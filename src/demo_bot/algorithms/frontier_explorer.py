#!/usr/bin/env python3
"""
Fast Directional Explorer for ROS 2
Optimized for speed: targets <5min exploration
Strategy: Directional commitment + wall-following + smart frontier selection
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Twist
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import numpy as np
from enum import Enum
import time

class ExplorationMode(Enum):
    DIRECTIONAL = 1  # Explore all frontiers in one direction
    WALL_FOLLOW = 2   # Follow walls systematically
    COVERAGE = 3      # Final cleanup

class FastDirectionalExplorer(Node):
    def __init__(self):
        super().__init__('fast_explorer')
        
        # QoS profile for costmap (must match Nav2's QoS)
        costmap_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.costmap_callback,
            costmap_qos  # Use proper QoS!
        )
        
        # TF2 for localization checking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.slam_ready = False  # Track SLAM initialization
        
        # Action client for navigation
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # State
        self.costmap = None
        self.current_mode = ExplorationMode.DIRECTIONAL
        self.explored_directions = set()
        self.last_goal_time = time.time()
        
        # GOAL TRACKING (Hysteresis/Stickiness)
        self.current_goal = None  # Currently active goal
        self.goal_in_progress = False  # Is a goal currently executing?
        self.goal_handle = None  # Handle to current goal
        self.failed_goals = []  # Track failed goals to avoid retrying
        
        # REGION-AWARE STATE (Directional Momentum)
        self.last_direction = None  # Track last direction for momentum
        self.recovery_in_progress = False
        
        # Publisher for recovery behavior (spinning)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # AREA COMMITMENT (prevents jumping between regions)
        
        # AREA COMMITMENT (prevents jumping between regions)
        self.current_area = None  # Track which area we're exploring (NORTH/SOUTH/EAST/WEST)
        self.area_commitment_count = 0  # How many goals completed in current area
        self.min_area_commitment = 3  # Explore at least 3 frontiers in an area before switching
        
        # Parameters (tuned for speed)
        self.min_frontier_size = 10  # pixels
        self.frontier_cluster_distance = 20  # pixels
        self.goal_timeout = 60.0  # seconds (increased for commitment)
        
        # Start exploration (REDUCED frequency for goal commitment)
        self.exploration_timer = self.create_timer(10.0, self.exploration_cycle)  # 10s instead of 5s
        
        self.get_logger().info('🚀 Fast Directional Explorer started!')
    
    def costmap_callback(self, msg):
        """Receive and process costmap"""
        self.costmap = msg
        self.get_logger().info(f'Costmap received: {msg.info.width}x{msg.info.height}')
    
    def check_slam_ready(self):
        """Check if SLAM has initialized and is providing localization"""
        try:
            # Try to get map→odom transform
            transform = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time(),  # Latest available
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # If we got here, SLAM is ready!
            if not self.slam_ready:
                self.get_logger().info('✅ SLAM localization ready! Starting exploration...')
                self.slam_ready = True
            
            return True
            
        except TransformException as ex:
            # Transform not available - SLAM not ready
            self.slam_ready = False
            return False
    
    def exploration_cycle(self):
        """Main exploration loop - only runs when no goal is active"""
        if self.costmap is None:
            self.get_logger().warn('Waiting for costmap...')
            return
        
        # CHECK SLAM LOCALIZATION FIRST!
        if not self.check_slam_ready():
            self.get_logger().warn('⏳ Waiting for SLAM to initialize (map→odom transform)...')
            return
        
        # HYSTERESIS: Skip if goal is still in progress!
        if self.goal_in_progress:
            elapsed = time.time() - self.last_goal_time
            self.get_logger().info(f'⏳ Goal in progress ({elapsed:.1f}s), waiting for completion...')
            
            # Timeout check
            if elapsed > self.goal_timeout:
                self.get_logger().warn(f'⚠️ Goal timeout ({self.goal_timeout}s), considering it failed')
                self.goal_in_progress = False
                if self.current_goal:
                    self.failed_goals.append(self.current_goal)
            else:
                return  # Skip this cycle, let the goal finish!
        
        # Find frontiers
        frontiers = self.detect_frontiers()
        
        if not frontiers:
            self.get_logger().warn('⚠️ No frontiers found! Attempting 360° spin recovery...')
            self.spin_360()
            return
        
        # Select best frontier based on current mode
        goal = self.select_best_frontier(frontiers)
        
        # Check if this goal was already tried and failed
        if goal and goal in self.failed_goals:
            self.get_logger().warn(f'Skipping previously failed goal: {goal}')
            return
        
        if goal:
            self.send_nav_goal(goal)
    
    def spin_360(self):
        """Perform a 360 degree spin to update costmap/SLAM"""
        self.get_logger().info('🔄 RECOVERY: Spinning 360° to clear map...')
        
        twist = Twist()
        twist.angular.z = 1.0  # rad/s
        
        # Publish for 6.5 seconds (approx 2*pi at 1.0 rad/s)
        # We can't block easily here, so we'll just publish a few times
        # Better approach: Publish and rely on next cycle to pick up
        
        # Ideally this should be a state, but for simplicity we burst publish
        for _ in range(10):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
            
        self.get_logger().info('🔄 Spin command sent')
        # Clear failed goals to retry them after spin
        self.failed_goals.clear()
    
    def detect_frontiers(self):
        """
        Detect frontier points (boundaries between known free and unknown)
        Returns list of (x, y, cluster_id, direction)
        """
        if self.costmap is None:
            return []
        
        width = self.costmap.info.width
        height = self.costmap.info.height
        data = np.array(self.costmap.data).reshape((height, width))
        
        frontiers = []
        
        # Simple frontier detection: free cells adjacent to unknown
        for y in range(1, height-1):
            for x in range(1, width-1):
                if data[y, x] == 0:  # Free space
                    # Check neighbors for unknown (-1)
                    neighbors = [
                        data[y-1, x], data[y+1, x],
                        data[y, x-1], data[y, x+1]
                    ]
                    if -1 in neighbors:  # Adjacent to unknown
                        # Calculate direction from robot
                        direction = self.calculate_direction(x, y)
                        frontiers.append((x, y, direction))
        
        self.get_logger().info(f'Found {len(frontiers)} frontier cells')
        
        # Cluster frontiers
        clustered = self.cluster_frontiers(frontiers)
        return clustered
    
    def calculate_direction(self, x, y):
        """Calculate cardinal direction from robot (N, S, E, W)"""
        # Assuming robot is near center of costmap
        center_x = self.costmap.info.width // 2
        center_y = self.costmap.info.height // 2
        
        dx = x - center_x
        dy = y - center_y
        
        if abs(dx) > abs(dy):
            return 'EAST' if dx > 0 else 'WEST'
        else:
            return 'NORTH' if dy > 0 else 'SOUTH'
    
    def cluster_frontiers(self, frontiers):
        """Group nearby frontiers into clusters"""
        if not frontiers:
            return []
        
        clusters = []
        visited = set()
        
        for i, (x, y, direction) in enumerate(frontiers):
            if i in visited:
                continue
            
            cluster = [(x, y)]
            visited.add(i)
            
            # Find nearby points
            for j, (x2, y2, _) in enumerate(frontiers):
                if j not in visited:
                    dist = np.sqrt((x - x2)**2 + (y - y2)**2)
                    if dist < self.frontier_cluster_distance:
                        cluster.append((x2, y2))
                        visited.add(j)
            
            if len(cluster) >= self.min_frontier_size:
                # Cluster center
                cx = int(np.mean([p[0] for p in cluster]))
                cy = int(np.mean([p[1] for p in cluster]))
                clusters.append((cx, cy, direction, len(cluster)))
        
        self.get_logger().info(f'Clustered into {len(clusters)} frontier regions')
        return clusters
    
    def select_best_frontier(self, frontiers):
        """
        PROXIMITY-FIRST STRATEGY with expanding search radius
        
        Strategy:
        1. Try to find frontiers within 1m (close)
        2. If none, expand to 2m (nearby)
        3. If none, expand to 4m (medium)
        4. If none, pick ANY frontier (far - last resort)
        
        Within each radius, pick the largest cluster for better coverage
        """
        if not frontiers:
            return None
        
        # Get robot position (center of costmap)
        center_x = self.costmap.info.width // 2
        center_y = self.costmap.info.height // 2
        resolution = self.costmap.info.resolution
        
        # Calculate distances for all frontiers
        frontiers_with_dist = []
        for x, y, direction, size in frontiers:
            # Distance in pixels
            dist_px = np.sqrt((x - center_x)**2 + (y - center_y)**2)
            # Distance in meters
            dist_m = dist_px * resolution
            frontiers_with_dist.append((x, y, direction, size, dist_px, dist_m))
        
        # EXPANDING SEARCH RADIUS STRATEGY
        search_radii = [
            (1.0, "very close"),
            (2.0, "close"),
            (4.0, "nearby"),
            (8.0, "medium"),
            (float('inf'), "far")  # Last resort
        ]
        
        for radius_m, radius_name in search_radii:
            # Filter frontiers within this radius
            candidates = [f for f in frontiers_with_dist if f[5] <= radius_m]
            
            if candidates:
                # Found frontiers in this radius!
                
                # DIRECTIONAL MOMENTUM: Prioritize frontiers in same direction
                if self.last_direction and radius_m <= 4.0:  # Only for near/medium range
                    momentum_candidates = [f for f in candidates if f[2] == self.last_direction]
                    if momentum_candidates:
                        self.get_logger().info(f'🚀 Momentum: Keeping direction {self.last_direction}')
                        candidates = momentum_candidates
                
                # Sort by SIZE (pick largest cluster for better coverage)
                candidates.sort(key=lambda f: f[3], reverse=True)
                
                x, y, direction, size, dist_px, dist_m = candidates[0]
                
                # Update last direction
                self.last_direction = direction
                
                self.get_logger().info(
                    f'📍 Selected {radius_name} frontier: '
                    f'{direction} at ({x}, {y}), '
                    f'dist={dist_m:.2f}m, size={size}'
                )
                
                return self.grid_to_world(x, y)
        
        # Should never reach here (last radius is infinite)
        return None
    
    def pick_new_area(self, by_direction):
        """Pick the area with the most frontiers (richest unexplored region)"""
        # Count frontiers in each direction
        frontier_counts = {d: len(f) for d, f in by_direction.items()}
        
        # Pick direction with most frontiers
        best_direction = max(frontier_counts, key=frontier_counts.get)
        
        self.get_logger().info(f'Area counts: {frontier_counts}')
        
        return best_direction
    
    def grid_to_world(self, x, y):
        """Convert grid coordinates to world coordinates"""
        origin = self.costmap.info.origin
        resolution = self.costmap.info.resolution
        
        world_x = origin.position.x + (x * resolution)
        world_y = origin.position.y + (y * resolution)
        
        return (world_x, world_y)
    
    def send_nav_goal(self, goal_pos):
        """Send navigation goal to Nav2 with callbacks"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 action server not available!')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_pos[0]
        goal_msg.pose.pose.position.y = goal_pos[1]
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(f'📍 Sending goal: ({goal_pos[0]:.2f}, {goal_pos[1]:.2f})')
        
        # Mark goal as in progress
        self.current_goal = goal_pos
        self.goal_in_progress = True
        self.last_goal_time = time.time()
        
        # Send goal with callback
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Called when Nav2 accepts/rejects the goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('❌ Goal rejected by Nav2')
            self.goal_in_progress = False
            if self.current_goal:
                self.failed_goals.append(self.current_goal)
            return
        
        self.get_logger().info('✅ Goal accepted, navigating...')
        self.goal_handle = goal_handle
        
        # Get result callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Called when Nav2 finishes the goal (success or failure)"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('🎉 Goal reached successfully!')
            self.goal_in_progress = False
        elif status == 5:  # ABORTED
            self.get_logger().warn('⚠️ Goal aborted')
            self.goal_in_progress = False
            if self.current_goal:
                self.failed_goals.append(self.current_goal)
        elif status == 6:  # CANCELED
            self.get_logger().warn('🚫 Goal canceled')
            self.goal_in_progress = False
        else:
            self.get_logger().warn(f'Goal finished with status: {status}')
            self.goal_in_progress = False
        
        # Clear current goal
        self.current_goal = None
        self.goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    explorer = FastDirectionalExplorer()
    rclpy.spin(explorer)
    explorer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
