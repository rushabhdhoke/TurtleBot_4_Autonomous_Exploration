#!/usr/bin/env python3
"""
Auto-undock script for TurtleBot4.
Automatically undocks the robot from its charging station before exploration.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Undock
import time


class AutoUndock(Node):
    def __init__(self):
        super().__init__('auto_undock')
        self.get_logger().info('Auto-undock node started')
        
        # Create action client for undocking
        self._action_client = ActionClient(self, Undock, '/undock')
        
    def send_undock_goal(self):
        """Send undock action goal to robot"""
        self.get_logger().info('Waiting for undock action server...')
        self._action_client.wait_for_server()
        
        goal_msg = Undock.Goal()
        self.get_logger().info('Sending undock goal...')
        
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Undock goal rejected')
            return False
            
        self.get_logger().info('Undock goal accepted, waiting for result...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        self.get_logger().info(f'Undock completed! Result: {result}')
        return True


def main(args=None):
    rclpy.init(args=args)
    
    auto_undock = AutoUndock()
    
    # Wait a bit for robot to initialize
    time.sleep(5)
    
    # Attempt to undock
    success = auto_undock.send_undock_goal()
    
    if success:
        auto_undock.get_logger().info('Robot successfully undocked! Ready for exploration.')
    else:
        auto_undock.get_logger().warn('Undock failed or robot already undocked.')
    
    auto_undock.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
