from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

def generate_launch_description():
    """
    Launch TurtleBot4 Simulation (Headless + SLAM + Nav2).
    - Default World: Maze (lighter than Depot)
    - Headless Gazebo (no GUI)
    - SLAM Toolbox active
    - Nav2 active
    - RViz visualization
    """
    
    # Get package directories
    pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_turtlebot4_gz_gui_plugins = get_package_share_directory('turtlebot4_gz_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')
    pkg_irobot_create_gz_bringup = get_package_share_directory('irobot_create_gz_bringup')
    pkg_irobot_create_gz_plugins = get_package_share_directory('irobot_create_gz_plugins')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    demo_bot = get_package_share_directory('demo_bot')
    
    # Declare World Argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze',
        description='Gazebo world to launch (maze, depot, warehouse)'
    )
    
    # Set Gazebo resource path (Copied from sim.launch.py)
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(pkg_turtlebot4_gz_bringup, 'worlds'),
            os.path.join(pkg_irobot_create_gz_bringup, 'worlds'),
            str(Path(pkg_turtlebot4_description).parent.resolve()),
            str(Path(pkg_irobot_create_description).parent.resolve())
        ])
    )

    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=':'.join([
            os.path.join(pkg_turtlebot4_gz_gui_plugins, 'lib'),
            os.path.join(pkg_irobot_create_gz_plugins, 'lib')
        ])
    )
    
    # Launch Gazebo (Headless)
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', ['-s -r ', LaunchConfiguration('world'), '.sdf']) # -s: Server only (Headless), -r: Run immediately
        ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ])

    # Spawn Robot with SLAM and Nav2
    spawn_launch = PathJoinSubstitution([pkg_turtlebot4_gz_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawn_launch]),
        launch_arguments={
            'model': 'standard',
            'rviz': 'true',
            'slam': 'true',
            'nav2': 'true',
            'localization': 'false',
            'params_file': PathJoinSubstitution([demo_bot, 'config', 'nav2_fast_frontier.yaml'])
        }.items()
    )
    
    return LaunchDescription([
        world_arg,
        gz_resource_path,
        gz_gui_plugin_path,
        gazebo,
        clock_bridge,
        spawn_robot
    ])
