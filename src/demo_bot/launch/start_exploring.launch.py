from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch Autonomous Exploration Logic.
    - Auto-undock script
    - Explore Lite node
    
    Assumes simulation (headless_sim.launch.py) is already running.
    """
    
    demo_bot = get_package_share_directory('demo_bot')
    
    # Auto-undock node (starts immediately)
    auto_undock = ExecuteProcess(
        cmd=[PathJoinSubstitution([demo_bot, '..', '..', 'lib', 'demo_bot', 'auto_undock.py'])],
        output='screen'
    )
    
    # Fast Explorer node (starts after 5 seconds to allow undocking to initiate/complete)
    fast_explorer = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[PathJoinSubstitution([demo_bot, '..', '..', 'lib', 'demo_bot', 'frontier_explorer.py'])],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        auto_undock,
        fast_explorer
    ])
