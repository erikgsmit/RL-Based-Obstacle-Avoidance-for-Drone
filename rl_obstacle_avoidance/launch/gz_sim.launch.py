from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_rl_drone = get_package_share_directory('rl_obstacle_avoidance')

    world_file = PathJoinSubstitution([pkg_rl_drone, 'worlds', 'drone_world.sdf'])
    bridge_config = PathJoinSubstitution([pkg_rl_drone, 'config', 'bridge.yaml'])

    # Declare bridge config as a launch argument
    declare_bridge_config = DeclareLaunchArgument(
        'config_file',
        default_value=bridge_config,
        description='YAML file for ROS-Gazebo bridge'
    )

    return LaunchDescription([
        declare_bridge_config,  # Add bridge argument

        # Launch Gazebo with the world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={'gz_args': world_file}.items(),
        ),

        # Start ROS-Gazebo Bridge
        RosGzBridge(
            bridge_name="ros_gz_bridge",
            config_file=bridge_config,
        ),
        
         Node(
            package='rl_obstacle_avoidance',
            executable='q_learning',
            name='q_learning_agent',
            output='screen'
        ),
    ])
