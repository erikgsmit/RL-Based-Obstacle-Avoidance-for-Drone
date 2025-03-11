from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_rl_drone = get_package_share_directory('rl_obstacle_avoidance')

    world_file = PathJoinSubstitution([pkg_rl_drone, 'worlds', 'drone_world.sdf'])
    # model_file = PathJoinSubstitution([pkg_rl_drone, 'models', 'parrot_bebop_2', 'model.sdf'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='Path to the Gazebo world file'
        ),

        # Launch Gazebo with the world file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
            ),
            launch_arguments={'gz_args': world_file}.items(),
        ),

       
    ])
