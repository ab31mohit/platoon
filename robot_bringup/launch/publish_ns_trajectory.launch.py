from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
import yaml
import os
from ament_index_python.packages import get_package_share_directory


def load_robots_config(context, *args, **kwargs):
    """Load robot configurations from the YAML file and create nodes."""
    # Get the path to the YAML file
    config_file_path = LaunchConfiguration('config_file').perform(context)
    
    # Load the YAML file
    with open(config_file_path, 'r') as file:
        config = yaml.safe_load(file)
    
    nodes = []
    
    # Create a node for each robot in the config
    for robot in config['robots']:
        # run the trajectory (path) visualization for every robot
        node = Node(
            package='robot_bringup', 
            executable='ns_robot_trajectory_node.py',
            name=f'{robot["name"]}_trajectory_node',
            namespace=robot['namespace'],
            parameters=[{
                'robot_name': robot['name'],
                'odom_topic': robot['odom_topic'],
                'trajectory_topic': robot['trajectory_topic']
            }],
            output='screen'
        )
        nodes.append(node)
    
    return nodes


def generate_launch_description():
    """Generate launch description with multiple trajectory nodes."""
    # Define the launch configuration variable
    pkg_dir = get_package_share_directory('robot_bringup')  # Replace with your package name
    default_config_path = os.path.join(pkg_dir, 'param', 'ns_trajectory.yaml')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the YAML file containing robot configurations'
    )
    
    # Use OpaqueFunction to load robots at launch time
    spawn_robots = OpaqueFunction(function=load_robots_config)
    
    return LaunchDescription([
        config_file_arg,
        spawn_robots
    ])