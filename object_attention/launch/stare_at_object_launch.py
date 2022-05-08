import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('object_attention')
    
    objects_file = os.path.join(example_dir, 'params/params.yaml')
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')



    # Specify the actions
    move_cmd = Node(
        package='object_attention',
        executable='stare_node',
        name='stare_node',
        output='screen',
        parameters=[objects_file])

   
    
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)

    # Declare the launch options


    ld.add_action(move_cmd)

    return ld