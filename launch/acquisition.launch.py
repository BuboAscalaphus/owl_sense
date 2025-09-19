from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import pyvizionsdk as sdk


def generate_launch_description():
    pkg_dir = get_package_share_directory('owl_sense')
    
    camera_node_path_cmd = DeclareLaunchArgument(
        'camera_config_path',
        default_value=pkg_dir + '/config/camera_config.yaml',
        description='Full path to the config file of the camera node')

    _, cameras = sdk.VxDiscoverCameraDevices()
    camera_nodes = []
    for id, cam in enumerate(cameras):
        camera_node = Node(
            package='owl_sense',
            executable='camera_node',
            name='camera_node',
            parameters=[
                LaunchConfiguration('camera_config_path'),
                {'camera_id': id,
                 'camera_name':cam}
            ],
            output='screen',
            namespace=f'camera{id}'
        )
        camera_nodes.append(camera_node)

    return LaunchDescription([
        camera_node_path_cmd,
        *camera_nodes
    ])
