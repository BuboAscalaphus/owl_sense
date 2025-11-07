from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import pyvizionsdk as sdk


def generate_launch_description():
    pkg_dir = get_package_share_directory('owl_sense')
    default_cfg = os.path.join(pkg_dir, 'config', 'cameras', 'ar082.app.yaml')

    camera_config_arg = DeclareLaunchArgument(
        'camera_config_path',
        default_value=default_cfg,
        description='Path to the camera YAML (not a ROS params file)'
    )

    # === FIX: discover cameras like your old file ===
    _, cameras = sdk.VxDiscoverCameraDevices()
    camera_nodes = []

    if not cameras:
        # Fallback: start one node anyway
        cameras = ['']

    for idx, cam in enumerate(cameras):
        camera_nodes.append(
            Node(
                package='owl_sense',
                executable='camera_node',
                name='camera_node',
                namespace=f'camera{idx}',
                output='screen',
                parameters=[{
                    # <== THE ONLY REAL FIX
                    'driver_params_file': LaunchConfiguration('camera_config_path'),
                    'camera_name': str(cam),
                }],
            )
        )

    return LaunchDescription([camera_config_arg, *camera_nodes])




