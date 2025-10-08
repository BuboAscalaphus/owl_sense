from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import pyvizionsdk as sdk

def generate_launch_description():
    pkg_dir = get_package_share_directory('owl_sense')

    # Generic args (no camera knobs here)
    camera_impl_arg = DeclareLaunchArgument(
        'camera_impl',
        default_value='owl_sense.camera_ar082:CameraAR082x',
        description='Python path to the camera driver (module:Class)'
    )
    driver_yaml_arg = DeclareLaunchArgument(
        'driver_params_file',
        default_value=os.path.join(pkg_dir, 'config', 'cameras', 'ar082.params.yaml'),
        description='Path to per-camera driver YAML'
    )

    camera_impl = LaunchConfiguration('camera_impl')
    driver_yaml = LaunchConfiguration('driver_params_file')

    # Discover cameras via SDK and spawn a node per camera
    _, cameras = sdk.VxDiscoverCameraDevices()
    camera_nodes = []
    for idx, cam_name in enumerate(cameras):
        camera_nodes.append(
            Node(
                package='owl_sense',
                executable='camera_node',
                name=f'camera_node_{idx}',
                namespace=f'camera{idx}',
                output='screen',
                parameters=[{
                    'camera_impl': camera_impl,
                    'driver_params_file': driver_yaml,
                    'camera_name': cam_name,
                    # Optional: per-node timeout override
                    'frame_timeout_ms': 120
                }]
            )
        )

    return LaunchDescription([
        camera_impl_arg,
        driver_yaml_arg,
        *camera_nodes
    ])

