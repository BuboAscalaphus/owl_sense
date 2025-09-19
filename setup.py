from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'owl_sense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davide',
    maintainer_email='davide.botturi@prospecto.cloud',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = owl_sense.camera_node:main',
            'image_subscriber = owl_sense.image_subscriber:main',
            'sensor_node = owl_sense.sensor_node:main',  # if used
        ],
    },
)
