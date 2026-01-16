from setuptools import setup
import os
from glob import glob

package_name = 'shadow_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shadow Control Team',
    maintainer_email='your_email@example.com',
    description='Vision-based bimanual teleoperation for humanoid robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = nodes.pose_publisher:main',
            'pose_subscriber = nodes.pose_subscriber:main',
            'image_viewer = nodes.image_viewer:main',
            'robot_controller = nodes.robot_controller:main',
        ],
    },
)
