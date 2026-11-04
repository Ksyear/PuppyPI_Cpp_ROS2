from setuptools import setup

package_name = 'puppy_lidar'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ksyear',
    maintainer_email='72062965+Ksyear@users.noreply.github.com',
    description='LDROBOT LD06 LiDAR driver and continuous odometry-free 2D mapping for PuppyPi (ROS 2 Humble).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ld06_driver = puppy_lidar.ld06_driver:main',
            'lidar_mapper = puppy_lidar.mapper_node:main',
        ],
    },
)
