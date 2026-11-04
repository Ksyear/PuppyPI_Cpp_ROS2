from setuptools import setup

package_name = 'puppy_vr_bridge'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/vr_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ksyear',
    maintainer_email='72062965+Ksyear@users.noreply.github.com',
    description='Meta Quest / phone Protocol v2 (UDP 5001 JSON) receiver and FPV UDP sender (5006) for PuppyPi on ROS 2 Humble.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_protocol_v2_receiver = puppy_vr_bridge.receiver_node:main',
            'camera_udp_sender = puppy_vr_bridge.camera_udp_sender:main',
        ],
    },
)
