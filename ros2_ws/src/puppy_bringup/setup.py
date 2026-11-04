from setuptools import setup

package_name = 'puppy_bringup'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    scripts=['scripts/puppypi-camera-detect'],
    install_requires=['setuptools'],
    entry_points={'console_scripts': ['media_server = puppy_bringup.media_server:main']},
    zip_safe=True,
    maintainer='Ksyear',
    maintainer_email='72062965+Ksyear@users.noreply.github.com',
    description='PuppyPi Media v1 server (camera and LiDAR map for VR/browser) and USB camera mode selector.',
    license='Apache-2.0',
)
