from setuptools import setup, find_packages

package_name = 'tello_hand_move'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    data_files=
    [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': 
        [
            'teleop_wasd_altitude = tello_hand_move.teleop_wasd_altitude:main',
        ],
    },
)