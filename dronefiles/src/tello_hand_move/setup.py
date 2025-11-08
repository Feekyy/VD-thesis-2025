from setuptools import setup, find_packages

package_name = 'tello_hand_move'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(include=[package_name, f'{package_name}.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': 
        [
            'teleop_wasd_altitude = tello_hand_move.teleop_wasd_altitude:main',
        ],
    },
)