from setuptools import setup

package_name = 'tello_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@localhost',
    description='Tello circle detector nodes',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_camera_contour = tello_camera.tello_camera_contour:main',
            'tello_camera_hough = tello_camera.tello_camera_hough:main',
        ],
    },
)
