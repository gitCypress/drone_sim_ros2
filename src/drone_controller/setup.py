from setuptools import find_packages, setup

package_name = 'drone_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cypress',
    maintainer_email='2730196143@qq.com',
    description='Drone controller package for AirSim simulation with image saving, keyboard teleoperation and mission control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver_node = drone_controller.image_saver_node:main',
            'keyboard_teleop_node = drone_controller.keyboard_teleop_node:main',
            'mission_node = drone_controller.mission_node:main',
        ],
    },
)
