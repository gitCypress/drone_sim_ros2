from setuptools import find_packages, setup

package_name = 'airsim_image_saver'

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
    maintainer_email='cypress@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_saver_node = airsim_image_saver.image_saver_node:main',
            'keyboard_teleop_node = airsim_image_saver.keyboard_teleop_node:main',
            'mission_node = airsim_image_saver.mission_node:main',
        ],
    },
)
