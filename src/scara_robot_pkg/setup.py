from setuptools import find_packages, setup
from glob import glob

package_name = 'scara_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/models/conveyor_belt_2', glob('models/conveyor_belt_2/*.config') + glob('models/conveyor_belt_2/*.sdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='apostolos-ubuntu-pc',
    maintainer_email='apostolos-ubuntu-pc@todo.todo',
    description='SCARA robot ROS 2 package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pick_place_cycle = scara_robot_pkg.pick_place_cycle:main',
            'scene_publisher = scara_robot_pkg.scene_publisher:main',
            'move_joints = scara_robot_pkg.move_joints:main',
        ],
    },
)