from glob import glob 
from setuptools import find_packages, setup

package_name = 'my_robot_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=['my_robot_goal'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/trajectory_publisher_launch.py', 'launch/robot_plan.launch.py']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'my_pose_goal'],
    zip_safe=True,
    maintainer='safa',
    maintainer_email='safa@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ 
        'console_scripts': [
            'my_robot_goal = my_robot_goal.my_robot_goal:main',
            'member_subscriber = my_pose_goal.member_subscriber:main',
        ],
    },
)
