from setuptools import setup, find_packages

setup(
    name='my_pose_goal',
    version='0.0.0',
    packages=['my_pose_goal'],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'member_subscriber=my_pose_goal.member_subscriber:main',
        ],
    },
)




