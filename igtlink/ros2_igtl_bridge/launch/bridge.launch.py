#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'node',
            default_value='server',
            description='OpenIGTLBridge Mode: client or server'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='18944',
            description='OpenIGTLBridge port number'
        ),
        DeclareLaunchArgument(
            'ip',
            default_value='172.16.43.1',
            description='OpenIGTLBridge IP address'
        ),

        LogInfo(msg=['node: ', LaunchConfiguration('node')]),
        LogInfo(msg=['port: ', LaunchConfiguration('port')]),
        LogInfo(msg=['ip: ', LaunchConfiguration('ip')]),
    ])
