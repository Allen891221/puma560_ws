# -*- coding: utf-8 -*-
"""RViz2 + joint_state_publisher_gui：在不開 Gazebo 的情況下檢查 DH 座標系。

    ros2 launch puma560_description display.launch.py
    ros2 launch puma560_description display.launch.py show_axes:=false joint_limits:=real
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('puma560_description')
    xacro_file = os.path.join(pkg, 'urdf', 'puma560.urdf.xacro')

    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' show_axes:=', LaunchConfiguration('show_axes'),
        ' joint_limits:=', LaunchConfiguration('joint_limits'),
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('show_axes', default_value='true'),
        DeclareLaunchArgument('joint_limits', default_value='default',
                              description='default（±180°）或 real（僅供參考）'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'rviz', 'puma560.rviz')]),
    ])
