# -*- coding: utf-8 -*-
"""PUMA560 教學模擬主要 launch：Gazebo Fortress + ros2_control + 教學 GUI（+ 選配 RViz2）

    ros2 launch puma560_gazebo teaching.launch.py
    ros2 launch puma560_gazebo teaching.launch.py rviz:=true
    ros2 launch puma560_gazebo teaching.launch.py software_rendering:=true render_engine:=ogre
    ros2 launch puma560_gazebo teaching.launch.py joint_limits:=real show_axes:=false

啟動順序：
    Gazebo → robot_state_publisher → 生成手臂（ros_gz_sim create）
    → 生成完成後啟動 joint_state_broadcaster → forward_position_controller → GUI
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('puma560_description')
    gz_pkg = get_package_share_directory('puma560_gazebo')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(desc_pkg, 'urdf', 'puma560.urdf.xacro')
    controllers_file = os.path.join(gz_pkg, 'config', 'controllers.yaml')
    world_file = os.path.join(gz_pkg, 'worlds', 'teaching.sdf')
    rviz_file = os.path.join(desc_pkg, 'rviz', 'puma560.rviz')

    rviz = LaunchConfiguration('rviz')
    gui = LaunchConfiguration('gui')
    joint_limits = LaunchConfiguration('joint_limits')

    robot_description = ParameterValue(Command([
        'xacro ', xacro_file,
        ' use_gazebo:=true',
        ' controllers_file:=', controllers_file,
        ' show_axes:=', LaunchConfiguration('show_axes'),
        ' joint_limits:=', joint_limits,
    ]), value_type=str)

    # --- Gazebo Fortress ---
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [
            '-r -v 2 ', LaunchConfiguration('world'),
            ' --render-engine ', LaunchConfiguration('render_engine')]}.items())

    # /clock：Gazebo → ROS（use_sim_time 需要）
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', '-name', 'puma560'],
        output='screen')

    # 手臂生成後，把 Gazebo 相機移到手臂附近（位置 (1.0,−0.8,1.3)，朝向手臂）
    camera_req = ('pose: {position: {x: 1.0, y: -0.8, z: 1.3}, '
                  'orientation: {x: -0.1720, y: 0.0697, z: 0.9107, w: 0.3690}}')
    # GUI 的 3D 場景載入需要一點時間：等 /gui/move_to/pose 服務出現，再等 5 秒後送出
    camera = ExecuteProcess(
        cmd=['bash', '-c',
             'for i in $(seq 1 30); do ign service -l | grep -q /gui/move_to/pose && break; '
             'sleep 1; done; sleep 5; '
             'ign service -s /gui/move_to/pose --reqtype ignition.msgs.GUICamera '
             '--reptype ignition.msgs.Boolean --timeout 5000 --req "%s"' % camera_req],
        output='log')

    jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen')

    fpc = Node(
        package='controller_manager', executable='spawner',
        arguments=['forward_position_controller', '--controller-manager', '/controller_manager'],
        output='screen')

    teaching_gui = Node(
        package='puma560_teaching_gui', executable='teaching_gui',
        parameters=[{'use_sim_time': True, 'joint_limits': joint_limits}],
        condition=IfCondition(gui), output='screen')

    rviz_node = Node(
        package='rviz2', executable='rviz2', arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz), output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false', description='是否開啟 RViz2'),
        DeclareLaunchArgument('world', default_value=world_file,
                              description='Gazebo world 檔（預設為本套件的 teaching.sdf）'),
        DeclareLaunchArgument('gui', default_value='true', description='是否開啟教學 GUI'),
        DeclareLaunchArgument('show_axes', default_value='true', description='是否畫出各 link 座標軸'),
        DeclareLaunchArgument('joint_limits', default_value='default',
                              description='default（±180°）或 real（僅供參考）'),
        DeclareLaunchArgument('software_rendering', default_value='false',
                              description='true：設定 LIBGL_ALWAYS_SOFTWARE=1（VM / 無 GPU）'),
        DeclareLaunchArgument('render_engine', default_value='ogre2',
                              description='ogre2（預設）或 ogre（舊版，軟體渲染較穩）'),

        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1',
                               condition=IfCondition(LaunchConfiguration('software_rendering'))),

        gazebo, clock_bridge, rsp, spawn,
        # 手臂生成完成後才啟動控制器，避免 controller_manager 尚未就緒
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, camera])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[fpc])),
        RegisterEventHandler(OnProcessExit(target_action=fpc, on_exit=[teaching_gui])),
        rviz_node,
    ])
