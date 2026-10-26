#!/usr/bin/env python3
"""nav_verify.launch.py —— **只拉导航栈 + RViz**（不跑探索），给操作者现场验证 nav2 用。

为什么单独做这个 launch（用户 2026-10-26 指示）：
  代价地图/规划器的问题与"探索建图"关系不大，重跑 180 s 整轮既慢又混入大量无关变量。
  这里只起：Gazebo 世界 + 机器人 + 车端建图/nav2 栈（auto_explore=false）+ RViz + 计划体检。
  操作者用 RViz 的 "2D Goal Pose" 手动给目标，就能直接看到：
    * RViz 里的 Inflation/Global costmap 是否真的把窄缝判成障碍；
    * Terminal 里 nav_plan_audit 每来一条 /plan 打一行"最小净宽 / 最长窄段 / 是否穿窄处"；
    * RViz 里 /plan_audit/markers：红点=计划压过的窄处，黄/橙球=整条计划最窄点。

用法：
  ros2 launch stemm_cartographer_exploration_sim nav_verify.launch.py \
      world:=<...world> gazebo_gui:=true rviz:=true pass_width:=1.05
"""
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.substitutions import EnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg = get_package_share_directory('stemm_cartographer_exploration_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_cfg = os.path.join(sim_pkg, 'config', 'sim_vision.rviz')

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wheeltec_two_wheel_model'),
            'launch', 'sim.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gazebo_gui'),
            'use_sim_time': use_sim_time,
        }.items())

    # 车端栈：auto_explore=false → 只起 cartographer + nav2 + 生命周期，不自动探索
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('stemm_cartographer_exploration'),
            'launch', 'stemm_cartographer_auto_mapping.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'auto_explore': 'false',
        }.items())

    # 车端 system_preflight 会校验 wheeltec-robot.service 等现场服务；
    # 仿真里必须把 sim_shims 垫片前置到 PATH，否则 preflight 直接失败把整栈拆掉
    # （第一次起 nav_verify 就栽在这：'wheeltec-robot.service is not active'）。
    shim_dir = os.path.join(get_package_prefix('stemm_cartographer_exploration_sim'),
                            'lib', 'stemm_cartographer_exploration_sim', 'sim_shims')

    audit = Node(package='stemm_cartographer_exploration_sim',
                 executable='nav_plan_audit', name='nav_plan_audit', output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'pass_width': LaunchConfiguration('pass_width'),
                              'narrow_len_max': LaunchConfiguration('narrow_len_max'),
                              'car_width': LaunchConfiguration('car_width'),
                              'veto': LaunchConfiguration('plan_veto'),
                              'veto_max': LaunchConfiguration('plan_veto_max'),
                              'log_path': LaunchConfiguration('plan_audit_log')}])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        SetEnvironmentVariable('PATH', [shim_dir, ':', EnvironmentVariable('PATH')]),
        DeclareLaunchArgument(
            'world',
            default_value='/home/kbf/stemm_sim_ws/install/wheeltec_two_wheel_model/share/'
                          'wheeltec_two_wheel_model/world/aws_small_house.world',
            description='Gazebo 世界文件（默认小房子）'),
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('pass_width', default_value='1.05'),
        DeclareLaunchArgument('narrow_len_max', default_value='1.00'),
        DeclareLaunchArgument('car_width', default_value='0.47'),
        DeclareLaunchArgument('plan_veto', default_value='true',
            description='计划穿窄处时直接取消该目标（"宁可不走"）；false=只报警'),
        DeclareLaunchArgument('plan_veto_max', default_value='10'),
        DeclareLaunchArgument('plan_audit_log', default_value=''),
        robot,
        # 车端栈必须**等机器人把传感器喂起来**再起：sensor_preflight 只等 8 s，
        # 和 Gazebo 同时启动会撞上"odom 还没动"直接判死（实测连续两次整栈被拆掉）。
        DeclareLaunchArgument('stack_delay', default_value='12.0'),
        TimerAction(period=LaunchConfiguration('stack_delay'), actions=[stack]),
        audit, rviz,
    ])
