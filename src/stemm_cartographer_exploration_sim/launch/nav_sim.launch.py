#!/usr/bin/env python3
"""nav_sim.launch.py —— **不用 Gazebo** 的 nav2 测试台（用户 2026-10-26 指示）。

组成：
  fake_base        运动学积分 /cmd_vel + 用离线地图 raycast 出 /scan（含真机雷达盲区）
  nav2 (bringup)   planner/controller/bt/behaviors + 两张代价地图，参数用车上那份
                   `stemm_safe_nav2_params.yaml`（所以测的就是车上那套配置）
  map_server       把离线地图发成 /map
  nav_plan_audit   每条 /plan 量"最小净宽/最长窄段"，穿窄处可 veto

没有 Gazebo ⇒ 起栈 5 s 级别、完全确定、不会再有物理接触/起栈抢时序的问题。
RViz 里给 2D Goal Pose 即可；地图与起点用 map_yaml / start_x,y,yaw 指定。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_PARAMS = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
              'stemm_safe_nav2_params.yaml')


def generate_launch_description():
    sim_pkg = get_package_share_directory('stemm_cartographer_exploration_sim')
    rviz_cfg = os.path.join(sim_pkg, 'config', 'sim_vision.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    base = Node(package='stemm_cartographer_exploration_sim', executable='fake_base',
                name='fake_base', output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'map_yaml': LaunchConfiguration('map_yaml'),
                             'start_x': LaunchConfiguration('start_x'),
                             'start_y': LaunchConfiguration('start_y'),
                             'start_yaw': LaunchConfiguration('start_yaw'),
                             'beams': LaunchConfiguration('beams')}])

    map_server = Node(package='nav2_map_server', executable='map_server',
                      name='map_server', output='screen',
                      parameters=[{'use_sim_time': use_sim_time,
                                   'yaml_filename': LaunchConfiguration('map_yaml')}])
    lifecycle_maps = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                          name='lifecycle_manager_maps', output='screen',
                          parameters=[{'use_sim_time': use_sim_time,
                                       'autostart': True,
                                       'node_names': ['map_server']}])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': LaunchConfiguration('nav_params'),
                          'autostart': 'true',
                          'use_composition': 'False'}.items())

    audit = Node(package='stemm_cartographer_exploration_sim', executable='nav_plan_audit',
                 name='nav_plan_audit', output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'pass_width': LaunchConfiguration('pass_width'),
                              'narrow_len_max': LaunchConfiguration('narrow_len_max'),
                              'car_width': LaunchConfiguration('car_width'),
                              'veto': LaunchConfiguration('plan_veto'),
                              'log_path': LaunchConfiguration('plan_audit_log')}])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_yaml',
                              default_value='/home/kbf/stemm_sim_ws/maps/explore_latest.yaml'),
        DeclareLaunchArgument('nav_params', default_value=NAV_PARAMS),
        DeclareLaunchArgument('start_x', default_value='0.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('beams', default_value='833'),
        DeclareLaunchArgument('pass_width', default_value='1.05'),
        DeclareLaunchArgument('narrow_len_max', default_value='1.00'),
        DeclareLaunchArgument('car_width', default_value='0.47'),
        DeclareLaunchArgument('plan_veto', default_value='true'),
        DeclareLaunchArgument('plan_audit_log', default_value=''),
        DeclareLaunchArgument('rviz', default_value='true'),
        base, map_server, lifecycle_maps,
        # nav2 等雷达/地图先出现再起（避免"TF 没来就规划"的噪声失败）
        TimerAction(period=2.0, actions=[nav2]),
        audit, rviz,
    ])
