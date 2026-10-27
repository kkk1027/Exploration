#!/usr/bin/env python3
"""nav_only.launch.py —— 只开"导航栈"的窄道/通行性测试台（2026-10-27）。

为什么单独做一个（前三轮踩的坑都在这行注释里）：
  * `nav_verify.launch.py` 会把车端 manager/RRT/看门狗/session_driver 一起拉起，
    而 `mapping_tf_guard` 是"门"节点：它按**系统时间**比对 sim 时间戳的 TF，
    判不健康就 exit 2 → 整栈自拆；外来目标还会被 session/watchdog 取消。
  * `nav_sim.launch.py` 用 fake_base，没有物理，会把车"推"进占据格。
  ⇒ 本文件 = Gazebo 物理 + **静态地图**（不跑 SLAM，坐标与世界同帧）
    + `nav2_bringup/navigation_launch.py`（车端那份参数）+ 计划体检 + 接触探针。
    不含 manager / RRT / watchdog / session_driver / tf_guard → 目标不会被抢，
    也没有车端安全闸（这正是"把口径交还 nav2"要观察的形态）。

坐标约定：静态地图由"从世界 (0,0,0,yaw=0) 起图"的轮次生成，机器人也生成在
(0,0,0)，并发布 map→odom_combined 单位变换 ⇒ **map 帧 == world 帧**（不用再做换算）。

用法：
  ros2 launch stemm_cartographer_exploration_sim nav_only.launch.py
  ros2 launch ... nav_only.launch.py map_yaml:=/path/map.yaml rviz:=true gui:=false
然后给目标（map 帧）：
  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: -7.7, y: -0.2}, orientation: {z: -0.707, w: 0.707}}}}"
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_PARAMS = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
              'stemm_safe_nav2_params.yaml')
DEFAULT_WORLD = ('/home/kbf/stemm_sim_ws/install/wheeltec_two_wheel_model/share/'
                 'wheeltec_two_wheel_model/world/aws_small_house.world')


def generate_launch_description():
    sim_pkg = get_package_share_directory('stemm_cartographer_exploration_sim')
    rviz_cfg = os.path.join(sim_pkg, 'config', 'sim_vision.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')

    # 1) 机器人活着（Gazebo 物理 + TF + /odom_combined），默认生成在世界原点
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wheeltec_two_wheel_model'), 'launch', 'sim.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui'),
                          'use_sim_time': use_sim_time}.items())

    # 2) 静态地图（不跑 SLAM）+ map→odom_combined 单位变换 ⇒ map 帧 == world 帧
    map_server = Node(package='nav2_map_server', executable='map_server',
                      name='map_server', output='screen',
                      parameters=[{'use_sim_time': use_sim_time,
                                   'yaml_filename': map_yaml,
                                   'topic_name': '/map'}])
    lifecycle_maps = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                          name='lifecycle_manager_maps', output='screen',
                          parameters=[{'use_sim_time': use_sim_time,
                                       'autostart': True,
                                       'node_names': ['map_server']}])
    map_to_odom = Node(package='tf2_ros', executable='static_transform_publisher',
                       name='map_to_odom_static', output='screen',
                       arguments=['0', '0', '0', '0', '0', '0',
                                  'map', 'odom_combined'])

    # 3) 只要 nav2（车端那份参数；不起 manager/RRT/watchdog/session/tf_guard）
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'params_file': LaunchConfiguration('nav_params'),
                          'autostart': 'true',
                          'use_composition': 'False'}.items())

    # 4) 计划体检（每条 /plan 量最小净宽）+ 接触探针（有物理才有真值）
    audit = Node(package='stemm_cartographer_exploration_sim', executable='nav_plan_audit',
                 name='nav_plan_audit', output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'pass_width': LaunchConfiguration('pass_width'),
                              'veto': LaunchConfiguration('plan_veto'),
                              'log_path': LaunchConfiguration('plan_audit_log')}])
    contacts = Node(package='stemm_cartographer_exploration_sim', executable='collision_probe',
                    name='collision_probe', output='screen',
                    parameters=[{'use_sim_time': use_sim_time,
                                 'log_path': LaunchConfiguration('contact_log')}])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=DEFAULT_WORLD),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('map_yaml',
                              default_value='/home/kbf/stemm_sim_ws/maps/explore_latest.yaml'),
        DeclareLaunchArgument('nav_params', default_value=NAV_PARAMS),
        DeclareLaunchArgument('pass_width', default_value='1.05'),
        DeclareLaunchArgument('plan_veto', default_value='false'),
        DeclareLaunchArgument('plan_audit_log', default_value=''),
        DeclareLaunchArgument('contact_log', default_value=''),
        sim, map_server, lifecycle_maps, map_to_odom, nav2, audit, contacts, rviz,
    ])
