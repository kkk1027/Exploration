"""实机自主探索栈的仿真接入（域 42，全程无 GUI）。

组装：
  1) 仿真机器人层  wheeltec_two_wheel_model/_t_classic.launch.py（官方差速插件 + 标定补偿）
  2) 实机探索栈    stemm_cartographer_exploration/stemm_cartographer_auto_mapping.launch.py（源码零改动）
  3) 会话驱动      session_driver（模拟小程序：到点调 set_mapping_done，之后跟踪返航、存图）

时序口径（操作者 2026-10-20 确认）：
  * 结束建图 = 调用与实机完全相同的那条服务 `/stemm_cartographer/set_mapping_done`
  * 探索限时 180 s，从"收到第一个探索目标"算起；返航不计入、且不设上限（同实机）
  * 返航目标 = map 原点（= 会话起点），由 cartographer_manager 自己处理
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, SetEnvironmentVariable,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node


def generate_launch_description():
    model = get_package_share_directory('wheeltec_two_wheel_model')
    nav_share = get_package_share_directory('wheeltec_two_wheel_nav')
    rviz_cfg = os.path.join(nav_share, 'rviz', 'slam.rviz')
    explore = get_package_share_directory('stemm_cartographer_exploration')
    sim_pkg = get_package_share_directory('stemm_cartographer_exploration_sim')

    world_cfg = LaunchConfiguration('world')
    explore_sec = LaunchConfiguration('explore_sec')
    stack_delay = LaunchConfiguration('stack_delay')
    frontier_source = LaunchConfiguration('frontier_source')
    auto_explore = LaunchConfiguration('auto_explore')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_out = LaunchConfiguration('map_out')

    # stack:=false 时只起机器人层 + 取证/脱困节点（P0 单元测试用，不拉车端整套栈）
    stack_on = IfCondition(LaunchConfiguration('stack'))

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(model, 'launch', '_t_classic.launch.py')),
        launch_arguments=[('gui', 'false'), ('world', world_cfg),
                          ('use_sim_time', use_sim_time),
                          ('spawn_x', LaunchConfiguration('spawn_x')),
                          ('spawn_y', LaunchConfiguration('spawn_y')),
                          ('spawn_yaw', LaunchConfiguration('spawn_yaw'))])

    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(explore, 'launch',
                         'stemm_cartographer_auto_mapping.launch.py')),
        launch_arguments=[('auto_explore', auto_explore),
                          ('frontier_source', frontier_source),
                          ('use_sim_time', use_sim_time)],
        condition=stack_on)

    # 与栈同时启动的运行时参数补丁（独立进程，0.5 s 重试，抢在 guard 的 30 s 判定前落地）
    patch_sim_time = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('stemm_cartographer_exploration_sim'),
                          'lib', 'stemm_cartographer_exploration_sim', 'patch_sim_time'),
             LaunchConfiguration('disable_retreat')],
        output='screen', condition=stack_on)

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    arbiter = Node(package='stemm_cartographer_exploration_sim', executable='frontier_arbiter',
                   name='frontier_arbiter', output='screen', condition=stack_on,
                   parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                                'republish_sec': 1.5, 'max_candidates': 6,
                                'blacklist_sec': 30.0, 'blacklist_radius': 0.5}])

    # P0-1：撞障取证（只读）。与机器人层同时启动，覆盖整轮（含启动期）。
    probe = Node(package='stemm_cartographer_exploration_sim',
                 executable='collision_probe', name='collision_probe',
                 output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'log_path': LaunchConfiguration('probe_log')}])

    # P0-3：雷达引导的"旋转 + 前进"脱困（与栈同时起，等车端节点就绪后自行等待触发）
    supervisor = Node(package='stemm_cartographer_exploration_sim',
                      executable='escape_supervisor', name='escape_supervisor',
                      output='screen',
                      parameters=[{'use_sim_time': use_sim_time,
                                   'enabled': LaunchConfiguration('escape_supervisor'),
                                   'log_path': LaunchConfiguration('supervisor_log')}])

    driver = Node(
        package='stemm_cartographer_exploration_sim',
        executable='session_driver',
        name='sim_session_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'explore_sec': explore_sec,
                     'map_out': map_out}])

    # 把仿真垫片目录前置到 PATH（只影响本次 launch 起的进程）
    shim_dir = os.path.join(
        get_package_prefix('stemm_cartographer_exploration_sim'),
        'lib', 'stemm_cartographer_exploration_sim', 'sim_shims')

    return LaunchDescription([
        SetEnvironmentVariable('PATH', [shim_dir, ':', EnvironmentVariable('PATH')]),
        DeclareLaunchArgument('world', default_value=os.path.join(
            model, 'world', 'custom_room_2ms.world'),
            description='仿真世界（默认 2 ms 步长版）'),
        DeclareLaunchArgument('rviz', default_value='true',
            description='是否启动 rviz2（地图/激光/机器人模型可视化）'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('explore_sec', default_value='180.0'),
        DeclareLaunchArgument('frontier_source', default_value='rrt'),
        DeclareLaunchArgument('auto_explore', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map_out', default_value=os.path.expanduser(
            '~/stemm_sim_ws/maps/explore_latest')),
        DeclareLaunchArgument('probe_log', default_value='',
            description='撞障取证 JSONL 路径（空=只打日志）'),
        DeclareLaunchArgument('supervisor_log', default_value='',
            description='脱困监督 JSONL 路径（空=只打日志）'),
        DeclareLaunchArgument('escape_supervisor', default_value='true',
            description='是否启用雷达引导的旋转+前进脱困（P0）'),
        DeclareLaunchArgument('stack', default_value='true',
            description='是否启动车端探索栈（false=只起机器人+取证/脱困，单元测试用）'),
        DeclareLaunchArgument('disable_retreat', default_value='1',
            description='1=运行期关闭车端盲退（P0 默认）；0=保留（对照基线）'),
        DeclareLaunchArgument(
            'stack_delay', default_value='65.0',
            description='探索栈延迟启动秒数：Gazebo 里机器人 spawn + EKF 出 odom_combined '
                        '约需 30~40 s，而探索栈内的 mapping_tf_guard 只有 30 s 超时'),
        robot,
        probe,
        TimerAction(period=stack_delay, actions=[stack, patch_sim_time, arbiter, supervisor]),
        TimerAction(period=stack_delay, actions=[driver]),
        rviz])
