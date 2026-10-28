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
import subprocess
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_PARAMS = ('/home/kbf/wheeltec_ros2/src/stemm_cartographer_exploration/config/'
              'stemm_safe_nav2_params.yaml')
VISION_PATCH = '/home/kbf/stemm_exploration_sim_ws/harness/vision_nav_params.py'
DEFAULT_WORLD = ('/home/kbf/stemm_sim_ws/install/wheeltec_two_wheel_model/share/'
                 'wheeltec_two_wheel_model/world/aws_small_house.world')


def generate_launch_description():
    sim_pkg = get_package_share_directory('stemm_cartographer_exploration_sim')
    rviz_cfg = os.path.join(sim_pkg, 'config', 'sim_vision.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')

    # 0) ★ 出生竞态兜底：spawn_entity 用 TRANSIENT_LOCAL 订阅 /robot_description，
    #    而 nav_only 里只有仿真侧一个 RSP，偶发拿不到 latched 消息 → 永久卡在
    #    "Waiting for entity xml on /robot_description"（实测连续 3 次）。
    #    这里并行再发一份同源 URDF（内容与 RSP 一致），保证 spawn 一定能拿到。
    urdf_pub = Node(package='stemm_cartographer_exploration_sim',
                    executable='urdf_publisher', name='urdf_publisher', output='screen',
                    parameters=[{'use_sim_time': use_sim_time,
                                 'model_file': LaunchConfiguration('robot_model'),
                                 'topic': '/robot_description',
                                 'period': 1.0, 'count': 30}])

    # 1) 机器人活着（Gazebo 物理 + TF + /odom_combined），默认生成在世界原点
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('wheeltec_two_wheel_model'), 'launch', 'sim.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world'),
                          'gui': LaunchConfiguration('gui'),
                          'spawn_x': LaunchConfiguration('spawn_x'),
                          'spawn_y': LaunchConfiguration('spawn_y'),
                          'spawn_yaw': LaunchConfiguration('spawn_yaw'),
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
    # 2026-10-27：**不能**用 map→odom 单位变换硬指。实测雷达与静态地图完全对不上
    # （左/右/前/后距离无任何一致项 ⇒ 车并不在静态地图的原点）。
    # 正确做法：用 AMCL 对静态地图做定位，由它发布 map→odom。
    amcl = Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'alpha1': 0.2, 'alpha2': 0.2, 'alpha3': 0.2, 'alpha4': 0.2,
                             'alpha5': 0.2,
                             'base_frame_id': 'base_footprint',
                             'global_frame_id': 'map',
                             'odom_frame_id': 'odom_combined',
                             'scan_topic': 'scan',
                             'laser_max_range': 30.0,
                             'laser_min_range': 0.15,
                             'laser_model_type': 'likelihood_field',
                             'max_particles': 2000, 'min_particles': 500,
                             'tf_broadcast': True,
                             # 初始位姿 = 建图那一轮的出生点 (0,0,0)
                             'set_initial_pose': True,
                             # 初始位姿 = 出生点（同一组参数），保证"假定的位姿"与物理一致
                             'initial_pose': {'x': LaunchConfiguration('spawn_x'),
                                              'y': LaunchConfiguration('spawn_y'),
                                              'z': 0.0,
                                              'yaw': LaunchConfiguration('spawn_yaw')}}])
    lifecycle_amcl = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                          name='lifecycle_manager_amcl', output='screen',
                          parameters=[{'use_sim_time': use_sim_time,
                                       'autostart': True,
                                       'node_names': ['amcl']}])

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

    # 2026-10-27 踩坑：nav2 与 map_server 同时起时，StaticLayer 还没收到 latch 的 /map，
    # 全局代价地图就按最小尺寸初始化 → 之后任何目标都是 "Goal pose is out of costmap!"。
    # 故把 nav2 延后启动（默认 8 s）。
    nav2_delayed = TimerAction(period=LaunchConfiguration('nav_start_delay'),
                               actions=[nav2])

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': use_sim_time}],
                condition=IfCondition(LaunchConfiguration('rviz')))

    # 5) 视觉障碍源（方案A）。★ 2026-10-27 修正：本文件原先**完全没有**视觉链路，
    #    导致 nav_only 里出现"雷达看不见的低矮/健身区障碍在代价地图里根本不存在"，
    #    于是用 nav_only 做的通行性测试会把"视觉层缺失"误判成"nav2 能过"。
    #    现在两件事一起做，且任何一步失败都 loud fail（绝不静默跑成纯雷达基线）：
    #      a) 启动前把 nav2 的 costmap plugins 注入视觉图层（plugins 只在 configure 生效）；
    #      b) 起 vision_obstacle_grid 节点，发布 /vision/live_cloud 等话题。
    vision = Node(package='stemm_cartographer_exploration_sim',
                  executable='vision_obstacle_grid', name='vision_obstacle_grid',
                  output='screen',
                  parameters=[{'use_sim_time': use_sim_time,
                               'cloud_topic': '/camera/depth/points',
                               'global_frame': 'map',
                               'self_filter_radius': LaunchConfiguration('vision_self_radius'),
                               'tf_use_stamp': LaunchConfiguration('vision_tf_stamp'),
                               'z_max': 1.80,
                               'low_z_max': 0.35,
                               'dump_path': LaunchConfiguration('vision_dump'),
                               'decay_sec': LaunchConfiguration('vision_decay_sec'),
                               'accum_decay_sec': LaunchConfiguration('vision_accum_decay_sec'),
                               'low_accum_decay_sec': LaunchConfiguration('vision_low_accum_decay_sec')}])

    def _vision_setup(context, *_args, **_kwargs):
        """按 vision:= 开关注入/撤销 nav2 视觉图层，并**验证真的进去了**。"""
        on = LaunchConfiguration('vision').perform(context).lower() in ('1', 'true', 'yes')
        mode = LaunchConfiguration('vision_mode').perform(context)
        params_file = LaunchConfiguration('nav_params').perform(context)
        if not on:
            subprocess.run([sys.executable, VISION_PATCH, '--restore'],
                           capture_output=True, text=True)
            print('[nav_only] ⚠ vision:=false —— 视觉层**未**接入代价地图，'
                  '本轮只有雷达静态基线，低矮障碍必然被当成可通行。', flush=True)
            return []
        r = subprocess.run([sys.executable, VISION_PATCH, '--apply', '--global-mode', mode],
                           capture_output=True, text=True)
        print(r.stdout.strip(), flush=True)
        if r.returncode != 0:
            raise RuntimeError(f'[nav_only] 视觉图层注入失败 (rc={r.returncode})：{r.stderr.strip()}')
        # 验证：两个 costmap 的 plugins 必须真的含视觉图层，否则拒绝启动
        try:
            d = yaml.safe_load(open(params_file, encoding='utf-8'))
        except Exception as e:                                  # noqa: BLE001
            raise RuntimeError(f'[nav_only] 参数文件解析失败：{e}') from e
        for node_name, tag in (('local_costmap', 'vision_voxel_layer'),
                               ('global_costmap', 'vision_obstacle_layer')):
            got = d[node_name][node_name]['ros__parameters'].get('plugins') or []
            if tag not in got:
                raise RuntimeError(f'[nav_only] {node_name}.plugins 缺 {tag}：{got}')
            print(f'[nav_only] ✓ {node_name}.plugins = {got}', flush=True)
        print(f'[nav_only] ✓ 视觉层已接入（global-mode={mode}），'
              f'视觉节点订阅 /camera/depth/points', flush=True)
        return [vision]

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
        DeclareLaunchArgument('nav_start_delay', default_value='8.0'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('robot_model',
                              default_value=os.path.join(
                                  get_package_share_directory('wheeltec_two_wheel_model'),
                                  'urdf', 'two_wheel_diff_sim.urdf.xacro'),
                              description='nav_only 兜底发布 /robot_description 用的模型'),
        # 视觉层（与 sim_explore.launch.py 的默认值逐项对齐，便于两份测试台互相对照）
        DeclareLaunchArgument('vision', default_value='true',
                              description='false = 纯雷达基线（代价地图不含视觉层）'),
        DeclareLaunchArgument('vision_mode', default_value='accum',
                              description='global 层视觉源：accum(默认，累积云+不清除) / '
                                          'livemark(实时云+不清除) / live(实时云+清除，'
                                          '实测路径会穿过视觉障碍) / lowaccum'),
        DeclareLaunchArgument('vision_dump', default_value='/tmp/navonly_vision',
                              description='视觉栅格落盘前缀（留空会让 launch 报 malformed arg，踩过）'),
        DeclareLaunchArgument('vision_self_radius', default_value='0.38'),
        DeclareLaunchArgument('vision_tf_stamp', default_value='true',
                              description='true=按点云时间戳取 TF（修旋转鬼影）；false=旧行为'),
        DeclareLaunchArgument('vision_decay_sec', default_value='0.0'),
        DeclareLaunchArgument('vision_accum_decay_sec', default_value='120.0',
                              description='累积云衰减秒数(-1=永不衰减；120=与 round.sh 的 accum 档一致)'),
        DeclareLaunchArgument('vision_low_accum_decay_sec', default_value='0.0'),
        urdf_pub, sim, map_server, lifecycle_maps, amcl, lifecycle_amcl, nav2_delayed, audit,
        contacts, rviz,
        OpaqueFunction(function=_vision_setup),
        RegisterEventHandler(OnShutdown(on_shutdown=[
            ExecuteProcess(cmd=[sys.executable, VISION_PATCH, '--restore'], output='screen')])),
    ])
