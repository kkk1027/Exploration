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
                            IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PythonExpression)
from ament_index_python.packages import get_package_prefix
from launch_ros.actions import Node


def generate_launch_description():
    model = get_package_share_directory('wheeltec_two_wheel_model')
    nav_share = get_package_share_directory('wheeltec_two_wheel_nav')
    # 方案A：默认用仿真包自带的配置（已含视觉栅格/代价地图/视觉点云/规划路径），
    # 厂商 slam.rviz 保持不动；需要旧视图时用 rviz_config:=<path> 覆盖。
    rviz_cfg = LaunchConfiguration('rviz_config')
    explore = get_package_share_directory('stemm_cartographer_exploration')
    sim_pkg = get_package_share_directory('stemm_cartographer_exploration_sim')
    default_rviz_cfg = os.path.join(sim_pkg, 'config', 'sim_vision.rviz')
    if not os.path.exists(default_rviz_cfg):
        default_rviz_cfg = os.path.join(nav_share, 'rviz', 'slam.rviz')

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
                          ('use_sim_time', use_sim_time),
                          # 车端 escape_controller 的 enabled 参数在 __init__ 缓存，
                          # 运行期 ros2 param set 不生效（实测仍在转 0.40 rad/s 抢 /cmd_vel），
                          # 必须用启动开关关掉；P0 里由 escape_supervisor（旋转+前进）取代它。
                          ('escape_controller', LaunchConfiguration('car_escape'))],
        condition=stack_on)

    # 与栈同时启动的运行时参数补丁（独立进程，0.5 s 重试，抢在 guard 的 30 s 判定前落地）
    patch_sim_time = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('stemm_cartographer_exploration_sim'),
                          'lib', 'stemm_cartographer_exploration_sim', 'patch_sim_time'),
             LaunchConfiguration('disable_retreat'),
             LaunchConfiguration('explore_backstep')],
        output='screen', condition=stack_on)

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    arbiter = Node(package='stemm_cartographer_exploration_sim', executable='frontier_arbiter',
                   name='frontier_arbiter', output='screen', condition=stack_on,
                   parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                                'republish_sec': 1.5, 'max_candidates': 6,
                                'blacklist_sec': 30.0, 'blacklist_radius': 0.5,
                                # D0（2026-10-23 发现）：这个参数以前根本没传给仲裁节点，
                                # 历史约 140 轮日志里 "[v3]" 出现 0 次 —— 一直是 v1 在跑。
                                # 默认保持 v1（= 历史实测口径），要试 v3 必须显式 scoring:=v3。
                                'scoring': LaunchConfiguration('scoring')}])

    # P0-1：撞障取证（只读）。与机器人层同时启动，覆盖整轮（含启动期）。
    probe = Node(package='stemm_cartographer_exploration_sim',
                 executable='collision_probe', name='collision_probe',
                 output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'log_path': LaunchConfiguration('probe_log')}])

    # 方案A：深度相机 → 视觉障碍（补 2D 雷达的结构性盲区，见 P0_验收记录 §3b）
    #   节点本身只发话题；nav2 侧的图层注入由 harness/vision_nav_params.py 在启动前完成
    #   （nav2 的 plugins 列表只能在启动时生效），两侧都用 vision:= 开关，一键回退。
    vision = Node(package='stemm_cartographer_exploration_sim',
                  executable='vision_obstacle_grid', name='vision_obstacle_grid',
                  output='screen',
                  condition=IfCondition(LaunchConfiguration('vision')),
                  parameters=[{'use_sim_time': use_sim_time,
                               'cloud_topic': '/camera/depth/points',
                               'global_frame': 'map',
                               'self_filter_radius': LaunchConfiguration('vision_self_radius'),
                               'z_max': 1.80,
                               'low_z_max': 0.35,
                               'dump_path': LaunchConfiguration('vision_dump'),
                               'decay_sec': LaunchConfiguration('vision_decay_sec')}])

    # P1-2：静止看门狗 —— 车端前向安全闸会让 nav2 "在导航但不动"，这里 5 s 级识别并自愈
    watchdog = Node(package='stemm_cartographer_exploration_sim',
                    executable='explore_watchdog', name='explore_watchdog',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('watchdog')),
                    parameters=[{'use_sim_time': use_sim_time,
                                 'enabled': True,
                                 'stall_sec': LaunchConfiguration('watchdog_stall_sec'),
                                 'force_escape': LaunchConfiguration('force_escape'),
                                 'log_path': LaunchConfiguration('watchdog_log')}])

    # ② nav2 生命周期 nudge：guard 的 90 s 超时是车端硬编码、不能改，
    #    所以由仿真侧把"卡在 inactive"的节点用 change_state 推起来（不改车端源码）。
    # LaunchConfiguration 之间不能做算术 → 用 PythonExpression 求"栈启动后的延迟"
    t_nudge = PythonExpression([
        'float("', LaunchConfiguration('stack_delay'), '") + 10.0'])
    t_vision = PythonExpression([
        'float("', LaunchConfiguration('stack_delay'), '") + float("',
        LaunchConfiguration('vision_delay'), '")'])
    t_rviz = PythonExpression([
        'float("', LaunchConfiguration('stack_delay'), '") + float("',
        LaunchConfiguration('rviz_delay'), '")'])

    nudge = Node(package='stemm_cartographer_exploration_sim',
                 executable='nav2_lifecycle_nudge', name='nav2_lifecycle_nudge',
                 output='screen', condition=stack_on,
                 parameters=[{'use_sim_time': use_sim_time,
                              'enabled': LaunchConfiguration('nudge'),
                              'log_path': LaunchConfiguration('nudge_log')}])

    # P0-3：雷达引导的"旋转 + 前进"脱困（与栈同时起，等车端节点就绪后自行等待触发）
    supervisor = Node(package='stemm_cartographer_exploration_sim',
                      executable='escape_supervisor', name='escape_supervisor',
                      output='screen',
                      parameters=[{'use_sim_time': use_sim_time,
                                   'enabled': LaunchConfiguration('escape_supervisor'),
                                   'unit_max_sec': LaunchConfiguration('supervisor_unit_max_sec'),
                                   'recovery_timeout_sec': LaunchConfiguration('supervisor_recovery_timeout'),
                                   'rotate_min_range': LaunchConfiguration('supervisor_rotate_min_range'),
                                   'advance_stop_range': LaunchConfiguration('supervisor_advance_stop_range'),
                                   'log_path': LaunchConfiguration('supervisor_log')}])


    driver = Node(
        package='stemm_cartographer_exploration_sim',
        executable='session_driver',
        name='sim_session_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'explore_sec': explore_sec,
                     'map_out': map_out}])

    # 就绪门闸脚本（仅保留供实验，不接进主链；见 stack_delay 的注释）
    ready_gate = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('stemm_cartographer_exploration_sim'),
                          'lib', 'stemm_cartographer_exploration_sim',
                          'wait_robot_ready'),
             '--settle', '3.0', '--min-odom', '20', '--timeout', '75.0'],
        output='screen')

    # 把仿真垫片目录前置到 PATH（只影响本次 launch 起的进程）
    shim_dir = os.path.join(
        get_package_prefix('stemm_cartographer_exploration_sim'),
        'lib', 'stemm_cartographer_exploration_sim', 'sim_shims')

    return LaunchDescription([
        SetEnvironmentVariable('PATH', [shim_dir, ':', EnvironmentVariable('PATH')]),
        DeclareLaunchArgument('world', default_value=os.path.join(
            model, 'world', 'custom_room_2ms.world'),
            description='仿真世界（默认 2 ms 步长版）'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_cfg,
            description='rviz2 配置：默认仿真包��� sim_vision.rviz（含视觉层显示）'),
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
        DeclareLaunchArgument('car_escape', default_value='false',
            description='是否启动车端 stemm_escape_controller（P0 默认 false，由 escape_supervisor 取代）'),
        DeclareLaunchArgument('escape_supervisor', default_value='true',
            description='是否启用雷达引导的旋转+前进脱困（P0）'),
        DeclareLaunchArgument('corridor_width', default_value='1.05',
            description='前向净空预检的走廊净宽下限（m）；抬高可挡掉"看着能过、实际过不去"的口子'),
        DeclareLaunchArgument('scoring', default_value='v1',
            description='前沿收益函数版本：v1=未知格数/就近（历史实测口径），v2=邻域未知占比，'
                        'v3=朝前未知+贴已知边界（此前从未真正生效，需单独 A/B）'),
        DeclareLaunchArgument('supervisor_unit_max_sec', default_value='8.0',
            description='脱困单元硬顶秒数（单元测试时调小以快速触发降级）'),
        DeclareLaunchArgument('supervisor_rotate_min_range', default_value='0.34',
            description='脱困旋转时"扫过扇区"的最小测距门槛（低于此值不转）'),
        DeclareLaunchArgument('supervisor_advance_stop_range', default_value='0.34',
            description='脱困前进时前向 ±45° 的最小测距门槛'),
        DeclareLaunchArgument('supervisor_recovery_timeout', default_value='40.0',
            description='累计受阻多少秒后退出脱困（用户要求的 40 s 兜底）'),
        DeclareLaunchArgument('explore_backstep', default_value='1',
            description='阶段2：探索期是否允许受控小步倒车（<=0.15 m @0.05 m/s，雷达复核后方；'
                        '用户 2026-10-24 批准）'),
        DeclareLaunchArgument('watchdog', default_value='true',
            description='是否启用静止看门狗（车端安全闸导致的"在导航但不动"）'),
        DeclareLaunchArgument('force_escape', default_value='false',
            description='P0-甲：静止时是否强制请求脱困单元（默认关闭，需单独 A/B）'),
        DeclareLaunchArgument('watchdog_stall_sec', default_value='5.0',
            description='静止多少秒判定卡死并自愈（取消目标+上报受阻区）'),
        DeclareLaunchArgument('watchdog_log', default_value='',
            description='静止事件 JSONL 路径（空=只打日志）'),
        DeclareLaunchArgument('nudge_log', default_value='',
            description='生命周期 nudge 事件 JSONL 路径（空=只打日志）'),
        DeclareLaunchArgument('vision', default_value='false',
            description='是否启动视觉障碍源（方案A）。nav2 图层注入需同时 apply 参数补丁'),
        DeclareLaunchArgument('vision_decay_sec', default_value='0.0',
            description='P0-己：视觉累积格衰减秒数（0=关闭=保持旧的单调累积；>0 会淡出长期未观测的格）'),
        DeclareLaunchArgument('vision_dump', default_value='',
            description='视觉累积栅格的落盘前缀（空=不落盘；事故复盘用）'),
        DeclareLaunchArgument('vision_self_radius', default_value='0.38',
            description='视觉障碍的自身过滤半径（m，以机器人原点为圆心）'),
        DeclareLaunchArgument('vision_delay', default_value='25.0',
            description='视觉源/看门狗/nudge 相对"栈启动"再延迟多少秒（起栈降载）'),
        DeclareLaunchArgument('rviz_delay', default_value='30.0',
            description='rviz 相对"栈启动"再延迟多少秒（渲染会抢 CPU）'),
        DeclareLaunchArgument('nudge', default_value='true',
            description='是否启用 nav2 生命周期 nudge（推卡在 inactive 的节点起来）'),
        DeclareLaunchArgument('stack', default_value='true',
            description='是否启动车端探索栈（false=只起机器人+取证/脱困，单元测试用）'),
        DeclareLaunchArgument('disable_retreat', default_value='1',
            description='1=运行期关闭车端盲退（P0 默认）；0=保留（对照基线）'),
        DeclareLaunchArgument(
            'stack_delay', default_value='65.0',
            description='探索栈延迟启动秒数。2026-10-23 实测：改成"机器人就绪即起栈"'
                        '（约 25 s）会让 Nav2 生命周期在仿真还没稳时超时'
                        '（ready_guard 90 s 内未激活、整轮被打断），故恢复固定延迟。'),
        robot,
        probe,
        # 2026-10-23 起栈降载：视觉源/看门狗/nudge 都挪到"栈启动之后"再起。
        # 实测起栈窗口（gzserver+cartographer+nav2 生命周期）对 CPU 极敏感，
        # 多压一个点云节点/一个 rviz 就可能让 controller_server 停在 inactive、
        # ready_guard 90 s 超时、整轮 0 派发。视觉节点在 map 帧出现前本来也是空转。
        TimerAction(period=stack_delay, actions=[stack, patch_sim_time, arbiter, supervisor]),
        TimerAction(period=stack_delay, actions=[driver]),
        TimerAction(period=t_nudge, actions=[nudge]),
        TimerAction(period=t_vision, actions=[vision, watchdog]),
        TimerAction(period=t_rviz, actions=[rviz])])
