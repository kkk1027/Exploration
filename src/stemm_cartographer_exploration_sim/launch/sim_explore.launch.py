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
        # gazebo 客户端（界面）由 GUI 开关控制：GUI=true 时起 gzclient
        launch_arguments=[('gui', LaunchConfiguration('gazebo_gui')), ('world', world_cfg),
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
             LaunchConfiguration('explore_backstep'),
             LaunchConfiguration('retreat_max'),
             LaunchConfiguration('retreat_interval')],
        output='screen', condition=stack_on)

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])

    arbiter = Node(package='stemm_cartographer_exploration_sim', executable='frontier_arbiter',
                   name='frontier_arbiter', output='screen', condition=stack_on,
                   parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
                                'republish_sec': 1.5,
                                # 关键旋钮：每 tick 最多校验几个候选。设成 launch 参数后
                                # 才能做"候选管线是否够宽"的 A/B（此前写死 6）。
                                'max_candidates': LaunchConfiguration('max_candidates'),
                                'blacklist_sec': 30.0, 'blacklist_radius': 0.5,
                                # D0（2026-10-23 发现）：这个参数以前根本没传给仲裁节点，
                                # 历史约 140 轮日志里 "[v3]" 出现 0 次 —— 一直是 v1 在跑。
                                # 默认保持 v1（= 历史实测口径），要试 v3 必须显式 scoring:=v3。
                                'scoring': LaunchConfiguration('scoring'),
                                # 注意：这个 launch 参数以前**根本没接到仲裁器**
                                # （声明了 corridor_width，但节点参数里没有映射），
                                # 所以 CORRIDOR_W=0.85 一直是空操作（实测日志仍打 1.05）。
                                'corridor_width_m': LaunchConfiguration('corridor_width'),
                                # 通行保证式判据（与场景无关；A/B 用）
                                'pass_width_m': LaunchConfiguration('pass_width'),
                                'narrow_len_max_m': LaunchConfiguration('narrow_len_max'),
                                'unknown_span_max_m': LaunchConfiguration('unknown_span_max'),
                                # P9-⑦：与 nav2 allow_unknown:false 对齐
                                'unknown_block': LaunchConfiguration('unknown_block'),
                                'width_unknown_block': LaunchConfiguration('width_unknown_block'),
                                # P9-A：沿途 pinch 的可退出性判定（默认关；NOEXIT=1 打开）
                                'noexit_enabled': LaunchConfiguration('noexit_enabled'),
                                'noexit_pinch_m': LaunchConfiguration('noexit_pinch_m'),
                                'noexit_short_m': LaunchConfiguration('noexit_short_m'),
                                'noexit_retreat_m': LaunchConfiguration('noexit_retreat_m'),
                                'noexit_turn_m': LaunchConfiguration('noexit_turn_m'),
                                'noexit_retreat_need_m': LaunchConfiguration('noexit_retreat_need_m'),
                                'noexit_turn_hold_m': LaunchConfiguration('noexit_turn_hold_m'),
                                'contact_fail_enabled': LaunchConfiguration('contact_fail'),
                                'contact_fail_max': LaunchConfiguration('contact_fail_max'),
                                'contact_fail_cooldown_sec': LaunchConfiguration('contact_fail_cooldown'),
                                # P9-CORR：走廊级记忆（本轮内存，不跨轮持久）
                                'corridor_block_enabled': LaunchConfiguration('corridor_block'),
                                'corridor_radius_m': LaunchConfiguration('corridor_radius'),
                                'corridor_ttl_sec': LaunchConfiguration('corridor_ttl'),
                                'corridor_ttl_max_sec': LaunchConfiguration('corridor_ttl_max'),
                                # V2-③ 后置 veto：默认关，实验时 veto:=true
                                'veto_enabled': LaunchConfiguration('veto'),
                                'veto_min_esc_dirs': LaunchConfiguration('veto_min_esc'),
                                'veto_occ_frac_max': LaunchConfiguration('veto_occ_max'),
                                'veto_max_per_session': LaunchConfiguration('veto_max'),
                                'veto_cooldown_sec': LaunchConfiguration('veto_cooldown'),
                                # P9-⑥：对 nav2 实际规划出的 /plan 做窄处体检（可选强制取消）
                                'plan_guard_log': LaunchConfiguration('plan_guard_log'),
                                'plan_guard_veto': LaunchConfiguration('plan_guard_veto'),
                                # P9-L1/L2：卡死治理（确定过不去 → 报失败/换目标）
                                'failfast_enabled': LaunchConfiguration('failfast'),
                                'failfast_window_sec': LaunchConfiguration('failfast_window'),
                                'failfast_drift_m': LaunchConfiguration('failfast_drift'),
                                'failfast_front_m': LaunchConfiguration('failfast_front'),
                                'failfast_confirm_hits': LaunchConfiguration('failfast_confirm'),
                                'failfast_ttl_sec': LaunchConfiguration('failfast_ttl'),
                                'failfast_hard_radius_m': LaunchConfiguration('failfast_hard_radius'),
                                'failfast_zone_radius_m': LaunchConfiguration('failfast_zone_radius'),
                                'failfast_max_actions': LaunchConfiguration('failfast_max_actions'),
                                'failfast_escalate_sec': LaunchConfiguration('failfast_escalate'),
                                'failfast_comp_radius_m': LaunchConfiguration('failfast_comp_radius'),
                                # V1 闸门（落点级）：与 veto 搭配才覆盖全部派发通道
                                'gate_enabled': LaunchConfiguration('gate'),
                                # 候选后验门（2026-10-27）：用当前全局代价地图复核落点
                                'cost_gate': LaunchConfiguration('cost_gate'),
                                'cost_gate_hard': LaunchConfiguration('cost_gate_hard'),
                                'cost_gate_disc_m': LaunchConfiguration('cost_gate_disc'),
                                'cost_gate_free_ratio': LaunchConfiguration('cost_gate_free'),
                                # 后验修复：不合格候选 → 就近随机采样改投最近的合格点
                                'cost_repair': LaunchConfiguration('cost_repair'),
                                'cost_repair_radius_m': LaunchConfiguration('cost_repair_radius'),
                                'cost_repair_samples': LaunchConfiguration('cost_repair_samples'),
                                'cost_repair_keep_unknown': LaunchConfiguration('cost_repair_keep_unknown'),
                                'cost_repair_min_unknown': LaunchConfiguration('cost_repair_min_unknown'),
                                'gate_esc_dirs_min': LaunchConfiguration('gate_min_esc'),
                                'gate_occ_soft': LaunchConfiguration('gate_occ_soft'),
                                # 探索末段"停车友好"偏置需要知道本轮总时长
                                'explore_sec': LaunchConfiguration('explore_sec'),
                                'probe_log': LaunchConfiguration('candidate_probe_log'),
                                'timeline_log': LaunchConfiguration('timeline_log')}])

    # P0-1：撞障取证（只读）。与机器人层同时启动，覆盖整轮（含启动期）。
    probe = Node(package='stemm_cartographer_exploration_sim',
                 executable='collision_probe', name='collision_probe',
                 output='screen',
                 parameters=[{'use_sim_time': use_sim_time,
                              'log_path': LaunchConfiguration('probe_log')}])

    # 候选合并器（2026-10-27）：仲裁合格目标优先，TTL 到期回落到 RRT 原始样本。
    # 背景：把 manager 直接指到 /validated_frontier_goal（单源化）后，manager 会长时间
    # 停在 waiting_for_rrt_goal（实测一轮里连续 93 s）：仲裁器窄道预检一拍没找到候选就
    # 不发布，而 manager 只认这一个源 ⇒ 供给断流 ⇒ 车静止但 RViz 还留着上一条 /plan。
    merge = Node(package='stemm_cartographer_exploration_sim',
                 executable='frontier_merge', name='frontier_merge',
                 output='screen',
                 condition=IfCondition(LaunchConfiguration('goal_merge')),
                 parameters=[{'use_sim_time': use_sim_time,
                              'validated_topic': '/validated_frontier_goal',
                              'raw_topic': '/assigned_frontier_goal',
                              'out_topic': '/merged_frontier_goal',
                              'validated_ttl': LaunchConfiguration('merge_ttl'),
                              'raw_min_period': LaunchConfiguration('merge_raw_period'),
                              'log_path': LaunchConfiguration('merge_log')}])

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
                               'decay_sec': LaunchConfiguration('vision_decay_sec'),
                               'accum_decay_sec': LaunchConfiguration('vision_accum_decay_sec'),
                               'low_accum_decay_sec': LaunchConfiguration('vision_low_accum_decay_sec')}])

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
                                   # P9-⑤（用户 2026-10-26 批准"短距离小速度倒车"）：探索期受控小步倒车
                                   'explore_reverse_enabled': LaunchConfiguration('supervisor_explore_reverse'),
                                   'explore_reverse_max_m': LaunchConfiguration('supervisor_explore_reverse_max_m'),
                                   'explore_reverse_speed': LaunchConfiguration('supervisor_explore_reverse_speed'),
                                   'explore_reverse_max_count': LaunchConfiguration('supervisor_explore_reverse_max_count'),
                                   # C v2 口径（2026-10-26 用真实数据重定）：状态+静默计时 + front 门槛
                                   'explore_reverse_front_m': LaunchConfiguration('supervisor_explore_reverse_front'),
                                   'explore_reverse_block_sec': LaunchConfiguration('supervisor_explore_reverse_block_sec'),
                                   'explore_reverse_takeover_busy': LaunchConfiguration('supervisor_explore_reverse_takeover'),
                                   'explore_reverse_rear_clear_m': LaunchConfiguration('supervisor_explore_reverse_rear_clear'),
                                   # P9-S0：接触即倒车（默认关）
                                   'contact_reverse_enabled': LaunchConfiguration('supervisor_contact_reverse'),
                                   'contact_reverse_window_sec': LaunchConfiguration('supervisor_contact_reverse_window'),
                                   'contact_reverse_max': LaunchConfiguration('supervisor_contact_reverse_max'),
                                   'contact_reverse_cooldown_sec': LaunchConfiguration('supervisor_contact_reverse_cool'),
                                   'contact_reverse_front_m': LaunchConfiguration('supervisor_contact_reverse_front'),
                                   'log_path': LaunchConfiguration('supervisor_log')}])


    driver = Node(
        package='stemm_cartographer_exploration_sim',
        executable='session_driver',
        name='sim_session_driver',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'explore_sec': explore_sec,
                     'return_wait_goal_sec': LaunchConfiguration('return_wait_goal_sec'),
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
        DeclareLaunchArgument('gazebo_gui', default_value='false',
            description='是否启动 gazebo 客户端界面（gzclient）'),
        DeclareLaunchArgument('rviz', default_value='true',
            description='是否启动 rviz2（地图/激光/机器人模型可视化）'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('explore_sec', default_value='180.0'),
        DeclareLaunchArgument('return_wait_goal_sec', default_value='20.0',
            description='探索限时到点时若候选仍在执行，等它结束再返航的上限秒数（0=关闭，旧行为）'),
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
        DeclareLaunchArgument('retreat_max', default_value='2',
            description='探索期受控倒车每局步数上限（原 2 步对 0.34 m 贴脸不够用）'),
        DeclareLaunchArgument('retreat_interval', default_value='8.0',
            description='探索期受控倒车最小间隔秒数（原 8 s 太慢）'),
        DeclareLaunchArgument('pass_width', default_value='0.90',
            description='通行保证带宽下限（车宽 0.47 + 2×0.215 ≈ 0.90，与场景无关）'),
        DeclareLaunchArgument('narrow_len_max', default_value='1.00',
            description='窄段累计长度上限（超过即否决：不允许"勉强能过"）'),
        DeclareLaunchArgument('unknown_span_max', default_value='0.35',
            description='连续未知段长度上限（未知按障碍算，只允许短跨度例外）'),
        DeclareLaunchArgument('unknown_block', default_value='false',
            description='P9-⑦：候选判据是否与 nav2 allow_unknown:false 一致（禁止跨未知区、落点必须已知自由）'),
        DeclareLaunchArgument('width_unknown_block', default_value='false',
            description='P9-D：判宽时把未知格按障碍计（修"半侧没观测到的缝被量成宽路"）'),
        DeclareLaunchArgument('noexit_enabled', default_value='false',
            description='P9-A：pass_width 只作上限，[0.752, pass_width) 的路径按"能不能退出"放行/否决'),
        DeclareLaunchArgument('noexit_pinch_m', default_value='0.752',
            description='A：低于此净宽视为 pinch（=1.6×车宽 0.47）'),
        DeclareLaunchArgument('noexit_short_m', default_value='0.35',
            description='A：pinch 穿透长度 <= 此值直接放行（只擦车角）'),
        DeclareLaunchArgument('noexit_retreat_m', default_value='1.20',
            description='A：pinch 进入侧要求可回退的窗口长度'),
        DeclareLaunchArgument('noexit_turn_m', default_value='0.81',
            description='A：pinch 内存在此自由直径即可掉头（车对角线）'),
        DeclareLaunchArgument('noexit_retreat_need_m', default_value='1.00',
            description='A：回退窗口内 w>=pinch 的连续长度要求'),
        DeclareLaunchArgument('contact_fail', default_value='false',
            description='P9-S0：物理接触时取消当前目标（默认关）'),
        DeclareLaunchArgument('contact_fail_max', default_value='6',
            description='P9-S0：每轮最多因接触取消几次目标'),
        DeclareLaunchArgument('corridor_block', default_value='false',
            description='P9-CORR：把"穿窄处"的窄点记成不可通行走廊并拒绝穿过（本轮内存）'),
        DeclareLaunchArgument('corridor_radius', default_value='0.35',
            description='P9-CORR：走廊半径（m）'),
        DeclareLaunchArgument('corridor_ttl', default_value='30.0',
            description='P9-CORR：走廊基础记忆时长（s，命中递增）'),
        DeclareLaunchArgument('corridor_ttl_max', default_value='180.0',
            description='P9-CORR：走廊记忆上限（s）'),
        DeclareLaunchArgument('contact_fail_cooldown', default_value='3.0',
            description='P9-S0：接触取消的冷却（s）'),
        DeclareLaunchArgument('noexit_turn_hold_m', default_value='0.30',
            description='A：掉头空间要连续多长（避免一格毛刺被当成凹室）'),
        DeclareLaunchArgument('corridor_width', default_value='1.05',
            description='前向净空预检的走廊净宽下限（m）；抬高可挡掉"看着能过、实际过不去"的口子'),
        DeclareLaunchArgument('timeline_log', default_value='',
            description='观测工作流 v2：仲裁器每拍一行时间线 JSONL（空=不写）'),
        DeclareLaunchArgument('candidate_probe_log', default_value='',
            description='候选质量探针 JSONL 路径（空=不记录）'),
        # 2026-10-25 十轮实测：6 → 14 后 manager 自派占比 55% → 41%（仲裁派发 28% → 43%），
        # 全绿 10/10、接触 0、返航失败 0 与基线持平，内存/CPU 采样无差异。
        # 原因：每 tick 只校验前 6 个候选，一失败就记"本轮候选全部被拒"，
        # 而候选池里常有一百多个点没被看过。
        DeclareLaunchArgument('max_candidates', default_value='14',
            description='每 tick 校验的候选数上限；越大越不容易"候选全灭"但更吃 CPU'),
        DeclareLaunchArgument('gate', default_value='false',
            description='V1 候选闸门（落点级硬闸门：esc_dirs/comp_r2 拒、occ 软惩罚）'),
        DeclareLaunchArgument('gate_min_esc', default_value='3',
            description='闸门可逃逸方向下限'),
        DeclareLaunchArgument('gate_occ_soft', default_value='0.30',
            description='闸门复杂区软惩罚阈值（2 m 内障碍占比）'),
        # 候选后验门（2026-10-27）：候选是在"当时的地图"上选的，等雷达/视觉扫到那一带，
        # 同一位置可能已被膨胀/占据/视觉障碍覆盖。下发前用当前全局代价地图复核落点。
        DeclareLaunchArgument('goal_merge', default_value='false',
            description='起候选合并器 frontier_merge（仲裁优先 + RRT 兜底）'),
        DeclareLaunchArgument('merge_ttl', default_value='5.0',
            description='仲裁合格目标的独占秒数'),
        DeclareLaunchArgument('merge_raw_period', default_value='0.5',
            description='原始样本限速（秒/条）'),
        DeclareLaunchArgument('merge_log', default_value='',
            description='合并器统计 jsonl 路径（空=只打印）'),
        DeclareLaunchArgument('cost_gate', default_value='false',
            description='候选后验门开关'),
        DeclareLaunchArgument('cost_gate_hard', default_value='99',
            description='落点格代价 >= 该值即拒（99=致命/内切）'),
        DeclareLaunchArgument('cost_gate_disc', default_value='0.35',
            description='圆盘半径 m：盘内不得有致命格'),
        DeclareLaunchArgument('cost_gate_free', default_value='0.80',
            description='圆盘内 /map 已知空闲比例下限'),
        DeclareLaunchArgument('cost_repair', default_value='true',
            description='后验不合格候选改为"就近修复"而不是否决'),
        DeclareLaunchArgument('cost_repair_radius', default_value='1.20',
            description='修复采样半径 m'),
        DeclareLaunchArgument('cost_repair_samples', default_value='24',
            description='每个候选的修复采样次数'),
        DeclareLaunchArgument('cost_repair_keep_unknown', default_value='0.30',
            description='修复点至少保留原候选未知格数的比例'),
        DeclareLaunchArgument('cost_repair_min_unknown', default_value='20',
            description='修复点附近未知格绝对下限'),
        # 2026-10-25 20 轮 vs 20 轮 A/B（docs/P7 第 24 节）：
        #   基线（无 veto）∶静止超时 28（1.40/轮）、归因率 24%
        #   有 veto（额度 6、冷却 10 s）∶静止超时 19（0.95/轮，−32%）、归因率 11.5%
        #   两组全绿率同为 19/20 —— 也就是说这是三步里唯一"既降卡死又不退化"的改动。
        DeclareLaunchArgument('veto', default_value='true',
            description='V2-③ 后置 veto：对非本节点派发的目标做硬闸门否决（只在探索期，默认关）'),
        DeclareLaunchArgument('veto_min_esc', default_value='3',
            description='后置 veto 的可逃逸方向下限（低于该值即否决）'),
        DeclareLaunchArgument('veto_occ_max', default_value='0.30',
            description='后置 veto 的 2 m 内障碍占比上限（达到即否决）'),
        DeclareLaunchArgument('veto_max', default_value='6',
            description='后置 veto 每局最多否决次数'),
        DeclareLaunchArgument('veto_cooldown', default_value='10.0',
            description='后置 veto 冷却秒数（两次否决之间的最小间隔）'),
        DeclareLaunchArgument('failfast', default_value='false',
            description='P9-L1/L2：是否启用"确定过不去→报失败/换目标"（默认关，A/B 打开）'),
        DeclareLaunchArgument('failfast_window', default_value='20.0',
            description='L1 滑窗秒数（离线标定取 20 s）'),
        DeclareLaunchArgument('failfast_drift', default_value='0.20',
            description='L1 窗口内位姿包围盒对角线上限（m）'),
        DeclareLaunchArgument('failfast_front', default_value='0.60',
            description='L1 净空证据：front_min_range 中位下限（m）'),
        DeclareLaunchArgument('failfast_confirm', default_value='2',
            description='同一 2 m 区内多少秒内命中几次才动手'),
        DeclareLaunchArgument('failfast_ttl', default_value='60.0',
            description='拉黑/禁入区存活秒数'),
        DeclareLaunchArgument('failfast_hard_radius', default_value='1.2',
            description='本轮禁入区半径（m）：候选落点在此半径内直接丢弃'),
        DeclareLaunchArgument('failfast_zone_radius', default_value='2.0',
            description='"同一区域"判定半径（m）'),
        DeclareLaunchArgument('failfast_max_actions', default_value='8',
            description='每轮最多触发几次判失败'),
        DeclareLaunchArgument('failfast_escalate', default_value='40.0',
            description='同区累计多少秒后升级为禁入区'),
        DeclareLaunchArgument('failfast_comp_radius', default_value='6.0',
            description='死角连通域半径（m）：确认过不去后把这个范围内的整块自由区域拉黑'),
        DeclareLaunchArgument('plan_guard_log', default_value='false',
            description='P9-⑥：记录 nav2 /plan 的最小净宽与最长窄段（False=不体检）'),
        DeclareLaunchArgument('plan_guard_veto', default_value='false',
            description='P9-⑥：计划穿窄处时取消该目标（宁可不走）'),
        DeclareLaunchArgument('scoring', default_value='v1',
            description='前沿收益函数版本：v1=未知格数/就近（历史实测口径），v2=邻域未知占比，'
                        'v3=朝前未知+贴已知边界（此前从未真正生效，需单独 A/B）'),
        DeclareLaunchArgument('supervisor_unit_max_sec', default_value='8.0',
            description='脱困单元硬顶秒数（单元测试时调小以快速触发降级）'),
        DeclareLaunchArgument('supervisor_rotate_min_range', default_value='0.34',
            description='脱困旋转时"扫过扇区"的最小测距门槛（低于此值不转）'),
        DeclareLaunchArgument('supervisor_advance_stop_range', default_value='0.34',
            description='脱困前进时前向 ±45° 的最小测距门槛'),
        DeclareLaunchArgument('supervisor_explore_reverse', default_value='false',
            description='P9：探索期是否允许脱困监督做一次受控小步倒车（雷达复核后方净空；用户 2026-10-26 批准）'),
        DeclareLaunchArgument('supervisor_explore_reverse_max_m', default_value='0.15',
            description='探索期受控倒车单步上限（m）'),
        DeclareLaunchArgument('supervisor_explore_reverse_speed', default_value='0.06',
            description='探索期受控倒车速度（m/s，小速度）'),
        DeclareLaunchArgument('supervisor_explore_reverse_max_count', default_value='4',
            description='探索期受控倒车每局步数上限'),
        DeclareLaunchArgument('supervisor_explore_reverse_front', default_value='0.70',
            description='C v2：前向 ±20° 最小测距 ≤ 此值才值得倒车（车端安全闸 front_stop=0.65）'),
        DeclareLaunchArgument('supervisor_explore_reverse_block_sec', default_value='6.0',
            description='C v2：车端闸按住且不再发速度需持续多少秒才接管（实测段时长 p75=5.3 s）'),
        DeclareLaunchArgument('supervisor_explore_reverse_takeover', default_value='true',
            description='C v2：是否允许在有活跃目标时接管（吸收态本身就是"目标还在、车端零速"）'),
        DeclareLaunchArgument('supervisor_explore_reverse_rear_clear', default_value='0.40',
            description='C v2：探索期倒车前要求后方 ±30° 最小净空（m）'),
        DeclareLaunchArgument('supervisor_contact_reverse', default_value='false',
            description='P9-S0：物理接触后（/collision_contact_zone）允许一次受限倒车（雷达复核后方）'),
        DeclareLaunchArgument('supervisor_contact_reverse_window', default_value='5.0',
            description='P9-S0：接触后多少秒内仍允许触发（s）'),
        DeclareLaunchArgument('supervisor_contact_reverse_cool', default_value='3.0',
            description='P9-S0：接触倒车冷却（s）'),
        DeclareLaunchArgument('supervisor_contact_reverse_front', default_value='0.90',
            description='P9-S0：接触倒车要求前向 ±20° 测距 <= 此值（前方已开阔就不倒）'),
        DeclareLaunchArgument('supervisor_contact_reverse_max', default_value='3',
            description='P9-S0：每局接触触发的倒车次数上限'),
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
        DeclareLaunchArgument('vision_low_accum_decay_sec', default_value='120.0',
            description='P9-⑧b：低矮带累积记忆窗口（s）'),
        DeclareLaunchArgument('vision_accum_decay_sec', default_value='-1.0',
            description='P9-⑧：累积视觉层（-> 全局代价地图）的独立衰减窗口，-1=沿用 vision_decay_sec'),
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
        merge,
        # 2026-10-23 起栈降载：视觉源/看门狗/nudge 都挪到"栈启动之后"再起。
        # 实测起栈窗口（gzserver+cartographer+nav2 生命周期）对 CPU 极敏感，
        # 多压一个点云节点/一个 rviz 就可能让 controller_server 停在 inactive、
        # ready_guard 90 s 超时、整轮 0 派发。视觉节点在 map 帧出现前本来也是空转。
        TimerAction(period=stack_delay, actions=[stack, patch_sim_time, arbiter, supervisor]),
        TimerAction(period=stack_delay, actions=[driver]),
        TimerAction(period=t_nudge, actions=[nudge]),
        TimerAction(period=t_vision, actions=[vision, watchdog]),
        TimerAction(period=t_rviz, actions=[rviz])])
