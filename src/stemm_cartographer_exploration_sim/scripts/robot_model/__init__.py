"""robot_model —— 车体画像与阈值派生（S5 M1，唯一几何事实源）。

为什么单独成包
--------------
改造前"车体几何"在代码/配置里有 **4 处独立定义**（S5 §1.1，2026-11-13 实测）：

    launch/sim_explore.launch.py   pass_width 0.90 / narrow_floor 0.58
    scripts/patch_sim_time         FOOTPRINT [[-0.18,±0.235],[0.32,±0.235]]
    scripts/collision_probe        body_x_min / body_x_max / body_half_width
    scripts/escape_supervisor      同上（又抄了一份）

四处只要有一处没跟着改，"换车"就退化成"几何半新半旧"。物理接触的第 2 层根因
（`ObstaclesCritic.consider_footprint: false`，只按底盘中心判碰撞）本质就是
"同一辆车在导航栈里有两套宽度"。

本包把几何收敛到 `config/robot_profiles/*.yaml`，其余全部由派生量给出。

两条铁律（S5 §2）
-----------------
1. **阈值不许配，只许算。** "通行需要多宽"不是参数，是
   `body_width + 2 × side_margin`。profile 里只写几何与余量，不写阈值。
2. **能力用 flag 表达，不用机型名。** 判据里只允许
   `if not profile.can_rotate_in_place`，不允许 `if profile.kinematics == 'ackermann'`。

导出
----
    RobotProfile        画像对象（构造即校验并算出全部派生量）
    load_profile()      按名字或路径加载
    available_profiles() 列出可加载的 profile 名
    ProfileError        内容不合法（启动即报错，不许半套几何跑起来）
"""

from .profile import (  # noqa: F401
    ProfileError,
    RobotProfile,
    available_profiles,
    load_profile,
    profile_search_dirs,
)

__all__ = [
    'ProfileError',
    'RobotProfile',
    'available_profiles',
    'load_profile',
    'profile_search_dirs',
]
