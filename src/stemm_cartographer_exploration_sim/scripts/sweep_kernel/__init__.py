"""sweep_kernel —— 扫掠式可通行性内核（S5 M2）。

导出
----
    SweepResult      一次扫掠的结果（含 as_dict 便于落盘）
    sweep_path       对路径扫掠（内部重采样 + 切向 yaw）
    sweep_poses      对位姿序列扫掠
    resample_path    路径等距重采样
    transform_footprint  车身系 → 世界系
    point_in_polygon 点是否在多边形内（与扫掠本体**同一套**扫描线判据）

    lateral_width_axis_aligned   改造前的轴向法向测宽（**仅测试用**，
                                 用于证明斜向通道被高估 1.41 倍）
"""

from .sweep import (  # noqa: F401
    SweepResult,
    lateral_width_axis_aligned,
    point_in_polygon,
    resample_path,
    smooth_yaw,
    sweep_path,
    sweep_poses,
    transform_footprint,
)

__all__ = [
    'SweepResult',
    'lateral_width_axis_aligned',
    'point_in_polygon',
    'resample_path',
    'smooth_yaw',
    'sweep_path',
    'sweep_poses',
    'transform_footprint',
]
