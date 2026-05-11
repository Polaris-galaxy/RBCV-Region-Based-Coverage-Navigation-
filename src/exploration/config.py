"""配置与数据结构：检测框、观测、矩形格、路径参数."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal


Meter = float


@dataclass
class DetectionZone:
    """单个检测框（覆盖区）。

    coords 建议使用 **地图/世界系** 多边形顶点 ``[(x,y), ...]``（有序，首尾不必重复）。
    也可只给轴对齐矩形 ``bbox_xyxy=(xmin, ymin, xmax, ymax)``，由载入逻辑补全为多边形。
    """

    zone_id: str
    polygon_xy: list[tuple[float, float]] | None = None
    bbox_xyxy: tuple[float, float, float, float] | None = None
    # 覆盖语义：是否在多边形内部即算「到过覆盖格」——与 coverage 分辨率一致时使用
    label: str = ""
    # 记录在 JSON 里的可选基础权重（若不在 InitialRegionWeight 里单独给）
    base_weight_hint: Meter | None = None


@dataclass
class SpatialRectPrior:
    """均匀矩形格划分后，对**落在该轴对齐矩形内**的格子中心施加额外权重加成（再与检测框聚合、归一化）。

    用于给「大门 / 重点走廊」等非检测框区域加权；坐标与地图系一致（米）。
    """

    xmin: float
    ymin: float
    xmax: float
    ymax: float
    boost: float = 0.15


@dataclass
class InitialRegionWeight:
    """对 **指定分区**（可按 zone_id 或 polygon 包含关系匹配）给定初始权重。"""

    name: str
    weight: float
    applies_to_zone_ids: list[str] | None = None
    # 矩形格 row,col 前缀或通配可由上层解释；此处保持简单仅用 zone_id


@dataclass
class ObjectObservation:
    """单次视觉观测（已通过你的检测算法得到 bbox 与世界坐标系下代表点）。"""

    timestamp_s: float
    zone_id: str
    class_name: str
    # 代表点（如检测框中心投影到地面）
    x: float
    y: float
    confidence: float = 1.0
    track_id: int | None = None  # 若视觉侧有 tracker 可直接填


@dataclass
class AggregationParams:
    """检测框在完成探索后如何从「物体数量」落成权重。"""

    # 可将数量映射到权重：[0,max_objects] -> [weight_floor, weight_ceil]
    weight_floor: float = 0.15
    weight_ceil: float = 1.0
    # w_zone = clamp(w_floor + (n_unique / scale) * (w_ceil - w_floor), ...)
    count_scale: float = 5.0
    # 与初始权重融合：multiplicative mix
    blend_with_initial: Literal["mul", "add", "max"] = "mul"


@dataclass
class StartSamplingParams:
    """随机选起始格：**高权重格子** 与 **贴工作区边界** 的格子概率更高。"""

    # 与 ``rect_weights`` 相乘后进入未归一化采样质量
    weight_boost: float = 1.35
    # 贴边格额外加的质量（相对 uniform_floor）
    edge_boost: float = 0.75
    # 每格至少有的质量，避免零概率
    uniform_floor: float = 0.07
    # 判断「贴边」：与 workspace bbox 距离小于该阈值 (m) 即视为边缘格
    edge_tol_m: float = 0.08


@dataclass
class PathPlannerParams:
    """奖励–代价近似：高权重矩形格给予高「奖励」，低权重格随机 sprinkle 探索奖励。"""

    cost_per_meter: float = 1.0
    # 经过一个矩形格的额外奖励（最终会转为负代价近似）
    reward_high_weight_coef: float = 2.0
    exploration_bonus_low_weight: float = 0.35
    n_random_bonus_cells: int = 12
    random_seed: int = 0
    # 归一化权重得到 0..1 再参与
    epsilon: float = 1e-6
