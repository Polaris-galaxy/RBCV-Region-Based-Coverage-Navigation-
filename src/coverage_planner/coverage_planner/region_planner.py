"""按手工分区独立规划：根据每个区域的"形状"自动选择候选生成策略.

动机:
    全图统一参数容易出现两类问题:
        1. 方正房间被反射顶点 / 中轴产生的密集候选挤满, 解中圆数偏多;
        2. 窄通道又因为六边形过疏丢失关键采样, 角点附近反复重叠.
    本模块按照分区掩码 (来自 :mod:`region_io`) 把规划问题切分为若干独立子问题,
    每个子问题根据 **形状指标** 自动判定候选策略:

        - "open"   : 高矩形度 + 平均自由半径 ≥ ``r``      ->  仅六边形 + 少量反射 (减少冗余)
        - "narrow" : 平均自由半径 < ``0.6 r``             ->  中轴 + 反射, 关闭六边形
        - "mixed"  : 介于两者之间                          ->  全部启用, reflex 稀疏化

设计上 **完全独立** 于 :class:`PlannerConfig`: 仅利用现有 ``plan_coverage``,
只负责为每个分区构造合适的 config 并合并解.
"""
from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Iterable

import numpy as np

from .map_io import GridMap, preprocess
from .planner import PlannerConfig, PlannerResult, plan_coverage


@dataclass
class RegionShape:
    """分区形状指标."""

    region_id: int
    label: str
    free_px: int
    bbox_h: int
    bbox_w: int
    rectangularity: float       # area / bbox_area, 1.0 = 完美矩形
    width_ratio: float          # 2 * median(edt) / r ≈ 平均通道宽度 / r
    aspect_ratio: float         # max / min(bbox 边)
    kind: str                   # "open" / "narrow" / "mixed" / "tiny"


@dataclass
class RegionPlan:
    region_id: int
    shape: RegionShape
    config: PlannerConfig
    result: PlannerResult


@dataclass
class PartitionResult:
    """逐区规划合并结果."""

    region_plans: list[RegionPlan] = field(default_factory=list)

    @property
    def total_circles(self) -> int:
        return sum(rp.result.selected_points.shape[0] for rp in self.region_plans)

    def all_points(self) -> np.ndarray:
        parts = [
            rp.result.selected_points
            for rp in self.region_plans
            if rp.result.selected_points.shape[0] > 0
        ]
        if not parts:
            return np.zeros((0, 2), dtype=np.int32)
        return np.concatenate(parts, axis=0)


def classify_region(
    region_mask: np.ndarray,
    edt_local: np.ndarray,
    r_px: float,
    region_id: int = 0,
    rect_thresh: float = 0.78,
    open_width_factor: float = 1.0,
    narrow_width_factor: float = 0.6,
    tiny_px: int = 25,
) -> RegionShape:
    """根据 ``region_mask`` 与同形状 ``edt_local`` 输出 :class:`RegionShape`.

    Args:
        region_mask: ``[H, W]`` bool, 当前分区的可规划自由区 (= ``free ∩ region``).
        edt_local: ``[H, W]`` float32, 与 ``region_mask`` 同尺寸的距离场;
            **建议使用 region 子区上单独 preprocess 得到的 edt**,
            否则相邻分区交界会被当作"墙"使 EDT 偏小.
        r_px: 观测半径 (像素).
    """
    free_px = int(region_mask.sum())
    if free_px == 0:
        return RegionShape(region_id, str(region_id), 0, 0, 0, 0.0, 0.0, 0.0, "tiny")

    ys, xs = np.where(region_mask)
    bbox_h = int(ys.max() - ys.min() + 1)
    bbox_w = int(xs.max() - xs.min() + 1)
    bbox_area = max(1, bbox_h * bbox_w)
    rectangularity = free_px / bbox_area
    aspect = max(bbox_h, bbox_w) / max(1, min(bbox_h, bbox_w))

    edt_vals = edt_local[region_mask]
    if edt_vals.size == 0:
        median_edt = 0.0
    else:
        median_edt = float(np.median(edt_vals))
    width_ratio = (2.0 * median_edt) / max(r_px, 1e-6)

    if free_px < tiny_px:
        kind = "tiny"
    elif width_ratio < narrow_width_factor:
        kind = "narrow"
    elif width_ratio >= open_width_factor and rectangularity >= rect_thresh:
        kind = "open"
    else:
        kind = "mixed"

    return RegionShape(
        region_id=region_id,
        label=str(region_id),
        free_px=free_px,
        bbox_h=bbox_h,
        bbox_w=bbox_w,
        rectangularity=float(rectangularity),
        width_ratio=float(width_ratio),
        aspect_ratio=float(aspect),
        kind=kind,
    )


def config_for_kind(base: PlannerConfig, kind: str) -> PlannerConfig:
    """按区域类型生成 :class:`PlannerConfig`.

    在 ``base`` 之上做 **最小化覆盖修改**, 用户预先在 base 设的字段 (``r``,
    ``coverage_ratio``, ``ils_*`` 等) 仍然生效.
    """
    if kind == "open":
        return replace(
            base,
            use_medial=False,
            use_hex=True,
            use_reflex=False,
            random_extra=min(base.random_extra, 50),
            ils_max_iter=min(base.ils_max_iter, 30),
        )
    if kind == "narrow":
        return replace(
            base,
            use_medial=True,
            use_hex=False,
            use_reflex=True,
            random_extra=min(base.random_extra, 200),
        )
    if kind == "tiny":
        return replace(
            base,
            use_medial=False,
            use_hex=False,
            use_reflex=False,
            random_extra=max(20, min(base.random_extra, 80)),
            enable_ils=False,
        )
    return replace(base, use_medial=True, use_hex=True, use_reflex=True)


def plan_partitions(
    free_full: np.ndarray,
    labels: np.ndarray,
    r_px: float,
    base_config: PlannerConfig | None = None,
    region_ids: Iterable[int] | None = None,
    pad_px: int = 4,
    verbose: bool = False,
) -> PartitionResult:
    """对 ``labels`` 中每个 ``id>=1`` 的分区独立规划并合并解.

    Args:
        free_full: ``[H, W]`` bool, **整图** 自由空间 (visibility 计算依赖它).
        labels: ``[H, W]`` int, ``0`` 不属于任何分区, ``k>=1`` 为第 k 个分区.
        r_px: 观测半径 (像素).
        base_config: 共用的 ``PlannerConfig``. 留空则按 ``r_px`` 给一份合理默认.
        region_ids: 仅规划指定 ID; 默认扫描 ``labels`` 中出现的所有 id.
        pad_px: 局部 EDT 计算时 region 子掩码向自由空间扩张几像素,
            缓解 "切断墙" 影响. 取 ``ceil(r_px/2)`` 比较稳, 默认 4.

    Notes:
        当前实现给每个区独立调 :func:`plan_coverage`, 因而 visibility 在子图内
        运算. 子图之外的障碍仍被自然忽略 (子图边界默认是非自由),
        若分区交界处有跨界遮挡场景, 可后续改为 "全图 visibility, 子图候选/目标".
    """
    if base_config is None:
        base_config = PlannerConfig(
            r=r_px,
            coverage_ratio=1.0,
            target_subsample=4,
            min_corridor=2.0,
            augment_for_full_cover=True,
            random_extra=200,
            enable_ils=True,
            ils_max_iter=40,
            ils_patience=10,
            verbose=verbose,
        )
    else:
        base_config = replace(base_config, r=r_px, verbose=verbose or base_config.verbose)

    if region_ids is None:
        region_ids = sorted(int(k) for k in np.unique(labels) if k > 0)
    else:
        region_ids = sorted(int(k) for k in region_ids)

    h, w = free_full.shape
    out: list[RegionPlan] = []
    for rid in region_ids:
        region_mask_full = (labels == rid) & free_full
        if not region_mask_full.any():
            continue

        ys, xs = np.where(region_mask_full)
        y0 = max(0, int(ys.min()) - pad_px)
        y1 = min(h, int(ys.max()) + 1 + pad_px)
        x0 = max(0, int(xs.min()) - pad_px)
        x1 = min(w, int(xs.max()) + 1 + pad_px)

        free_local_full = free_full[y0:y1, x0:x1].copy()
        region_local = region_mask_full[y0:y1, x0:x1]
        free_local_planning = free_local_full & region_local

        # 用 region-local 的可规划自由域 preprocess (EDT/skeleton 不被分区
        # 边界外的"虚墙"扭曲).
        gmap_local = preprocess(free_local_planning, min_corridor=base_config.min_corridor)

        shape = classify_region(
            region_mask=gmap_local.free,
            edt_local=gmap_local.edt,
            r_px=r_px,
            region_id=rid,
        )
        cfg = config_for_kind(base_config, shape.kind)

        if verbose:
            print(
                f"[region {rid}] kind={shape.kind} free_px={shape.free_px} "
                f"bbox={shape.bbox_h}x{shape.bbox_w} rect={shape.rectangularity:.2f} "
                f"width_ratio={shape.width_ratio:.2f}"
            )

        result = plan_coverage(gmap_local, cfg)

        if result.selected_points.shape[0] > 0:
            shifted = result.selected_points.copy()
            shifted[:, 0] += y0
            shifted[:, 1] += x0
            result = PlannerResult(
                gmap=GridMap(
                    free=free_full,
                    edt=np.zeros((1, 1), dtype=np.float32),
                    skeleton=np.zeros((1, 1), dtype=bool),
                    skel_dist=np.zeros((1, 1), dtype=np.float32),
                ),
                candidates=result.candidates,
                selected_indices=result.selected_indices,
                selected_points=shifted.astype(np.int32),
                coverage_ratio=result.coverage_ratio,
                target_count=result.target_count,
                target_xy=result.target_xy,
                greedy_size=result.greedy_size,
                ils_size=result.ils_size,
                ils_stats=result.ils_stats,
            )

        out.append(RegionPlan(region_id=rid, shape=shape, config=cfg, result=result))

    return PartitionResult(region_plans=out)
