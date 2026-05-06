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

import math
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
    trimmed_points: np.ndarray | None = None
    trimmed_region_ids: np.ndarray | None = None
    trim_stats: dict | None = None

    @property
    def total_circles(self) -> int:
        if self.trimmed_points is not None:
            return int(self.trimmed_points.shape[0])
        return sum(rp.result.selected_points.shape[0] for rp in self.region_plans)

    @property
    def raw_total_circles(self) -> int:
        """精修前各分区独立解之和."""
        return sum(rp.result.selected_points.shape[0] for rp in self.region_plans)

    def all_points(self) -> np.ndarray:
        if self.trimmed_points is not None:
            return self.trimmed_points
        return self._concat_region_points()

    def all_region_ids(self) -> np.ndarray:
        """与 ``all_points()`` 对齐的分区 ID 序列."""
        if self.trimmed_points is not None and self.trimmed_region_ids is not None:
            return self.trimmed_region_ids
        ids: list[int] = []
        for rp in self.region_plans:
            n = rp.result.selected_points.shape[0]
            if n > 0:
                ids.extend([rp.region_id] * n)
        return np.asarray(ids, dtype=np.int64)

    def _concat_region_points(self) -> np.ndarray:
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
    corridor_aspect_thresh: float = 3.0,
    corridor_width_thresh: float = 1.5,
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
    elif (
        aspect >= corridor_aspect_thresh
        and width_ratio < corridor_width_thresh
    ):
        kind = "corridor"
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
    if kind == "corridor":
        # 走廊会跳过 plan_coverage 直接沿中轴线放圆, 这里仅给一份保守 fallback
        # config (若 _plan_corridor_local 失败回退用).
        return replace(
            base,
            use_medial=True,
            use_hex=False,
            use_reflex=True,
            random_extra=min(base.random_extra, 100),
            enable_ils=False,
        )
    return replace(base, use_medial=True, use_hex=True, use_reflex=True)


def _plan_corridor_local(
    gmap_local: GridMap,
    r_px: float,
    step_factor: float = math.sqrt(3.0),
    push_into_free: bool = True,
) -> np.ndarray:
    """沿走廊中轴线按六边形单行步距 ``√3·r`` 等距放置圆心.

    几何依据:
        宽度不超过 ``r`` 的单行覆盖, 相邻圆心间距取 ``√3·r`` (约 1.732 r)
        即可保证整条带状区域被覆盖, 与正六边形蜂窝单行步距一致.
        宽度更大的走廊也能被等距单行覆盖, 留少量交叠.

    实现:
        1. 取 ``gmap_local.skeleton`` 上所有像素;
        2. 用 PCA 主方向把这些点投影到一维弧长;
        3. 沿一维投影按 ``step = step_factor * r_px`` 等距挑点;
        4. 可选地把每个点沿 EDT 梯度向自由空间深处微推, 提升单点视距.

    Args:
        gmap_local: 走廊子图.
        r_px: 视距半径 (像素).
        step_factor: 步距系数, 默认 ``√3 ≈ 1.732``.
        push_into_free: 是否做最后一步 EDT 微推.

    Returns:
        ``[N, 2]`` int32 圆心 (row, col), 已经在 ``gmap_local`` 局部坐标系内.
    """
    from .visibility import _push_into_free as _push

    skel = gmap_local.skeleton
    if not skel.any():
        ys, xs = np.where(gmap_local.free)
        if ys.size == 0:
            return np.zeros((0, 2), dtype=np.int32)
        skel_pts = np.stack([ys, xs], axis=1).astype(np.float64)
    else:
        ys, xs = np.where(skel)
        skel_pts = np.stack([ys, xs], axis=1).astype(np.float64)

    if skel_pts.shape[0] == 1:
        only = skel_pts[0].astype(np.int32)[None, :]
        return only

    centroid = skel_pts.mean(axis=0)
    centered = skel_pts - centroid
    cov = centered.T @ centered
    _eigvals, eigvecs = np.linalg.eigh(cov)
    main_dir = eigvecs[:, -1]
    proj = centered @ main_dir

    order = np.argsort(proj)
    sorted_pts = skel_pts[order]
    sorted_proj = proj[order]

    step = max(1.0, float(step_factor) * float(r_px))
    next_t = sorted_proj[0]
    out: list[np.ndarray] = []
    for k, t in enumerate(sorted_proj):
        if t >= next_t:
            out.append(sorted_pts[k])
            next_t = t + step
    if not out:
        out.append(sorted_pts[0])
    pts = np.asarray(out, dtype=np.int32)

    if push_into_free and gmap_local.edt is not None:
        pts = _push(
            pts.astype(np.int32),
            gmap_local.edt.astype(np.float32),
            gmap_local.free,
            target_edt=0.45 * float(r_px),
            max_walk=max(2, int(0.45 * float(r_px))),
        ).astype(np.int32)

    return pts


def globally_trim_solution(
    free_full: np.ndarray,
    selected_points: np.ndarray,
    r_px: float,
    target_mask_full: np.ndarray | None = None,
    target_subsample: int = 4,
    n_rays: int | None = None,
    max_passes: int = 3,
    verbose: bool = False,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """对合并后的全图解做"全图可见性下的删圆"局部搜索, 削减跨分区冗余.

    与 ``plan_coverage`` 内部 ILS 不同, 这里 visibility / 目标都是 **整图** 的,
    因此一个分区的圆有可能覆盖到另一分区边缘的目标, 从而允许整体减圆.

    Args:
        free_full: 整图 ``[H, W]`` bool 自由空间.
        selected_points: ``[N, 2]`` int32, 合并后的圆心 (row, col).
        r_px: 视距半径 (像素).
        target_mask_full: 必须覆盖的整图目标掩码; ``None`` 取 ``free_full``.
        target_subsample: 目标格步长 (与 :class:`PlannerConfig.target_subsample` 一致).
        n_rays: 极坐标射线数, ``None`` 走默认.
        max_passes: 最多反复几轮 try_remove (每轮删一遍).
        verbose: 打印中文进度.

    Returns:
        (new_points, keep_index, stats):
            ``new_points`` 是裁剪后保留的 ``[M, 2]`` 圆心,
            ``keep_index`` 是它们在 ``selected_points`` 中的下标,
            ``stats`` 记录前后圆数与裁剪轮次.
    """
    from .map_io import GridMap
    from .planner import _subsample_targets
    from .refine import SolutionState, _try_remove_pass
    from .visibility import build_coverage

    n_sel = int(selected_points.shape[0])
    if n_sel <= 1:
        keep = np.arange(n_sel, dtype=np.int64)
        return selected_points.copy(), keep, {
            "before": n_sel, "after": n_sel, "removed": 0, "passes": 0,
        }

    if target_mask_full is None:
        target_mask_full = free_full

    candidates = selected_points.astype(np.int32)

    if verbose:
        print(
            f"[全局精修] 在整图自由空间上重建可见性矩阵：候选数={n_sel}，"
            f"半径 r={r_px:.2f} 像素…"
        )

    gmap_full = GridMap(
        free=free_full,
        edt=np.zeros((1, 1), dtype=np.float32),
        skeleton=np.zeros((1, 1), dtype=bool),
        skel_dist=np.zeros((1, 1), dtype=np.float32),
    )
    coverage, _free_idx, free_xy = build_coverage(
        gmap_full, candidates, r=r_px, n_rays=n_rays
    )

    in_target = target_mask_full[free_xy[:, 0], free_xy[:, 1]]
    sub_mask = _subsample_targets(free_xy, free_full.shape, target_subsample)
    base_target = in_target & sub_mask

    selected_idx = list(range(n_sel))
    state = SolutionState.from_selected(
        coverage, base_target, candidates, selected_idx
    )

    actually_covered = state.cover_count > 0
    state.target_mask = base_target & actually_covered

    if verbose:
        print(
            f"[全局精修] 全图目标={int(base_target.sum())}，"
            f"实际可覆盖={int(state.target_mask.sum())}；开始尝试删圆…"
        )

    total_removed = 0
    pass_used = 0
    for it in range(max_passes):
        before = state.size
        rm = _try_remove_pass(state, verbose=False)
        total_removed += rm
        pass_used = it + 1
        if verbose:
            print(
                f"[全局精修] 第 {pass_used}/{max_passes} 轮：删除 {rm} 个，"
                f"当前圆盘数={state.size}"
            )
        if rm == 0 or state.size == before:
            break

    keep = state.selected_indices().astype(np.int64)
    new_points = candidates[keep].copy()
    stats = {
        "before": n_sel,
        "after": int(keep.size),
        "removed": total_removed,
        "passes": pass_used,
    }
    if verbose:
        print(
            f"[全局精修] 完成：圆盘数 {n_sel} → {keep.size}（共减少 "
            f"{total_removed} 个）"
        )
    return new_points, keep, stats


def plan_partitions(
    free_full: np.ndarray,
    labels: np.ndarray,
    r_px: float,
    base_config: PlannerConfig | None = None,
    region_ids: Iterable[int] | None = None,
    pad_px: int = 4,
    global_trim: bool = True,
    trim_target_subsample: int | None = None,
    trim_max_passes: int = 3,
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
    n_regions = sum(
        1 for rid in region_ids if ((labels == rid) & free_full).any()
    )
    progress_i = 0
    out: list[RegionPlan] = []
    for rid in region_ids:
        region_mask_full = (labels == rid) & free_full
        if not region_mask_full.any():
            continue

        progress_i += 1
        if verbose:
            print(
                f"[分区进度] 第 {progress_i}/{n_regions} 个："
                f"正在规划分区 ID={rid} …"
            )

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
                f"[分区 {rid}] 类型={shape.kind} 可走像素={shape.free_px} "
                f"包围盒={shape.bbox_h}×{shape.bbox_w} "
                f"长方形程度={shape.rectangularity:.2f} "
                f"宽度比={shape.width_ratio:.2f} "
                f"长宽比={shape.aspect_ratio:.2f}"
            )

        if shape.kind == "corridor":
            corr_pts = _plan_corridor_local(gmap_local, r_px=r_px)
            if verbose:
                print(
                    f"[分区 {rid}] 走廊模式：沿中轴 √3·r 步距生成 "
                    f"{corr_pts.shape[0]} 个圆心"
                )
            free_xy_local = np.argwhere(gmap_local.free).astype(np.int32)
            result = PlannerResult(
                gmap=gmap_local,
                candidates=corr_pts.copy(),
                selected_indices=list(range(corr_pts.shape[0])),
                selected_points=corr_pts.astype(np.int32),
                coverage_ratio=1.0,
                target_count=int(free_xy_local.shape[0]),
                target_xy=free_xy_local,
                greedy_size=corr_pts.shape[0],
                ils_size=corr_pts.shape[0],
                ils_stats=None,
            )
        else:
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

    partition = PartitionResult(region_plans=out)

    if global_trim and partition.raw_total_circles >= 2:
        merged_pts = partition._concat_region_points()
        merged_ids = []
        for rp in out:
            n = rp.result.selected_points.shape[0]
            if n > 0:
                merged_ids.extend([rp.region_id] * n)
        merged_ids_arr = np.asarray(merged_ids, dtype=np.int64)

        sub = (
            trim_target_subsample
            if trim_target_subsample is not None
            else base_config.target_subsample
        )
        if verbose:
            print(
                f"[全局精修] 启动后处理：合并各分区共 {merged_pts.shape[0]} 个圆，"
                f"准备做整图删圆精修。"
            )
        new_pts, keep_idx, stats = globally_trim_solution(
            free_full=free_full,
            selected_points=merged_pts,
            r_px=r_px,
            target_subsample=sub,
            max_passes=trim_max_passes,
            verbose=verbose,
        )
        partition.trimmed_points = new_pts.astype(np.int32)
        partition.trimmed_region_ids = merged_ids_arr[keep_idx]
        partition.trim_stats = stats

    return partition
