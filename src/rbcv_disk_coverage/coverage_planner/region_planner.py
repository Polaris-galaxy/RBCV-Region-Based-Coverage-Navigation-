"""按手工分区独立规划：根据每个区域的"形状"自动选择候选生成策略.

动机:
    全图统一参数容易出现两类问题:
        1. 方正房间被反射顶点 / 中轴产生的密集候选挤满, 解中圆数偏多;
        2. 窄通道又因为六边形过疏丢失关键采样, 角点附近反复重叠.
    本模块按照分区掩码 (来自 :mod:`region_io`) 把规划问题切分为若干独立子问题,
    每个子问题根据 **形状指标** 自动判定候选策略; **mixed** 类型在单个分区内再以
    **EDT 门控** 分流中轴候选（开阔核偏重六边形、狭窄/靠墙带保留中轴）, 近似
    「手绘大框内的自动细分」, 详见 :func:`config_for_kind`.

        - "rect_room": 高矩形度 + 平均自由半径 ≥ ``r``      ->  仅六边形 + 少量反射 (减少冗余)
        - "narrow" : 平均自由半径 < ``0.6 r``             ->  中轴 + 反射, 关闭六边形
        - "mixed"  : 介于两者之间                          ->  全部启用 + 窄部 EDT 限域中轴

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
from .set_cover import find_unreachable_targets
from .visibility import build_coverage

from .candidates import _dedup
from .corridor_split import wide_corridor_strip_centers


def _plan_one_region_task(
    free_full: np.ndarray,
    labels: np.ndarray,
    rid: int,
    r_px: float,
    base_config: PlannerConfig,
    pad_px: int,
    verbose: bool,
    capture_stdout: bool = False,
) -> tuple[RegionPlan | None, np.ndarray, np.ndarray, str]:
    """单分区规划任务（供线程/进程池并行调用）。

    Args:
        capture_stdout: 仅并行且需要汇总日志时设为 True，避免多线程 print 交叉；
            串行模式下应为 False，否则会吞掉正常控制台输出。

    Returns:
        (region_plan_or_none, pool_candidates_global, pool_candidate_rids, log_text)
    """
    if not capture_stdout:
        rp, pool_pts, pool_rids = _plan_one_region_task_impl(
            free_full=free_full,
            labels=labels,
            rid=rid,
            r_px=r_px,
            base_config=base_config,
            pad_px=pad_px,
            verbose=verbose,
        )
        return rp, pool_pts, pool_rids, ""

    import io
    from contextlib import redirect_stdout

    buf = io.StringIO()
    with redirect_stdout(buf):
        rp, pool_pts, pool_rids = _plan_one_region_task_impl(
            free_full=free_full,
            labels=labels,
            rid=rid,
            r_px=r_px,
            base_config=base_config,
            pad_px=pad_px,
            verbose=verbose,
        )
    return rp, pool_pts, pool_rids, buf.getvalue()


def _plan_one_region_task_impl(
    free_full: np.ndarray,
    labels: np.ndarray,
    rid: int,
    r_px: float,
    base_config: PlannerConfig,
    pad_px: int,
    verbose: bool,
) -> tuple[RegionPlan | None, np.ndarray, np.ndarray]:
    """单分区规划实现（不捕获 stdout）。"""
    h, w = free_full.shape
    region_mask_full = (labels == rid) & free_full
    if not region_mask_full.any():
        return None, np.zeros((0, 2), dtype=np.int32), np.zeros((0,), dtype=np.int64)

    ys, xs = np.where(region_mask_full)
    y0 = max(0, int(ys.min()) - pad_px)
    y1 = min(h, int(ys.max()) + 1 + pad_px)
    x0 = max(0, int(xs.min()) - pad_px)
    x1 = min(w, int(xs.max()) + 1 + pad_px)

    free_local_full = free_full[y0:y1, x0:x1].copy()
    region_local = region_mask_full[y0:y1, x0:x1]
    free_local_planning = free_local_full & region_local

    gmap_local = preprocess(
        free_local_planning, min_corridor=base_config.min_corridor
    )
    shape = classify_region(
        region_mask=gmap_local.free,
        edt_local=gmap_local.edt,
        r_px=r_px,
        region_id=rid,
        corridor_width_thresh=float(base_config.corridor_classify_width_ratio_max),
    )
    if base_config.use_hex_first and shape.kind != "corridor":
        # 新流水线下不再按 kind 预先关掉某类候选；plan_coverage 内部会分流到
        # plan_coverage_hex_first（先 hex，再针对未覆盖的残余区补 medial/reflex）。
        cfg = replace(base_config)
    else:
        cfg = config_for_kind(base_config, shape.kind)

    if (
        base_config.force_hex_first_rectangle_rooms
        and shape.kind not in ("corridor", "tiny", "narrow")
        and shape.rectangularity >= base_config.rect_room_rectangularity_min
        and shape.width_ratio >= base_config.rect_room_width_ratio_min
    ):
        cfg = replace(cfg, use_hex_first=True)

    shape_final = shape

    if verbose:
        flow = "Hx" if cfg.use_hex_first and shape.kind != "corridor" else shape.kind
        print(
            f"[r{rid}] {shape.kind}/{flow} px={shape.free_px} "
            f"AR={shape.aspect_ratio:.1f} WR={shape.width_ratio:.1f}r"
        )

    if shape.kind == "corridor":
        corr_pts, co_meta = _plan_corridor_local(
            gmap_local,
            r_px=r_px,
            resolution_m_per_px=cfg.resolution_m_per_px,
            corridor_strip_width_m=cfg.corridor_strip_width_m,
            enable_strip_split=cfg.enable_corridor_strip_split,
            alpha=cfg.alpha,
            corridor_spine_step_floor_frac=cfg.corridor_spine_step_floor_frac,
        )
        fallback = (
            corr_pts.shape[0] == 0
            or not _corridor_quick_covers_all_free_pixels(
                gmap_local, corr_pts, r_px=r_px, n_rays=cfg.n_rays
            )
        )
        if fallback:
            # 自适应步距未全覆盖 (常见于多分支走廊): 收紧到固定 1.25·r 兜底
            tight_pts, tight_meta = _plan_corridor_local(
                gmap_local,
                r_px=r_px,
                step_factor=1.25,
                resolution_m_per_px=cfg.resolution_m_per_px,
                corridor_strip_width_m=cfg.corridor_strip_width_m,
                enable_strip_split=cfg.enable_corridor_strip_split,
                alpha=cfg.alpha,
                corridor_spine_step_floor_frac=cfg.corridor_spine_step_floor_frac,
            )
            if tight_pts.shape[0] and _corridor_quick_covers_all_free_pixels(
                gmap_local, tight_pts, r_px=r_px, n_rays=cfg.n_rays
            ):
                corr_pts = tight_pts
                co_meta = tight_meta
                fallback = False
        if fallback:
            if verbose:
                print(f"[r{rid}] corridor fallback→plan_coverage")
            result = plan_coverage(gmap_local, cfg)
        else:
            if verbose:
                suf = f" {co_meta}" if co_meta else ""
                print(f"[r{rid}] corridor n={corr_pts.shape[0]}{suf}")
            if co_meta:
                shape_final = replace(shape, corridor_plan_meta=co_meta)
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

    local_cands = np.asarray(result.candidates, dtype=np.int32)
    if local_cands.shape[0] > 0:
        cand_global = local_cands.copy().astype(np.int32)
        cand_global[:, 0] += y0
        cand_global[:, 1] += x0
        pool_pts = cand_global
        pool_rids = np.full(cand_global.shape[0], rid, dtype=np.int64)
    else:
        pool_pts = np.zeros((0, 2), dtype=np.int32)
        pool_rids = np.zeros((0,), dtype=np.int64)

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

    return (
        RegionPlan(region_id=rid, shape=shape_final, config=cfg, result=result),
        pool_pts,
        pool_rids,
    )


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
    # "rect_room": 规整开阔矩形（语义上对应「先六边形铺满」）。
    kind: str                   # "rect_room" / "narrow" / "mixed" / "tiny" / "corridor"
    corridor_plan_meta: str = ""  # 走廊专有：划分线采样摘要


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


def _any_hex_first_core_freeze_enabled(region_plans: list[RegionPlan]) -> bool:
    """是否存在启用 hex_first 核区 EDT 冻结的分区（用于整图精修对齐分区内策略）。"""
    for rp in region_plans:
        c = rp.config
        if (
            c.use_hex_first
            and c.hex_first_pure_lattice
            and c.hex_first_protect_core_edt_factor is not None
        ):
            return True
    return False


def _hex_first_global_freeze_mask(
    points: np.ndarray,
    region_ids: np.ndarray,
    r_px: float,
    edt_full: np.ndarray,
    rid_to_cfg: dict[int, PlannerConfig],
) -> np.ndarray | None:
    """整图候选/已选圆上与分区内一致的「高净空 hex 核」冻结掩码."""
    n = int(points.shape[0])
    if n == 0:
        return None
    h, w = edt_full.shape
    freeze = np.zeros(n, dtype=bool)
    for i in range(n):
        y = int(np.clip(points[i, 0], 0, h - 1))
        x = int(np.clip(points[i, 1], 0, w - 1))
        rid = int(region_ids[i])
        cfg = rid_to_cfg.get(rid)
        if cfg is None:
            continue
        prot = cfg.hex_first_protect_core_edt_factor
        if (
            prot is None
            or not cfg.use_hex_first
            or not cfg.hex_first_pure_lattice
        ):
            continue
        if float(edt_full[y, x]) >= float(prot) * float(r_px):
            freeze[i] = True
    return freeze if freeze.any() else None


def classify_region(
    region_mask: np.ndarray,
    edt_local: np.ndarray,
    r_px: float,
    region_id: int = 0,
    rect_thresh: float = 0.78,
    open_width_factor: float = 1.0,
    narrow_width_factor: float = 0.6,
    corridor_aspect_thresh: float = 2.5,
    corridor_width_thresh: float = 4.5,
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
        return RegionShape(
            region_id, str(region_id), 0, 0, 0, 0.0, 0.0, 0.0, "tiny", ""
        )

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
        # 规整矩形开阔区：语义上对应「先做六边形整区铺设」的策略入口。
        kind = "rect_room"
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
    if kind in ("rect_room", "open"):
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
    # 全开候选时，用大房间中心 EDT 过高的中轴点会挤占冗余；仅用「局部净空 ≤ ~0.62r」
    # 的中轴点服务窄口/沿墙走势，开阔核交给六边形（与「核六边形 + 边缘专项」思想一致）。
    return replace(
        base,
        use_medial=True,
        use_hex=True,
        use_reflex=True,
        medial_max_edt_factor=0.62,
    )


def _corridor_spaced_on_skeleton_component(
    skel_pts: np.ndarray,
    r_px: float,
    step_factor: float | None = None,
    edt: np.ndarray | None = None,
    alpha: float = 0.95,
    step_floor_frac: float | None = 0.42,
) -> np.ndarray:
    """在单个连通 skel 点集上按 PCA 主轴投影取样 (局部坐标).

    步距策略:
        * 若 ``edt`` 给出且 ``step_factor is None`` -> **几何自适应步距**:
              半通道宽 ``ρ = edt(p)``, 沿主轴下一步距
                  ``step(p) = α · 2 · sqrt(r² − ρ²)`` (ρ < r)
                  ``      = α · √3 · r``               (ρ ≥ r)
            并设下界 ``≥ step_floor_frac · √3 · α · r``，避免 ρ→r 时步距塌成 1px、脊线过密。
        * 若 ``step_factor`` 给出: 退化为旧版固定步距 = ``factor·r``.

    Args:
        skel_pts: ``[K, 2]`` float, 单列连通分支的中轴像素.
        r_px: 半径 (像素).
        step_factor: 退化为固定步距 (兼容旧调用); ``None`` 走自适应路径.
        edt: ``[H, W]`` float, 整图 EDT, 用于查每个采样点局部 ρ.
        alpha: 自适应步距安全系数 (0~1).
    """
    if skel_pts.shape[0] == 0:
        return np.zeros((0, 2), dtype=np.int32)
    if skel_pts.shape[0] == 1:
        return skel_pts.astype(np.int32)

    centroid = skel_pts.mean(axis=0)
    centered = skel_pts - centroid
    cov = centered.T @ centered
    _eigvals, eigvecs = np.linalg.eigh(cov)
    main_dir = eigvecs[:, -1]
    proj = centered @ main_dir

    order = np.argsort(proj)
    sorted_pts = skel_pts[order]
    sorted_proj = proj[order]

    if edt is not None and step_factor is None:
        yy = sorted_pts[:, 0].astype(int)
        xx = sorted_pts[:, 1].astype(int)
        local_r = np.asarray(edt[yy, xx], dtype=np.float32)

        out_idx: list[int] = [0]
        last_t = float(sorted_proj[0])
        for k in range(1, sorted_proj.size):
            t = float(sorted_proj[k])
            ref_lr = float(local_r[out_idx[-1]])
            if ref_lr >= float(r_px):
                step = float(r_px) * math.sqrt(3.0) * alpha
            else:
                val = max(0.0, float(r_px) ** 2 - ref_lr ** 2)
                step = alpha * 2.0 * math.sqrt(val)
                step = max(step, 1.0)
            if step_floor_frac is not None and float(step_floor_frac) > 0.0:
                floor = float(step_floor_frac) * math.sqrt(3.0) * float(r_px) * float(alpha)
                step = max(step, floor)
            if t - last_t >= step:
                out_idx.append(k)
                last_t = t
        return sorted_pts[out_idx].astype(np.int32)

    step = max(1.0, float(step_factor or math.sqrt(3.0)) * float(r_px))
    next_t = sorted_proj[0]
    out: list[np.ndarray] = []
    for k, t in enumerate(sorted_proj):
        if t >= next_t:
            out.append(sorted_pts[k])
            next_t = t + step
    if not out:
        out.append(sorted_pts[0])
    return np.asarray(out, dtype=np.int32)


def _plan_corridor_local(
    gmap_local: GridMap,
    r_px: float,
    step_factor: float | None = None,
    push_into_free: bool = True,
    resolution_m_per_px: float | None = None,
    corridor_strip_width_m: float = 3.4,
    enable_strip_split: bool = True,
    alpha: float = 0.95,
    corridor_spine_step_floor_frac: float | None = 0.42,
) -> tuple[np.ndarray, str]:
    """沿走廊敷设圆心：窄道单脊线；宽道在条带分界上附加划分线采样.

    米制划条依赖 ``resolution_m_per_px``；未提供时仅走中轴（与旧版一致）。
    返回 ``(centers, meta)``，``meta`` 为各骨架分量的 ``hx2`` 式摘要（h/v + 划线条数）。
    """
    from scipy.ndimage import label as nd_label

    from .visibility import _push_into_free as _push

    skel = gmap_local.skeleton
    segments: list[np.ndarray] = []
    meta_bits: list[str] = []

    if skel.any():
        labeled, ncomp = nd_label(skel, structure=np.ones((3, 3), dtype=int))
        for comp in range(1, ncomp + 1):
            ys, xs = np.where(labeled == comp)
            comp_pts = np.stack([ys, xs], axis=1).astype(np.float64)
            strip = np.zeros((0, 2), dtype=np.int32)
            n_lines = 0
            orient_tag = ""
            if (
                enable_strip_split
                and resolution_m_per_px is not None
                and float(resolution_m_per_px) > 0.0
            ):
                strip, n_lines, orient_tag = wide_corridor_strip_centers(
                    gmap_local.free,
                    gmap_local.edt,
                    comp_pts,
                    float(r_px),
                    float(resolution_m_per_px),
                    float(corridor_strip_width_m),
                    float(alpha),
                )
                if n_lines > 0 and strip.shape[0] > 0 and orient_tag != "narrow":
                    o = orient_tag[0] if orient_tag else "?"
                    meta_bits.append(f"{o}x{n_lines}")

            seg = _corridor_spaced_on_skeleton_component(
                comp_pts,
                r_px,
                step_factor=step_factor,
                edt=gmap_local.edt if step_factor is None else None,
                alpha=float(alpha),
                step_floor_frac=corridor_spine_step_floor_frac,
            )
            chunks = [c for c in (strip, seg) if isinstance(c, np.ndarray) and c.size]
            if not chunks:
                continue
            merged = np.vstack(chunks).astype(np.int32)
            merged = _dedup(merged, max(2.0, 0.52 * float(r_px)))
            if merged.size:
                segments.append(merged)

    meta = ";".join(meta_bits)
    # 无障碍中轴时使用整块自由网格会严重误导 PCA → 交由上层 fallback plan_coverage。
    if not segments:
        return np.zeros((0, 2), dtype=np.int32), meta

    pts = np.vstack(segments).astype(np.int32)

    if push_into_free and gmap_local.edt is not None:
        pts = _push(
            pts.astype(np.int32),
            gmap_local.edt.astype(np.float32),
            gmap_local.free,
            target_edt=0.45 * float(r_px),
            max_walk=max(2, int(0.45 * float(r_px))),
        ).astype(np.int32)

    return pts, meta


def _corridor_quick_covers_all_free_pixels(
    gmap_local: GridMap,
    centers: np.ndarray,
    r_px: float,
    n_rays: int | None,
) -> bool:
    """圆心集在 **当前子图** 栅格可见性下是否触及全部自由像素 (步长 1)."""
    if centers.shape[0] == 0:
        return False
    cov, _, _free_xy = build_coverage(
        gmap_local, centers.astype(np.int32), r=r_px, n_rays=n_rays
    )
    if cov.shape[1] == 0:
        return True
    unreachable = find_unreachable_targets(
        cov, np.ones(cov.shape[1], dtype=bool)
    )
    return unreachable.size == 0


def globally_trim_solution(
    free_full: np.ndarray,
    selected_points: np.ndarray,
    r_px: float,
    target_mask_full: np.ndarray | None = None,
    target_subsample: int = 4,
    n_rays: int | None = None,
    max_passes: int = 3,
    verbose: bool = False,
    freeze_mask: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray, dict]:
    """对合并后的全图解做"全图可见性下的删圆"局部搜索, 削减跨分区冗余.

    与 ``plan_coverage`` 内部 ILS 不同, 这里 visibility / 目标都是 **整图** 的,
    因此一个分区的圆有可能覆盖到另一分区边缘的目标, 从而允许整体减圆.

    Args:
        free_full: 整图 ``[H, W]`` bool 自由空间.
        selected_points: ``[N, 2]`` int32, 合并后的圆心 (row, col).
        r_px: 视距半径 (像素).
        target_mask_full: 必须覆盖的整图目标掩码; ``None`` 取 ``free_full``.
        target_subsample: 目标格步长。**注意**: 若 >1, 删圆时只保证稀疏采样格
            仍被覆盖, 可能在采样格之间的自由空间留下大片“视觉空洞”. 分区规划
            仍可用 ``PlannerConfig.target_subsample``; 全局精修默认单独用更密
            的目标 (见 ``plan_partitions.trim_target_subsample``).
        n_rays: 极坐标射线数, ``None`` 走默认.
        max_passes: 最多反复几轮 try_remove (每轮删一遍).
        verbose: 打印中文进度.
        freeze_mask: 与 ``selected_points``（即候选）同长；``True`` 的圆不允许删除
            （与 :func:`refine._try_remove_pass` 一致），用于与 hex_first 核区保护对齐。

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
        rm = _try_remove_pass(state, verbose=False, freeze_mask=freeze_mask)
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


def globally_optimize_solution(
    free_full: np.ndarray,
    candidates: np.ndarray,
    initial_selected_idx: np.ndarray | list[int],
    r_px: float,
    target_mask_full: np.ndarray | None = None,
    target_subsample: int = 1,
    n_rays: int | None = None,
    mode: str = "ils",
    ils_max_iter: int = 25,
    ils_patience: int = 6,
    enable_swap3: bool = True,
    swap3_max_attempts: int | None = 100000,
    time_limit: float | None = None,
    soft_target_min_edt: float = 0.0,
    soft_isolated_max_area_px: int = 0,
    seed: int = 0,
    verbose: bool = False,
    greedy_edt_tiebreak: bool = True,
    freeze_mask: np.ndarray | None = None,
    merge_identical_pool: bool = True,
    overlap_tiebreak: bool = True,
    greedy_nnz_tiebreak: bool = True,
) -> tuple[np.ndarray, dict]:
    """整图候选池上的强力 post-processing.

    与 :func:`globally_trim_solution` (仅 ``try_remove``) 不同, 本函数:
        - **候选池** = 各分区生成的候选 (而非仅初解), 因此可以做 ``swap_2_for_1`` /
          ``swap_3_for_2``: 删 2 个 / 3 个分区内圆 + 加入 1 个 / 2 个未选候选, 真正
          降总圆数.
        - 直接复用 ``refine.ils`` 完整局部搜索 (含 try_remove + swap2 + swap3 + 扰动).

    Args:
        candidates: ``[N, 2]`` int32, **整图坐标** 的候选池 (含初解).
        initial_selected_idx: 初解在 ``candidates`` 中的下标.
        mode:
            - ``"local"``      : 以初解 warm-start, 只做 try_remove + 局部 swap,
                                  不做扰动; 更保留六边形布局, 空旷区更稳.
            - ``"ils"``        : 以初解 warm-start 跑完整 ILS (含扰动).
            - ``"regreedy"``   : 先在 pool 上重新跑 greedy 取代初解, 再跑 ILS (更激进).
            - ``"remove-only"``: 仅 ``try_remove`` 多轮 (旧 ``globally_trim_solution`` 行为).
        soft_target_min_edt: **目标软化** 阈值 (像素). 若 > 0, 把整图自由空间中
            ``EDT < soft_target_min_edt`` (即贴近障碍/位于小空隙里) 的目标格从
            ILS 的硬约束中剔除. 直观语义: "这种像素允许由周围圆近似覆盖, 不强制
            为它们专门留圆", 能显著减少在杂物密集区的冗余圆. 0 = 关闭, 推荐
            ``0.08 ~ 0.15 * r`` (例 r=50 px → 4 ~ 8 px).
        soft_isolated_max_area_px: **小连通域剔除** 阈值. 若 > 0, 对目标掩码做
            连通域分析, 面积 ``< soft_isolated_max_area_px`` 的"孤立小自由片段"
            从硬约束中剔除. 多用于补漏地图清洗后仍残留的细小自由噪点.
        greedy_edt_tiebreak: 仅当 ``mode=="regreedy"`` 时有效；整图重新贪心时用
            圆心 EDT 作第三级 tie-break（与 :attr:`PlannerConfig.greedy_edt_tiebreak` 对齐）。
        freeze_mask: 长度与 ``candidates`` 一致；``True`` 的候选不参与删圆 / 作为 swap
            被删端（与 :func:`local_search` 一致），用于整图阶段对齐 hex_first 核区保护。

    Returns:
        ``(new_idx, stats)``: ``new_idx`` 为最终选中下标 (在 ``candidates`` 中).
    """
    from .map_io import GridMap
    from .planner import _subsample_targets
    from .refine import SolutionState, _greedy_repair, _try_remove_pass, ils, local_search
    from .set_cover import greedy_set_cover

    candidates = np.asarray(candidates, dtype=np.int32)
    initial_selected_idx = np.asarray(initial_selected_idx, dtype=np.int64).ravel()
    n_pool = int(candidates.shape[0])
    n_init = int(initial_selected_idx.size)

    if n_pool == 0:
        return np.zeros((0,), dtype=np.int64), {
            "before": 0, "after": 0, "removed": 0, "mode": mode, "passes": 0,
        }

    if target_mask_full is None:
        target_mask_full = free_full

    if verbose:
        print(
            f"[全局精修-{mode}] 候选池规模={n_pool}（其中初解 {n_init}），"
            f"r={r_px:.2f} 像素，正在整图自由空间上重建可见性矩阵…"
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

    from .visibility import (
        merge_freeze_masks_after_prune,
        prune_identical_coverage_rows,
        remap_pool_indices_after_prune,
    )

    if merge_identical_pool:
        from scipy.ndimage import distance_transform_edt

        n_before_merge = int(candidates.shape[0])
        edt_prune = distance_transform_edt(free_full.astype(bool))
        coverage, candidates, pool_otn = prune_identical_coverage_rows(
            coverage, candidates, edt=edt_prune.astype(np.float32),
        )
        candidates = candidates.astype(np.int32, copy=False)
        initial_selected_idx = remap_pool_indices_after_prune(
            initial_selected_idx, pool_otn
        )
        freeze_mask = merge_freeze_masks_after_prune(
            freeze_mask, pool_otn, int(candidates.shape[0])
        )
        n_pool = int(candidates.shape[0])
        if verbose and n_before_merge != n_pool:
            print(
                f"[全局精修-{mode}] 合并完全相同覆盖行：候选池 "
                f"{n_before_merge} → {n_pool}"
            )

    in_target = target_mask_full[free_xy[:, 0], free_xy[:, 1]]
    sub_mask = _subsample_targets(free_xy, free_full.shape, target_subsample)
    base_target = in_target & sub_mask

    # 移除整池都覆盖不到的目标 (避免 ILS 试图覆盖不可达像素).
    col_count = np.asarray((coverage > 0).sum(axis=0)).ravel()
    coverable = col_count > 0
    target = base_target & coverable
    n_dropped = int(base_target.sum() - target.sum())
    if verbose:
        if n_dropped:
            print(
                f"[全局精修-{mode}] 整图目标 {int(base_target.sum())} 中 "
                f"{n_dropped} 个无候选可达，已剔除"
            )
        else:
            print(
                f"[全局精修-{mode}] 整图目标 {int(target.sum())}，全部至少有一个候选"
            )

    # ---- 目标软化: 贴近障碍 / 位于小连通空隙的像素不计入硬约束 ----
    if float(soft_target_min_edt) > 0.0 or int(soft_isolated_max_area_px) > 0:
        from scipy.ndimage import (
            distance_transform_edt,
            label as cc_label,
        )

        n_before_soft = int(target.sum())
        soft_mask = np.zeros(target.shape, dtype=bool)

        if float(soft_target_min_edt) > 0.0:
            edt_full = distance_transform_edt(free_full)
            edt_vals = edt_full[free_xy[:, 0], free_xy[:, 1]]
            soft_by_edt = edt_vals < float(soft_target_min_edt)
            soft_mask = soft_mask | soft_by_edt
            if verbose:
                print(
                    f"[全局精修-{mode}] 软化 EDT < {soft_target_min_edt:.2f} px "
                    f"的目标格 {int((soft_by_edt & target).sum())} 个 "
                    f"(允许小空隙/贴墙像素由周围圆近似覆盖)"
                )

        if int(soft_isolated_max_area_px) > 0:
            target_2d = np.zeros(free_full.shape, dtype=bool)
            target_2d[free_xy[:, 0], free_xy[:, 1]] = target
            cc_labels, n_cc = cc_label(target_2d)
            if n_cc > 0:
                sizes = np.bincount(cc_labels.ravel())
                sizes[0] = 0
                small = (sizes > 0) & (sizes < int(soft_isolated_max_area_px))
                small_mask_2d = small[cc_labels]
                soft_by_iso = small_mask_2d[free_xy[:, 0], free_xy[:, 1]]
                soft_mask = soft_mask | soft_by_iso
                if verbose:
                    print(
                        f"[全局精修-{mode}] 软化 < {soft_isolated_max_area_px} px "
                        f"的孤立目标连通域 {int(small.sum())} 块 "
                        f"({int((soft_by_iso & target).sum())} 像素)"
                    )

        target = target & ~soft_mask
        n_softened = n_before_soft - int(target.sum())
        if verbose and n_softened > 0:
            print(
                f"[全局精修-{mode}] 目标格 {n_before_soft} → {int(target.sum())} "
                f"(软化共 {n_softened}, 占 {100 * n_softened / max(n_before_soft, 1):.1f}%)"
            )

    init_list = [int(i) for i in initial_selected_idx if 0 <= int(i) < n_pool]

    if mode == "regreedy":
        tbp = None
        if greedy_edt_tiebreak:
            from scipy.ndimage import distance_transform_edt

            edt_full = distance_transform_edt(free_full.astype(bool)).astype(
                np.float64
            )
            h, w = free_full.shape
            yy = np.clip(candidates[:, 0], 0, h - 1)
            xx = np.clip(candidates[:, 1], 0, w - 1)
            tbp = edt_full[yy, xx]
        sel, n_cov = greedy_set_cover(
            coverage,
            target_mask=target,
            coverage_ratio=1.0,
            verbose=False,
            tiebreak_priority=tbp,
            overlap_tiebreak=overlap_tiebreak,
            nnz_tiebreak=greedy_nnz_tiebreak,
        )
        if verbose:
            print(
                f"[全局精修-regreedy] 整图重新贪心：圆数={len(sel)} "
                f"(覆盖 {n_cov} / {int(target.sum())})；接下来跑 ILS"
            )
        warm = list(sel)
    elif mode == "remove-only":
        warm = init_list
    elif mode in ("ils", "local"):
        warm = init_list
    else:
        raise ValueError(f"unknown mode={mode!r}")

    if mode == "remove-only":
        state = SolutionState.from_selected(coverage, target, candidates, warm)
        actually_covered = state.cover_count > 0
        state.target_mask = target & actually_covered
        total_removed = 0
        passes = 0
        for it in range(ils_max_iter):
            before = state.size
            rm = _try_remove_pass(state, verbose=False, freeze_mask=freeze_mask)
            total_removed += rm
            passes = it + 1
            if verbose:
                print(
                    f"[全局精修-remove-only] 第 {passes} 轮：删除 {rm} 个，"
                    f"当前圆盘数={state.size}"
                )
            if rm == 0 or state.size == before:
                break
        new_idx = state.selected_indices().astype(np.int64)
        stats = {
            "before": n_init,
            "after": int(new_idx.size),
            "removed": total_removed,
            "passes": passes,
            "mode": mode,
        }
    elif mode == "local":
        state = SolutionState.from_selected(coverage, target, candidates, warm)
        repaired = _greedy_repair(state, verbose=False)
        improved = local_search(
            state,
            r=r_px,
            enable_swap2=True,
            enable_swap3=enable_swap3,
            swap3_max_attempts=swap3_max_attempts,
            verbose=False,
            freeze_mask=freeze_mask,
        )
        new_idx = state.selected_indices().astype(np.int64)
        stats = {
            "before": n_init,
            "after": int(new_idx.size),
            "removed": int(n_init - new_idx.size),
            "passes": 1,
            "mode": mode,
            "repaired": int(repaired),
            "local_moves": int(improved),
        }
    else:
        new_sel, ils_stats = ils(
            coverage=coverage,
            target_mask=target,
            candidates_xy=candidates,
            initial_selected=warm,
            r=r_px,
            max_iter=ils_max_iter,
            patience=ils_patience,
            enable_swap3=enable_swap3,
            swap3_max_attempts=swap3_max_attempts,
            time_limit=time_limit,
            seed=seed,
            verbose=verbose,
            freeze_mask=freeze_mask,
        )
        new_idx = np.asarray(sorted(new_sel), dtype=np.int64)
        stats = {
            "before": n_init,
            "after": int(new_idx.size),
            "removed": int(n_init - new_idx.size),
            "passes": int(ils_stats.get("iterations", 0)),
            "mode": mode,
            "elapsed": float(ils_stats.get("elapsed", 0.0)),
            "history": ils_stats.get("history", []),
        }

    if verbose:
        print(
            f"[全局精修-{mode}] 完成：圆数 {n_init} → {int(new_idx.size)} "
            f"(净减少 {n_init - int(new_idx.size)} 个)"
        )

    return new_idx, stats


def _coord_key(pts: np.ndarray) -> np.ndarray:
    """把 (y, x) int32 坐标编成 int64 hash, 用于精确去重 / 查表."""
    return pts[:, 0].astype(np.int64) * (1 << 30) + pts[:, 1].astype(np.int64)


def plan_partitions(
    free_full: np.ndarray,
    labels: np.ndarray,
    r_px: float,
    base_config: PlannerConfig | None = None,
    region_ids: Iterable[int] | None = None,
    pad_px: int = 4,
    global_trim: bool = True,
    global_trim_mode: str = "local",
    trim_target_subsample: int = 1,
    trim_max_passes: int = 3,
    trim_ils_max_iter: int = 12,
    trim_ils_patience: int = 4,
    trim_enable_swap3: bool = True,
    trim_swap3_max_attempts: int | None = 30000,
    trim_time_limit: float | None = 180.0,
    trim_soft_target_factor: float = 0.0,
    trim_soft_isolated_max_area_px: int = 0,
    parallel_regions: bool = True,
    n_jobs: int | None = None,
    parallel_backend: str = "process",
    parallel_log: bool = True,
    verbose: bool = False,
    resolution_m_per_px: float | None = None,
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
        global_trim_mode: 全局后处理模式, 见 :func:`globally_optimize_solution`:

            - ``"local"`` (默认): 各分区候选并集 + try_remove + 局部 swap,
              不做扰动, 更保留六边形初始布局, 适合空旷区视觉效果.
            - ``"ils"``: 各分区候选并集 + 完整 ILS (try_remove + 2换1 +
              3换2 + 扰动). 更激进, 但可能打乱空旷区规则布局.
            - ``"regreedy"``: 在并集池上重新跑全图贪心覆盖, 再 ILS. 最激进.
            - ``"remove-only"``: 旧行为 (仅 try_remove, 已不推荐, 仅作对比).

        trim_target_subsample: 全局精修目标格步长. 默认 1 (全像素).
        trim_max_passes: ``remove-only`` 模式下的最大轮数; 也作为 ``ils`` 模式下
            ``ils_max_iter`` 的下限保护 (二者取最大).
        trim_ils_max_iter / trim_ils_patience / trim_enable_swap3 / trim_time_limit:
            ILS 行为参数, 默认即可. 大图慢的话可设 ``trim_time_limit=60.0``.
        trim_soft_target_factor: ``soft_target_min_edt = factor * r_px``. 见
            :func:`globally_optimize_solution`. 推荐 ``0.08 ~ 0.15``, 在杂物
            密集 (货架/桌椅) 场景能多减 10-30% 圆数. 0 = 关闭 (默认).
        trim_soft_isolated_max_area_px: 见 :func:`globally_optimize_solution`.
            通常 ``< (0.3 * r_px) ** 2`` 较稳, 默认 0 关闭.
        resolution_m_per_px: 米/像素，与 ROS 地图 ``yaml`` 中 ``resolution`` 一致；
            写入 ``PlannerConfig.resolution_m_per_px`` 以启用走廊 **>strip 米宽** 时的
            划条分界采样。不传则走廊仅走单脊线中轴（与旧版一致）。

    Notes:
        实现给每个区独立调 :func:`plan_coverage`, visibility 在子图内运算, 然后在
        全局后处理阶段把所有候选并到一起在整图 visibility 上做 ILS, 因此跨分区
        冗余圆基本能被精确削掉.

        parallel_regions / n_jobs / parallel_backend:
            分区级并行默认开启（``parallel_regions=True``、``parallel_backend="process"``）；
            仅当有效分区数 ≥ 2 时才会并行，单分区仍为串行。纯 NumPy/CPU 密集场景下
            ``thread`` 常受 GIL 限制，多核利用率低；需要可改 ``parallel_backend="thread"``。
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

    if resolution_m_per_px is not None and float(resolution_m_per_px) > 0:
        base_config = replace(
            base_config, resolution_m_per_px=float(resolution_m_per_px)
        )

    if region_ids is None:
        region_ids = sorted(int(k) for k in np.unique(labels) if k > 0)
    else:
        region_ids = sorted(int(k) for k in region_ids)

    h, w = free_full.shape
    n_regions = sum(
        1 for rid in region_ids if ((labels == rid) & free_full).any()
    )
    out: list[RegionPlan] = []
    pool_pts_per_region: list[np.ndarray] = []
    pool_rid_per_region: list[np.ndarray] = []

    # 并行：默认开启 process 后端以吃满多核；纯 Python/NumPy 重计算下 thread 受 GIL 限制。
    if parallel_regions and n_regions >= 2:
        from concurrent.futures import (
            ProcessPoolExecutor,
            ThreadPoolExecutor,
            as_completed,
        )

        backend = str(parallel_backend).strip().lower()
        if backend not in ("thread", "process"):
            raise ValueError(
                f"parallel_backend must be 'thread' or 'process', got {parallel_backend!r}"
            )
        max_workers = int(n_jobs) if n_jobs is not None else None
        if verbose:
            print(
                f"[partition] ×{n_regions} {backend} "
                f"w={max_workers or 'auto'}"
            )
        if verbose and backend == "thread":
            print(
                "[partition] thread 常为 GIL 所限；需多核请用 process / RBCV_PAR_BACKEND。"
            )

        futures = {}
        Executor = ThreadPoolExecutor if backend == "thread" else ProcessPoolExecutor
        with Executor(max_workers=max_workers) as ex:
            for rid in region_ids:
                # 进程间传参会拷贝数组；但区域数量通常较少，此处换取明显墙钟时间收益。
                fut = ex.submit(
                    _plan_one_region_task,
                    free_full,
                    labels,
                    int(rid),
                    float(r_px),
                    base_config,
                    int(pad_px),
                    bool(verbose),
                    bool(verbose and parallel_log),
                )
                futures[fut] = int(rid)

            done_i = 0
            for fut in as_completed(futures):
                done_i += 1
                rp, pool_pts, pool_rids, log_text = fut.result()
                if verbose and not parallel_log:
                    print(f"[p] {done_i}/{n_regions} id={futures[fut]}")
                if verbose and parallel_log and log_text.strip():
                    rid_done = futures[fut]
                    print(f"[p] r{rid_done}\n{log_text.rstrip()}")
                if rp is not None:
                    out.append(rp)
                if pool_pts.shape[0] > 0:
                    pool_pts_per_region.append(pool_pts)
                    pool_rid_per_region.append(pool_rids)

        out.sort(key=lambda x: x.region_id)
    else:
        progress_i = 0
        for rid in region_ids:
            region_mask_full = (labels == rid) & free_full
            if not region_mask_full.any():
                continue
            progress_i += 1
            if verbose:
                print(f"[p] {progress_i}/{n_regions} rid={rid}")
            rp, pool_pts, pool_rids, _log = _plan_one_region_task(
                free_full=free_full,
                labels=labels,
                rid=int(rid),
                r_px=float(r_px),
                base_config=base_config,
                pad_px=int(pad_px),
                verbose=bool(verbose),
                capture_stdout=False,
            )
            if rp is not None:
                out.append(rp)
            if pool_pts.shape[0] > 0:
                pool_pts_per_region.append(pool_pts)
                pool_rid_per_region.append(pool_rids)

    partition = PartitionResult(region_plans=out)

    rid_to_cfg = {rp.region_id: rp.config for rp in out}
    edt_for_trim: np.ndarray | None = None
    if (
        global_trim
        and partition.raw_total_circles >= 2
        and base_config.global_trim_protect_hex_core
        and _any_hex_first_core_freeze_enabled(out)
    ):
        from scipy.ndimage import distance_transform_edt

        edt_for_trim = distance_transform_edt(free_full.astype(bool))

    if global_trim and partition.raw_total_circles >= 2:
        merged_pts = partition._concat_region_points()
        merged_ids = []
        for rp in out:
            n = rp.result.selected_points.shape[0]
            if n > 0:
                merged_ids.extend([rp.region_id] * n)
        merged_ids_arr = np.asarray(merged_ids, dtype=np.int64)

        freeze_merged = None
        if edt_for_trim is not None:
            freeze_merged = _hex_first_global_freeze_mask(
                merged_pts,
                merged_ids_arr,
                r_px,
                edt_for_trim,
                rid_to_cfg,
            )

        sub = trim_target_subsample
        region_free = free_full & (labels > 0)

        if global_trim_mode == "remove-only":
            if verbose:
                print(
                    f"[全局精修] 模式=remove-only：合并各分区共 "
                    f"{merged_pts.shape[0]} 个圆，准备做整图删圆精修。"
                )
            new_pts, keep_idx, stats = globally_trim_solution(
                free_full=free_full,
                selected_points=merged_pts,
                r_px=r_px,
                target_mask_full=region_free,
                target_subsample=sub,
                max_passes=trim_max_passes,
                verbose=verbose,
                freeze_mask=freeze_merged,
            )
            partition.trimmed_points = new_pts.astype(np.int32)
            partition.trimmed_region_ids = merged_ids_arr[keep_idx]
            partition.trim_stats = stats
            return partition

        # ----- ILS / regreedy 模式: 用各分区候选并集做完整 ILS -----
        if pool_pts_per_region:
            pool_raw = np.vstack(pool_pts_per_region).astype(np.int32)
            pool_rid_raw = np.concatenate(pool_rid_per_region).astype(np.int64)
        else:
            pool_raw = merged_pts.astype(np.int32)
            pool_rid_raw = merged_ids_arr.copy()

        # 整图坐标精确去重 (同位置不同分区视作同一候选, 取首次出现的 rid).
        keys = _coord_key(pool_raw)
        _, first_idx = np.unique(keys, return_index=True)
        first_idx = np.sort(first_idx)
        pool_pts = pool_raw[first_idx]
        pool_rids = pool_rid_raw[first_idx]
        pool_keys = _coord_key(pool_pts)
        pool_lookup = {int(k): int(i) for i, k in enumerate(pool_keys)}

        sel_keys = _coord_key(merged_pts)
        init_idx = np.array(
            [pool_lookup.get(int(k), -1) for k in sel_keys], dtype=np.int64
        )
        missing_mask = init_idx < 0
        if missing_mask.any():
            extra = merged_pts[missing_mask].astype(np.int32)
            extra_rids = merged_ids_arr[missing_mask]
            n_old = pool_pts.shape[0]
            pool_pts = np.vstack([pool_pts, extra]).astype(np.int32)
            pool_rids = np.concatenate([pool_rids, extra_rids]).astype(np.int64)
            new_keys = _coord_key(extra)
            for i, k in enumerate(new_keys):
                pool_lookup[int(k)] = n_old + i
            missing_idx = np.where(missing_mask)[0]
            for j, mi in enumerate(missing_idx):
                init_idx[mi] = n_old + j

        freeze_pool = None
        if edt_for_trim is not None:
            freeze_pool = _hex_first_global_freeze_mask(
                pool_pts, pool_rids, r_px, edt_for_trim, rid_to_cfg
            )

        if verbose:
            print(
                f"[全局精修] 模式={global_trim_mode}：合并各分区共 "
                f"{merged_pts.shape[0]} 个已选圆 + 候选并集池 "
                f"{pool_pts.shape[0]} 个，准备做整图 ILS 精修。"
            )

        ils_iter = max(int(trim_max_passes), int(trim_ils_max_iter))
        soft_min_edt = float(trim_soft_target_factor) * float(r_px)
        new_idx, stats = globally_optimize_solution(
            free_full=free_full,
            candidates=pool_pts,
            initial_selected_idx=init_idx,
            r_px=r_px,
            target_mask_full=region_free,
            target_subsample=sub,
            mode=global_trim_mode,
            ils_max_iter=ils_iter,
            ils_patience=trim_ils_patience,
            enable_swap3=trim_enable_swap3,
            swap3_max_attempts=trim_swap3_max_attempts,
            time_limit=trim_time_limit,
            soft_target_min_edt=soft_min_edt,
            soft_isolated_max_area_px=int(trim_soft_isolated_max_area_px),
            seed=base_config.seed,
            verbose=verbose,
            greedy_edt_tiebreak=base_config.greedy_edt_tiebreak,
            freeze_mask=freeze_pool,
            merge_identical_pool=base_config.merge_identical_coverage_candidates,
            overlap_tiebreak=base_config.overlap_tiebreak,
            greedy_nnz_tiebreak=base_config.greedy_nnz_tiebreak,
        )
        partition.trimmed_points = pool_pts[new_idx].astype(np.int32)
        partition.trimmed_region_ids = pool_rids[new_idx]
        partition.trim_stats = stats

    return partition
