"""顶层 DGR (Divide-Greedy-Refine) 流水线.

阶段:
    1. 预处理: 距离场 + 中轴.
    2. 候选点生成 (中轴 + 六边形 + 反射顶点).
    3. 覆盖矩阵构建 (栅格可见性).
    4. 候选完备性补全 (保证全覆盖在理论上可行).
    5. 贪心 Set Cover (Johnson 1974).
    6. ILS 局部搜索精修 (try_remove + swap_2_for_1 + 扰动).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .candidates import (
    _dedup,
    generate_candidates,
    hex_candidates,
    medial_candidates,
    random_candidates,
    reflex_candidates,
)
from .map_io import GridMap, preprocess
import time as _time

from .refine import SolutionState, _greedy_repair, ils, local_search
from .set_cover import find_unreachable_targets, greedy_set_cover
from .visibility import augment_for_feasibility, build_coverage


@dataclass
class PlannerConfig:
    r: float = 32.0
    alpha: float = 0.9
    use_medial: bool = True
    use_hex: bool = True
    use_reflex: bool = True
    # 新流水线：先全片六边形铺设 → 残余区域再用中轴/反射圆补齐。开启后忽略 use_medial/use_hex/use_reflex 的预先组合，
    # 改由"是否覆盖到"决定哪里走中轴、哪里走反射；单分区内可见性只算一次合并候选池。
    use_hex_first: bool = False
    # hex_first 残余分析时，在残余像素 mask 周围膨胀 hex_first_search_factor*r 像素得到补圆候选搜索区。
    # 1.5r 较 1.0r 更能把"靠墙/凹角的反射顶点候选"也带入池，避免开阔分区的角落欠覆盖。
    hex_first_search_factor: float = 1.5
    # hex_first 第一阶段六边形候选的 min_room_radius_factor*r；略高一点可减少
    # 「大厅靠边一圈」对 medial/reflex 的依赖，视觉上更均匀（略增耗时）。
    hex_first_room_radius_factor: float = 0.55
    # hex_first 区内精修：默认 **仅 local_search（无扰动）**，保留六边形格局；
    # 设为 True 则走完整 ILS（含扰动），圆数可能更少但空旷区布局易被打乱。
    hex_first_use_full_ils: bool = False
    # 区内分流（近似「开阔核 + 狭窄/靠边带」）:
    # 若为非 None，只保留 EDT <= medial_max_edt_factor * r 的中轴候选，削弱大厅中心冗余中轴。
    medial_max_edt_factor: float | None = None
    # 若为非 None，六边形仅此 EDT 阈值以上放点：min_room_radius = factor * r（默认 Hex 内部 0.5r）。
    hex_min_room_radius_factor: float | None = None
    random_extra: int = 0
    n_rays: int | None = None
    coverage_ratio: float = 1.0
    target_subsample: int = 1
    min_corridor: float = 1.0
    augment_for_full_cover: bool = True
    overlap_tiebreak: bool = True
    # 贪心 set cover 第三级 tie-break：圆心 EDT 大者优先（空旷核优先、缓解开阔区靠边扎堆）
    greedy_edt_tiebreak: bool = True
    enable_ils: bool = True
    ils_max_iter: int = 30
    ils_patience: int = 6
    ils_perturb_min: float = 0.05
    ils_perturb_max: float = 0.20
    ils_enable_swap3: bool = True
    ils_swap3_max_attempts: int | None = 100000
    ils_block_perturb_every: int = 3
    ils_block_factor: float = 5.0
    ils_time_limit: float | None = None
    verbose: bool = False
    seed: int = 0
    extras: dict = field(default_factory=dict)


@dataclass
class PlannerResult:
    gmap: GridMap
    candidates: np.ndarray
    selected_indices: list[int]
    selected_points: np.ndarray
    coverage_ratio: float
    target_count: int
    target_xy: np.ndarray
    greedy_size: int = 0
    ils_size: int = 0
    ils_stats: dict | None = None


def _subsample_targets(
    free_xy: np.ndarray, gmap_shape: tuple[int, int], stride: int
) -> np.ndarray:
    if stride <= 1:
        return np.ones(free_xy.shape[0], dtype=bool)
    h, w = gmap_shape
    keep_grid = np.zeros((h // stride + 1, w // stride + 1), dtype=bool)
    bys = free_xy[:, 0] // stride
    bxs = free_xy[:, 1] // stride
    mask = np.zeros(free_xy.shape[0], dtype=bool)
    for k, (by, bx) in enumerate(zip(bys, bxs)):
        if not keep_grid[by, bx]:
            keep_grid[by, bx] = True
            mask[k] = True
    return mask


def plan_coverage(
    free_or_gmap: np.ndarray | GridMap,
    config: PlannerConfig,
) -> PlannerResult:
    """端到端流水线."""
    if isinstance(free_or_gmap, GridMap):
        gmap = free_or_gmap
    else:
        gmap = preprocess(free_or_gmap, min_corridor=config.min_corridor)

    if config.use_hex_first:
        return plan_coverage_hex_first(gmap, config)

    if config.verbose:
        print(
            f"[规划] 栅格尺寸={gmap.shape} 可走格数={gmap.n_free} 半径r={config.r}"
        )

    candidates = generate_candidates(
        gmap,
        r=config.r,
        alpha=config.alpha,
        use_medial=config.use_medial,
        use_hex=config.use_hex,
        use_reflex=config.use_reflex,
        medial_max_edt_factor=config.medial_max_edt_factor,
        hex_min_room_radius_factor=config.hex_min_room_radius_factor,
        random_extra=config.random_extra,
        random_seed=config.seed,
    )
    if config.verbose:
        print(f"[规划] 候选点数量: {candidates.shape[0]}")

    if config.verbose:
        print("[规划] 正在构建覆盖矩阵（候选数较多时可能较耗时）…")

    coverage, _free_idx, free_xy = build_coverage(
        gmap, candidates, r=config.r, n_rays=config.n_rays
    )
    if config.verbose:
        print(
            f"[规划] 覆盖矩阵: 形状={coverage.shape} 非零元={coverage.nnz}"
        )

    target_mask = _subsample_targets(
        free_xy, gmap.shape, config.target_subsample
    )
    if config.verbose:
        print(
            f"[规划] 目标点数 |U|（采样步长={config.target_subsample}）"
            f"={int(target_mask.sum())}"
        )

    if config.augment_for_full_cover:
        unreach = find_unreachable_targets(coverage, target_mask)
        if unreach.size > 0:
            if config.verbose:
                print(f"[规划] {unreach.size} 个目标不可达，正在补全候选…")
            candidates, coverage = augment_for_feasibility(
                gmap, candidates, coverage, free_xy, target_mask,
                r=config.r, n_rays=config.n_rays, verbose=config.verbose,
            )
            if config.verbose:
                print(
                    f"[规划] 补全后候选数: {candidates.shape[0]}，"
                    f"覆盖矩阵非零元={coverage.nnz}"
                )

        residual = find_unreachable_targets(coverage, target_mask)
        if residual.size > 0:
            target_mask = target_mask.copy()
            target_mask[residual] = False
            if config.verbose:
                print(
                    f"[规划] 剔除 {residual.size} 个残余硬不可达目标"
                    f"（多为栅格噪声，占可走格"
                    f"{100*residual.size/max(1, gmap.n_free):.4f}%）"
                )

    tbp = None
    if config.greedy_edt_tiebreak:
        tbp = gmap.edt[candidates[:, 0], candidates[:, 1]].astype(
            np.float64, copy=False
        )

    selected, n_covered = greedy_set_cover(
        coverage,
        target_mask=target_mask,
        coverage_ratio=config.coverage_ratio,
        overlap_tiebreak=config.overlap_tiebreak,
        tiebreak_priority=tbp,
        verbose=config.verbose,
    )
    target_total = int(target_mask.sum())
    greedy_ratio = n_covered / max(target_total, 1)
    greedy_size = len(selected)
    if config.verbose:
        print(
            f"[规划] 贪心：圆盘数={greedy_size}，"
            f"已覆盖={n_covered}/{target_total} "
            f"（{100*greedy_ratio:.2f}%）"
        )

    ils_stats = None
    ils_size = greedy_size
    if config.enable_ils and selected:
        if config.verbose:
            print("[规划] 正在执行迭代局部搜索（ILS）…")
        selected, ils_stats = ils(
            coverage=coverage,
            target_mask=target_mask,
            candidates_xy=candidates,
            initial_selected=selected,
            r=config.r,
            max_iter=config.ils_max_iter,
            patience=config.ils_patience,
            perturb_min=config.ils_perturb_min,
            perturb_max=config.ils_perturb_max,
            enable_swap3=config.ils_enable_swap3,
            swap3_max_attempts=config.ils_swap3_max_attempts,
            block_perturb_every=config.ils_block_perturb_every,
            block_factor=config.ils_block_factor,
            time_limit=config.ils_time_limit,
            seed=config.seed,
            verbose=config.verbose,
        )
        ils_size = len(selected)
        if config.verbose:
            print(
                f"[规划] 迭代局部搜索：圆盘数={ils_size}（贪心初解 {greedy_size}，"
                f"减少 {greedy_size-ils_size}，"
                f"降幅 {100*(greedy_size-ils_size)/max(greedy_size,1):.1f}%），"
                f"耗时 {ils_stats['elapsed']:.1f} 秒"
            )

    final_cover_mask = np.zeros(coverage.shape[1], dtype=bool)
    indptr = coverage.indptr
    indices = coverage.indices
    for c in selected:
        cols = indices[indptr[c]:indptr[c + 1]]
        final_cover_mask[cols] = True
    final_covered = int((final_cover_mask & target_mask).sum())
    final_ratio = final_covered / max(target_total, 1)

    selected_pts = (
        candidates[selected] if selected else np.zeros((0, 2), dtype=np.int32)
    )

    return PlannerResult(
        gmap=gmap,
        candidates=candidates,
        selected_indices=list(selected),
        selected_points=selected_pts,
        coverage_ratio=final_ratio,
        target_count=target_total,
        target_xy=free_xy[target_mask],
        greedy_size=greedy_size,
        ils_size=ils_size,
        ils_stats=ils_stats,
    )


def plan_coverage_hex_first(
    gmap: GridMap,
    config: PlannerConfig,
) -> PlannerResult:
    """两阶段流水线：**先六边形铺满，再针对未覆盖的残余区域补圆**。

    与 :func:`plan_coverage` 的区别：
        1. 不预先按"形状"决定候选组合；
        2. **阶段 A** 单独跑一次「纯六边形 → 贪心」得到一个粗解 (大空间区
           域基本由这一步覆盖)；
        3. **阶段 B** 计算尚未被阶段 A 覆盖到的残余目标 mask, 仅在该 mask
           周围 ``hex_first_search_factor * r`` 像素内追加 **中轴**(窄过道)
           与 **反射顶点**(墙边/凹角) 候选；大厅中心因此不会被中轴/反射候选
           污染, ILS 决策空间更小。
        4. 阶段 C 合并候选, 重建 visibility; 以**六边形贪心初解**为种子做修补,
           再默认用 **local_search（无扰动）** 删圆/替换；可选完整 ILS。

    几何/工程理由:
        - 在开阔区域六边形是已知最少圆覆盖；
        - 在窄过道/墙边附近, 中轴与反射顶点能精确补漏；
        - 「先做后判」保留了原 `mixed` 模式中 EDT 阈值过滤的最大优点
          (开阔核仅靠 hex), 同时 **省去手工 EDT 阈值调参**。

    Args:
        gmap: 已经 ``preprocess`` 的 GridMap (region 子图)。
        config: 复用 :class:`PlannerConfig` 中的 r, alpha, ils_*, target_subsample
            等字段；本函数仅额外读取 ``hex_first_room_radius_factor`` 与
            ``hex_first_search_factor``。

    Returns:
        :class:`PlannerResult` (与 ``plan_coverage`` 同结构)。
    """
    from scipy.ndimage import binary_dilation

    r = config.r

    # ===== 阶段 A: 纯六边形候选 → 临时 build_coverage → greedy =====
    hex_min_rr = float(config.hex_first_room_radius_factor) * float(r)
    hex_pts = hex_candidates(gmap, r, min_room_radius=hex_min_rr)

    if hex_pts.shape[0] == 0:
        if config.verbose:
            print("[hex_first] 阶段A 无六边形候选 → 回退默认流水线")
        return plan_coverage(gmap, replace_config_use_hex_first(config, False))

    coverage_hex, _free_idx, free_xy = build_coverage(
        gmap, hex_pts, r=r, n_rays=config.n_rays
    )
    target_mask_full = _subsample_targets(
        free_xy, gmap.shape, config.target_subsample
    )
    tbp_hex = None
    if config.greedy_edt_tiebreak:
        tbp_hex = gmap.edt[hex_pts[:, 0], hex_pts[:, 1]].astype(
            np.float64, copy=False
        )
    sel_hex, _ = greedy_set_cover(
        coverage_hex,
        target_mask=target_mask_full,
        coverage_ratio=config.coverage_ratio,
        overlap_tiebreak=config.overlap_tiebreak,
        tiebreak_priority=tbp_hex,
        verbose=False,
    )

    cov_mask_hex = np.zeros(coverage_hex.shape[1], dtype=bool)
    indptr_h = coverage_hex.indptr
    indices_h = coverage_hex.indices
    for c in sel_hex:
        cov_mask_hex[indices_h[indptr_h[c]:indptr_h[c + 1]]] = True
    uncov_in_target = target_mask_full & ~cov_mask_hex
    n_uncov = int(uncov_in_target.sum())
    n_target_total = int(target_mask_full.sum())

    # ===== 阶段 B: 残余区域分析 → 中轴 / 反射候选 =====
    extra_parts: list[np.ndarray] = []
    n_med = n_ref = 0
    if n_uncov > 0:
        uncov_2d = np.zeros(gmap.shape, dtype=bool)
        sel_yx = free_xy[uncov_in_target]
        if sel_yx.shape[0] > 0:
            uncov_2d[sel_yx[:, 0], sel_yx[:, 1]] = True
        rad = max(2, int(round(float(config.hex_first_search_factor) * r)))
        struct = np.ones((rad * 2 + 1, rad * 2 + 1), dtype=bool)
        search_region = binary_dilation(uncov_2d, structure=struct) & gmap.free

        if gmap.skeleton.any():
            med_full = medial_candidates(gmap, r, alpha=config.alpha)
            if med_full.shape[0] > 0:
                keep_med = search_region[med_full[:, 0], med_full[:, 1]]
                med_in = med_full[keep_med]
                n_med = int(med_in.shape[0])
                if n_med > 0:
                    extra_parts.append(med_in)

        ref_full = reflex_candidates(gmap, r)
        if ref_full.shape[0] > 0:
            keep_ref = search_region[ref_full[:, 0], ref_full[:, 1]]
            ref_in = ref_full[keep_ref]
            n_ref = int(ref_in.shape[0])
            if n_ref > 0:
                extra_parts.append(ref_in)

    if config.random_extra > 0:
        rand_pts = random_candidates(
            gmap, config.random_extra,
            seed=config.seed, min_clearance=0.3 * r,
        )
        if rand_pts.shape[0] > 0:
            extra_parts.append(rand_pts)

    # ===== 阶段 C: 合并候选 → 完整 build_coverage → greedy + ILS =====
    parts = [hex_pts] + extra_parts
    candidates = np.concatenate(
        [p for p in parts if p.shape[0] > 0], axis=0
    ).astype(np.int32)
    candidates = _dedup(candidates, 0.45 * r)

    if config.verbose:
        uncov_pct = 100.0 * n_uncov / max(n_target_total, 1)
        print(
            f"[hex_first] A: hex {hex_pts.shape[0]}→选 {len(sel_hex)} "
            f"(残余 {uncov_pct:.1f}%)；"
            f"B: medial+reflex 补 {n_med + n_ref}；"
            f"C: 候选 {candidates.shape[0]}"
        )

    coverage, _free_idx, free_xy = build_coverage(
        gmap, candidates, r=r, n_rays=config.n_rays
    )
    target_mask = _subsample_targets(
        free_xy, gmap.shape, config.target_subsample
    )

    if config.augment_for_full_cover:
        unreach = find_unreachable_targets(coverage, target_mask)
        if unreach.size > 0:
            candidates, coverage = augment_for_feasibility(
                gmap, candidates, coverage, free_xy, target_mask,
                r=r, n_rays=config.n_rays, verbose=False,
            )
        residual = find_unreachable_targets(coverage, target_mask)
        if residual.size > 0:
            target_mask = target_mask.copy()
            target_mask[residual] = False

    hex_selected_pts = hex_pts[sel_hex] if sel_hex else np.zeros((0, 2), dtype=np.int32)
    cand_lookup = {
        (int(y), int(x)): int(i)
        for i, (y, x) in enumerate(candidates.tolist())
    }
    initial_selected = [
        cand_lookup[(int(y), int(x))]
        for y, x in hex_selected_pts.tolist()
        if (int(y), int(x)) in cand_lookup
    ]

    state = SolutionState.from_selected(
        coverage, target_mask, candidates, initial_selected
    )
    repaired = _greedy_repair(state, verbose=False)
    selected = state.selected_indices().tolist()
    n_covered = int(((state.cover_count > 0) & target_mask).sum())
    target_total = int(target_mask.sum())
    greedy_size = len(selected)

    ils_stats: dict | None = None
    ils_size = greedy_size
    if config.enable_ils and selected:
        if config.hex_first_use_full_ils:
            selected, ils_stats = ils(
                coverage=coverage,
                target_mask=target_mask,
                candidates_xy=candidates,
                initial_selected=selected,
                r=r,
                max_iter=config.ils_max_iter,
                patience=config.ils_patience,
                perturb_min=config.ils_perturb_min,
                perturb_max=config.ils_perturb_max,
                enable_swap3=config.ils_enable_swap3,
                swap3_max_attempts=config.ils_swap3_max_attempts,
                block_perturb_every=config.ils_block_perturb_every,
                block_factor=config.ils_block_factor,
                time_limit=config.ils_time_limit,
                seed=config.seed,
                verbose=False,
            )
            ils_size = len(selected)
        else:
            t_ls0 = _time.time()
            st = SolutionState.from_selected(
                coverage, target_mask, candidates, list(selected)
            )
            local_search(
                st,
                r=r,
                enable_swap2=True,
                enable_swap3=config.ils_enable_swap3,
                max_passes=min(int(config.ils_max_iter), 28),
                swap3_max_attempts=config.ils_swap3_max_attempts,
                verbose=False,
            )
            selected = st.selected_indices().tolist()
            ils_size = len(selected)
            ils_stats = {
                "elapsed": _time.time() - t_ls0,
                "mode": "local_search",
                "iterations": 1,
            }

    if config.verbose:
        ref = (
            "full_ils"
            if config.hex_first_use_full_ils
            else "local_search"
        )
        print(
            f"[hex_first] hex初解={len(initial_selected)} + 修补={repaired} "
            f"→ 局部初解={greedy_size} → {ref}={ils_size}"
            + (
                f" ({ils_stats['elapsed']:.1f}s)"
                if ils_stats is not None else ""
            )
        )

    final_cover_mask = np.zeros(coverage.shape[1], dtype=bool)
    indptr = coverage.indptr
    indices = coverage.indices
    for c in selected:
        cols = indices[indptr[c]:indptr[c + 1]]
        final_cover_mask[cols] = True
    final_covered = int((final_cover_mask & target_mask).sum())
    final_ratio = final_covered / max(target_total, 1)

    selected_pts = (
        candidates[selected] if selected else np.zeros((0, 2), dtype=np.int32)
    )

    return PlannerResult(
        gmap=gmap,
        candidates=candidates,
        selected_indices=list(selected),
        selected_points=selected_pts,
        coverage_ratio=final_ratio,
        target_count=target_total,
        target_xy=free_xy[target_mask],
        greedy_size=greedy_size,
        ils_size=ils_size,
        ils_stats=ils_stats,
    )


def replace_config_use_hex_first(cfg: PlannerConfig, val: bool) -> PlannerConfig:
    """轻量重置 ``use_hex_first`` 字段，用于回退到默认流水线避免无限递归。"""
    from dataclasses import replace as _dc_replace
    return _dc_replace(cfg, use_hex_first=val)
