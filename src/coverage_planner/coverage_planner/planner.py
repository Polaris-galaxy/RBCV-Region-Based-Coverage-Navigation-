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

from .candidates import generate_candidates
from .map_io import GridMap, preprocess
from .refine import ils
from .set_cover import find_unreachable_targets, greedy_set_cover
from .visibility import augment_for_feasibility, build_coverage


@dataclass
class PlannerConfig:
    r: float = 32.0
    alpha: float = 0.9
    use_medial: bool = True
    use_hex: bool = True
    use_reflex: bool = True
    random_extra: int = 0
    n_rays: int | None = None
    coverage_ratio: float = 1.0
    target_subsample: int = 1
    min_corridor: float = 1.0
    augment_for_full_cover: bool = True
    overlap_tiebreak: bool = True
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

    selected, n_covered = greedy_set_cover(
        coverage,
        target_mask=target_mask,
        coverage_ratio=config.coverage_ratio,
        overlap_tiebreak=config.overlap_tiebreak,
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
