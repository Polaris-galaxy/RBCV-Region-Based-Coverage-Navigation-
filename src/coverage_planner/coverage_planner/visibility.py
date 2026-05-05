"""栅格可见性 + 圆盘覆盖矩阵。

对每个候选点 ``c``, 计算其有效观测区
    S(c) = Disk(c, r) ∩ Visible(c)
其中 ``Visible(c) = {x ∈ Free : 线段 cx ⊂ Free}``。

实现方式: 极坐标 ray-marching。从 ``c`` 出发, 在角度 ``θ ∈ [0, 2π)``
上以分辨率 ``Δθ ≈ 1/r`` 撒射线, 沿射线步进, 命中障碍即停止;
途中所经像素均为可见。

输出的 "覆盖" 用 **稀疏布尔矩阵** ``A[ |C| , |U| ]`` 表示, 其中:
    - ``U`` 是待覆盖元素集合 (此实现里 ``U = Free pixels``);
    - ``A[i, j] = True`` 表示候选点 i 能观测到自由像素 j。
"""
from __future__ import annotations

import math

import numpy as np
from scipy.sparse import csr_matrix, vstack as sp_vstack

from .map_io import GridMap


def _visible_disk(
    free: np.ndarray,
    cy: int,
    cx: int,
    r: float,
    n_rays: int,
) -> np.ndarray:
    """从 ``(cy, cx)`` 出发计算半径 ``r`` 内的可见像素布尔图 (局部裁剪)。

    返回 ``[H, W]`` bool, 与 ``free`` 等大;
    ``True`` 仅出现在以 ``(cy, cx)`` 为中心、半径 ``r`` 的方框内。
    """
    h, w = free.shape
    vis = np.zeros((h, w), dtype=bool)

    if not (0 <= cy < h and 0 <= cx < w and free[cy, cx]):
        return vis

    angles = np.linspace(0.0, 2.0 * math.pi, n_rays, endpoint=False)
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    n_steps = int(math.ceil(r)) + 1
    ts = np.arange(n_steps + 1, dtype=np.float32)
    ts = np.minimum(ts, r)

    ys_all = cy + np.outer(ts, sin_a)
    xs_all = cx + np.outer(ts, cos_a)

    yi_all = np.round(ys_all).astype(np.int32)
    xi_all = np.round(xs_all).astype(np.int32)

    in_bounds = (yi_all >= 0) & (yi_all < h) & (xi_all >= 0) & (xi_all < w)

    blocked = np.zeros(n_rays, dtype=bool)
    for k in range(n_steps + 1):
        yk = yi_all[k]
        xk = xi_all[k]
        ib = in_bounds[k]
        active = (~blocked) & ib
        if not active.any():
            break
        idx = np.where(active)[0]
        ys = yk[idx]
        xs = xk[idx]
        is_obs = ~free[ys, xs]
        vis[ys[~is_obs], xs[~is_obs]] = True
        blocked[idx[is_obs]] = True

    return vis


def build_coverage(
    gmap: GridMap,
    candidates: np.ndarray,
    r: float,
    n_rays: int | None = None,
) -> tuple[csr_matrix, np.ndarray, np.ndarray]:
    """构建覆盖关系稀疏矩阵。

    Args:
        gmap: 预处理地图。
        candidates: ``[N, 2]`` 候选点 (row, col)。
        r: 观测半径 (像素)。
        n_rays: 极坐标扫描射线数, 默认 ``ceil(2*pi*r)``。

    Returns:
        coverage: ``csr_matrix[N, M]``, 行=候选点, 列=自由像素索引;
                  ``True`` 表示可见。
        free_index: ``[H, W]`` int32, 自由像素的全局编号
                    (障碍处为 -1)。
        free_xy: ``[M, 2]`` int32, 第 j 个自由像素的 (row, col)。
    """
    h, w = gmap.shape
    free = gmap.free
    if n_rays is None:
        n_rays = max(64, int(math.ceil(2.0 * math.pi * r)))

    free_flat_idx = -np.ones((h, w), dtype=np.int64)
    ys, xs = np.where(free)
    free_flat_idx[ys, xs] = np.arange(ys.size, dtype=np.int64)
    free_xy = np.stack([ys, xs], axis=1).astype(np.int32)
    n_free = ys.size

    indptr: list[int] = [0]
    indices: list[np.ndarray] = []

    for cy, cx in candidates:
        cy = int(cy)
        cx = int(cx)
        vis = _visible_disk(free, cy, cx, r, n_rays)
        if not vis.any():
            indptr.append(indptr[-1])
            continue
        rr = max(0, cy - int(math.ceil(r)))
        cc = max(0, cx - int(math.ceil(r)))
        re = min(h, cy + int(math.ceil(r)) + 1)
        ce = min(w, cx + int(math.ceil(r)) + 1)
        local_vis = vis[rr:re, cc:ce]
        local_idx = free_flat_idx[rr:re, cc:ce]
        sel = local_vis & (local_idx >= 0)
        cols = local_idx[sel].astype(np.int64)
        cols.sort()
        indices.append(cols)
        indptr.append(indptr[-1] + cols.size)

    if indices:
        flat_indices = np.concatenate(indices)
    else:
        flat_indices = np.zeros((0,), dtype=np.int64)
    data = np.ones(flat_indices.size, dtype=bool)
    coverage = csr_matrix(
        (data, flat_indices, np.asarray(indptr, dtype=np.int64)),
        shape=(candidates.shape[0], n_free),
    )
    return coverage, free_flat_idx, free_xy


def _dedup_points(pts: np.ndarray, dedup_radius: float) -> np.ndarray:
    if pts.shape[0] <= 1 or dedup_radius <= 0:
        return pts
    from scipy.spatial import cKDTree

    tree = cKDTree(pts)
    keep = np.ones(pts.shape[0], dtype=bool)
    for i in range(pts.shape[0]):
        if not keep[i]:
            continue
        for j in tree.query_ball_point(pts[i], r=dedup_radius):
            if j > i:
                keep[j] = False
    return pts[keep]


def _push_into_free(
    pts: np.ndarray,
    edt: np.ndarray,
    free: np.ndarray,
    target_edt: float,
    max_walk: int = 8,
) -> np.ndarray:
    """对每个候选 ``p``, 沿 edt 梯度向自由空间深处微移到 ``edt(p) >= target``.

    用 8 邻域局部最大爬山, 最多 ``max_walk`` 步. 候选若已经满足 (或四
    周都不更深), 保持原位.
    """
    h, w = edt.shape
    out = pts.copy()
    for k in range(out.shape[0]):
        py, px = int(out[k, 0]), int(out[k, 1])
        if not free[py, px]:
            continue
        for _ in range(max_walk):
            if edt[py, px] >= target_edt:
                break
            best_e = edt[py, px]
            best_y, best_x = py, px
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    ny, nx = py + dy, px + dx
                    if 0 <= ny < h and 0 <= nx < w and free[ny, nx]:
                        e = edt[ny, nx]
                        if e > best_e:
                            best_e = e
                            best_y, best_x = ny, nx
            if best_y == py and best_x == px:
                break
            py, px = best_y, best_x
        out[k, 0] = py
        out[k, 1] = px
    return out


def augment_for_feasibility(
    gmap: GridMap,
    candidates: np.ndarray,
    coverage: csr_matrix,
    free_xy: np.ndarray,
    target_mask: np.ndarray,
    r: float,
    n_rays: int | None = None,
    dedup_radius: float | None = None,
    max_iters: int = 3,
    max_extra: int | None = None,
    push_into_free: bool = True,
    push_target_edt_factor: float = 0.35,
    verbose: bool = False,
) -> tuple[np.ndarray, csr_matrix]:
    """检测无任何候选可覆盖的"必须覆盖"目标, 把它们自身位置补入候选集.

    采用迭代式补全: 第 i 轮 dedup 半径减半, 直到所有可达目标
    都被覆盖, 或达到 ``max_iters`` 上限.

    保证 ``coverage_ratio=1.0`` 的全覆盖至少在理论上可行 (个别因
    栅格化误差仍不可达的 "残余" 目标交由上层 planner 决定如何处理).

    Args:
        gmap, free_xy, r, n_rays: 同 ``build_coverage``.
        candidates, coverage: 现有候选集与覆盖矩阵.
        target_mask: ``[|U|], bool``, 必须覆盖的目标.
        dedup_radius: 第一轮的最小间距 (默认 ``0.3*r``).
        max_iters: 迭代上限.
        max_extra: 单轮新增上限 (None=不限).

    Returns:
        (new_candidates, new_coverage).
    """
    if dedup_radius is None:
        dedup_radius = 0.3 * r

    cur_dedup = dedup_radius
    for it in range(max_iters):
        col_count = np.asarray((coverage > 0).sum(axis=0)).ravel()
        unreachable = target_mask & (col_count == 0)
        n_unreach = int(unreachable.sum())
        if n_unreach == 0:
            if verbose and it == 0:
                print("[augment] all targets reachable, nothing to do")
            break

        uy = free_xy[unreachable, 0]
        ux = free_xy[unreachable, 1]
        pts = np.stack([uy, ux], axis=1).astype(np.int32)

        if push_into_free:
            pts = _push_into_free(
                pts, gmap.edt, gmap.free,
                target_edt=push_target_edt_factor * r,
                max_walk=max(4, int(push_target_edt_factor * r)),
            )

        pts = _dedup_points(pts, cur_dedup)

        if max_extra is not None and pts.shape[0] > max_extra:
            idx = np.linspace(0, pts.shape[0] - 1, max_extra).astype(int)
            pts = pts[idx]

        if verbose:
            print(
                f"[augment] iter {it}: {n_unreach} unreachable -> "
                f"+{pts.shape[0]} candidates (dedup={cur_dedup:.1f}, "
                f"push={push_into_free})"
            )

        new_cov, _free_idx, _free_xy = build_coverage(
            gmap, pts, r=r, n_rays=n_rays
        )
        candidates = np.concatenate([candidates, pts], axis=0)
        coverage = sp_vstack([coverage, new_cov], format="csr")

        cur_dedup = max(1.0, cur_dedup / 2.0)

    return candidates, coverage
