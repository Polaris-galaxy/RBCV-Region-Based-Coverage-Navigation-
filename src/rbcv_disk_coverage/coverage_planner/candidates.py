"""候选观测点生成。

策略:
    1. 中轴自适应步长 (适合通道, 处理窄通道关键)
    2. 六边形点阵 (适合宽阔房间)
    3. 反射顶点附近 (障碍凹角附近, 艺术馆问题最优解所在)

输出统一为 ``np.ndarray[N, 2]``, 行向量 ``[row, col]``。
"""
from __future__ import annotations

import math

import numpy as np
from scipy.ndimage import label as cc_label

from .map_io import GridMap


def _step_length(local_radius: float, r: float, alpha: float) -> float:
    """中轴上以局部半径 ``local_radius`` 计算的最大允许步长。

    通道宽 ``w = 2*local_radius``, 圆心位于中轴沿轴向覆盖长度
        L(w) = 2 * sqrt(r^2 - (w/2)^2)
    带安全系数 ``alpha`` (典型 0.85~0.95)。
    若 ``local_radius >= r``, 中轴已脱离通道范畴, 返回较大步长以
    交给六边形铺设处理。
    """
    if local_radius >= r:
        return r * math.sqrt(3.0) * alpha
    val = r * r - local_radius * local_radius
    if val <= 0.0:
        return 1.0
    return max(1.0, alpha * 2.0 * math.sqrt(val))


def _trace_skeleton(skel: np.ndarray) -> list[np.ndarray]:
    """把骨架二值图按 8-连通连通分量切分, 每分量返回点列。

    返回的点列只是"连通分量内的所有像素"集合 (顺序未保证),
    在采样时按贪心最近邻方式行走。
    """
    structure = np.ones((3, 3), dtype=bool)
    labels, n = cc_label(skel, structure=structure)
    if n == 0:
        return []
    components: list[np.ndarray] = []
    for k in range(1, n + 1):
        ys, xs = np.where(labels == k)
        if ys.size == 0:
            continue
        components.append(np.stack([ys, xs], axis=1))
    return components


def _walk_component(
    pts: np.ndarray,
    edt: np.ndarray,
    r: float,
    alpha: float,
    min_step: float,
) -> list[tuple[int, int]]:
    """在一个骨架连通分量上自适应步长行走, 返回采样点列表。

    简化做法: 按"未被任何已选点覆盖"准则选下一个点。
    距离用平方欧氏 (像素), 用 KD-Tree 加速近邻查询。
    """
    from scipy.spatial import cKDTree

    if pts.shape[0] == 0:
        return []

    tree = cKDTree(pts)
    n = pts.shape[0]
    placed_mask = np.zeros(n, dtype=bool)

    start_idx = int(np.argmax(edt[pts[:, 0], pts[:, 1]]))
    selected: list[int] = [start_idx]
    placed_mask[start_idx] = True

    queue = [start_idx]
    while queue:
        idx = queue.pop()
        y, x = pts[idx]
        local_r = float(edt[y, x])
        step = max(min_step, _step_length(local_r, r, alpha))

        nbrs = tree.query_ball_point([y, x], r=step * 1.05)
        nbrs_arr = np.asarray(nbrs, dtype=np.int64)

        if nbrs_arr.size == 0:
            continue
        for nb in nbrs_arr:
            placed_mask[nb] = True

        farther = tree.query_ball_point([y, x], r=step * 2.0)
        for j in farther:
            if placed_mask[j]:
                continue
            d = float(np.hypot(pts[j, 0] - y, pts[j, 1] - x))
            if abs(d - step) < step * 0.25:
                selected.append(j)
                placed_mask[j] = True
                queue.append(j)

    return [(int(pts[i, 0]), int(pts[i, 1])) for i in selected]


def medial_candidates(
    gmap: GridMap,
    r: float,
    alpha: float = 0.9,
    min_step: float = 2.0,
) -> np.ndarray:
    """沿中轴生成自适应步长候选点。"""
    components = _trace_skeleton(gmap.skeleton)
    out: list[tuple[int, int]] = []
    for comp in components:
        out.extend(_walk_component(comp, gmap.edt, r, alpha, min_step))
    if not out:
        return np.zeros((0, 2), dtype=np.int32)
    return np.asarray(out, dtype=np.int32)


def _hex_collect_for_phase(
    h: int,
    w: int,
    free: np.ndarray,
    edt: np.ndarray,
    r: float,
    min_room_radius: float,
    ox: float,
    oy: float,
    *,
    lattice_spacing_scale: float = 1.0,
) -> list[tuple[int, int]]:
    """固定相位 (ox,oy) 下枚举可走且满足净空阈值的六角格点."""
    sc = max(float(lattice_spacing_scale), 1e-6)
    dx = r * math.sqrt(3.0) * sc
    dy = 1.5 * r * sc
    pts: list[tuple[int, int]] = []
    j = 0
    y = oy + dy / 2.0
    while y < h:
        row_off = (dx / 2.0) if (j % 2 == 1) else 0.0
        x = ox + dx / 2.0 + row_off
        while x < w:
            yi = int(round(y))
            xi = int(round(x))
            if (
                0 <= yi < h
                and 0 <= xi < w
                and bool(free[yi, xi])
                and float(edt[yi, xi]) >= float(min_room_radius)
            ):
                pts.append((yi, xi))
            x += dx
        y += dy
        j += 1
    return pts


def hex_candidates(
    gmap: GridMap,
    r: float,
    min_room_radius: float | None = None,
    *,
    optimize_phase: bool = False,
    phase_search_steps: int = 9,
    lattice_spacing_scale: float = 1.0,
) -> np.ndarray:
    """在 ``edt >= min_room_radius`` 的"宽阔房间"区域铺设六边形点阵。

    Args:
        gmap: 预处理地图。
        r: 观测半径 (像素)。
        min_room_radius: 仅在 ``edt`` 不小于该值的位置放点;
            默认为 ``0.5 * r`` , 即只在距障碍 ``>= r/2`` 的开阔处。
        optimize_phase: 在周期 ``[0,dx)×[0,dy)`` 内 coarse 搜索相位, 优先
            「合法六角格点数多、且离障碍更远（EDT 和更大）」，使大厅空白
            区视觉对齐更规整。False 等价于相位 (0,0) (与旧网格锚定一致)。
        phase_search_steps: 相位搜索每轴采样数 ``≥2``.
        lattice_spacing_scale: 六角格距整体放大系数 (相对理论紧覆盖 ``≥1``).
            默认 ``1.0``; 略大于 ``1`` (如 ``1.03``) 可略减格点数, 依赖残余
            贪心/补圆仍达全覆盖, 需在具体地图上验证.
    """
    if min_room_radius is None:
        min_room_radius = 0.5 * r

    free = np.asarray(gmap.free, dtype=bool)
    edt = gmap.edt
    h, w = free.shape
    sc = max(float(lattice_spacing_scale), 1e-6)
    dx = r * math.sqrt(3.0) * sc
    dy = 1.5 * r * sc

    steps = max(2, int(phase_search_steps))
    ox_vals = np.linspace(0.0, dx, num=steps, endpoint=False).astype(np.float64)
    oy_vals = np.linspace(0.0, dy, num=steps, endpoint=False).astype(np.float64)

    best_pts: list[tuple[int, int]] = []
    best_key = (-1, -1.0)

    if not optimize_phase:
        ox_vals = np.array([0.0], dtype=np.float64)
        oy_vals = np.array([0.0], dtype=np.float64)

    for ox in ox_vals:
        for oy in oy_vals:
            pts = _hex_collect_for_phase(
                h,
                w,
                free,
                edt,
                r,
                float(min_room_radius),
                float(ox),
                float(oy),
                lattice_spacing_scale=sc,
            )
            if not pts:
                key = (0, 0.0)
            else:
                ed_sum = sum(float(edt[y, x]) for y, x in pts)
                key = (len(pts), ed_sum)
            if key > best_key:
                best_key = key
                best_pts = pts

    if not best_pts:
        return np.zeros((0, 2), dtype=np.int32)
    return np.asarray(best_pts, dtype=np.int32)


def reflex_candidates(
    gmap: GridMap,
    r: float,
    inset_factor: float = 0.4,
    min_inset_px: float = 4.0,
    sparse_radius_factor: float = 0.5,
) -> np.ndarray:
    """检测障碍凹角并在自由空间深处生成候选 (大幅减少边界冗余).

    关键改进 vs 旧实现:
        1. **深内缩**: 沿向内法向行进, 直到 ``edt(p) >= inset_factor * r``
           或撞墙为止. 避免候选贴墙导致单圆有效面积过小.
        2. **稀疏化**: 边界像素处理后, 在 ``sparse_radius_factor*r`` 内
           dedup, 防止一段长墙生成一长串候选.

    Args:
        gmap: 预处理地图 (含 ``edt`` , ``free`` ).
        r: 观测半径 (像素).
        inset_factor: 候选距墙目标比例 (相对 r).
        min_inset_px: 即使 edt 不够大也至少内缩这么多像素.
        sparse_radius_factor: 同段墙上候选间最小距离 (相对 r).
    """
    free = gmap.free
    edt = gmap.edt
    obstacle = ~free
    h, w = free.shape

    boundary = np.zeros_like(free)
    boundary[1:-1, 1:-1] = obstacle[1:-1, 1:-1] & (
        free[:-2, 1:-1] | free[2:, 1:-1] | free[1:-1, :-2] | free[1:-1, 2:]
    )

    bys, bxs = np.where(boundary)
    if bys.size == 0:
        return np.zeros((0, 2), dtype=np.int32)

    target_edt = inset_factor * r
    max_walk = max(int(math.ceil(target_edt)) + 2, int(min_inset_px) + 2)

    pts: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    for by, bx in zip(bys, bxs):
        y0 = max(by - 1, 0)
        y1 = min(by + 2, h)
        x0 = max(bx - 1, 0)
        x1 = min(bx + 2, w)
        patch = free[y0:y1, x0:x1]
        n_free = int(patch.sum())
        if n_free <= 4:
            continue

        ys_f, xs_f = np.where(patch)
        cy = ys_f.mean() + y0
        cx = xs_f.mean() + x0
        vy = cy - by
        vx = cx - bx
        norm = math.hypot(vy, vx)
        if norm < 1e-6:
            continue
        vy /= norm
        vx /= norm

        chosen_y, chosen_x = -1, -1
        best_edt = -1.0
        for s in range(1, max_walk + 1):
            ty = int(round(by + s * vy))
            tx = int(round(bx + s * vx))
            if not (0 <= ty < h and 0 <= tx < w):
                break
            if not free[ty, tx]:
                break
            e = float(edt[ty, tx])
            if e > best_edt:
                best_edt = e
                chosen_y, chosen_x = ty, tx
            if e >= target_edt and s >= min_inset_px:
                break

        if chosen_y < 0:
            continue
        key = (chosen_y, chosen_x)
        if key not in seen:
            seen.add(key)
            pts.append(key)

    if not pts:
        return np.zeros((0, 2), dtype=np.int32)
    pts_arr = np.asarray(pts, dtype=np.int32)

    sparse_r = sparse_radius_factor * r
    if sparse_r > 1 and pts_arr.shape[0] > 1:
        pts_arr = _dedup(pts_arr, sparse_r)

    return pts_arr


def _dedup(pts: np.ndarray, min_sep: float) -> np.ndarray:
    """去除距离过近的重复候选点。"""
    if pts.shape[0] <= 1:
        return pts
    from scipy.spatial import cKDTree

    tree = cKDTree(pts)
    keep = np.ones(pts.shape[0], dtype=bool)
    for i in range(pts.shape[0]):
        if not keep[i]:
            continue
        nbrs = tree.query_ball_point(pts[i], r=min_sep)
        for j in nbrs:
            if j > i:
                keep[j] = False
    return pts[keep]


def random_candidates(
    gmap: GridMap,
    n: int,
    seed: int = 0,
    min_clearance: float = 0.0,
) -> np.ndarray:
    """在 ``edt >= min_clearance`` 的自由空间内随机撒 ``n`` 个候选点.

    供 ILS 用作 swap 备选, 缓解 "区域所有候选都被选中导致无可替换" 问题.
    """
    h, w = gmap.shape
    if min_clearance > 0:
        valid = gmap.free & (gmap.edt >= min_clearance)
    else:
        valid = gmap.free
    ys, xs = np.where(valid)
    if ys.size == 0 or n <= 0:
        return np.zeros((0, 2), dtype=np.int32)
    rng = np.random.default_rng(seed)
    n = min(n, ys.size)
    idx = rng.choice(ys.size, size=n, replace=False)
    return np.stack([ys[idx], xs[idx]], axis=1).astype(np.int32)


def generate_candidates(
    gmap: GridMap,
    r: float,
    alpha: float = 0.9,
    use_medial: bool = True,
    use_hex: bool = True,
    use_reflex: bool = True,
    medial_max_edt_factor: float | None = None,
    hex_min_room_radius_factor: float | None = None,
    dedup_radius: float | None = None,
    reflex_inset_factor: float = 0.4,
    reflex_sparse_factor: float = 0.5,
    random_extra: int = 0,
    random_min_clearance: float | None = None,
    random_seed: int = 0,
) -> np.ndarray:
    """组合多类候选点并去重.

    Args:
        gmap: 预处理地图.
        r: 观测半径 (像素).
        alpha: 中轴步长安全系数 (0~1).
        use_medial / use_hex / use_reflex: 是否启用对应来源.
        dedup_radius: 去重最小间距, 默认 ``0.4*r``.
        medial_max_edt_factor: 若设置, 仅保留 ``edt <= factor * r`` 的中轴点
            （大厅中心 EDT 高处的中轴不参与，让给六边形）。
        hex_min_room_radius_factor: 若设置, ``hex_candidates`` 使用 ``factor * r`` 作
            ``min_room_radius``, 略高于默认 0.5 可把六边形更推向开阔核。
        random_extra: 额外随机候选数 (>0 时启用).
        random_min_clearance: 随机候选要求 ``edt >= 该值`` , 默认 ``0.3*r``.
        random_seed: 随机种子.

    Returns:
        ``[N, 2]`` int32, 行向量 ``[row, col]``.
    """
    if dedup_radius is None:
        dedup_radius = 0.55 * r
    if random_min_clearance is None:
        random_min_clearance = 0.3 * r

    parts: list[np.ndarray] = []
    if use_medial:
        med = medial_candidates(gmap, r, alpha=alpha)
        if (
            med.shape[0] > 0
            and medial_max_edt_factor is not None
        ):
            cap = float(medial_max_edt_factor) * float(r)
            yy = med[:, 0]
            xx = med[:, 1]
            keep = gmap.edt[yy, xx] <= cap
            med = med[keep]
        parts.append(med)
    if use_hex:
        min_rr = None
        if hex_min_room_radius_factor is not None:
            min_rr = float(hex_min_room_radius_factor) * float(r)
        parts.append(hex_candidates(gmap, r, min_room_radius=min_rr))
    if use_reflex:
        parts.append(reflex_candidates(
            gmap, r,
            inset_factor=reflex_inset_factor,
            sparse_radius_factor=reflex_sparse_factor,
        ))
    if random_extra > 0:
        parts.append(
            random_candidates(
                gmap, random_extra,
                seed=random_seed,
                min_clearance=random_min_clearance,
            )
        )

    parts = [p for p in parts if p.shape[0] > 0]
    if not parts:
        return np.zeros((0, 2), dtype=np.int32)
    pts = np.concatenate(parts, axis=0)
    return _dedup(pts, dedup_radius)
