"""宽走廊按物理宽度切条 + 在内部划分线上采样圆心。

约定:
    - 输入/输出均为 **像素栅格坐标** (row, col)，不修改地图数组本身。
    - 米制宽度依赖 ``resolution_m_per_px``（米/像素，与 ROS map 一致）。

规则（与用户描述对齐）:
    - 估计走廊横截面宽度 ``W_m``（由中轴上 EDT 中位数: ``2*median(EDT)`` 換算）.
    - 若 ``W_m <= strip_width_m``（默认 **3.4 m**，见 ``PlannerConfig.corridor_strip_width_m``）：不划条，由调用方退回 **单脊线**
      的中轴取样。
    - 若 ``W_m > strip_width_m``：取 ``n = ceil(W_m / strip_width_m)``，将横截面方向
      均分为 ``n + 1`` 条带子，并在 **中间的 n 条划分线** 上沿走廊走向按步距
      布点；**n > 1 时线在走廊方向上交错半步**（并排六边形取样），减少冗余圆。
"""
from __future__ import annotations

import math

import numpy as np


def _unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.array([1.0, 0.0], dtype=np.float64)
    return (v / n).astype(np.float64)


def corridor_component_axes(comp_pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """PCA 主轴 (沿走廊) 与垂直于走廊的单位向量."""
    centroid = np.mean(comp_pts, axis=0)
    c = comp_pts.astype(np.float64) - centroid
    cov = c.T @ c
    _, eigvecs = np.linalg.eigh(cov)
    main = _unit(eigvecs[:, -1])
    orth = _unit(eigvecs[:, -2])
    return main, orth


def corridor_orientation_label(main_dir: np.ndarray) -> str:
    """``horizontal``: 走廊主要沿列 / x。"""
    # 索引 (row, col) = (y, x)；主导方向 |col 分量| 更大则认为横向走廊。
    if abs(main_dir[1]) >= abs(main_dir[0]):
        return "horizontal"
    return "vertical"


def sample_division_line_on_grid(
    free: np.ndarray,
    centroid: np.ndarray,
    orth: np.ndarray,
    main: np.ndarray,
    t_delim: float,
    m_min: float,
    m_max: float,
    r_px: float,
    alpha: float,
    line_index: int = 0,
    stagger_across_lines: bool = False,
) -> np.ndarray:
    """在走廊走向上取样；多条划分线时奇数条线错开半步距以降低并排重叠."""
    step = max(1.0, float(r_px) * math.sqrt(3.0) * float(alpha))
    off = (
        (0.5 * step)
        if (stagger_across_lines and line_index % 2 == 1)
        else 0.0
    )
    out: list[tuple[int, int]] = []
    s = float(m_min) + off
    while s <= float(m_max) + 1e-6:
        p = centroid + float(t_delim) * orth + s * main
        yi, xi = int(round(p[0])), int(round(p[1]))
        if 0 <= yi < free.shape[0] and 0 <= xi < free.shape[1] and free[yi, xi]:
            out.append((yi, xi))
        s += step
    if not out and m_max >= m_min:
        mc = 0.5 * (float(m_min) + float(m_max)) + off
        p = centroid + float(t_delim) * orth + mc * main
        yi, xi = int(round(p[0])), int(round(p[1]))
        if 0 <= yi < free.shape[0] and 0 <= xi < free.shape[1] and free[yi, xi]:
            out.append((yi, xi))
    if not out:
        return np.zeros((0, 2), dtype=np.int32)
    return np.asarray(out, dtype=np.int32)


def wide_corridor_strip_centers(
    free: np.ndarray,
    edt: np.ndarray,
    comp_pts: np.ndarray,
    r_px: float,
    resolution_m_per_px: float,
    strip_width_m: float,
    alpha: float = 0.9,
) -> tuple[np.ndarray, int, str]:
    """对单条骨架连通分量若足够宽则返回划分线圆心。

    Returns:
        (centers, n_lines, orient). ``n_lines==0`` 表示不启用划条（窄走廊）。
    """
    if (
        comp_pts.shape[0] == 0
        or resolution_m_per_px <= 0
        or strip_width_m <= 0
    ):
        return np.zeros((0, 2), dtype=np.int32), 0, "unknown"

    yy = np.clip(comp_pts[:, 0].astype(np.int64), 0, edt.shape[0] - 1)
    xx = np.clip(comp_pts[:, 1].astype(np.int64), 0, edt.shape[1] - 1)
    rho = np.asarray(edt[yy, xx], dtype=np.float64)
    width_px = 2.0 * float(np.median(rho))
    width_m = width_px * float(resolution_m_per_px)

    if width_m <= float(strip_width_m) + 1e-9:
        return np.zeros((0, 2), dtype=np.int32), 0, "narrow"

    n = int(math.ceil(width_m / float(strip_width_m)))
    n_strips = n + 1
    n_lines = n

    centroid = np.mean(comp_pts.astype(np.float64), axis=0)
    rho_px = max(float(np.median(rho)), 1.0)

    dy = float(comp_pts[:, 0].max() - comp_pts[:, 0].min()) + 1e-9
    dx = float(comp_pts[:, 1].max() - comp_pts[:, 1].min()) + 1e-9

    if dx >= dy * 1.15:
        main = np.array([0.0, 1.0], dtype=np.float64)
        orth = np.array([1.0, 0.0], dtype=np.float64)
        orient = "horizontal"
        tc = float(np.dot(centroid, orth))
        t_min = tc - rho_px
        t_max = tc + rho_px
        pm = comp_pts.astype(np.float64) @ main
        m_min, m_max = float(pm.min()), float(pm.max())
    elif dy >= dx * 1.15:
        main = np.array([1.0, 0.0], dtype=np.float64)
        orth = np.array([0.0, 1.0], dtype=np.float64)
        orient = "vertical"
        tc = float(np.dot(centroid, orth))
        t_min = tc - rho_px
        t_max = tc + rho_px
        pm = comp_pts.astype(np.float64) @ main
        m_min, m_max = float(pm.min()), float(pm.max())
    else:
        main, orth = corridor_component_axes(comp_pts)
        orient = corridor_orientation_label(main)
        proj_o = comp_pts.astype(np.float64) @ orth
        proj_m = comp_pts.astype(np.float64) @ main
        t_min, t_max = float(proj_o.min()), float(proj_o.max())
        m_min, m_max = float(proj_m.min()), float(proj_m.max())
        if (t_max - t_min) < 1.5 * rho_px:
            tc = float(np.dot(centroid, orth))
            t_min = tc - rho_px
            t_max = tc + rho_px

    if t_max - t_min < 1e-3 or m_max - m_min < 1e-3:
        return np.zeros((0, 2), dtype=np.int32), 0, orient

    delim_ts = [
        t_min + k * (t_max - t_min) / n_strips for k in range(1, n_strips)
    ]
    stagger = int(n_lines) > 1
    chunks: list[np.ndarray] = []
    for li, td in enumerate(delim_ts):
        seg = sample_division_line_on_grid(
            free,
            centroid,
            orth,
            main,
            td,
            m_min,
            m_max,
            r_px,
            alpha,
            line_index=li,
            stagger_across_lines=stagger,
        )
        if seg.shape[0]:
            chunks.append(seg)
    if not chunks:
        return np.zeros((0, 2), dtype=np.int32), int(n_lines), orient
    return np.vstack(chunks).astype(np.int32), n_lines, orient
