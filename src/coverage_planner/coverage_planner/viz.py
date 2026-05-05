"""结果可视化。"""
from __future__ import annotations

import numpy as np

from .planner import PlannerResult


def plot_result(
    result: PlannerResult,
    r: float,
    save_path: str | None = None,
    show_candidates: bool = False,
):
    """画地图 + 候选点 + 选中点 + 覆盖圆。"""
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    gmap = result.gmap
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.imshow(gmap.free, cmap="gray", origin="upper", interpolation="nearest")

    if show_candidates and result.candidates.shape[0] > 0:
        ax.scatter(
            result.candidates[:, 1],
            result.candidates[:, 0],
            s=4,
            c="orange",
            alpha=0.4,
            label=f"candidates ({result.candidates.shape[0]})",
        )

    pts = result.selected_points
    if pts.shape[0] > 0:
        for y, x in pts:
            ax.add_patch(
                Circle(
                    (float(x), float(y)),
                    radius=r,
                    fill=True,
                    facecolor="cyan",
                    edgecolor="blue",
                    alpha=0.18,
                    linewidth=0.7,
                )
            )
        ax.scatter(
            pts[:, 1],
            pts[:, 0],
            s=18,
            c="red",
            marker="x",
            label=f"selected ({pts.shape[0]})",
        )

    ax.set_title(
        f"coverage planner result | "
        f"selected={pts.shape[0]} | "
        f"covered={result.coverage_ratio*100:.2f}%"
    )
    ax.legend(loc="upper right")
    ax.set_aspect("equal")
    plt.tight_layout()
    if save_path:
        plt.savefig(save_path, dpi=150)
    return fig, ax


def coverage_mask(
    result: PlannerResult, r: float
) -> np.ndarray:
    """返回选中点几何并集 (不考虑可见性) 的覆盖掩码, 仅供可视化对比。"""
    gmap = result.gmap
    h, w = gmap.shape
    yy, xx = np.mgrid[0:h, 0:w]
    mask = np.zeros((h, w), dtype=bool)
    for cy, cx in result.selected_points:
        d2 = (yy - int(cy)) ** 2 + (xx - int(cx)) ** 2
        mask |= d2 <= r * r
    return mask & gmap.free
