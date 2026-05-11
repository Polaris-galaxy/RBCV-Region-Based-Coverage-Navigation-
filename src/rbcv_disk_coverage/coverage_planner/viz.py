"""结果可视化。"""
from __future__ import annotations

import math

import numpy as np

from .planner import PlannerResult


def setup_chinese_font() -> None:
    """让 matplotlib 能在 Windows / Linux 上显示中文标题/图例.

    多次调用是安全的, 仅修改 ``rcParams``.
    """
    import matplotlib

    matplotlib.rcParams["axes.unicode_minus"] = False
    matplotlib.rcParams["font.sans-serif"] = [
        "Microsoft YaHei", "SimHei", "Source Han Sans SC",
        "Noto Sans CJK SC", "PingFang SC", "WenQuanYi Zen Hei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["font.family"] = "sans-serif"


def plot_result(
    result: PlannerResult,
    r: float,
    save_path: str | None = None,
    show_candidates: bool = False,
    use_visibility_disk: bool = True,
):
    """绘制规划结果：地图 + 选中点 + 观测圆.

    Args:
        result: :class:`PlannerResult` 规划结果.
        r: 视距半径 (像素).
        save_path: 若给出则把图保存到该路径 (PNG, dpi=150).
        show_candidates: 是否一并把候选点撒上 (橙色小点).
        use_visibility_disk: ``True`` 时按可见性裁剪绘制 (圆被障碍切掉),
            ``False`` 时回退到几何完整圆.
    """
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle

    from .visibility import visible_disk

    gmap = result.gmap
    free = gmap.free
    h, w = free.shape
    fig, ax = plt.subplots(figsize=(12, 12))
    bg = np.where(free, 255, 0).astype(np.uint8)
    ax.imshow(bg, cmap="gray", vmin=0, vmax=255,
              origin="upper", interpolation="nearest")

    if show_candidates and result.candidates.shape[0] > 0:
        ax.scatter(
            result.candidates[:, 1],
            result.candidates[:, 0],
            s=4,
            c="orange",
            alpha=0.4,
            label=f"candidates ({result.candidates.shape[0]})",
            zorder=2,
        )

    pts = result.selected_points
    if pts.shape[0] > 0:
        if use_visibility_disk:
            rgba = np.zeros((h, w, 4), dtype=np.float32)
            color = (0.0, 0.6, 1.0)
            alpha = 0.22
            n_rays = max(64, int(math.ceil(2.0 * math.pi * r)))
            for y, x in pts:
                mask = visible_disk(free, int(y), int(x), r, n_rays)
                if not mask.any():
                    continue
                prev_a = rgba[mask, 3]
                new_a = prev_a + (1.0 - prev_a) * alpha
                for ch in range(3):
                    prev_c = rgba[mask, ch]
                    rgba[mask, ch] = (
                        prev_c * prev_a
                        + color[ch] * alpha * (1.0 - prev_a)
                    ) / np.maximum(new_a, 1e-6)
                rgba[mask, 3] = new_a
            ax.imshow(rgba, origin="upper", interpolation="nearest", zorder=1)
        else:
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
            zorder=3,
        )

    ax.set_title(
        f"coverage planner result | "
        f"selected = {pts.shape[0]} | "
        f"covered = {result.coverage_ratio*100:.2f}%"
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
