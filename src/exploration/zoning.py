"""在工作空间轴对齐包围盒上划分均匀矩形网格."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class GridRect:
    row: int
    col: int
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def centroid(self) -> tuple[float, float]:
        return (
            0.5 * (self.xmin + self.xmax),
            0.5 * (self.ymin + self.ymax),
        )


def build_uniform_grid(
    workspace_xyxy: tuple[float, float, float, float],
    *,
    cell_size_m: float,
) -> list[GridRect]:
    """``cell_size_m`` 为近似正方形边长；矩形数量 = ceil(W/cell) * ceil(H/cell)。

    若格子过多，可先增大 ``cell_size_m``（例如车体直径的 1.5~2.5 倍）。
    """
    xmin, ymin, xmax, ymax = workspace_xyxy
    w = max(xmax - xmin, 1e-9)
    h = max(ymax - ymin, 1e-9)
    nx = max(1, int(__import__("math").ceil(w / cell_size_m)))
    ny = max(1, int(__import__("math").ceil(h / cell_size_m)))
    dx = w / nx
    dy = h / ny
    cells: list[GridRect] = []
    for j in range(ny):
        for i in range(nx):
            cells.append(
                GridRect(
                    row=j,
                    col=i,
                    xmin=xmin + i * dx,
                    ymin=ymin + j * dy,
                    xmax=xmin + (i + 1) * dx,
                    ymax=ymin + (j + 1) * dy,
                )
            )
    return cells


def suggest_cell_size(
    workspace_xyxy: tuple[float, float, float, float],
    *,
    target_cells: int = 80,
    robot_footprint_m: float = 0.5,
) -> float:
    """启发式：使格子数接近 target_cells，且不小于机器人尺度。"""
    xmin, ymin, xmax, ymax = workspace_xyxy
    w = max(xmax - xmin, 1e-9)
    h = max(ymax - ymin, 1e-9)
    area = w * h
    s = (area / max(target_cells, 1)) ** 0.5
    return max(robot_footprint_m * 1.2, s)
