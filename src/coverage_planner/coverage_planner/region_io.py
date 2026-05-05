"""地图坐标系下的长方形分区：加载、栅格化、与自由空间取交.

与 :class:`RosMapInfo` / ``load_ros_map`` 配套使用：矩形顶点使用 **地图坐标**
(米), 与 ROS ``map_server`` / ``nav_msgs/OccupancyGrid`` 一致::

    cell_center_mx = origin_x + (col + 0.5) * resolution
    cell_center_my = origin_y + (row + 0.5) * resolution

判定规则：若某栅格 **中心点** 落在矩形闭区间 ``[xmin,xmax]×[ymin,ymax]`` 内，
则该格属于该区域（可按需在矩形外加半格 ``resolution/2`` 做膨胀）。

重叠区域：**先声明者优先**（保持标签稳定）；如需并集可对 ``labels > 0`` 处理。
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .map_io import RosMapInfo


@dataclass(frozen=True)
class RegionRect:
    """轴对齐长方形（地图坐标，单位米）."""

    id: str
    xmin: float
    ymin: float
    xmax: float
    ymax: float

    def __post_init__(self) -> None:
        xa, xb = sorted((float(self.xmin), float(self.xmax)))
        ya, yb = sorted((float(self.ymin), float(self.ymax)))
        object.__setattr__(self, "xmin", xa)
        object.__setattr__(self, "xmax", xb)
        object.__setattr__(self, "ymin", ya)
        object.__setattr__(self, "ymax", yb)


@dataclass
class RegionsSpec:
    """分区描述文件顶层结构."""

    frame_id: str
    regions: list[RegionRect]


def load_regions_json(path: str | Path) -> RegionsSpec:
    """从 JSON 载入分区定义.

    Schema::

        {
          "frame_id": "map",
          "regions": [
            {"id": "A", "xmin": 0.0, "ymin": 0.0, "xmax": 10.0, "ymax": 5.0}
          ]
        }
    """
    path = Path(path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    frame_id = str(raw.get("frame_id", "map"))
    items = raw.get("regions")
    if not isinstance(items, list):
        raise ValueError("regions.json: missing list 'regions'")
    regions: list[RegionRect] = []
    for i, it in enumerate(items):
        if not isinstance(it, dict):
            raise ValueError(f"regions[{i}] must be object")
        rid = str(it.get("id", f"region_{i}"))
        try:
            regions.append(
                RegionRect(
                    id=rid,
                    xmin=float(it["xmin"]),
                    ymin=float(it["ymin"]),
                    xmax=float(it["xmax"]),
                    ymax=float(it["ymax"]),
                )
            )
        except KeyError as e:
            raise KeyError(f"regions[{i}] missing field {e}") from e
    return RegionsSpec(frame_id=frame_id, regions=regions)


def save_regions_json(spec: RegionsSpec, path: str | Path) -> None:
    """写出 JSON（便于 RViz / 其它工具对齐后再保存）."""
    path = Path(path)
    payload = {
        "frame_id": spec.frame_id,
        "regions": [
            {
                "id": r.id,
                "xmin": r.xmin,
                "ymin": r.ymin,
                "xmax": r.xmax,
                "ymax": r.ymax,
            }
            for r in spec.regions
        ],
    }
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def rasterize_regions(
    spec: RegionsSpec,
    info: RosMapInfo,
    shape: tuple[int, int],
) -> np.ndarray:
    """将长方形列表栅格化为 ``labels[H,W]``.

    Returns:
        ``int32``, ``0`` 表示不属于任一给定矩形；``k>=1`` 为 ``spec.regions[k-1]``.
    """
    h, w = int(shape[0]), int(shape[1])
    ox, oy, _ = info.origin
    res = float(info.resolution)
    cols = np.arange(w, dtype=np.float64)
    rows = np.arange(h, dtype=np.float64)
    cx = ox + (cols + 0.5) * res
    cy = oy + (rows + 0.5) * res
    mx, my = np.meshgrid(cx, cy)

    labels = np.zeros((h, w), dtype=np.int32)
    for idx, reg in enumerate(spec.regions, start=1):
        inside = (
            (mx >= reg.xmin)
            & (mx <= reg.xmax)
            & (my >= reg.ymin)
            & (my <= reg.ymax)
        )
        labels = np.where((labels == 0) & inside, idx, labels)
    return labels


def union_region_mask(labels: np.ndarray) -> np.ndarray:
    """是否落在任一矩形内（按栅格中心判定后的并集）."""
    return labels > 0


def mask_free_with_regions(free: np.ndarray, labels: np.ndarray) -> np.ndarray:
    """自由空间 ∩ 分区并集."""
    if free.shape != labels.shape:
        raise ValueError(f"shape mismatch free={free.shape} labels={labels.shape}")
    return free & union_region_mask(labels)


def split_labels_by_region(labels: np.ndarray) -> dict[int, np.ndarray]:
    """``region_index -> boolean mask``，仅包含 ``labels`` 中出现过且 ``>0`` 的键."""
    ids = [int(k) for k in np.unique(labels) if k > 0]
    return {k: labels == k for k in ids}
