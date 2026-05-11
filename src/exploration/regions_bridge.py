"""将 RBCV ``regions.json``（与 ``coverage_planner.region_io`` 相同的分区格式）转为 ``DetectionZone``。

本模块不导入 ``coverage_planner``，只解析 JSON。
"""

from __future__ import annotations

import json
from pathlib import Path

from .config import DetectionZone
from .detection_zones import zone_from_dict


def load_zones_from_regions_json(path: str | Path) -> list[DetectionZone]:
    """从 ``{"frame_id": "...", "regions": [{"id","xmin","ymin","xmax","ymax"}]}`` 构建检测框列表.

    每个轴对齐分区对应一个 ``DetectionZone``（``bbox_xyxy`` + 由载入逻辑补全的多边形）。
    """
    path = Path(path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    items = raw.get("regions")
    if not isinstance(items, list):
        raise ValueError("regions.json: missing list 'regions'")
    zones: list[DetectionZone] = []
    for i, it in enumerate(items):
        if not isinstance(it, dict):
            raise ValueError(f"regions[{i}] must be object")
        rid = str(it.get("id", f"region_{i}"))
        try:
            xmin, ymin, xmax, ymax = (
                float(it["xmin"]),
                float(it["ymin"]),
                float(it["xmax"]),
                float(it["ymax"]),
            )
        except KeyError as e:
            raise KeyError(f"regions[{i}] missing field {e}") from e
        d: dict = {
            "zone_id": rid,
            "bbox_xyxy": [xmin, ymin, xmax, ymax],
            "label": str(it.get("label", "")),
        }
        if it.get("base_weight_hint") is not None:
            d["base_weight_hint"] = it["base_weight_hint"]
        zones.append(zone_from_dict(d))
    return zones
