"""检测框 JSON 读写（位置 + 覆盖语义）."""

from __future__ import annotations

import json
from pathlib import Path

from .config import DetectionZone


def _polygon_from_bbox(b: tuple[float, float, float, float]) -> list[tuple[float, float]]:
    xmin, ymin, xmax, ymax = b
    return [
        (xmin, ymin),
        (xmax, ymin),
        (xmax, ymax),
        (xmin, ymax),
    ]


def zone_to_dict(z: DetectionZone) -> dict:
    d: dict = {"zone_id": z.zone_id, "label": z.label}
    if z.polygon_xy:
        d["polygon_xy"] = z.polygon_xy
    if z.bbox_xyxy:
        d["bbox_xyxy"] = list(z.bbox_xyxy)
    if z.base_weight_hint is not None:
        d["base_weight_hint"] = z.base_weight_hint
    return d


def zone_from_dict(d: dict) -> DetectionZone:
    poly = d.get("polygon_xy")
    if poly:
        poly = [tuple(map(float, p)) for p in poly]
    bb = d.get("bbox_xyxy")
    if bb is not None:
        bb = tuple(map(float, bb))
    z = DetectionZone(
        zone_id=str(d["zone_id"]),
        polygon_xy=poly,
        bbox_xyxy=bb,
        label=str(d.get("label", "")),
        base_weight_hint=d.get("base_weight_hint"),
    )
    if z.polygon_xy is None and z.bbox_xyxy is not None:
        z.polygon_xy = _polygon_from_bbox(z.bbox_xyxy)
    return z


def save_detection_zones(zones: list[DetectionZone], path: str | Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {"version": 1, "zones": [zone_to_dict(z) for z in zones]}
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")


def load_detection_zones(path: str | Path) -> list[DetectionZone]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return [zone_from_dict(d) for d in payload["zones"]]
