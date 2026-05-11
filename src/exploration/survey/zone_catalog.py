"""载入带「覆盖观测几何」的检测框目录（survey_zones.json）。"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

from ..config import DetectionZone
from ..detection_zones import zone_from_dict


@dataclass
class CoverageSpec:
    """与 RBCV 圆盘覆盖对齐：圆心列表 + 半径；空圆心表示「框内任意位置均算在覆盖任务区内」。"""

    radius_m: float = 2.5
    sites_m: list[tuple[float, float]] = field(default_factory=list)


@dataclass
class SurveyZone:
    zone_id: str
    zone: DetectionZone
    coverage: CoverageSpec


def load_survey_zones(path: str | Path) -> list[SurveyZone]:
    """JSON schema::

        {
          "version": 2,
          "zones": [
            {
              "zone_id": "A",
              "bbox_xyxy": [x0,y0,x1,y1],
              "label": "",
              "base_weight_hint": 0.3,
              "coverage": { "radius_m": 2.5, "sites_m": [[x,y], ...] }
            }
          ]
        }

    亦可兼容旧版 ``detection_zones`` 文件（无 ``coverage``）：默认整框内都算覆盖区。
    """
    path = Path(path)
    raw = json.loads(path.read_text(encoding="utf-8"))
    items = raw.get("zones")
    if not isinstance(items, list):
        raise ValueError("survey_zones: missing list 'zones'")
    out: list[SurveyZone] = []
    for i, it in enumerate(items):
        if not isinstance(it, dict):
            raise ValueError(f"zones[{i}] must be object")
        zid = str(it.get("zone_id", f"zone_{i}"))
        d = {k: v for k, v in it.items() if k != "coverage"}
        if "zone_id" not in d:
            d["zone_id"] = zid
        dz = zone_from_dict(d)
        cov_raw = it.get("coverage")
        if cov_raw is None:
            cov = CoverageSpec()
        else:
            r = float(cov_raw.get("radius_m", 2.5))
            sites = cov_raw.get("sites_m") or []
            sites_t: list[tuple[float, float]] = [
                (float(p[0]), float(p[1])) for p in sites if isinstance(p, (list, tuple)) and len(p) >= 2
            ]
            cov = CoverageSpec(radius_m=r, sites_m=sites_t)
        out.append(SurveyZone(zone_id=dz.zone_id, zone=dz, coverage=cov))
    return out


def survey_zones_as_detection_list(rows: list[SurveyZone]) -> list[DetectionZone]:
    return [s.zone for s in rows]


def workspace_xyxy_from_survey(rows: list[SurveyZone], margin_m: float = 0.5) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for s in rows:
        z = s.zone
        if z.polygon_xy:
            for x, y in z.polygon_xy:
                xs.append(x)
                ys.append(y)
        elif z.bbox_xyxy is not None:
            x0, y0, x1, y1 = z.bbox_xyxy
            xs += [x0, x1]
            ys += [y0, y1]
        for sx, sy in s.coverage.sites_m:
            xs.append(sx)
            ys.append(sy)
            xs.append(sx + s.coverage.radius_m)
            xs.append(sx - s.coverage.radius_m)
            ys.append(sy + s.coverage.radius_m)
            ys.append(sy - s.coverage.radius_m)
    if not xs:
        return (0.0, 0.0, 1.0, 1.0)
    return (
        min(xs) - margin_m,
        min(ys) - margin_m,
        max(xs) + margin_m,
        max(ys) + margin_m,
    )
