"""检测框权重 + 初始权重 -> 矩形格权重."""

from __future__ import annotations

from collections import defaultdict

from shapely.geometry import Polygon, Point, box as shapely_box

from .config import (
    INITIAL_WEIGHT_CEIL,
    INITIAL_WEIGHT_DEFAULT,
    INITIAL_WEIGHT_FLOOR,
    AggregationParams,
    DetectionZone,
    InitialRegionWeight,
)
from .zoning import GridRect


def _poly(z: DetectionZone) -> Polygon:
    if z.polygon_xy:
        return Polygon(z.polygon_xy)
    assert z.bbox_xyxy is not None
    x0, y0, x1, y1 = z.bbox_xyxy
    return shapely_box(x0, y0, x1, y1)


def _clamp_initial(w: float) -> float:
    return max(INITIAL_WEIGHT_FLOOR, min(INITIAL_WEIGHT_CEIL, float(w)))


def resolve_initial_weights(zones: list[DetectionZone], priors: list[InitialRegionWeight]) -> dict[str, float]:
    """zone_id -> 初始权重（夹在 ``INITIAL_WEIGHT_FLOOR``..``INITIAL_WEIGHT_CEIL``）。"""
    zw: dict[str, float] = {}
    for z in zones:
        if z.base_weight_hint is not None:
            zw[z.zone_id] = _clamp_initial(z.base_weight_hint)
        else:
            zw[z.zone_id] = INITIAL_WEIGHT_DEFAULT

    for p in priors:
        if p.applies_to_zone_ids:
            for zid in p.applies_to_zone_ids:
                zw[zid] = _clamp_initial(p.weight)
    return zw


def finalize_zone_weights(
    zone_object_counts: dict[str, int],
    initial_z: dict[str, float],
    params: AggregationParams,
) -> dict[str, float]:
    """已完成探索后对 **检测框** 赋权重。"""
    fin: dict[str, float] = {}
    for zid, n in zone_object_counts.items():
        frac = min(1.0, float(n) / max(params.count_scale, 1e-9))
        w_count = params.weight_floor + frac * (params.weight_ceil - params.weight_floor)
        w0 = initial_z.get(zid, params.weight_floor)
        if params.blend_with_initial == "mul":
            fin[zid] = max(params.weight_floor, min(params.weight_ceil, w0 * (0.65 + 0.35 * w_count)))
        elif params.blend_with_initial == "add":
            fin[zid] = max(params.weight_floor, min(params.weight_ceil, 0.5 * (w0 + w_count)))
        else:
            fin[zid] = max(params.weight_floor, min(params.weight_ceil, max(w0, w_count)))
    # 未有计数的 zone 保留初始权重
    for zid, w0 in initial_z.items():
        fin.setdefault(zid, max(params.weight_floor, min(params.weight_ceil, w0)))
    return fin


def aggregate_rect_weights(
    cells: list[GridRect],
    zones: list[DetectionZone],
    zone_weights: dict[str, float],
    *,
    initial_rect_weights: dict[tuple[int, int], float] | None = None,
    empty_rect_default: float = 0.2,
    combine: str = "sum_norm",
) -> dict[tuple[int, int], float]:
    """每个矩形格：累加落入其 centroid 或与格相交的检测框权重，并可加格点先验。"""
    polys = {z.zone_id: _poly(z) for z in zones}
    rect_w: dict[tuple[int, int], float] = defaultdict(float)

    if initial_rect_weights:
        for (r, c), w in initial_rect_weights.items():
            rect_w[(r, c)] += w

    for cell in cells:
        key = (cell.row, cell.col)
        b = shapely_box(cell.xmin, cell.ymin, cell.xmax, cell.ymax)
        cxy = Point(cell.centroid())
        for zid, poly in polys.items():
            wz = zone_weights.get(zid, 0.0)
            if b.intersects(poly) or poly.contains(cxy):
                rect_w[key] += wz

    # 归一或填默认
    out: dict[tuple[int, int], float] = {}
    for cell in cells:
        key = (cell.row, cell.col)
        v = rect_w.get(key, 0.0)
        if v <= 0 and combine == "sum_norm":
            v = empty_rect_default
        out[key] = v

    if combine == "sum_norm":
        m = max(out.values()) if out else 1.0
        if m <= 1e-9:
            m = 1.0
        out = {k: min(1.0, float(v) / m) for k, v in out.items()}

    return out
