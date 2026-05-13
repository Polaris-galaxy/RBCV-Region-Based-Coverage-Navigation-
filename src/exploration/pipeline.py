"""端到端粘合：检测框 JSON、观测列表、划分、权重、路径。"""

from __future__ import annotations

from collections import defaultdict

from .aggregation import aggregate_rect_weights, finalize_zone_weights, resolve_initial_weights
from .config import (
    AggregationParams,
    DetectionZone,
    InitialRegionWeight,
    ObjectObservation,
    PathPlannerParams,
    SpatialRectPrior,
    StartSamplingParams,
)
from .zoning import GridRect, build_uniform_grid, suggest_cell_size
from .costmap import build_grid_cost_fields
from .fusion import count_unique_objects_in_zones
from .planner import plan_exploration_tour, sample_random_start_cell_key


def _spatial_priors_to_initial_rect_weights(
    cells: list[GridRect],
    priors: list[SpatialRectPrior],
) -> dict[tuple[int, int], float] | None:
    if not priors:
        return None
    acc: dict[tuple[int, int], float] = defaultdict(float)
    for cell in cells:
        cx, cy = cell.centroid()
        key = (cell.row, cell.col)
        for p in priors:
            if p.xmin <= cx <= p.xmax and p.ymin <= cy <= p.ymax:
                acc[key] += float(p.boost)
    return dict(acc)


def run_post_survey_planning(
    *,
    zones: list[DetectionZone],
    observations: list[ObjectObservation],
    workspace_xyxy: tuple[float, float, float, float],
    priors: list[InitialRegionWeight],
    agg: AggregationParams | None = None,
    path_params: PathPlannerParams | None = None,
    cell_size_m: float | None = None,
    merge_radius_m: float = 0.45,
    start_cell_key: tuple[int, int] | None = None,
    start_sampling: StartSamplingParams | None = None,
    spatial_rect_priors: list[SpatialRectPrior] | None = None,
) -> dict:
    agg = agg or AggregationParams()
    path_params = path_params or PathPlannerParams()

    per_class = count_unique_objects_in_zones(observations, merge_radius_m=merge_radius_m)
    per_zone = {z: sum(dc.values()) for z, dc in per_class.items()}
    init_w = resolve_initial_weights(zones, priors)
    zone_w = finalize_zone_weights(per_zone, init_w, agg)

    if cell_size_m is None:
        cell_size_m = suggest_cell_size(workspace_xyxy, target_cells=80, robot_footprint_m=0.5)
    cells = build_uniform_grid(workspace_xyxy, cell_size_m=cell_size_m)
    initial_rw = _spatial_priors_to_initial_rect_weights(cells, spatial_rect_priors or [])
    rect_w = aggregate_rect_weights(
        cells,
        zones,
        zone_w,
        initial_rect_weights=initial_rw,
    )

    edges, _trav = build_grid_cost_fields(cells, rect_w, path_params)

    rng = __import__("random").Random(int(path_params.random_seed))
    ss = start_sampling or StartSamplingParams()
    if start_cell_key is None and cells:
        start_cell_key = sample_random_start_cell_key(
            cells,
            rect_w,
            workspace_xyxy,
            rng=rng,
            weight_boost=ss.weight_boost,
            edge_boost=ss.edge_boost,
            uniform_floor=ss.uniform_floor,
            edge_tol_m=ss.edge_tol_m,
        )

    valid_keys = {(c.row, c.col) for c in cells}
    resolved_start = (
        start_cell_key
        if start_cell_key is not None and start_cell_key in valid_keys
        else ((cells[0].row, cells[0].col) if cells else (0, 0))
    )
    tour_keys = plan_exploration_tour(cells, edges, start_cell_key=resolved_start)

    centers = { (c.row, c.col): c.centroid() for c in cells }
    waypoints = [centers[k] for k in tour_keys if k in centers]

    return {
        "zone_weights": zone_w,
        "per_zone_class_counts": per_class,
        "rect_weights": rect_w,
        "cells": cells,
        "cell_size_m": cell_size_m,
        "tour_cell_keys": tour_keys,
        "waypoints_xy": waypoints,
        "graph_edges": edges,
        "start_cell_key": resolved_start,
        "random_seed_used": int(path_params.random_seed),
    }


def workspace_from_zones(zones: list[DetectionZone], margin_m: float = 1.0) -> tuple[float, float, float, float]:
    xs: list[float] = []
    ys: list[float] = []
    for z in zones:
        if z.polygon_xy:
            for x, y in z.polygon_xy:
                xs.append(x)
                ys.append(y)
        elif z.bbox_xyxy is not None:
            x0, y0, x1, y1 = z.bbox_xyxy
            xs += [x0, x1]
            ys += [y0, y1]
    if not xs:
        return (0, 0, 1, 1)
    return (
        min(xs) - margin_m,
        min(ys) - margin_m,
        max(xs) + margin_m,
        max(ys) + margin_m,
    )
