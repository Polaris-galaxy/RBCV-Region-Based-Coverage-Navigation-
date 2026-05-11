"""将矩形权重转为可导航图的「Traversal 代价」（高权重更愿意走）."""

from __future__ import annotations

import math

from .config import PathPlannerParams
from .zoning import GridRect


def weights_to_traversal_cost(
    rect_weights: dict[tuple[int, int], float],
    cells: list[GridRect],
    params: PathPlannerParams,
    *,
    bonus_cells: dict[tuple[int, int], float] | None = None,
) -> dict[tuple[int, int], float]:
    """``traversal_cost = base - reward`` 近似：越低越「愿意」经过。

    实际导航可再结合占据栅格；此处仅语义层权重。
    """
    rng = __import__("random").Random(params.random_seed)

    highs = sorted(rect_weights.values())
    median = highs[len(highs) // 2] if highs else 0.5

    bonus = bonus_cells or {}

    keys = {(c.row, c.col) for c in cells}

    extras: dict[tuple[int, int], float] = {}
    low = [k for k in keys if rect_weights.get(k, 0) <= median + 1e-9]
    rng.shuffle(low)
    for k in low[: params.n_random_bonus_cells]:
        extras[k] = params.exploration_bonus_low_weight

    out = {}
    for k in keys:
        w = min(1.0, max(0.0, rect_weights.get(k, 0)))
        rew = params.reward_high_weight_coef * w
        rew += bonus.get(k, 0.0)
        rew += extras.get(k, 0.0)

        travel = params.cost_per_meter * 1.0
        base = travel
        cost = base - rew
        out[k] = max(params.epsilon, cost)
    return out


def cell_graph_edges(
    cells: list[GridRect],
) -> list[tuple[tuple[int, int], tuple[int, int], float]]:
    """邻接矩形 4 连通，边长取中心欧氏距离。"""
    lookup = {(c.row, c.col): c for c in cells}
    edges: list[tuple[tuple[int, int], tuple[int, int], float]] = []
    for c in cells:
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nb = lookup.get((c.row + dr, c.col + dc))
            if not nb:
                continue
            x0, y0 = c.centroid()
            x1, y1 = nb.centroid()
            dist = math.hypot(x1 - x0, y1 - y0)
            edges.append(((c.row, c.col), (nb.row, nb.col), dist))
    return edges


def build_grid_cost_fields(
    cells: list[GridRect],
    rect_weights: dict[tuple[int, int], float],
    params: PathPlannerParams,
) -> tuple[list[tuple[int, int, int, float]], dict[tuple[int, int], float]]:
    """(from_idx, to_idx, directed_edge_index_in_list, weighted_edge_cost)

    idx 枚举 ``enumerate(cells)`` 顺序。"""
    key_to_idx = {(c.row, c.col): i for i, c in enumerate(cells)}
    traversal = weights_to_traversal_cost(rect_weights, cells, params)
    raw_edges = cell_graph_edges(cells)
    out: list[tuple[int, int, int, float]] = []

    triples: list[tuple[tuple[int, int], tuple[int, int], float]] = []
    for a, b, dist in raw_edges:
        wa = traversal[a]
        wb = traversal[b]
        w_edge = 0.5 * (wa + wb) * dist
        triples.append((a, b, w_edge))

    for ei, (a, b, w_edge) in enumerate(triples):
        ia, ib = key_to_idx[a], key_to_idx[b]
        out.append((ia, ib, ei, w_edge))
        out.append((ib, ia, ei, w_edge))

    return out, traversal
