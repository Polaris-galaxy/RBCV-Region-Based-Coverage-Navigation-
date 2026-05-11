# 包内模块 `coverage_planner/`

等半径圆盘覆盖规划核心代码（**不依赖 ROS**，仅需 Python 数值与图像库）。

| 文件 | 作用 |
|------|------|
| `map_io.py` | 栅格加载、`load_ros_map`、EDT、中轴、`GridMap` |
| `region_io.py` | **长方形分区 JSON** → `labels` 掩码，与地图 `origin`/`resolution` 对齐 |
| `region_planner.py` | **按分区形状自适应**（open/narrow/mixed/tiny），逐区独立 `plan_coverage` |
| `candidates.py` | 候选观测点生成（六边形 / 中轴 / 反射顶点 / 随机） |
| `visibility.py` | 可见性、覆盖矩阵、`augment_for_feasibility` |
| `set_cover.py` | 贪心 Set Cover（含 `overlap_tiebreak` 减少冗余重叠） |
| `refine.py` | ILS 局部精修（try_remove / swap_2_for_1 / swap_3_for_2） |
| `planner.py` | `plan_coverage` 顶层流水线 |
| `viz.py` | Matplotlib 可视化 |

对外 API 见上级目录 `README.md`；包级懒加载见 `__init__.py`。
