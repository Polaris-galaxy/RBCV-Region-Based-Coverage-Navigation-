# `src/` 工作区目录说明

本仓库在 `src/` 下按**职责**划分目录；**Python 包名**与**文件夹名**不必相同（当前仅 `coverage_planner` 为小包名，外层文件夹已改为直观名称）。

| 目录 | 肉眼可辨的作用 | 说明 |
|------|----------------|------|
| **`rbcv_disk_coverage/`** | **几何圆盘覆盖**（含障碍、可见性裁剪） | 内含 Python 包 **`coverage_planner`**（`import coverage_planner`）；脚本如 `draw_regions.py`、`demo_regions.py`。 |
| **`exploration/`** | **语义探索**：检测框 + rosbag + 视觉 JSONL、权重、均匀格、闭合路径 | 子包 **`exploration.survey`** 管覆盖几何与 rosbag 对齐；入口脚本 `scripts/run_semantic_stack.py`。 |
| **`map_tools/`** | 地图资源与 ROS1 **`/map` 发布** | catkin 包名 `map_tools`；`bootstrap` 会定位 `rbcv_disk_coverage` 以加载 `coverage_planner`。 |
| **`coverage_regions_ros/`** | ROS1 分区标签广播 | 订阅 `/map`，发布分区栅格。 |

环境变量（可选）：

- **`RBCV_DISK_COVERAGE_SRC`**：若源码不在默认相对路径，指向 **`rbcv_disk_coverage` 这一层**（其下应有子目录 `coverage_planner/`）。
- **`COVERAGE_PLANNER_SRC`**：同上，旧名兼容。

视觉检测与 bag **契约**见 `exploration/README.md` 与 `exploration/data/*.example.*`。
