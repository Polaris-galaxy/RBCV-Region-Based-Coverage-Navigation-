# `src/` 目录（RBCV 工程）

本目录为 **Region-Based Coverage Navigation（RBCV）** 的可运行代码与文档根。

本仓库离线主流程为纯 Python：从 `map_tools/maps/` 读取 ROS 风格 YAML+PGM 地图，
用 **`rbcv_disk_coverage/`**（Python 包名仍为 `coverage_planner`）划分分区并计算等半径覆盖点；可选用 **`exploration/`** 做检测语义、rosbag 对齐与加权探索回路。原 **exploration_ws** 已并入 **`exploration/`**。
ROS1 相关包为可选，用于在线发布 `/map` 与分区标签栅格供下游节点使用。

| 子目录 / 文件 | 说明 |
|----------------|------|
| `rbcv_disk_coverage/` | **几何圆盘覆盖**：Python 包 `coverage_planner`，`map_io` 与地图路径通常指向 `map_tools/maps` |
| `map_tools/` | ROS1：**地图资源**、`OccupancyGrid` 构建、静态 `/map` 发布节点 |
| `coverage_regions_ros/` | ROS1：订阅 `/map`，发布分区栅格（默认 JSON 在 `map_tools/maps`） |
| `rbcv_bringup/` | ROS1：**组合 launch**、**两阶段流水线**（`rbcv_stage1_*` → `survey_zones.json`；`rbcv_stage2_*` → 语义探索计划）；依赖 `map_tools`、`coverage_regions_ros` |
| `exploration/` | **语义探索**：检测框+覆盖圆、rosbag 位姿与视觉 JSONL、物体去重、矩形格权重、闭合 tour；详见 `exploration/survey/` |
| `Kimera部署指南.md` | Kimera（ROS1）部署步骤 |
| `ROS2终端指令.md` | ROS 2 CLI 备忘 |
| `requirements.txt` | Python 依赖（规划器、可视化、`shapely`/`rosbags`、pytest） |
| `USAGE.md` | **划分地图 → 覆盖** 端到端操作指南 |
| `FILES.md` | **仓库文件索引**：每个文件的用途一览 |
| `WORKSPACE_LAYOUT.md` | **目录分工说明**（几何覆盖 / 语义探索 / ROS） |

进入各子目录查看更细的 README。

## 操作指南

完整的「划分地图区域 → 圆盘覆盖」工作流见 **[USAGE.md](USAGE.md)**。
