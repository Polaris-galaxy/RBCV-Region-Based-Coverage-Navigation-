# `src/` 目录（RBCV 工程）

本目录为 **Region-Based Coverage Navigation（RBCV）** 的可运行代码与文档根。

本仓库离线主流程为纯 Python：从 `map_tools/maps/` 读取 ROS 风格 YAML+PGM 地图，
用 `coverage_planner` 划分分区并计算等半径覆盖点（可见性裁剪、Set Cover + ILS 精修）。
ROS1 相关包为可选，用于在线发布 `/map` 与分区标签栅格供下游节点使用。

| 子目录 / 文件 | 说明 |
|----------------|------|
| `coverage_planner/` | 纯 Python：规划与 `map_io`（地图路径通常指向 `map_tools/maps`） |
| `map_tools/` | ROS1：**地图资源**、`OccupancyGrid` 构建、静态 `/map` 发布节点 |
| `coverage_regions_ros/` | ROS1：订阅 `/map`，发布分区栅格（默认 JSON 在 `map_tools/maps`） |
| `Kimera部署指南.md` | Kimera（ROS1）部署步骤 |
| `ROS2终端指令.md` | ROS 2 CLI 备忘 |
| `requirements.txt` | 全套 Python 依赖（含规划器与示例可视化） |
| `USAGE.md` | **划分地图 → 覆盖** 端到端操作指南 |
| `FILES.md` | **仓库文件索引**：每个文件的用途一览 |

进入各子目录查看更细的 README。

## 操作指南

完整的「划分地图区域 → 圆盘覆盖」工作流见 **[USAGE.md](USAGE.md)**。
