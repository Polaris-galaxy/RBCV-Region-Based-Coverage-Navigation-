# `src/` 目录

本目录下放可复用代码与说明文档。

| 子目录 / 文件 | 说明 |
|----------------|------|
| `coverage_planner/` | 纯 Python：规划与 `map_io`（地图路径通常指向 `map_tools/maps`） |
| `map_tools/` | ROS1：**地图资源**、`OccupancyGrid` 构建、静态 `/map` 发布节点 |
| `coverage_regions_ros/` | ROS1：订阅 `/map`，发布分区栅格（默认 JSON 在 `map_tools/maps`） |
| `Kimera部署指南.md` | Kimera（ROS1）部署步骤 |
| `ROS2终端指令.md` | ROS 2 CLI 备忘 |
| `requirements.txt` | 全套 Python 依赖（含规划器与示例可视化） |
| `USAGE.md` | **划分地图 → 覆盖** 端到端操作指南 |

进入各子目录查看更细的 README。

## 操作指南

完整的「划分地图区域 → 圆盘覆盖」工作流见 **[USAGE.md](USAGE.md)**。
