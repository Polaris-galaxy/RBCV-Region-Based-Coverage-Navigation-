# 工作区说明

本目录为机器人 / 地图相关笔记与代码的集合；**当前不强制在此执行 ROS 编译**，后续可按需在 Ubuntu + catkin 或本机 Python 环境中操作。

## 内容索引

| 路径 | 说明 |
|------|------|
| `src/coverage_planner/` | 圆盘覆盖规划（Python，`map_io` / `region_io`） |
| `src/map_tools/` | **地图资源 + Python 桥接 + ROS1 静态 `/map` 发布**（与上一项共用地图文件） |
| `src/coverage_regions_ros/` | ROS1：分区标签栅格广播（订阅 `/map`） |
| `src/Kimera部署指南.md` | Kimera 语义 SLAM 部署备忘（ROS1） |
| `src/ROS2终端指令.md` | ROS 2 终端命令参考 |

各子目录内另有 **README.md** 便于快速查看。

## 操作指南

完整的「划分地图区域 → 圆盘覆盖」工作流见 **[src/USAGE.md](src/USAGE.md)**。
