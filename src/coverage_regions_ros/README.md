# coverage_regions_ros（ROS1 / catkin）

**ROS Noetic** 下可选功能包：订阅 `OccupancyGrid`（默认 `/map`），读取 **长方形分区 JSON**，发布与地图对齐的分区标签栅格。

该包的定位：

- 离线端：`coverage_planner` 负责“分区 → 覆盖点”规划（无需 ROS）。
- 在线端：本包把同一份分区 JSON 栅格化为 `OccupancyGrid`（labels），便于下游节点在 ROS 中对齐分区。

> **当前**：可先只看文档与脚本；不必在本机编译 ROS，后续在 Ubuntu catkin 工作空间中 `catkin_make` / `catkin build` 即可。

## 目录

| 子目录 | 说明 |
|--------|------|
| `scripts/` | Python 节点 |
| `launch/` | `roslaunch` 示例 |

## 改名（TODO）

若工程需要统一命名：同步修改 **`package.xml` 中 `<name>`** 与 **`CMakeLists.txt` 中 `project()`**，并替换 launch 里的包名。

## 简要用法（备忘）

```bash
# 分区 JSON 建议放在 map_tools/maps/，与离线脚本共用同一文件
roslaunch coverage_regions_ros publish_partition_labels.launch \
    regions_json:=$(rospack find map_tools)/maps/my_regions.json
```

详见各子目录 README。
