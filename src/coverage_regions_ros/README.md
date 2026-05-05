# coverage_regions_ros（ROS1 / catkin）

**ROS Noetic** 下可选功能包：订阅 `OccupancyGrid`（默认 `/map`），读取 **长方形分区 JSON**，发布与地图对齐的分区标签栅格。

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
roslaunch coverage_regions_ros publish_partition_labels.launch regions_json:=/绝对路径/regions.json
```

详见各子目录 README。
