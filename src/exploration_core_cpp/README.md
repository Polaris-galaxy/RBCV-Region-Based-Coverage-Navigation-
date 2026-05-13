# `exploration_core_cpp/` — 探索回路规划（可选加速）

与 `exploration/planner.py` 中 **最近邻 + 2-opt** 逻辑等价的独立可执行文件，便于在大规模均匀格（数百格以上）时缩短 CPU 时间。

## 构建

在 **本目录** 下执行：

```bash
cmake -S . -B build
cmake --build build --config Release
```

Windows 可执行文件通常在 `build/Release/rbcv_plan_tour.exe`，Linux/macOS 为 `build/rbcv_plan_tour`。

## 使用

1. 设置环境变量 **`RBCV_EXPLORATION_PLANNER_EXE`** 为上述可执行文件的绝对路径。  
2. 正常运行 `exploration/scripts/run_semantic_stack.py` 或 ROS **`rbcv_stage2_semantic_explore.launch`**；`plan_exploration_tour` 会自动探测该变量并调用原生实现。  
3. 未设置变量时 **自动回退** 至纯 Python，行为与旧版一致。

## 输入输出格式

- **stdin**：首行 `n start_idx num_edges`，随后 `num_edges` 行 `u v w`（与 `build_grid_cost_fields` 展开的有向边列表一致）。  
- **stdout**：一行闭合回路节点下标（末元素重复首节点）。

## 注意

- 仅加速 **拓扑Tour**；栅格权重、代价场、DBSCAN 融合等仍在 Python 中完成。  
- 若后续需要将 **代价场构建** 一并迁入 C++，可在此包内新增库目标并在 `exploration/costmap.py` 侧增加可选绑定。
