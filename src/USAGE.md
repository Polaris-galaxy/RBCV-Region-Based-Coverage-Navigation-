# 操作指南：地图分区 + 圆盘覆盖（RBCV）

> **RBCV** = Region-Based Coverage Navigation。  
> 目标：把 **`src/map_tools/maps/`** 里的栅格地图手动划分长方形分区，再在目录 **`src/rbcv_disk_coverage/`** 下用 Python 包 **`coverage_planner`** 对每个分区做半径 **2.5 m** 的检测圆覆盖。**离线步骤全程纯 Python，不需要 ROS。**

---

## 0. 前置环境（一次性）

在 **仓库根目录** 下的 `src/` 安装依赖（路径按本机克隆位置替换 `<仓库根>`）：

```powershell
cd <仓库根>\src
py -m pip install -r requirements.txt
```

示例（文件夹名为 `RBCV-Region-Based-Coverage-Navigation-` 时）：

```powershell
cd D:\Galaxy\其他\桌面\RBCV-Region-Based-Coverage-Navigation-\src
py -m pip install -r requirements.txt
```

依赖：`numpy` `scipy` `shapely` `rosbags`（探索与 rosbag）`scikit-image` `imageio` `matplotlib` `numba` `pytest`（`numba` 用于可见性 JIT 加速）。

> **`scripts/demo_regions.py` 等命令的工作目录必须是 `src/rbcv_disk_coverage`**，见 **§4**；其它 `scripts\` 同上。

---

## 1. 准备地图

地图统一放在 **`src/map_tools/maps/`**：

```
map_tools/maps/
├── map7.yaml + map7.pgm           # ROS map_server 风格（YAML 中 image 用相对路径）
├── map8.yaml + map8.pgm
└── regions_map7.example.json      # 示例分区
```

新地图：把 `*.yaml` + `*.pgm` 放进该目录，且 `yaml` 里 **`image:`** 字段使用 **相对文件名**（如 `image: map9.pgm`）。

> 想先看地图边界范围（米），在 **`src/rbcv_disk_coverage`** 下执行：
>
> ```powershell
> py scripts\preview_real_map.py ..\map_tools\maps\map7.yaml
> ```
>
> 终端会打印 `shape`、`resolution`，结合 `yaml` 里的 `origin` 即可推算 x/y 范围。

---

## 2. 划分长方形区域

以下两种方式任选其一，**都生成同一种 JSON**。运行前请 **`cd` 到 `src/rbcv_disk_coverage`**（与 **§4** 一致），再执行 `py scripts\...`。

### 方式 A：交互式拖框（推荐）

```powershell
py scripts\draw_regions.py ..\map_tools\maps\map7.yaml ..\map_tools\maps\my_regions.json
```

操作（窗口标题也有提示）：

| 操作 | 含义 |
|------|------|
| 鼠标左键拖动 | 画一个长方形分区（自动命名 `R0/R1/...`） |
| `n` | 终端里输入下一个矩形的字符串 id（如 `room_A`） |
| `u` | 撤销最后一个矩形 |
| `r` | 清空全部 |
| **`s`** | **保存** 到 `my_regions.json` |
| `q` | 关闭窗口 |

显示坐标轴单位为 **米**，已用 `RosMapInfo.origin + resolution` 对齐 ROS 约定。

> Windows 若遇到中文乱码：脚本会自动尝试注册 `Microsoft YaHei/SimHei` 等系统字体；
> 仍异常时请确认系统存在 `C:\Windows\Fonts\msyh.ttc`（微软雅黑）。

### 方式 B：直接编辑 JSON

复制 `regions_map7.example.json` 改名（如 `my_regions.json`），按需要增删矩形：

```json
{
  "frame_id": "map",
  "regions": [
    {"id": "room_A",  "xmin": -48.0, "ymin": -50.0, "xmax": -35.0, "ymax": -38.0},
    {"id": "corridor","xmin": -35.0, "ymin": -45.0, "xmax": -22.0, "ymax": -42.0}
  ]
}
```

字段：

- **`frame_id`**：通常 `map`，与发布占用栅格的帧一致；
- **`xmin/ymin/xmax/ymax`**：**米**，`map` 坐标系；
- **`id`**：任意字符串，仅作标识；
- 矩形可重叠：**列表中靠前者优先**（`region_io.rasterize_regions` 已实现）。

---

## 3. 验证分区对齐

工作目录：**`src/rbcv_disk_coverage`**。

```powershell
py scripts\preview_regions.py ..\map_tools\maps\map7.yaml ..\map_tools\maps\my_regions.json
```

终端会列出每个分区的：

| 字段 | 含义 |
|------|------|
| `矩形内像素` | 矩形覆盖到的栅格数 |
| `可走∩矩形` | 与自由空间相交的栅格数（**= 0 通常说明矩形落在障碍 / 地图外**） |

弹出窗口：左图叠加矩形边框，右图用不同颜色显示分区 ID（自动保存到 `scripts/分区叠图预览.png`）。

> 说明：`scripts/` 下的 `*.png` 为运行时生成文件，**可随时重新生成**，已在仓库根 `.gitignore` 忽略。

---

## 4. 圆盘覆盖（`demo_regions.py`）

先 **`cd` 到 `src/rbcv_disk_coverage`**。脚本参数：**地图 YAML**、**半径（米）**、**分区 JSON**。

```powershell
cd <仓库根>\src\rbcv_disk_coverage
py scripts\demo_regions.py ..\map_tools\maps\map7.yaml 2.5 ..\map_tools\maps\my_regions.json
```

**CMD：** `cd /d <仓库根>\src\rbcv_disk_coverage`。

**候选流水线（默认）：** **`RBCV_HEX_FIRST=1`**（默认开启）使用「**先六边形铺底 → 残余区域再补圆**」两阶段：

1. 阶段 A：每个分区先生成 **物理六边形格点**，默认对 hex 跑 **Set Cover 贪心子集**作为初解（圆数少）；若要 **格点全集铺底**（更整齐、更吃圆）设 **`PlannerConfig.hex_first_pure_lattice=True`**；
2. 阶段 B：把 **未被阶段 A 覆盖到的残余像素 mask** 周围 **`hex_first_search_factor·r`** 像素膨胀作为搜索区，**只在该搜索区内** 追加 **中轴候选**（窄过道）和 **反射顶点候选**（墙边/凹角）；
3. 阶段 C：**保留阶段 A 的六边形初解**，只用阶段 B 候选修补残余，然后做局部删圆/替换。

效果：大厅中心不会被中轴/反射候选污染，也不会被后续全池 greedy 打乱；窄过道仍然按既有的中轴方法补齐。要回到旧的"按形状预先分类"流水线（`rect_room` / `narrow` / `mixed` 等仅影响默认候选开关、不再强行 hex_first），设 `RBCV_HEX_FIRST=0`。

**分区内精修（hex_first）：** 默认 **不含扰动** 的局部搜索（删圆 + swap）。默认还会 **在六边形格网上搜索相位**（`hex_optimize_grid_phase`，步数 `hex_phase_search_steps`），并对 **初解中已选用的 hex 圆心** 在净空 ≥ `hex_first_protect_core_edt_factor·r` 时做 **局部冻结**（默认约 `0.58·r`；设为 `None` 关闭），减轻删圆 / swap 把大厅打散。追求更少圆数时可设 **`RBCV_HEX_FIRST_FULL_ILS=1`** 启用完整 ILS（含扰动），可能把空旷区规则布局打乱、叠圆感变强。

**默认删除策略：** `RBCV_TRIM_MODE=local`（默认）先做各分区 hex_first 区内精修，再在整图上对 **候选并集** 做删圆与 swap。**整图 global trim** 默认 **不** 再套一层 hex 核冻结（`PlannerConfig.global_trim_protect_hex_core=False`），以便跨区删掉冗余圆；若须保住大厅六角不被整图 swap 打散，可设 **`RBCV_GLOBAL_HEX_FREEZE=1`**（或代码里 `global_trim_protect_hex_core=True`）。若只追求最少圆数，也可设 `RBCV_TRIM_MODE=ils` 或 `regreedy`（更激进，可能打乱开阔区）。

**规整矩形大厅：** `classify_region` 在满足高长方形程度与足够净空宽时标记为 **`rect_room`**（等价于早期的 `open`，分区汇总里会看到该类型）。

**规整矩形强制六边形流水线：** `PlannerConfig.force_hex_first_rectangle_rooms=True`（默认开启）时，在满足 **`rect_room` 长方形程度 / 净空宽度阈值** 的 **`mixed`** 等区域上也会 **`use_hex_first=True`**（先六边形铺满再补残余 + 区内局部搜索）。

**走廊（狭长通道）：** `classify_region` 在 **`aspect_ratio ≥ 2.5`** 且 **`width_ratio` 小于 `corridor_classify_width_ratio_max`**（默认 **`4.5`**，即相对半径 `r` 的宽度指标；可在 `PlannerConfig.corridor_classify_width_ratio_max` 微调）时判为 **`corridor`**，**不走 hex_first**。圆心生成：

- **物理宽度不超过 `corridor_strip_width_m`（默认 **3.4 m**，与 `PlannerConfig.corridor_strip_width_m` 一致）**：沿中轴单行 + **几何自适应步距**（下一步距 `α·2·√(r² − ρ²)`，ρ 为局部 EDT），并设 **步距下界**（`corridor_spine_step_floor_frac·√3·α·r`），避免横截面局部很宽时 ρ→r 导致步距塌成 1px、脊线过密；失败再固定 `1.25·r`，再失败才整区 `plan_coverage`。
- **物理宽度大于该阈值**：设 `n = ⌈W_m / corridor_strip_width_m⌉`，在横截面方向均分为 **`n + 1`** 条带子，并在中间 **`n` 条划分线** 上沿走廊走向补采样；**`n > 1` 时各划分线在走廊方向上错位半步距**，与中轴取样合并去重复。**必须提供地图分辨率（米/像素）**：脚本 `demo_regions.py` 已从地图 YAML 传入 `plan_partitions(..., resolution_m_per_px=float(info.resolution))`；若直接在 Python 里调用 `plan_partitions`，请传入 **`resolution_m_per_px=info.resolution`**（与 `PlannerConfig.resolution_m_per_px` 二选一即可，`plan_partitions` 会合并进 config）。未提供分辨率时，走廊行为退化为「仅中轴」（与旧版一致）。
- **横竖走向**：对每个骨架连通分量单独判断（一条分区里可同时含横向与纵向走廊段）。
- **`RegionShape.corridor_plan_meta`**：启用划条时会有一个简短摘要（例如 `hx2`），便于对照日志。

以下为旧表述归档（阈值已放宽）：原先文档写「`width_ratio < 2.0`」仅代表更严的一种手工分类边界；当前默认 **`4.5`**，以便数米宽的条形大厅仍走走廊管线。**地图栅格坐标不变**，划条与中轴都只输出像素 `(row, col)`。

**分区并行（多核）：** `demo_regions.py` 默认 **按分区并行规划**。环境变量 **`RBCV_PAR_BACKEND`**：`process`（多进程，易吃满多核，**脚本未设置该变量时默认用它**）或 `thread`（多线程，受 GIL 影响，往往看不出多核满载）。**只有 1 个分区时并行几乎不加速**；**全局精修**在各分区结束后 **串行**，并行不缩短这一段。

并行示例（在与上面相同的 `cd` 之后、`py ...` **之前** 设置）——

PowerShell：`$env:RBCV_PAR_BACKEND="process"`

CMD：`set RBCV_PAR_BACKEND=process`

（在 **Python 里直接调** `plan_partitions` 时，默认已开启分区并行且后端为 `process`；若需与旧行为一致可设 `parallel_regions=False` 或 `parallel_backend="thread"`。）

**其它环境变量**在 CMD 用 `set KEY=value`，不要用 PowerShell 的 `$env:`。常与并行一起用的是 **`RBCV_FAST=1`**（进一步省时间，可能降质量）。

**提速（按需）：** `pip install -r requirements.txt` 已包含 `numba`（单点可见性 JIT，与分区并行互不替代）。其它：`RBCV_FAST=1`、`RBCV_N_RAYS=96`（可能漏细缝）、`COVERAGE_PLANNER_USE_NUMBA=0` 关闭 JIT；`RBCV_HEX_FIRST=0` 回退旧流水线；要更省圆、更慢见 §6 `RBCV_BEST`。**分区内部 ILS** 可用 `RBCV_REGION_ILS_TIME_LIMIT`（秒，如 `120`）避免单个大分区跑满迭代；**全局精修**用 `RBCV_TRIM_TIME_LIMIT`（二者不同：前者管各区内精修，后者管合并后的整图 ILS）。未设环境变量时，`demo_regions.py` 默认分区内 **180s**、全局精修 **600s**（见脚本顶部常量）。

**产出：** 终端汇总 + **`scripts/分区覆盖结果.png`**（可随时重跑，已 `.gitignore`）。

**可选：** `py -m pytest tests -q`。算法详情见 `src/rbcv_disk_coverage/docs/ALGORITHM.md`（本项目纯 CPU）。

---

## 5. 程序内调用（接入下游 Python）

```python
from coverage_planner import (
    load_ros_map, load_regions_json, rasterize_regions,
    PlannerConfig, plan_partitions,
)
from coverage_planner.map_io import meters_to_pixels

# 路径与示例脚本一致：工作目录为 src/rbcv_disk_coverage 时用 ../map_tools/maps/
# load_ros_map 默认已做 clean_map（闭合弥缝、抹黑封闭中空区等）；需原始栅格时传 apply_clean_map=False
free, info = load_ros_map("../map_tools/maps/map7.yaml")
spec = load_regions_json("../map_tools/maps/my_regions.json")
labels = rasterize_regions(spec, info, free.shape)

partition = plan_partitions(
    free_full=free,
    labels=labels,
    r_px=meters_to_pixels(2.5, info.resolution),
    base_config=PlannerConfig(
        overlap_tiebreak=True,
        ils_max_iter=40,
    ),
    resolution_m_per_px=float(info.resolution),
    verbose=True,
)

print("总圆数:", partition.total_circles)
pts_px = partition.all_points()      # [N, 2] 像素 (row, col)
```

像素 → 米（与 ROS 一致）：

```python
ox, oy, _ = info.origin
res = info.resolution
mx = ox + (pts_px[:, 1] + 0.5) * res
my = oy + (pts_px[:, 0] + 0.5) * res
```

`partition.region_plans[i]` 内含 `region_id / shape / config / result`，可单独取每个分区的结果。其中 **`shape.kind`** 含 **`rect_room`**、`corridor`、`narrow`、`mixed`、`tiny`；**走廊划条摘要**见 **`shape.corridor_plan_meta`**。

---

## 6. 调参与排错

### 6.1 一键“尽量最优”（更慢但更省圆）

`demo_regions.py` 支持 `RBCV_BEST=1`，自动启用更激进的全局精修（对 **所有分区** 生效）。**工作目录仍为 `src/rbcv_disk_coverage`。**

PowerShell：

```powershell
$env:RBCV_BEST="1"
py scripts\demo_regions.py ..\map_tools\maps\map7.yaml 2.5 ..\map_tools\maps\my_regions.json
```

CMD：

```cmd
set RBCV_BEST=1
py scripts\demo_regions.py ..\map_tools\maps\map7.yaml 2.5 ..\map_tools\maps\my_regions.json
```

常用精修参数（可选，覆盖 `RBCV_BEST` 默认值）：

- `RBCV_TRIM_MODE`: `ils` / `regreedy`（最激进）
- `RBCV_SOFT_FACTOR`: 软化贴墙/小缝隙像素（如 `0.10~0.15`）
- `RBCV_TRIM_TIME_LIMIT`: 全局精修时间上限（秒；`0` 表示不设上限）

| 现象 | 处理 |
|------|------|
| 某分区报 `px_in_rect∩free = 0` | 矩形落在障碍/地图外，回到 **第 2 步** 重画或调坐标 |
| 方正区域结果还偏密 | 把 `PlannerConfig.overlap_tiebreak=True`（默认开）；或减小 `ils_max_iter` 缩短运行 |
| 窄通道断开/不足 | `PlannerConfig(min_corridor=2.0)` 调大；或减小 `alpha`（`candidates._step_length`） |
| 两个相邻矩形交界出现密集圆 | 把交界处归并为一个矩形；或在 `coverage_regions_ros` 里通过 region 列表顺序控制归属 |
| 改了 yaml 后规划坐标不对 | 检查 `yaml` 中 **`origin`** 与实际地图原点一致，`resolution` 单位为米/像素 |

---

## 6. 语义探索回路（`exploration`：rosbag + 视觉 + 导航闭环）

**`src/exploration/`** 负责在**检测框 + 覆盖站位**上统计目标物、给框赋权、把整块工作区切成**均匀矩形格**合成权重，再生成**闭合探索路径**（起点=终点，奖励–代价近似 + 低权重格随机探索 bonus）。与 **§2–§4** 的几何圆盘覆盖互补：后者解决「看穿障碍到哪看」，前者解决「调查完后按语义再逛一遍」。

### 6.1 数据准备

1. **`survey_zones.json`**（示例：`exploration/data/survey_zones.example.json`）  
   - 每个 `zone`：`bbox_xyxy` / `polygon_xy`、`base_weight_hint`（可选）。  
   - **`coverage`**：由 RBCV **`demo_regions.py`** 得到的圆心填入 `sites_m`，`radius_m` 与规划半径一致；若无圆心可省略 `sites_m`，表示**整个人框内**都算可调查区域。  

2. **视觉输出 `detections.jsonl`**（示例：`exploration/data/detections.example.jsonl`）  
   - 每行 JSON：`t_s`、`class_name`、`x`、`y`（目标在**地图/世界系**下的代表点）、可选 `track_id`（**有则优先用于去重**）、`confidence`。  

3. **ROS1 bag** 中与 `t_s` **同一坐标系**的里程计话题（如 `/odom`），先用：  
   `py exploration\scripts\inspect_rosbag1.py <bag>` 查 topic。

### 6.2 一键生成导航计划 JSON

在 **`src/`** 下：

```powershell
py exploration\scripts\run_semantic_stack.py `
  --survey exploration\data\survey_zones.example.json `
  --bag D:\path\to\record.bag --topic /odom `
  --detections exploration\data\detections.example.jsonl `
  --out exploration\data\out_semantic_plan.json
```

- **仅自检**（无 rosbag）：省略 `--bag` / `--topic` / `--detections`，可用 `--priors-json` 给指定 `zone_id` 初始权重。  
- **矩形格上额外加权**（非检测框区域）：`--rect-priors-json` 传入轴对齐矩形列表，见脚本 `--help` 说明。

**分区 JSON → 检测框**：`exploration.regions_bridge.load_zones_from_regions_json`（再手工补上 `coverage` 段更佳）。

目录索引：**`src/WORKSPACE_LAYOUT.md`**；模块说明：**`src/exploration/README.md`**。

---

## 7. ROS1（后续步骤，可暂时忽略）

`coverage_regions_ros` 与 `map_tools` 已写好 catkin 包结构，未来在 Ubuntu Noetic 工作空间里：

```bash
catkin build && source devel/setup.bash
roslaunch map_tools publish_static_map.launch
roslaunch coverage_regions_ros publish_partition_labels.launch \
    regions_json:=$(rospack find map_tools)/maps/my_regions.json
```

会发布两个话题：

- `/map`（`nav_msgs/OccupancyGrid`）：原栅格地图
- `/region_partition_labels`（`OccupancyGrid`）：分区标签栅格（`-1` = 区外，`1..K` = 第 k 个分区）

下游 Python 节点订阅 `/region_partition_labels` 即可与本指南的离线流程对齐。

---

## 文件速查（相对仓库根）

| 用途 | 路径 |
|------|------|
| 地图 YAML/PGM、分区 JSON | `src/map_tools/maps/` |
| 分区解析（Python 模块） | `src/rbcv_disk_coverage/coverage_planner/region_io.py` |
| 分区自适应规划（形状分类、走廊划条、`plan_partitions`） | `src/rbcv_disk_coverage/coverage_planner/region_planner.py`、`corridor_split.py` |
| 交互划分脚本 | `src/rbcv_disk_coverage/scripts/draw_regions.py` |
| 分区预览脚本 | `src/rbcv_disk_coverage/scripts/preview_regions.py` |
| 端到端覆盖脚本 | `src/rbcv_disk_coverage/scripts/demo_regions.py` |
| 语义探索示例 / rosbag 工具 | `src/exploration/scripts/`、`src/exploration/README.md` |

更细的算法说明见 `src/rbcv_disk_coverage/docs/ALGORITHM.md`。
