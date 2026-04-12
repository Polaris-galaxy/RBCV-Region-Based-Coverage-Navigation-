# ROS 2 终端指令参考

> 本文档根据本机环境 **`ros2 --help` 及各子命令 `--help`** 整理（ROS 2 **Humble**）。  
> 其他发行版可能多出或缺少个别子命令；以你机器上 `ros2` 输出为准。  
> **通用用法**：`ros2 -h` 查看一级命令；`ros2 <命令> -h` 查看该命令说明；`ros2 <命令> <子命令> -h` 查看更细子命令。

---

## 使用前：加载环境

在已安装 ROS 2 的终端中先执行（路径按你的安装调整）：

```bash
source /opt/ros/<发行版名>/setup.bash
```

若使用工作空间 overlay，还需 `source ~/your_ws/install/setup.bash`。

---

## 顶层：`ros2`

```text
usage: ros2 [-h] [--use-python-default-buffering]
            Call `ros2 <command> -h` for more detailed usage. ...

options:
  -h, --help            show this help message and exit
  --use-python-default-buffering
                        Do not force line buffering in stdout and instead use
                        the python default buffering, which might be affected
                        by PYTHONUNBUFFERED/-u and depends on whatever stdout
                        is interactive or not
```

**说明**：`ros2` 是可扩展的命令行入口；具体功能见下列子命令。

---

## `ros2 run` — 运行节点可执行文件

```text
usage: ros2 run [-h] [--prefix PREFIX] package_name executable_name ...

positional arguments:
  package_name     Name of the ROS package
  executable_name  Name of the executable
  argv             Pass arbitrary arguments to the executable

options:
  -h, --help       show this help message and exit
  --prefix PREFIX  Prefix command, which should go before the executable.
                   (e.g. --prefix 'gdb -ex run --args').
```

**示例**：

```bash
ros2 demo_nodes_cpp talker
ros2 demo_nodes_cpp listener
```

---

## `ros2 launch` — 启动 Launch 文件

```text
usage: ros2 launch [-h] [-n] [-d] [-p | -s] [-a]
                   [--launch-prefix LAUNCH_PREFIX]
                   [--launch-prefix-filter LAUNCH_PREFIX_FILTER]
                   package_name [launch_file_name] [launch_arguments ...]

positional arguments:
  package_name          Name of the ROS package which contains the launch file
  launch_file_name      Name of the launch file
  launch_arguments      Arguments to the launch file; '<name>:=<value>' (for
                        duplicates, last one wins)

options:
  -h, --help            show this help message and exit
  -n, --noninteractive  Run non-interactively, with no terminal associated
  -d, --debug           Debug mode, more verbose output
  -p, --print, --print-description
                        Print the launch description without launching
  -s, --show-args, --show-arguments
                        Show arguments that may be given to the launch file
  -a, --show-all-subprocesses-output
                        Show all launched subprocesses' output
  --launch-prefix LAUNCH_PREFIX
                        Prefix before all executables (quote if spaces)
  --launch-prefix-filter LAUNCH_PREFIX_FILTER
                        Regex to filter which executables get --launch-prefix
```

**示例**：

```bash
ros2 my_pkg my_launch.py
ros2 my_pkg my_launch.py use_sim_time:=true
```

---

## `ros2 topic` — 话题

**一级子命令**：`list` · `echo` · `pub` · `hz` · `bw` · `info` · `type` · `find` · `delay`

部分命令支持：`--spin-time` · `-s/--use-sim-time` · `--no-daemon` · `--include-hidden-topics`（在 `topic` 组或子命令上）。

### `ros2 topic list`

列出话题。常用：`-t/--show-types` 显示类型；`-c/--count-topics` 只显示数量；`-v/--verbose` 详细信息。

### `ros2 topic echo`

订阅并打印消息。常用：`--qos-profile` · `--qos-depth` · `--field` · `--once` · `--csv` · `--filter`（Python 表达式，消息变量为 `m`）。

### `ros2 topic pub`

向话题发布。常用：`-r/--rate` 发布频率；`-1/--once` 发一条退出；`-t/--times` 发 N 次；`-w/--wait-matching-subscriptions`；各类 `--qos-*`。

### `ros2 topic hz`

统计发布频率。常用：`--window` · `--filter` · `--wall-time`。

### `ros2 topic bw`

统计话题带宽。常用：`--window`。

### `ros2 topic info`

话题信息。`-v/--verbose` 含节点名、GUID、QoS 等。

### `ros2 topic type`

查询话题消息类型。

### `ros2 topic find`

按消息类型查找话题。`-c` 只计数。

### `ros2 topic delay`

根据消息 `header` 时间戳估计延迟。`--window`。

---

## `ros2 service` — 服务

**一级子命令**：`list` · `call` · `find` · `type`

组级选项：`--include-hidden-services`。

### `ros2 service list`

`-t/--show-types` · `-c/--count-services`。

### `ros2 service call`

```text
usage: ros2 service call [-h] [-r N] service_name service_type [values]
```

`values` 为 YAML 格式请求字段；`-r` 按 Hz 重复调用。

### `ros2 service find`

按服务类型列出服务名。`-c` 只计数。

### `ros2 service type`

查询某服务名的类型。

---

## `ros2 action` — 动作

**一级子命令**：`list` · `info` · `send_goal`

### `ros2 action list`

`-t/--show-types` · `-c/--count-actions`。

### `ros2 action info`

`-t` · `-c`（只统计 client/server 数量）。

### `ros2 action send_goal`

```text
usage: ros2 action send_goal [-h] [-f] action_name action_type goal
```

`goal` 为 YAML；`-f/--feedback` 打印反馈。

---

## `ros2 node` — 节点

**一级子命令**：`list` · `info`

### `ros2 node list`

`-a/--all` 含隐藏节点；`-c/--count-nodes`。

### `ros2 node info`

`node_name`；`--include-hidden` 显示隐藏 topic/service/action。

---

## `ros2 param` — 参数

**一级子命令**：`list` · `get` · `set` · `dump` · `load` · `delete` · `describe`

### `ros2 param list`

可选 `node_name`；`--filter` 正则；`--param-prefixes`；`--param-type`。

### `ros2 param get` / `set` / `delete`

`node_name` + `parameter_name`（`set` 另有 `value`）。

### `ros2 param dump`

YAML 格式输出节点全部参数；`--output-dir` 等为弃用选项。

### `ros2 param load`

从 YAML 加载；`--no-use-wildcard` 不把 `/**` 命名空间参数加载进节点。

### `ros2 param describe`

查看已声明参数的说明信息。

---

## `ros2 pkg` — 软件包

**一级子命令**：`list` · `prefix` · `executables` · `create` · `xml`

### `ros2 pkg prefix`

`--share` 显示 share 目录。

### `ros2 pkg executables`

可选 `package_name`；`--full-path`。

### `ros2 pkg create`

创建新包：可选 `--build-type`（`cmake` / `ament_cmake` / `ament_python`）、`--dependencies`、`--node-name`、`--license` 等。

### `ros2 pkg xml`

`-t/--tag` 输出 `package.xml` 中指定标签。

---

## `ros2 interface` — 消息 / 服务 / 动作接口

**一级子命令**：`list` · `show` · `package` · `packages` · `proto`

### `ros2 interface list`

`-m/--only-msgs` · `-s/--only-srvs` · `-a/--only-actions`。

### `ros2 interface show`

显示 `.msg`/`.srv`/`.action` 定义；`--all-comments` / `--no-comments`；类型名可用 `-` 从 stdin 读。

### `ros2 interface packages`

列出提供接口的包（同样可用 `-m/-s/-a` 过滤）。

### `ros2 interface proto`

输出接口“原型”示例；`--no-quotes`。

---

## `ros2 bag` — 录制与回放（rosbag2）

**一级子命令**：`record` · `play` · `info` · `convert` · `list` · `reindex`

### `ros2 bag record`

常用：`-a/--all` 录全部；`-o` 输出目录；`-e/-x` 正则包含/排除话题；`--compression-*`；`--max-bag-size` / `--max-bag-duration` 分卷；`--use-sim-time` 等。详见 `ros2 bag record -h`。

### `ros2 bag play`

常用：`-r` 倍速；`-l` 循环；`--topics` 子集；`--remap`；`--clock` 发布 `/clock`；`-d` 延迟开始；`--start-offset`。详见 `ros2 bag play -h`。

### `ros2 bag info`

查看 bag 元数据与话题统计。

### `ros2 bag convert`

`-i` 输入 URI（可多次）；`-o` 输出选项 YAML（含 `output_bags`）。

### `ros2 bag list`

列出插件：`storage` · `converter` · `compressor` · `decompressor`；`--verbose`。

### `ros2 bag reindex`

重建 bag 元数据文件。

---

## `ros2 component` — 组件（Composable Nodes）

**一级子命令**：`list` · `load` · `unload` · `standalone` · `types`

### `ros2 component load`

向容器节点加载组件：`container_node_name` · `package_name` · `plugin_name`；支持 `-n` 节点名、`--remap-rule`、`--parameter` 等。

### `ros2 component unload`

`container_node_name` + 一个或多个 `component_uid`。

### `ros2 component standalone`

在独立容器运行某组件；`-c` 容器节点名。

### `ros2 component types`

列出 ament 索引中注册的组件类型；可选 `package_name`。

---

## `ros2 daemon` — CLI 守护进程

**一级子命令**：`start` · `stop` · `status`

用于加速 `ros2` 部分命令的图发现等；`start` 支持 `--debug`。

---

## `ros2 doctor` / `ros2 wtf` — 环境诊断

`wtf` 为 `doctor` 的别名。

**组选项**：`--report` · `--report-failed` · `--include-warnings`

**子命令**：`hello` — 多机网络连通性（话题名、周期、`--ttl`、`-1/--once` 等）。

---

## `ros2 lifecycle` — 生命周期节点

**一级子命令**：`nodes` · `get` · `set` · `list`

- **`nodes`**：列出具备 lifecycle 的节点（`-a` · `-c`）。
- **`get`**：查询状态；可省略 `node_name` 表示全部节点。
- **`set`**：`node_name` + `transition` 触发迁移。
- **`list`**：列出某节点可用迁移；`-a` 显示全部迁移。

---

## `ros2 multicast` — 组播探测

**一级子命令**：`send` · `receive`

用于 DDS/网络组播相关简单收发测试；`send` 有 `--ttl`。

---

## `ros2 security` — 安全（SROS2）

**一级子命令**（节选）：`create_keystore` · `create_enclave` · `create_permission` · `generate_artifacts` · `generate_policy` · `list_enclaves`

另有弃用别名：`create_key` → 请用 `create_enclave`；`list_keys` → `list_enclaves`。

- **`generate_policy`**：从当前 ROS 图生成策略 XML；支持 `--spin-time` 等。
- **`generate_artifacts`**：`-k` keystore 根路径，`-e` enclave 列表，`-p` 策略文件列表。

完整参数见：`ros2 security <子命令> -h`。

---

## 一级命令总览（本机 `ros2 --help`）

| 命令 | 说明 |
|------|------|
| `action` | 动作相关 |
| `bag` | 录制/回放 bag |
| `component` | 组件化节点 |
| `daemon` | CLI 守护进程 |
| `doctor` | 检查 ROS 安装与常见问题 |
| `interface` | 接口（msg/srv/action）信息 |
| `launch` | 运行 launch 文件 |
| `lifecycle` | 生命周期节点 |
| `multicast` | UDP 组播收发 |
| `node` | 节点信息 |
| `param` | 参数 |
| `pkg` | 软件包 |
| `run` | 运行可执行文件 |
| `security` | 安全相关 |
| `service` | 服务 |
| `topic` | 话题 |
| `wtf` | 等同于 `doctor` |

---

## 常用组合示例

```bash
# 话题
ros2 topic list -t
ros2 topic echo /chatter --once
ros2 topic pub /chatter std_msgs/msg/String "data: 'hello'" -1

# 服务
ros2 service list -t
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"

# 动作
ros2 action send_goal /fibonacci example_interfaces/action/Fibonacci "{order: 5}" -f

# 参数
ros2 param list
ros2 param get /talker qos_overrides

# 接口定义
ros2 interface show std_msgs/msg/String

# 录制
ros2 bag record -a -o my_bag
ros2 bag play my_bag
```

---

## 获取最新完整帮助

任意子命令的完整参数以终端为准：

```bash
ros2 <命令> -h
ros2 <命令> <子命令> -h
```

例如：`ros2 topic echo -h`、`ros2 bag record -h`。
