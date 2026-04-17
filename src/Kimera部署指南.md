# Kimera 语义 SLAM 部署指南（Ubuntu 20.04 + ROS1 Noetic）

> 目标：在您的机器人（底盘 3D 激光雷达 + 顶部双目相机）上，跑通 MIT-SPARK 开源的 Kimera 语义 SLAM 框架。
> 方案：**方案 A — 直接编译并运行官方源码**。
> 预计耗时：依赖安装 + 编译约半天～1 天，EuRoC Demo 跑通约半小时，对接自己相机约 1~2 天（含标定）。

---

## 目录

- [一、Kimera 各子仓库代码链接与介绍](#一kimera-各子仓库代码链接与介绍)
- [二、系统与硬件要求](#二系统与硬件要求)
- [三、依赖安装（按顺序执行）](#三依赖安装按顺序执行)
- [四、创建 catkin 工作空间并拉取源码](#四创建-catkin-工作空间并拉取源码)
- [五、编译](#五编译)
- [六、用 EuRoC 数据集跑通 Demo](#六用-euroc-数据集跑通-demo)
- [七、对接您自己的双目相机 + 激光雷达](#七对接您自己的双目相机--激光雷达)
- [八、常见踩坑与解决办法](#八常见踩坑与解决办法)
- [九、参考资料](#九参考资料)

---

## 一、Kimera 各子仓库代码链接与介绍

`MIT-SPARK/Kimera` 主仓库本身只是一个"导航页"（README + 文档），**真正的代码分布在下列子仓库里**。部署时我们需要把它们全部 clone 到同一个 catkin workspace 的 `src/` 下。

### 1. 核心定位建图模块

| 子模块 | 仓库地址 | 作用 | 关键依赖 |
|---|---|---|---|
| **Kimera-VIO** | <https://github.com/MIT-SPARK/Kimera-VIO> | 双目视觉-惯性里程计（VIO），Kimera 的定位核心 | GTSAM、OpenCV、OpenGV、DBoW2 |
| **Kimera-VIO-ROS** | <https://github.com/MIT-SPARK/Kimera-VIO-ROS> | Kimera-VIO 的 ROS 封装，订阅相机/IMU 话题 | Kimera-VIO、cv_bridge |
| **Kimera-RPGO** | <https://github.com/MIT-SPARK/Kimera-RPGO> | 鲁棒位姿图优化（Robust Pose Graph Optimization），用 PCM 算法剔除错误回环 | GTSAM |
| **Kimera-PGMO** | <https://github.com/MIT-SPARK/Kimera-PGMO> | Pose-Graph 与 Mesh 联合优化，回环时同步变形已建好的 mesh | GTSAM、Kimera-RPGO |

### 2. 语义建图模块

| 子模块 | 仓库地址 | 作用 | 关键依赖 |
|---|---|---|---|
| **Kimera-Semantics** | <https://github.com/MIT-SPARK/Kimera-Semantics> | 基于 Voxblox 的 3D 语义 TSDF 体素建图，支持贝叶斯语义融合 | Voxblox、PCL |
| **Voxblox** | <https://github.com/ethz-asl/voxblox> | ETH 开源的 TSDF 体素库，Kimera-Semantics 的底层 | protobuf、Eigen |
| **Kimera-Mesher** | 已合并进 Kimera-VIO | 实时三角网格重建 | — |

### 3. 多机器人 / 新一代方案

| 子模块 | 仓库地址 | 作用 |
|---|---|---|
| **Kimera-Multi** | <https://github.com/MIT-SPARK/Kimera-Multi> | 多机器人分布式协同 SLAM |
| **Hydra** | <https://github.com/MIT-SPARK/Hydra> | Kimera 的"继任者"，实时 3D 场景图（推荐关注） |

### 4. 官方数据集与工具

| 资源 | 链接 | 说明 |
|---|---|---|
| Kimera 主仓库 | <https://github.com/MIT-SPARK/Kimera> | 文档入口、论文列表 |
| uHumans2 Dataset | <https://web.mit.edu/sparklab/datasets/uHumans2/> | 官方语义 SLAM 评测数据集（Unity 仿真） |
| EuRoC MAV Dataset | <https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets> | Kimera-VIO 的标配测试集（双目 + IMU） |
| 论文 | <https://arxiv.org/abs/1910.02490> | ICRA 2020 Kimera 原始论文 |

---

## 二、系统与硬件要求

### 软件

| 项目 | 推荐版本 |
|---|---|
| 操作系统 | Ubuntu **20.04 LTS**（Focal） |
| ROS | **ROS1 Noetic** |
| 编译器 | GCC 9 / C++17 |
| CMake | ≥ 3.15 |
| Python | 3.8（Noetic 自带） |

### 硬件建议

- CPU：Intel i5 及以上（VIO 对单核性能敏感）
- 内存：≥ 8 GB（跑大场景建议 16 GB）
- GPU：**非必需**（Kimera-VIO 是纯 CPU 的，但做语义分割时需要）

### 传感器要求

- **双目相机**：已做过内参 + 外参标定，且**硬件同步或软件近似同步**；典型型号：RealSense D435i、ZED 2、Intel T265（不推荐，已停产）、MYNT EYE 等
- **IMU**：频率 ≥ 100 Hz，与相机时间对齐；一般和相机集成在同一模组最省事
- **3D 激光雷达**：Kimera 本体**不直接使用 LiDAR**，LiDAR 走另一套 SLAM（如 FAST-LIO2），两者最终在地图层融合，见第七节

---

## 三、依赖安装（按顺序执行）

以下命令全部在一个新开的终端中按顺序运行。

### 3.1 基础工具链

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git wget unzip pkg-config \
    python3-pip python3-dev python3-wstool python3-catkin-tools \
    libboost-all-dev libeigen3-dev libtbb-dev \
    libgoogle-glog-dev libgflags-dev \
    libmetis-dev
```

### 3.2 ROS Noetic（如果未安装）

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y \
    ros-noetic-cv-bridge ros-noetic-image-transport \
    ros-noetic-tf2-ros ros-noetic-pcl-ros \
    ros-noetic-rviz ros-noetic-rqt-image-view
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.3 OpenCV（Noetic 自带 4.2，够用）

```bash
sudo apt install -y libopencv-dev python3-opencv
```

> 若后续报错提示需要 OpenCV contrib，可手动编译 4.5.x + contrib。

### 3.4 GTSAM（Kimera 的核心依赖，**必须用指定分支**）

```bash
cd ~
git clone https://github.com/borglab/gtsam.git
cd gtsam
# Kimera-VIO 官方 README 推荐该 commit（2023 年前后版本）
git checkout 4.2a7
mkdir build && cd build
cmake .. \
    -DGTSAM_USE_SYSTEM_EIGEN=ON \
    -DGTSAM_POSE3_EXPMAP=ON \
    -DGTSAM_ROT3_EXPMAP=ON \
    -DGTSAM_TANGENT_PREINTEGRATION=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```

> ⚠️ 重要：`GTSAM_TANGENT_PREINTEGRATION=OFF` 必须设，否则 Kimera-VIO 运行时会因为预积分协方差计算方式不匹配而数值发散。

### 3.5 OpenGV

```bash
cd ~
git clone https://github.com/laurentkneip/opengv.git
cd opengv
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DEIGEN_INCLUDE_DIR=/usr/include/eigen3
make -j$(nproc)
sudo make install
sudo ldconfig
```

### 3.6 DBoW2（Kimera 自带 fork）

无需单独装，后面会随 Kimera-VIO 的 `thirdparty/` 一起编译。

### 3.7 protobuf（Voxblox 需要）

```bash
sudo apt install -y libprotobuf-dev protobuf-compiler
```

---

## 四、创建 catkin 工作空间并拉取源码

```bash
mkdir -p ~/kimera_ws/src
cd ~/kimera_ws
catkin config --extend /opt/ros/noetic \
              --cmake-args -DCMAKE_BUILD_TYPE=Release \
              -DGTSAM_TANGENT_PREINTEGRATION=OFF
cd src

# ── Kimera 主线 ──
git clone https://github.com/MIT-SPARK/Kimera-VIO.git
git clone https://github.com/MIT-SPARK/Kimera-VIO-ROS.git
git clone https://github.com/MIT-SPARK/Kimera-RPGO.git
git clone https://github.com/MIT-SPARK/Kimera-PGMO.git
git clone https://github.com/MIT-SPARK/Kimera-Semantics.git

# ── 第三方依赖 ──
git clone https://github.com/ethz-asl/voxblox.git
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/ethz-asl/gflags_catkin.git
git clone https://github.com/ethz-asl/glog_catkin.git
git clone https://github.com/ethz-asl/protobuf_catkin.git
git clone https://github.com/ethz-asl/minkindr.git
git clone https://github.com/ethz-asl/minkindr_ros.git

# 可选：想跑多机器人协同版本
# git clone https://github.com/MIT-SPARK/Kimera-Multi.git

cd ~/kimera_ws
```

> 💡 也可以用官方 `install/kimera_vio_ros_https.rosinstall` 一键拉取：
> ```bash
> cd ~/kimera_ws/src
> wstool init
> wstool merge Kimera-VIO-ROS/install/kimera_vio_ros_https.rosinstall
> wstool update -j8
> ```

---

## 五、编译

```bash
cd ~/kimera_ws
catkin build -j$(nproc)
# 若内存不足，改为：catkin build -j2
```

首次编译约 **15~40 分钟**。编译完成后：

```bash
echo "source ~/kimera_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 六、用 EuRoC 数据集跑通 Demo

### 6.1 下载 EuRoC（推荐 MH_01_easy）

```bash
mkdir -p ~/datasets/euroc && cd ~/datasets/euroc
wget http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/machine_hall/MH_01_easy/MH_01_easy.bag
```

### 6.2 启动 Kimera-VIO-ROS

```bash
# 终端 1
roscore

# 终端 2：启动 Kimera-VIO-ROS（EuRoC 配置）
roslaunch kimera_vio_ros kimera_vio_ros_euroc.launch

# 终端 3：RViz 可视化（launch 会自动启动，可跳过）
rviz -d ~/kimera_ws/src/Kimera-VIO-ROS/rviz/kimera_vio_euroc.rviz

# 终端 4：回放数据包
rosbag play ~/datasets/euroc/MH_01_easy.bag --clock
```

看到 RViz 里出现相机轨迹 + 稀疏点云 + 实时 mesh 即为成功。

### 6.3 加上 Kimera-Semantics 跑语义建图

EuRoC 本身没有语义标签，可以用 **uHumans2 数据集**（Unity 仿真，自带 2D 语义图）：

```bash
# 下载 uHumans2 官方 rosbag（见 https://web.mit.edu/sparklab/datasets/uHumans2/）
roslaunch kimera_semantics_ros kimera_semantics.launch
rosbag play uHumans2_xxx.bag --clock
```

---

## 七、对接您自己的双目相机 + 激光雷达

Kimera 本体只用**双目 + IMU**，激光雷达走**独立的 LiDAR SLAM** 再与 Kimera 融合。

### 7.1 双目相机接入（以 RealSense D435i 为例）

```bash
sudo apt install -y ros-noetic-realsense2-camera ros-noetic-realsense2-description
roslaunch realsense2_camera rs_camera.launch \
    enable_infra1:=true enable_infra2:=true \
    enable_gyro:=true enable_accel:=true \
    unite_imu_method:=linear_interpolation \
    enable_sync:=true
```

发布的关键话题：
- 左/右红外图：`/camera/infra1/image_rect_raw`、`/camera/infra2/image_rect_raw`
- IMU：`/camera/imu`
- 相机信息：`/camera/infra1/camera_info`、`/camera/infra2/camera_info`

### 7.2 相机标定（关键步骤，决定 VIO 精度）

使用 **Kalibr** 做双目 + IMU 联合标定：

```bash
# 安装 Kalibr（推荐 Docker 方式，避免环境冲突）
docker pull stereolabs/kalibr:latest
# 具体步骤见 https://github.com/ethz-asl/kalibr/wiki
```

标定输出的 YAML 文件要转成 Kimera 的格式，放到：

```
~/kimera_ws/src/Kimera-VIO-ROS/param/<your_robot>/
    ├── LeftCameraParams.yaml
    ├── RightCameraParams.yaml
    └── ImuParams.yaml
```

### 7.3 改造 launch 文件

复制一份 EuRoC 的 launch 当模板：

```bash
cp ~/kimera_ws/src/Kimera-VIO-ROS/launch/kimera_vio_ros_euroc.launch \
   ~/kimera_ws/src/Kimera-VIO-ROS/launch/kimera_vio_ros_myrobot.launch
```

修改关键参数：

```xml
<arg name="dataset_name" value="MyRobot"/>
<arg name="left_cam_topic"        default="/camera/infra1/image_rect_raw"/>
<arg name="right_cam_topic"       default="/camera/infra2/image_rect_raw"/>
<arg name="left_cam_info_topic"   default="/camera/infra1/camera_info"/>
<arg name="right_cam_info_topic"  default="/camera/infra2/camera_info"/>
<arg name="imu_topic"             default="/camera/imu"/>
```

### 7.4 激光雷达并行融合方案

推荐架构：

```
┌───────────────┐   双目+IMU   ┌─────────────┐
│ 双目相机 + IMU │───────────▶ │  Kimera-VIO │──┐
└───────────────┘              └─────────────┘  │
                                                ▼
                                        ┌────────────────┐
                                        │   TF 融合 /    │
                                        │  地图层拼接    │
                                        └────────────────┘
                                                ▲
┌──────────────┐   点云       ┌─────────────┐    │
│  3D 激光雷达  │───────────▶│  FAST-LIO2  │────┘
└──────────────┘              └─────────────┘
```

两种落地方式：
1. **LiDAR 做主定位，相机只做语义**：用 FAST-LIO2 输出机器人位姿 → Kimera-Semantics 用这个位姿给语义体素上色（需改 Kimera-Semantics 的 pose source）。
2. **两个 SLAM 并行，各管一张图**：LiDAR SLAM 出点云地图用于导航，Kimera 出语义 mesh 用于场景理解，通过 TF 对齐。

---

## 八、常见踩坑与解决办法

| 现象 | 原因 | 解决 |
|---|---|---|
| 编译 Kimera-VIO 时报 `gtsam` 找不到 | GTSAM 版本不对 / 没 ldconfig | 重装 GTSAM 4.2a7，运行 `sudo ldconfig` |
| VIO 刚启动就飘到无穷远 | `GTSAM_TANGENT_PREINTEGRATION=ON` 忘了关 | 重编 GTSAM，关掉此项 |
| `undefined reference to cv::...` | OpenCV 多版本冲突 | 确保只链接 Noetic 自带的 OpenCV 4.2 |
| IMU 和图像时间戳不同步 | 驱动没开 `enable_sync` | 检查相机驱动；或用 `message_filters::ApproximateTime` |
| 回环后 mesh 没更新 | 没启用 Kimera-PGMO | 在 launch 中设 `use_pgmo:=true` |
| RViz 里看不到语义颜色 | 语义图话题没订阅对 | 检查 `semantic_label_2_color.csv` 与 2D 分割话题是否匹配 |
| 编译 OOM 卡死 | 并行度太高 | 改 `catkin build -j2` 或 `-j1` |

---

## 九、参考资料

- Kimera 论文（ICRA 2020）: <https://arxiv.org/abs/1910.02490>
- Kimera-Semantics 论文: <https://arxiv.org/abs/1902.03288>
- Kimera-VIO 官方文档: <https://github.com/MIT-SPARK/Kimera-VIO/blob/master/docs/kimera_vio_install.md>
- Kimera-VIO-ROS 快速入门: <https://github.com/MIT-SPARK/Kimera-VIO-ROS#installation>
- GTSAM 文档: <https://gtsam.org/>
- Voxblox 文档: <https://voxblox.readthedocs.io/>
- Kalibr 标定教程: <https://github.com/ethz-asl/kalibr/wiki>
- FAST-LIO2（激光 SLAM 配合项）: <https://github.com/hku-mars/FAST_LIO>

---

## 十、推荐的下一步

1. **先跑通 EuRoC Demo**（第六节），确认环境无误。
2. **标定您自己的双目 + IMU**（第七节 7.2），这是精度的根本。
3. **接入真实相机跑室内数据**，先录 rosbag 离线调参，再上线。
4. **并行跑 FAST-LIO2**，把 LiDAR 位姿和地图融合进来。
5. 如果追求更新的技术栈，可以考虑直接上 **Hydra**（MIT-SPARK 的新一代 3D 场景图）。

---

> 本文档仅列出最常用路径。如果遇到文档未覆盖的问题，请结合报错信息查阅对应子仓库的 GitHub Issues。
