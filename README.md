# lidar_nav2_ws

##### 基于 Livox MID-360 3D LiDAR 的 ROS 2 自主导航工作空间，集成 LIO 里程计与 Nav2 导航框架，支持 Gazebo 仿真与实机部署。

---

## 系统架构

四轮滑移转向机器人 + Livox MID-360 3D LiDAR + IMU，通过 FAST-LIO 或 Point-LIO 获取 3D 里程计，经坐标系桥接和 3D→2D 转换后接入 Nav2 导航栈。

**TF 坐标系链：**
```
map → odom → base_footprint → chassis → livox_frame
```

**核心数据流：**
```
LiDAR/IMU
  → LIO 里程计 (FAST-LIO / Point-LIO)
  → lio_interface (TF 桥接: LIO 内部坐标系 → odom)
  → sensor_scan_generation (odom → base_footprint TF，发布 /odom)
  → pointcloud_to_laserscan (3D → 2D /scan)
  → 重定位 (map → odom TF)
  → Nav2 (DWB 局部规划 + Navfn 全局规划)
```
---
## 数据流转

### 仿真模式

**仿真建图启动链：** `mapping_sim.sh`

| 终端 | 节点 | 作用 |
|------|------|------|
| 1 | gui_teleop | GUI 键盘遥控 |
| 2 | FAST-LIO | 3D LiDAR 里程计 |
| 3 | lio_interface | 坐标系桥接 |
| 4 | get_urdf | Gazebo + robot_state_publisher + RViz |
| 5 | sensor_scan_generation | TF + /odom 发布 |
| 6 | pointcloud_to_laserscan | 3D → 2D 转换 |
| 7 | SLAM Toolbox | 建图（发布 map → odom TF） |

**仿真导航启动链：** `nav2_sim.sh`

| 终端 | 节点 | 作用 |
|------|------|------|
| 1 | FAST-LIO | 3D LiDAR 里程计 |
| 2 | lio_interface | 坐标系桥接 |
| 3 | get_urdf | Gazebo + robot_state_publisher + RViz |
| 4 | sensor_scan_generation | TF + /odom 发布 |
| 5 | pointcloud_to_laserscan | 3D → 2D 转换 |
| 6 | small_gicp_relocalization | 重定位（发布 map → odom TF） |
| 7 | Nav2 | 导航栈 |

### 实机模式


**实机建图启动链：** `mapping_real.sh`

| 终端 | 节点 | 作用 |
|------|------|------|
| 1 | livox_ros_driver2 | Livox MID-360 驱动 |
| 2 | FAST-LIO | 3D LiDAR 里程计 |
| 3 | lio_interface | 坐标系桥接 |
| 4 | gld_robot_description | 实机 URDF + RViz |
| 5 | sensor_scan_generation | TF + /odom 发布 |
| 6 | pointcloud_to_laserscan | 3D → 2D 转换 |
| 7 | SLAM Toolbox | 建图（发布 map → odom TF） |

**实机导航启动链：** `nav2_real.sh`

| 终端 | 节点 | 作用 |
|------|------|------|
| 1 | livox_ros_driver2 | Livox MID-360 驱动 |
| 2 | FAST-LIO | 3D LiDAR 里程计 |
| 3 | lio_interface | 坐标系桥接 |
| 4 | gld_robot_description | 实机 URDF + RViz |
| 5 | sensor_scan_generation | TF + /odom 发布 |
| 6 | pointcloud_to_laserscan | 3D → 2D 转换 |
| 7 | small_gicp_relocalization | 重定位（发布 map → odom TF） |
| 8 | Nav2 | 导航栈 |
---
### 仿真与实机的区别

| | 仿真 | 实机 |
|---|---|---|
| LiDAR 输入 | Gazebo ray sensor (`/livox/lidar`) | livox_ros_driver2 (`/livox/lidar`) |
| 机器人模型 | `get_urdf` (simple_car.urdf) | `gld_robot_description` (gld_robot_description.urdf) |
| 点云格式转换 | 不需要（FAST-LIO）/ 需要 ign_sim_pointcloud_tool（Point-LIO） | 不需要 |
| use_sim_time | True（LIO 管线）/ False（Nav2） | False |

## 功能包简介

### 仿真相关

| 包名 | 路径 | 用途 |
|------|------|------|
| `get_urdf` | `src/get_urdf/` | Gazebo 仿真环境，包含四轮小车 URDF 模型、世界文件和 RViz 配置 |
| `livox_laser_simulation_RO2` | `src/livox_laser_simulation_RO2/` | Livox LiDAR Gazebo 仿真插件，提供 xacro 宏在 URDF 中挂载 Livox 传感器 |
| `ign_sim_pointcloud_tool` | `src/ign_sim_pointcloud_tool/` | 仿真点云格式转换，将 Gazebo 的 XYZ 点云添加 ring/time 字段转为 Velodyne 格式（Point-LIO 仿真必需） |

### 实机相关

| 包名 | 路径 | 用途 |
|------|------|------|
| `gld_robot_description` | `src/gld_robot_description/` | 实机机器人 URDF 模型，包含 Livox MID-360、RealSense D456/D405、Orbbec Gemini 深度相机 |
| `livox_ros_driver2` | `src/livox_ros_driver2/` | Livox MID-360 实机驱动（SMBU 修改版），同时发布 CustomMsg 和 PointCloud2 格式，内置 IMU |

### 里程计 (LIO)

| 包名 | 路径 | 用途 |
|------|------|------|
| `FAST_LIO` | `src/localization/FAST_LIO/` | FAST-LIO2 里程计，基于 ikd-Tree 的紧耦合 LiDAR-IMU 里程计，100Hz+ 输出 |
| `point_lio` | `src/localization/point_lio/` | Point-LIO 里程计，高带宽（4-8kHz），抗 IMU 饱和和剧烈振动 |
| `Sophus` | `src/localization/Sophus/` | 李群库（SO(2)/SO(3)/SE(2)/SE(3)），FAST-LIO 和 Point-LIO 的数学依赖 |

### 数据桥接

| 包名 | 路径 | 用途 |
|------|------|------|
| `lio_interface` | `src/lio_interface/` | LIO 里程计 → Nav2 TF 桥接，将 LIO 内部坐标系（camera_init/aft_mapped）转换到标准 odom 坐标系 |
| `sensor_scan_generation` | `src/sensor_scan_generation/` | 发布 odom → base_footprint TF 和 /odom 话题，数值微分计算速度 |

### 地图工具

| 包名 | 路径 | 用途 |
|------|------|------|
| `pcd2pgm-master` | `src/pcd2pgm-master/` | PCD 点云 → 2D 占用栅格地图（PGM），支持滤波和降噪 |

### 重定位

| 包名 | 路径 | 用途 |
|------|------|------|
| `small_gicp_relocalization` | `src/registration/small_gicp_relocalization/` | 基于 small_gicp 的重定位，发布 map → odom TF，替代 AMCL |
| `global_small_gicp_relocalization` | `src/registration/global_small_gicp_relocalization/` | 多分辨率 GICP 全局重定位 |
| `global_relocalization` | `src/registration/global_relocalization/` | 全局重定位早期原型 |
| `icp_registration` | `src/registration/icp_registration/` | PCL ICP 粗精两阶段重定位，支持 yaw 搜索 |

### 导航

| 包名 | 路径 | 用途 |
|------|------|------|
| `me_nav2_bringup` | `src/me_nav2_bringup/` | Nav2 集成启动包，包含 Nav2 参数、SLAM Toolbox 配置、3D→2D 转换参数、地图和 PCD 文件 |
| `gui_teleop` | `src/gui_teleop/` | tkinter GUI 键盘遥控，WASD 控制，支持速度调节和紧急停止 |

## 关键话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/livox/lidar` | PointCloud2 / CustomMsg | LiDAR 原始点云 |
| `/livox/imu` | Imu | LiDAR 内置 IMU |
| `/cloud_registered` | PointCloud2 | LIO 配准后的点云 |
| `/Odometry` | nav_msgs/Odometry | FAST-LIO 里程计 |
| `/aft_mapped_to_init` | nav_msgs/Odometry | Point-LIO 里程计 |
| `/registered_scan` | PointCloud2 | 转换到 odom 坐标系的点云 |
| `/registered_odometry` | nav_msgs/Odometry | 转换到 odom 坐标系的里程计 |
| `/odom` | nav_msgs/Odometry | Nav2 使用的里程计 |
| `/scan` | LaserScan | 2D 激光扫描（Nav2 costmap 使用） |
| `/cmd_vel` | Twist | Nav2 输出的速度指令 |
| `/initialpose` | PoseWithCovarianceStamped | RViz 初始位姿估计（重定位用） |
---
## 使用指南

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo（仿真需要）
- Livox-SDK2（实机需要，已预编译在 `livox_ros_driver2/3rdparty/`）

### 构建

```bash
source /opt/ros/humble/setup.bash
./build.sh
# 等价于: colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

每次修改代码后需重新构建。

### 仿真建图

```bash
source install/setup.bash
./mapping_sim.sh
```

1. 在 GUI 遥控窗口中用 WASD 驾驶机器人遍历环境
2. 在 RViz 中观察建图效果
3. 建图完成后保存：

```bash
./save_map.sh    # 保存 2D 占用栅格地图
./save_pcd.sh    # 保存 3D 点云
```

4. 将 PCD 文件移至 `src/me_nav2_bringup/pcd/`

### 仿真导航

1. 修改重定位配置中的 PCD 文件路径：
   `src/registration/small_gicp_relocalization/launch/small_gicp_relocalization_launch.py` 中的 `prior_pcd_file`

2. 修改 Nav2 地图路径：
   `src/me_nav2_bringup/launch/my_nav2_launch.py` 中的 `map_yaml_file`

3. 启动导航：
   ```bash
   ./nav2_sim.sh
   ```

4. 在 RViz 中使用 "2D Pose Estimate" 设置初始位姿，然后用 "Nav2 Goal" 发送目标点

### 实机建图

```bash
source install/setup.bash
./mapping_real.sh
```

流程同仿真建图，使用遥控器或手柄驾驶机器人。

### 实机导航

```bash
./nav2_real.sh
```

流程同仿真导航。

### 调试工具

```bash
./show_tf_tree.sh    # 生成 TF 树 PDF
```

## LIO 切换说明

默认使用 FAST-LIO，可在启动脚本中通过注释切换到 Point-LIO。

| | FAST-LIO | Point-LIO |
|---|---|---|
| 仿真 LiDAR 驱动 | `xfer_format=0` (PointCloud2) | `xfer_format=1` (CustomMsg) |
| 实机 LiDAR 驱动 | `xfer_format=0` | `xfer_format=1` |
| 仿真 config | `mid360.yaml` (lidar_type=5) | `mid360_sim.yaml` (lidar_type=2) |
| 实机 config | `mid360.yaml` (lidar_type=4) | `mid360_real.yaml` (lidar_type=1) |
| lio_interface | `fastlio_lio_interface_launch.py` (订阅 `/Odometry`) | `pointlio_lio_interface_launch.py` (订阅 `/aft_mapped_to_init`) |
| 仿真需 ign_sim_pointcloud_tool | 否（直接处理 PointCloud2） | 是（需要 ring/time 字段） |

**注意：** 两个 LIO 的 `lidar_type` 枚举定义不同，不要混淆。

## 关键配置文件

| 文件 | 说明 |
|------|------|
| `src/me_nav2_bringup/config/nav2_params.yaml` | Nav2 参数（DWB + Navfn） |
| `src/me_nav2_bringup/config/slam_toolbox_params.yaml` | SLAM Toolbox 参数 |
| `src/me_nav2_bringup/config/Pointcloud2d_3d.yaml` | 3D→2D 转换参数 |
| `src/localization/FAST_LIO/config/mid360.yaml` | FAST-LIO 配置 |
| `src/localization/point_lio/config/mid360_sim.yaml` | Point-LIO 仿真配置 |
| `src/localization/point_lio/config/mid360_real.yaml` | Point-LIO 实机配置 |
| `src/livox_ros_driver2/config/MID360_config.json` | LiDAR 网络配置 |
| `src/registration/icp_registration/config/icp.yaml` | ICP 配置 |
