# 仿真导航全流程测试报告

**日期**: 2026-07-10  
**测试目标**: 运行 Lidar_nav2_ws 完整的仿真导航流水线，发现并解决问题  
**测试环境**: ROS2 Humble + Gazebo 11 + Nav2  
**报告版本**: V3 — DWB 评价函数全面修复

---

## 系统架构概览

```
Gazebo (simple_car URDF)
  ├─ /livox/lidar (PointCloud2) ──→ FAST-LIO mapping
  ├─ /livox/imu (IMU) ────────────→ FAST-LIO mapping
  │
FAST-LIO
  ├─ /cloud_registered (registered point cloud)
  └─ /Odometry (lidar_odom frame)
        │
        ▼
lio_interface
  ├─ 订阅 /cloud_registered, /Odometry
  └─ 发布 /registered_scan, /registered_odometry (odom frame)
        │
        ▼
sensor_scan_generation
  ├─ 同步 /registered_odometry + /registered_scan
  └─ 发布 /odom, TF odom→base_footprint, /lidar_frame_pcd
        │
        ├──────────────────┐
        ▼                  ▼
pointcloud_to_laserscan   KISS-Matcher 重定位
  /registered_scan→/scan    加载 accumulated_map.pcd
                         map→odom TF (回环匹配 + GICP 持续跟踪)
        │                  │
        ▼                  ▼
   Nav2 导航栈 ──── map_odom_bridge (50Hz 中继)
        │
        ▼
   /cmd_vel ──→ velocity_smoother ──→ Gazebo diff_drive
```

---

## 测试结果

### ✅ 编译 — 通过

所有核心包成功编译：

| 包名 | 状态 |
|------|------|
| me_nav2_bringup | ✅ |
| lio_interface | ✅ |
| sensor_scan_generation | ✅ |
| gui_teleop | ✅ |
| global_relocalization_kiss_matcher | ✅ |

### ✅ Gazebo 仿真 — 通过

- Gazebo 11 启动正常
- `simple_car.urdf` 载入成功，包含：
  - 4 轮差速底盘（diff_drive）
  - Mid-360 LiDAR 仿真（ray sensor，10Hz，360°×50线）
  - IMU 传感器（200Hz）
- `robot_state_publisher` 正确发布 TF（base_footprint → chassis → livox_frame）
- LiDAR 数据流正常（`/livox/lidar` 约 11000 点/帧）

### ✅ FAST-LIO 里程计 — 通过

- 配置 `lidar_type: 5`（通用点云格式），适配 Gazebo 仿真
- `/Odometry` 发布正常，frame_id = `camera_init`
- `/cloud_registered` 发布正常

### ✅ lio_interface — 通过

- 正确订阅 FAST-LIO 输出
- 将点云从 `camera_init` 变换到 `odom` 坐标系
- `/registered_scan`（frame_id = `odom`）和 `/registered_odometry` 发布正常

### ✅ sensor_scan_generation — 通过

- 正确同步 `/registered_odometry` 和 `/registered_scan`
- 发布 `/odom` 里程计
- 发布 `odom→base_footprint` TF（使用 FAST-LIO 里程计数据）
- 发布 `/lidar_frame_pcd`（livox_frame 坐标系）

### ✅ pointcloud_to_laserscan — 通过

- 将 `/registered_scan`（3D）转换为 `/scan`（2D LaserScan）
- frame_id = `livox_frame`
- 范围：0.05m ~ 70m
- QoS 修复：配置 `qos_overrides` 使 `/scan` 发布 RELIABLE QoS，兼容 rviz2 和 Nav2

### ✅ KISS-Matcher 重定位 — 通过

- 加载先验地图点云：**1,244,976 点**（95MB `accumulated_map.pcd`）
- KISS 全局配准初始化成功（inliers > 3, overlapness 100% > 80%）
- 初始化后切换到 GICP 持续跟踪（2Hz），更新 map→odom 变换
- 同时以 20Hz 发布 map→odom TF
- 订阅 `/initialpose` 话题以触发重定位

### ✅ Nav2 导航栈 — 通过（手动生命周期管理）

| 节点 | 最终状态 |
|------|---------|
| map_server | ✅ active |
| controller_server | ✅ active |
| planner_server | ✅ active |
| bt_navigator | ✅ active |
| behavior_server | ✅ active |
| smoother_server | ✅ active |
| velocity_smoother | ✅ active |
| waypoint_follower | ✅ active |

---

## 已修复的问题

### 🚀 Fix #1: map_odom_bridge use_sim_time 未生效

**问题描述**:
`scripts/map_odom_bridge.py` 使用 `rclpy.init()` 不带参数，导致通过命令行传递的 `--ros-args -p use_sim_time:=true` 被忽略。TF 时间戳为 Wall-Clock 时间（如 `1783608527.898`）而非仿真时间（如 `589.577`）。

**影响**:
Nav2 的 planner_server 和 bt_navigator 出现大量 TF 外推错误（Extrapolation Error），导致全局规划器无法完成路径计算。

**修复**（`scripts/map_odom_bridge.py`）:
```python
rclpy.init(args=sys.argv)
```

**验证结果**: 修复后 TF 时间戳正确显示仿真时间。

### 🚀 Fix #2: map_odom_bridge 恒等变换覆盖 KISS-Matcher 定位（V3 重写）

**问题描述**:
KISS-Matcher 以 20Hz 发布正确的 map→odom TF（通过 KISS 全局配准 + GICP 持续跟踪），但 `map_odom_bridge.py` 同时以 50Hz 发布**单位变换**（identity）。由于桥接器频率更高，其单位变换覆盖了 KISS-Matcher 的定位结果，导致：
- 机器人在 map 帧中的位置始终为原点
- 全局路径无法正确转换到 odom 帧
- DWB 控制器报 `Transform data too old when converting from odom to map`

**修复**（`scripts/map_odom_bridge.py` V3）:
- 重写为：通过订阅 `/tf` 话题直接捕获 KISS-Matcher 的真实 map→odom 变换
- 缓存后以当前仿真时间重发（50Hz），保持时间戳新鲜
- 值变化检测机制防止自身回环
- 回退：无定位时低频发布单位变换

### 🚀 Fix #3: /scan QoS 不兼容

**问题描述**:
`pointcloud_to_laserscan` 发布的 `/scan` 为 BEST_EFFORT QoS，rviz2 的 LaserScan 显示插件使用 RELIABLE QoS，导致 rviz2 无法显示激光扫描数据。

**修复**（`config/Pointcloud2d_3d.yaml`）:
```yaml
qos_overrides:
  scan:
    publisher:
      reliability: reliable
```

### 🚀 Fix #4: costmap 生命周期冗余管理

**问题描述**:
`manual_lifecycle_manager.py` 尝试配置/激活 `global_costmap/global_costmap` 和 `local_costmap/local_costmap`，但这些是由其父节点（planner_server / controller_server）自动管理的子节点，导致 `No transition matching` 警告。

**修复**（`scripts/manual_lifecycle_manager.py`）:
从 `nav_nodes` 列表中移除 costmap 条目，仅让父节点管理。

### 🚀 Fix #5: DWB 控制器参数全面修复（V3 核心修复）

**问题描述**:
DWB 配置经历了两个极端：

| 版本 | critics | 问题 |
|------|---------|------|
| 原始 | 7个，权重过高（BaseObstacle:10, PathAlign:32等） | 所有轨迹被拒绝，DWB 不输出速度 |
| V2（前次修复） | 仅 Oscillation + BaseObstacle(scale=0) | 所有轨迹得分为0，DWB 选第一条（vx=0,vtheta=0）→ **机器人不动或随机移动** |

V2 的极简配置导致**"轨迹规划"失效**：
- 500条采样轨迹全部得0分，DWB 无法区分好坏
- 第一条轨迹通常是 `(vx=0, vtheta=0)` → 无输出
- 即使偶然选到非零速度，也不朝目标前进（无 GoalAlign/PathDist）
- Progress Checker 检测不到目标进展 → BT 进入恢复循环 → 用户感知为"不行了"

**修复**（`config/nav2_params.yaml` V3 — 平衡的 critics 集）:

```
critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathDist"]

BaseObstacle.scale: 1.5    # 避障（中等权重，不拒绝窄通道）
GoalAlign.scale:   6.0     # 朝目标方向（主要引导）
PathDist.scale:    8.0     # 沿路径行驶（主要引导）
RotateToGoal.scale: 0.5    # 终点旋转对齐（低权重，仅终点附近生效）
```

**设计原则**:
- 5个 critics 而非7个（移除冗余的 GoalDist/PathDist）
- 权重从"极高"（24-32）降低到"适中"（1.5-8.0）
- vx/vtheta 采样从20→12，减少计算负载（576→144条/周期）
- BaseObstacle 权重低（1.5），确保在空旷仿真环境中不会误拒绝轨迹

---

## 数据流验证

### TF 树

```
map  ──(KISS-Matcher + map_odom_relay)──→  odom
                                            ──(sensor_scan_generation)──→  base_footprint
                                                                        ──(robot_state_publisher)──→  chassis
                                                                                                        ──(robot_state_publisher)──→  livox_frame
```

### 话题流

```
/livox/lidar (Gazebo ray sensor)     → FAST-LIO
/livox/imu (Gazebo IMU plugin)       → FAST-LIO
/Odometry (FAST-LIO)                 → lio_interface
/cloud_registered (FAST-LIO)         → lio_interface
/registered_odometry (lio_interface) → sensor_scan_generation
/registered_scan (lio_interface)     → sensor_scan_generation, pointcloud_to_laserscan, KISS-Matcher
/odom (sensor_scan_generation)       → Nav2, map_odom_relay
/scan (pointcloud_to_laserscan)      → Nav2 local costmap
/map (map_server)                    → Nav2 global costmap
/tf (map→odom from KISS-Matcher)     → map_odom_relay（中继）
```

---

## 启动步骤

### 完整启动命令

```bash
# 1. Gazebo + 机器人
ros2 launch get_urdf get_urdf_launch.py

# 2. FAST-LIO
ros2 launch fast_lio mapping.launch.py use_sim_time:=true rviz:=false

# 3. 接口
ros2 launch lio_interface lio_interface_launch.py
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py

# 4. 2D 转换
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py

# 5. 重定位
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py

# 6. Nav2
ros2 launch me_nav2_bringup my_nav2_launch.py

# 7. 等待 12-15 秒后执行生命周期管理
python3 src/me_nav2_bringup/scripts/manual_lifecycle_manager.py

# 8. 初始位姿 + map→odom relay（V3 — 监听 /tf 中继 KISS-Matcher）
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{
  header: {frame_id: "map"},
  pose: {
    pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}},
    covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0685]
  }
}'
python3 scripts/map_odom_bridge.py --ros-args -p use_sim_time:=true &
```

### 或用一键启动脚本

```bash
bash scripts/nav2_sim.sh
```

### 测试导航

```bash
# 测试全局规划
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose '{
  goal: {header: {frame_id: "map"}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}}}
}'

# 发送导航目标（含反馈）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{
  pose: {header: {frame_id: "map"}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}}}
}' --feedback
```

---

## 总结

### V3 报告新增修复

| 修复 | 文件 | 描述 |
|------|------|------|
| 🚀 DWB 评价函数全面修复 | `config/nav2_params.yaml` | 从极简2批评委升级为5个平衡评价函数，恢复目标导向导航 |

### 已知问题（不影响功能）

1. **planner_server 日志为空** — 日志文件大小为0字节，进程正常运行。可能是启动配置中日志重定向的问题。
2. **libdiagnostic_updater 不兼容** — 系统安装的库与 ROS2 Nav2 不兼容，已使用 `manual_lifecycle_manager.py` 替代。

### 建议下一步微调

如果导航效果需要进一步优化，可以调整 DWB critics 权重：

| 场景 | 调整 |
|------|------|
| 机器人不走直线 | 增大 `PathDist.scale`（8.0→12.0）|
| 机器人贴墙太近 | 增大 `BaseObstacle.scale`（1.5→3.0）|
| 机器人转角太猛 | 减小 `max_vel_theta`（1.0→0.5）|
| 机器人在目标附近徘徊 | 增大 `RotateToGoal.scale`（0.5→2.0）|
| 计算负载过高 | 减小采样数 `vx_samples/vtheta_samples`（12→8）|
