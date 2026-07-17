#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(dirname -- "$SCRIPT_DIR")"
cd "$WORKSPACE_ROOT" || exit 1

# 仿真导航启动脚本

# 杀死之前的 Gazebo 进程
# killall -9 gzserver gzclient

# 键盘控制
# ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 可能会导致 /cmd_vel 话题被占用
# gui控制小车
gnome-terminal --title="GUI控制" -- bash -c "
source install/setup.bash;
ros2 run gui_teleop gui_teleop_node"

# -----------------------------------------------------------------------------------
# # 使用fast-lio作为里程计（仿真环境下必须 use_sim_time:=true）
gnome-terminal --title="FAST-LIO 里程计" -- bash -c "
source install/setup.bash;
ros2 launch fast_lio mapping.launch.py use_sim_time:=true rviz:=false"

# 里程计接口
gnome-terminal --title="lio_interface" -- bash -c "
source install/setup.bash;
ros2 launch lio_interface lio_interface_launch.py"

# ------------------------------------------------------------------------------------

# # 使用point-lio作为里程计
# gnome-terminal --title="点云格式转换器" -- bash -c "
# source install/setup.bash;
# ros2 run ign_sim_pointcloud_tool ign_sim_pointcloud_tool_node --ros-args \
#   -p pcd_topic:=/livox/lidar \
#   -p n_scan:=50 \
#   -p horizon_scan:=360 \
#   -p ang_bottom:=7.22 \
#   -p ang_res_y:=1.248"

# gnome-terminal --title="Point-LIO 里程计" -- bash -c "
# source install/setup.bash;
# ros2 launch point_lio point_lio.launch.py use_sim_time:=True\
#   point_lio_cfg_dir:=/home/px4/Lidar_nav2_ws/src/localization/point_lio/config/mid360_sim.yaml"

# gnome-terminal --title="lio_interface" -- bash -c "
# source install/setup.bash;
# ros2 launch lio_interface pointlio_lio_interface_launch.py"

# ------------------------------------------------------------------------------------


# Gazebo 仿真环境
gnome-terminal --title="Gazebo 仿真" -- bash -c "
killall -9 gzserver gzclient;
source install/setup.bash;
ros2 launch get_urdf get_urdf_launch.py"

gnome-terminal --title="sensor_scan_generation" -- bash -c "
source install/setup.bash;
ros2 launch sensor_scan_generation sensor_scan_generation_launch.py"

gnome-terminal --title="3d点云转2d" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup pointcloud_to_laserscan_launch.py"

# gnome-terminal --title="small_gicp 重定位" -- bash -c "
# source install/setup.bash;
# ros2 launch small_gicp_relocalization small_gicp_relocalization_launch.py"

gnome-terminal --title="KISS + GICP 重定位" -- bash -c "
source install/setup.bash;
ros2 launch global_relocalization_kiss_matcher global_kiss_matcher_relocalization_launch.py"

gnome-terminal --title="Nav2 导航" -- bash -c "
source install/setup.bash;
ros2 launch me_nav2_bringup my_nav2_launch.py"

# =====================================================
# 等待所有 Nav2 节点就绪后，手动过渡生命周期
# 因 libdiagnostic_updater 系统库与 ROS2 不兼容，
# 系统自带的 lifecycle_manager 会闪退，此处用 Python 替代
# =====================================================
sleep 12
gnome-terminal --title="Lifecycle Manager (手动)" -- bash -c "
source install/setup.bash;
python3 src/me_nav2_bringup/scripts/manual_lifecycle_manager.py;
sleep 3;
echo '====== 设置初始位姿给 KISS-Matcher 触发重定位 ======';
ros2 topic pub --once /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0, y: 0, z: 0}, orientation: {x: 0, y: 0, z: 0, w: 1}}, covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.0685]}}' 2>&1;
echo '====== 启动 map->odom bridge (50Hz 刷新防止 TF 超时) ======';
python3 scripts/map_odom_bridge.py --ros-args -p use_sim_time:=true &
sleep 2;
echo '=== 系统就绪 === 如需调整位姿，在 RViz 中使用 2D Pose Estimate';
exec bash"
