"""实机导航完整启动文件 (Jetson + ESP32)

一键启动所有节点:
  - livox_ros_driver2 (LiDAR)
  - FAST-LIO (里程计)
  - lio_interface (TF 桥接)
  - gld_robot_description (URDF)
  - sensor_scan_generation
  - pointcloud_to_laserscan (3D→2D)
  - serial_bridge (ESP32 串口通信)
  - KISS-Matcher / small_gicp 全局重定位
  - Nav2 导航栈
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ─── 参数 ───
    port_arg = DeclareLaunchArgument(
        'port', default_value='/dev/ttyUSB0',
        description='ESP32 串口设备路径')

    # ─── 1. Livox 驱动 ───
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('livox_ros_driver2'),
                         'launch', 'fast_lio_msg_MID360_launch.py')),
    )

    # ─── 2. FAST-LIO ───
    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'),
                         'launch', 'mapping.launch.py')),
    )

    # ─── 3. lio_interface ───
    lio_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lio_interface'),
                         'launch', 'fastlio_lio_interface_launch.py')),
    )

    # ─── 4. 机器人 URDF ───
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gld_robot_description'),
                         'launch', 'gld_robot_description_launch.py')),
    )

    # ─── 5. sensor_scan_generation ───
    sensor_scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sensor_scan_generation'),
                         'launch', 'sensor_scan_generation_launch.py')),
    )

    # ─── 6. 3D→2D 转换 ───
    pcl2laser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('me_nav2_bringup'),
                         'launch', 'pointcloud_to_laserscan_launch.py')),
    )

    # ─── 7. serial_bridge (ESP32) ───
    serial_bridge_node = Node(
        package='serial_bridge',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': 921600,
            'wheel_radius': 0.098,       # 98mm
            'track_width': 0.444,          # 444mm
            'publish_tf': True,
            'auto_mode': True,
        }]
    )

    # ─── 8. Nav2 ───
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('me_nav2_bringup'),
                         'launch', 'my_nav2_launch.py')),
    )

    return LaunchDescription([
        port_arg,
        livox_launch,
        fast_lio_launch,
        lio_interface_launch,
        urdf_launch,
        sensor_scan_launch,
        pcl2laser_launch,
        serial_bridge_node,
        nav2_launch,
    ])
