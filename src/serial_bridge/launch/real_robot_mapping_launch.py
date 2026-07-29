"""实机建图完整启动文件 (Jetson + ESP32 + SLAM Toolbox)

与 navigation 的区别: 不启动 Nav2, 而是启动 SLAM Toolbox 在线建图
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
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
            'auto_mode': False,           # 建图时用遥控器手动控制
        }]
    )

    # ─── 8. SLAM Toolbox ───
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py'),
            launch_arguments={
                'slam_params_file': os.path.join(
                    get_package_share_directory('me_nav2_bringup'),
                    'config', 'slam_toolbox_params.yaml')
            }.items()
        ),
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
        slam_launch,
    ])
