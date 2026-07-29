"""serial_bridge 启动文件

用法:
    ros2 launch serial_bridge serial_bridge_launch.py [port:=/dev/ttyUSB0]
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_bridge',
            executable='serial_bridge',
            name='serial_bridge',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port', default='/dev/ttyUSB0'),
                'baudrate': 921600,
                'wheel_radius': 0.098,       # 98mm
                'track_width': 0.444,          # 444mm
                'publish_tf': True,
                'auto_mode': True,            # 启动后自动进入 ROS2 控制模式
            }]
        )
    ])
