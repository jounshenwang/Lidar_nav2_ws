#!/usr/bin/env python3
"""持续累积 /cloud_registered 点云，Ctrl+C 时保存为 PCD 文件"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import sys
import signal

OUTPUT = "/home/px4/Lidar_nav2_ws/src/me_nav2_bringup/pcd/accumulated_map.pcd"


class Accumulator(Node):
    def __init__(self):
        super().__init__("pcd_accumulator")
        self.all_points = []
        self.field_names = None
        self.sub = self.create_subscription(
            PointCloud2, "/cloud_registered", self.cb, 10
        )
        self.get_logger().info("开始累积 /cloud_registered ... Ctrl+C 保存并退出")

    def cb(self, msg):
        names = [f.name for f in msg.fields]
        if self.field_names is None:
            self.field_names = names

        n = msg.width * msg.height
        ps = msg.point_step
        data = msg.data
        for i in range(n):
            vals = []
            off = i * ps
            for f in msg.fields:
                fo = off + f.offset
                if f.datatype == 7:
                    v = struct.unpack_from("<f", data, fo)[0]
                elif f.datatype == 4:
                    v = struct.unpack_from("<H", data, fo)[0]
                elif f.datatype == 2:
                    v = struct.unpack_from("<B", data, fo)[0]
                else:
                    v = struct.unpack_from("<f", data, fo)[0]
                vals.append(v)
            self.all_points.append(vals)

        total = len(self.all_points)
        if total % 10000 < n:
            self.get_logger().info(f"已累积 {total} 个点 ...")


def main():
    rclpy.init()
    node = Accumulator()

    def save_and_exit(sig=None, frame=None):
        if not node.all_points:
            node.get_logger().info("无数据，直接退出")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(0)

        lines = [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            "FIELDS " + " ".join(node.field_names),
            "SIZE " + " ".join("4" for _ in node.field_names),
            "TYPE " + " ".join("F" for _ in node.field_names),
            "COUNT " + " ".join("1" for _ in node.field_names),
            f"WIDTH {len(node.all_points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(node.all_points)}",
            "DATA ascii",
        ]
        for vals in node.all_points:
            lines.append(" ".join(str(v) for v in vals))

        with open(OUTPUT, "w") as f:
            f.write("\n".join(lines) + "\n")

        node.get_logger().info(
            f"已保存 {len(node.all_points)} 个点 -> {OUTPUT}"
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, save_and_exit)
    signal.signal(signal.SIGTERM, save_and_exit)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        save_and_exit()


if __name__ == "__main__":
    main()