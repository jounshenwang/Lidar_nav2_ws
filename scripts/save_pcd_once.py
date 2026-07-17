#!/usr/bin/env python3
"""保存一帧 /cloud_registered 点云为 PCD 文件"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import struct
import sys


class PcdSaver(Node):
    def __init__(self):
        super().__init__("pcd_saver")
        self.pcd = None
        self.sub = self.create_subscription(
            PointCloud2, "/cloud_registered", self.cb, 10
        )

    def cb(self, msg):
        self.pcd = msg


def main():
    OUTPUT = "/home/px4/Lidar_nav2_ws/src/me_nav2_bringup/pcd/saved_map.pcd"

    rclpy.init()
    node = PcdSaver()

    print("等待 /cloud_registered 数据...", flush=True)
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.pcd is not None:
            break

    if node.pcd is None:
        print("ERROR: 10 秒内未收到点云", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    msg = node.pcd
    n_points = msg.width * msg.height

    # 字段类型映射: ROS sensor_msgs datatype -> PCD type
    dt_map = {1: "U", 2: "U", 4: "U", 7: "F", 8: "F"}

    lines = [
        "# .PCD v0.7 - Point Cloud Data file format",
        "VERSION 0.7",
        "FIELDS " + " ".join(f.name for f in msg.fields),
        "SIZE " + " ".join("4" for _ in msg.fields),
        "TYPE " + " ".join(dt_map.get(f.datatype, "F") for f in msg.fields),
        "COUNT " + " ".join(str(f.count) for f in msg.fields),
        f"WIDTH {msg.width}",
        f"HEIGHT {msg.height}",
        "VIEWPOINT 0 0 0 1 0 0 0",
        f"POINTS {n_points}",
        "DATA ascii",
    ]

    data = msg.data
    ps = msg.point_step
    for i in range(n_points):
        vals = []
        off = i * ps
        for f in msg.fields:
            fo = off + f.offset
            if f.datatype == 7:      # FLOAT32
                v = struct.unpack_from("<f", data, fo)[0]
            elif f.datatype == 4:    # UINT16
                v = struct.unpack_from("<H", data, fo)[0]
            elif f.datatype == 2:    # UINT8
                v = struct.unpack_from("<B", data, fo)[0]
            else:
                v = struct.unpack_from("<f", data, fo)[0]
            vals.append(str(v))
        lines.append(" ".join(vals))

    with open(OUTPUT, "w") as f:
        f.write("\n".join(lines) + "\n")

    print(f"OK: {n_points} points -> {OUTPUT}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()