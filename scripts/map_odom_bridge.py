#!/usr/bin/env python3
"""
KISS-Matcher map->odom TF 中继节点（V3 — 直接订阅 /tf）

KISS-Matcher 以 20Hz 发布 map->odom，但时间戳固定为 last_scan_time + 0.1s，
导致 Nav2 控制器因变换"过时"无法做 TF 变换（"Transform data too old"）。

此节点直接订阅 /tf 话题，从中提取 KISS-Matcher 的 map->odom 变换，
缓存后以当前仿真时间重新发布（50Hz），彻底消除时间戳不匹配问题。
反馈回路防护：通过比对变换值变化检测真正来自 KISS-Matcher 的更新。

回退：尚未收到 KISS-Matcher 有效变换时，低频发布单位变换。
"""
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import tf2_ros
import geometry_msgs.msg
from tf2_msgs.msg import TFMessage
from rclpy.time import Time

# 回退发布频率（Hz）— 无定位时使用单位变换
FALLBACK_HZ = 2.0
# 中继发布频率（Hz）— 缓存 KISS-Matcher 的变换后以当前时间重发
RELAY_HZ = 50.0
# 变换值变化检测阈值
EPS = 1e-5


class MapOdomRelay(Node):
    def __init__(self):
        super().__init__('map_odom_relay')

        # TF 广播器（用于发布我们的中继变换）
        self.br = tf2_ros.TransformBroadcaster(self)

        # 直接订阅 /tf 话题，捕获 KISS-Matcher 发布的原始 map->odom
        # 使用与 TF 发布者兼容的 QoS（KeepLast 100, RELIABLE, VOLATILE）
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.tf_sub = self.create_subscription(
            TFMessage, '/tf', self.tf_callback, tf_qos
        )

        # 缓存状态
        self.kiss_transform = None       # geometry_msgs.msg.TransformStamped
        self.last_relay = None           # 上次中继的值（用于去重检测）
        self.have_kiss_transform = False
        self.fallback_active = True
        self._relay_logged = False
        self._fallback_logged = False

        # 周期发布
        self.create_timer(1.0 / RELAY_HZ, self.publish_transform)

        self.get_logger().info(
            'map->odom relay v3 started. '
            'Subscribing to /tf for KISS-Matcher map->odom, '
            f'relaying at {RELAY_HZ} Hz (fallback {FALLBACK_HZ} Hz).'
        )

    def tf_callback(self, msg: TFMessage):
        """从 /tf 消息中提取 KISS-Matcher 的 map->odom"""
        for tf in msg.transforms:
            if tf.header.frame_id == 'map' and tf.child_frame_id == 'odom':
                # 检查是否为非单位变换（单位变换可能来自我们的回退或其他节点）
                is_identity = (
                    abs(tf.transform.translation.x) < EPS
                    and abs(tf.transform.translation.y) < EPS
                    and abs(tf.transform.translation.z) < EPS
                    and abs(tf.transform.rotation.w - 1.0) < EPS
                )
                if is_identity:
                    continue

                # 检查是否与上次中继的值不同（避免自身回环）
                if self.last_relay is not None:
                    lr = self.last_relay
                    diff_x = abs(tf.transform.translation.x - lr.transform.translation.x)
                    diff_y = abs(tf.transform.translation.y - lr.transform.translation.y)
                    diff_z = abs(tf.transform.translation.z - lr.transform.translation.z)
                    diff_qw = abs(tf.transform.rotation.w - lr.transform.rotation.w)
                    if diff_x < EPS and diff_y < EPS and diff_z < EPS and diff_qw < EPS:
                        continue  # 与上次中继相同 → 忽略

                # 这是 KISS-Matcher 的真实更新
                self.kiss_transform = tf
                self.have_kiss_transform = True
                self.fallback_active = False
                if not self._relay_logged:
                    self._relay_logged = True
                    self._fallback_logged = False
                    self.get_logger().info(
                        f'KISS-Matcher map->odom update: '
                        f'trans=({tf.transform.translation.x:.3f}, '
                        f'{tf.transform.translation.y:.3f}, '
                        f'{tf.transform.translation.z:.3f})'
                    )

    def publish_transform(self):
        now = self.get_clock().now()

        if self.have_kiss_transform and self.kiss_transform is not None:
            # === 中继模式：用当前时间戳重发 KISS-Matcher 的变换 ===
            msg = geometry_msgs.msg.TransformStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'map'
            msg.child_frame_id = 'odom'
            msg.transform = self.kiss_transform.transform
            self.br.sendTransform(msg)

            # 缓存本次中继的值，用于去重检测
            self.last_relay = msg
        else:
            # === 回退模式：低频发布单位变换 ===
            # 降频：每秒只发 FALLBACK_HZ 次
            if not self._fallback_should_publish(now):
                return

            msg = geometry_msgs.msg.TransformStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = 'map'
            msg.child_frame_id = 'odom'
            msg.transform.translation.x = 0.0
            msg.transform.translation.y = 0.0
            msg.transform.translation.z = 0.0
            msg.transform.rotation.x = 0.0
            msg.transform.rotation.y = 0.0
            msg.transform.rotation.z = 0.0
            msg.transform.rotation.w = 1.0
            self.br.sendTransform(msg)

            if not self._fallback_logged:
                self._fallback_logged = True
                self._relay_logged = False
                self.get_logger().warn(
                    'KISS-Matcher map->odom not yet available. '
                    'Publishing identity fallback.'
                )

    def _fallback_should_publish(self, now) -> bool:
        """回退降频控制：用内部计时判断是否该发"""
        interval_ns = int(1.0 / FALLBACK_HZ * 1e9)
        if not hasattr(self, '_last_fallback_time'):
            self._last_fallback_time = now
            return True
        if (now - self._last_fallback_time).nanoseconds >= interval_ns:
            self._last_fallback_time = now
            return True
        return False


def main():
    rclpy.init(args=sys.argv)
    node = MapOdomRelay()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
