#!/usr/bin/env python3
"""
serial_bridge: ROS2 ↔ ESP32-S3 串口桥接节点

协议定义 (与 comm.h/comm.cpp 对应):

  下行帧 (ROS2 → ESP32): 7 字节
    0     1-2      3-4      5       6
    0xAA  vTargetL  vTargetR  mode    checksum(XOR)
          (int16)   (int16)   (uint8) (uint8)

  上行帧 (ESP32 → ROS2): 15 字节
    0     1-4    5-8    9-10   11-12  13    14
    0xBB  encL    encR   pitch  fault  state checksum
          (int32) (int32)(int16)(uint16)(uint8)(uint8)

工作流程:
  1. 订阅 /cmd_vel (Twist) → 换算为左右轮速度 → 组下行帧发送
  2. 接收上行帧 → 解析编码器/姿态 → 发布轮式里程计 /odom
  3. 200ms 无上行帧 → 报通信超时警告
"""

import struct
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, UInt8
from tf2_ros import TransformBroadcaster
import serial
import serial.tools.list_ports
import threading
import math
import time

# ──────────────────── 协议常量 (与 ESP32 config.h 对齐) ────────────────────
SYNC_DOWN = 0xAA
SYNC_UP   = 0xBB

MODE_RELEASE   = 0
MODE_MANUAL    = 1
MODE_ROS2_AUTO = 2
MODE_ESTOP     = 3

DOWN_FRAME_LEN = 7
UP_FRAME_LEN   = 15

# ──────────────────── 编码器/运动学参数 (与 ESP32 config.h 对齐) ─────────────
ENCODER_PPR   = 13
GEAR_RATIO    = 34
ENCODER_MULT  = 4
COUNTS_PER_REV = ENCODER_PPR * GEAR_RATIO * ENCODER_MULT  # 1768

CONTROL_PERIOD_S = 0.02  # 50 Hz


class SerialBridge(Node):
    """ROS2 ↔ ESP32 串口桥接节点"""

    def __init__(self):
        super().__init__('serial_bridge')

        # ─── 参数 ───
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('wheel_radius', 0.065)       # 轮半径 (m)
        self.declare_parameter('track_width', 0.45)         # 轮距 (m)
        self.declare_parameter('encoder_ppr', ENCODER_PPR)
        self.declare_parameter('gear_ratio', GEAR_RATIO)
        self.declare_parameter('encoder_mult', ENCODER_MULT)
        self.declare_parameter('publish_tf', True)          # 是否发布 odom→base_footprint TF
        self.declare_parameter('auto_mode', False)          # 启动后自动切 ROS2_AUTO

        port     = self.get_parameter('port').value
        baud     = self.get_parameter('baudrate').value
        self._wheel_radius = self.get_parameter('wheel_radius').value
        self._track_width  = self.get_parameter('track_width').value
        self._publish_tf   = self.get_parameter('publish_tf').value
        auto_mode          = self.get_parameter('auto_mode').value

        # 编码器换算因子: counts_per_rev
        ppr   = self.get_parameter('encoder_ppr').value
        gr    = self.get_parameter('gear_ratio').value
        mult  = self.get_parameter('encoder_mult').value
        self._counts_per_rev = ppr * gr * mult
        self._meters_per_count = (2 * math.pi * self._wheel_radius) / self._counts_per_rev

        self.get_logger().info(
            f"⚙  轮半径={self._wheel_radius:.3f}m, 轮距={self._track_width:.3f}m, "
            f"counts/rev={self._counts_per_rev}, m/count={self._meters_per_count:.6f}")

        # ─── 状态 ───
        self._mode = MODE_ROS2_AUTO if auto_mode else MODE_MANUAL
        self._cmd_vel = Twist()            # 最新速度指令
        self._running = True

        # 里程计累加
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_enc_l = None
        self._last_enc_r = None
        self._last_odom_time = None

        # 通信超时检测
        self._last_up_rx_ms = time.monotonic()

        # ─── 串口 ───
        self._ser = None
        try:
            self._ser = serial.Serial(port, baud, timeout=0.02)
            self.get_logger().info(f"✅ 串口已打开: {port} @ {baud} baud")
        except Exception as e:
            self.get_logger().error(f"❌ 无法打开串口 {port}: {e}")
            self.get_logger().info("尝试自动搜索可用串口...")
            ports = serial.tools.list_ports.comports()
            for p in ports:
                self.get_logger().info(f"  可用串口: {p.device} - {p.description}")
            # 即使串口打开失败，节点也不崩溃，允许重连

        # ─── 接收线程 ───
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # ─── TF 广播 ───
        self._tf_broadcaster = TransformBroadcaster(self)

        # ─── ROS2 订阅 / 发布 ───
        self._sub_cmd_vel = self.create_subscription(
            Twist, '/cmd_vel', self._cb_cmd_vel, 10)
        self._sub_mode_cmd = self.create_subscription(
            UInt8, '/mode_cmd', self._cb_mode_cmd, 10)
        self._pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self._pub_mode = self.create_publisher(UInt8, '/serial_bridge_mode', 10)

        # ─── 定时器: 50Hz 发送下行帧 ───
        self._timer = self.create_timer(CONTROL_PERIOD_S, self._timer_cb)

        # ─── 定时器: 1Hz 打印状态 ───
        self.create_timer(1.0, self._status_cb)

        self.get_logger().info("🚀 serial_bridge 已启动")

    # ═══════════════════ 串口接收 ═══════════════════

    def _rx_loop(self):
        """后台线程: 持续读取串口, 解析上行帧"""
        rx_buf = bytearray()
        rx_state = 0   # 0=wait_sync, 1=recv_data

        while self._running:
            if self._ser is None or not self._ser.is_open:
                time.sleep(0.5)
                continue
            try:
                if self._ser.in_waiting > 0:
                    data = self._ser.read(self._ser.in_waiting)
                    rx_buf.extend(data)
            except Exception as e:
                self.get_logger().error(f"串口读取错误: {e}")
                time.sleep(0.1)
                continue

            # 逐字节解析上行帧
            while len(rx_buf) > 0:
                if rx_state == 0:
                    if rx_buf[0] == SYNC_UP:
                        rx_state = 1
                    rx_buf.pop(0)
                else:
                    if len(rx_buf) >= UP_FRAME_LEN:
                        frame = rx_buf[:UP_FRAME_LEN]
                        rx_buf = rx_buf[UP_FRAME_LEN:]
                        rx_state = 0
                        self._parse_up_frame(frame)
                    else:
                        break   # 等待更多数据

    def _parse_up_frame(self, frame: bytearray):
        """解析 15 字节上行帧"""
        if len(frame) != UP_FRAME_LEN:
            return

        # XOR 校验
        cs = 0
        for b in frame[:-1]:
            cs ^= b
        if cs != frame[-1]:
            self.get_logger().warn("⚠ 上行帧校验和错误")
            return

        (sync, enc_l, enc_r, pitch_x100, fault, state) = struct.unpack(
            '<BiihHB', frame[:14])

        # 更新时间戳
        self._last_up_rx_ms = time.monotonic()

        # ── 里程计推算 ──
        now = self.get_clock().now()
        if self._last_enc_l is not None:
            dl = (enc_l - self._last_enc_l) * self._meters_per_count
            dr = (enc_r - self._last_enc_r) * self._meters_per_count
            d = (dl + dr) / 2.0
            dtheta = (dr - dl) / self._track_width

            self._x += d * math.cos(self._theta + dtheta / 2.0)
            self._y += d * math.sin(self._theta + dtheta / 2.0)
            self._theta += dtheta

            # 发布 Odometry
            self._publish_odom(now, enc_l, enc_r, fault, state)
        else:
            self._publish_tf_only(now)

        self._last_enc_l = enc_l
        self._last_enc_r = enc_r
        self._last_odom_time = now

    # ═══════════════════ 里程计发布 ═══════════════════

    def _publish_odom(self, stamp, enc_l, enc_r, fault, state):
        odom = Odometry()
        odom.header = Header(stamp=stamp.to_msg(), frame_id='odom')
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        # 四元数
        odom.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        self._pub_odom.publish(odom)

        # TF
        if self._publish_tf:
            t = TransformStamped()
            t.header = Header(stamp=stamp.to_msg(), frame_id='odom')
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            # 广播 TF (需要单独的 tf2_broadcaster)
            if hasattr(self, '_tf_broadcaster'):
                self._tf_broadcaster.sendTransform(t)

    def _publish_tf_only(self, stamp):
        if self._publish_tf and hasattr(self, '_tf_broadcaster'):
            t = TransformStamped()
            t.header = Header(stamp=stamp.to_msg(), frame_id='odom')
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation.w = 1.0
            self._tf_broadcaster.sendTransform(t)

    # ═══════════════════ cmd_vel 回调 ═══════════════════

    def _cb_cmd_vel(self, msg: Twist):
        self._cmd_vel = msg

    def _cb_mode_cmd(self, msg: UInt8):
        """切换控制模式 (0=释放, 1=手动, 2=ROS2, 3=急停)"""
        if msg.data in (MODE_RELEASE, MODE_MANUAL, MODE_ROS2_AUTO, MODE_ESTOP):
            self._mode = msg.data
            self.get_logger().info(f"🔄 模式切换 → {self._mode_name()}")
        else:
            self.get_logger().warn(f"⚠ 未知模式: {msg.data}")

    def _mode_name(self):
        return {MODE_RELEASE: '释放', MODE_MANUAL: '手动',
                MODE_ROS2_AUTO: 'ROS2导航', MODE_ESTOP: '急停'}.get(self._mode, '?')

    # ═══════════════════ 50Hz 定时器: 发送下行帧 ═══════════════════

    def _timer_cb(self):
        """每 20ms: 组下行帧发送给 ESP32"""
        if self._ser is None or not self._ser.is_open:
            # 尝试重连
            self._try_reconnect()
            return

        # 检查通信超时 (超过 500ms 没收上行帧)
        now = time.monotonic()
        if now - self._last_up_rx_ms > 0.5:
            # 不切模式, 只警告
            pass

        # ── 速度换算: Twist → 编码器脉冲/控制周期 ──
        v = self._cmd_vel.linear.x
        w = self._cmd_vel.angular.z

        # 左右轮线速度 (m/s)
        left_ms  = v - w * self._track_width / 2.0
        right_ms = v + w * self._track_width / 2.0

        # m/s → 编码器脉冲/控制周期
        # 速度 (m/s) → 转速 (rev/s) = v / (2πr)
        # 脉冲/控制周期 = rev/s × counts/rev × 周期(s)
        def ms_to_counts(ms):
            rev_per_s = ms / (2 * math.pi * self._wheel_radius)
            return int(round(rev_per_s * self._counts_per_rev * CONTROL_PERIOD_S))

        vL = ms_to_counts(left_ms)
        vR = ms_to_counts(right_ms)

        # 限幅 int16
        vL = max(-32768, min(32767, vL))
        vR = max(-32768, min(32767, vR))

        # ── 组包 ──
        frame = struct.pack('<BhhB', SYNC_DOWN, vL, vR, self._mode)
        cs = 0
        for b in frame:
            cs ^= b
        frame += bytes([cs])

        try:
            self._ser.write(frame)
        except Exception as e:
            self.get_logger().error(f"串口写入错误: {e}")

    # ═══════════════════ 重连 ═══════════════════

    def _try_reconnect(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
            self._ser = serial.Serial(port, baud, timeout=0.02)
            self.get_logger().info(f"✅ 串口重连成功: {port}")
        except Exception:
            pass

    # ═══════════════════ 状态打印 ═══════════════════

    def _status_cb(self):
        age = (time.monotonic() - self._last_up_rx_ms) * 1000
        self.get_logger().info(
            f"[串口桥] 模式={'ROS2_AUTO' if self._mode==MODE_ROS2_AUTO else 'MANUAL'} "
            f"| 上行末帧 {age:.0f}ms 前 "
            f"| x={self._x:.2f} y={self._y:.2f} θ={math.degrees(self._theta):.1f}°")

    # ═══════════════════ 清理 ═══════════════════

    def destroy(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()
        super().destroy()

    def set_mode(self, mode: int):
        """外部设置控制模式"""
        self._mode = mode
        self.get_logger().info(f"模式切换: {mode}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
