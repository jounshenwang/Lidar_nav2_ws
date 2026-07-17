#!/usr/bin/env python3
"""
Nav2 手动生命周期管理器
替代因系统库不兼容(libdiagnostic_updater.so)而无法运行的 lifecycle_manager。
使用 rclpy 创建节点并设置 use_sim_time=True，确保与 nav2 节点时间域一致。
通过轮询等待每个节点的 change_state 服务就绪后再执行 transition，避免固定延迟导致的竞态条件。
对于 activate 使用异步+轮询方式，避免 costmap 激活慢导致 service call 超时。
"""
import sys
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition

# 超时时间（秒）
SERVICE_TIMEOUT = 30.0     # 等待每个节点服务就绪的最大时间
SVC_CALL_TIMEOUT = 10.0    # change_state 服务调用同步等待的超时（仅 configure 用）
POLL_INTERVAL = 1.0        # 轮询状态间隔
STATE_TIMEOUT = 60.0       # 等待状态变化的最大时间（activate 可能很慢）
CONFIGURE_GAP = 2.0        # configure 完成后等待时间
ACTIVATE_GAP = 3.0         # activate 完成后等待时间
BT_WAIT = 8.0              # 等待 FollowPath action 就绪然后激活 bt_navigator

# 目标状态映射
TARGET_STATE = {
    Transition.TRANSITION_CONFIGURE: 'inactive',
    Transition.TRANSITION_ACTIVATE: 'active',
}


class ManualLifecycleManager(Node):
    def __init__(self):
        super().__init__('manual_lifecycle_manager')
        # use_sim_time 由 ROS2 框架自动声明，无需手动 declare

    def call_change_state(self, node_name, transition_id):
        """
        调用生命周期节点的 change_state 服务。
        对于 configure: 同步等待结果
        对于 activate: 异步发送 + 轮询状态（costmap 激活可能很慢）
        """
        target_state = TARGET_STATE.get(transition_id, 'unknown')
        service_name = f'{node_name}/change_state'
        client = self.create_client(ChangeState, service_name)

        if not client.wait_for_service(timeout_sec=SVC_CALL_TIMEOUT):
            self.get_logger().error(f'{node_name}: change_state service 不可用')
            return False

        req = ChangeState.Request()
        req.transition.id = transition_id

        if transition_id == Transition.TRANSITION_CONFIGURE:
            # configure 通常很快，同步等待
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=SVC_CALL_TIMEOUT)
            if future.result() is not None and future.result().success:
                self.get_logger().info(f'{node_name}: configure 成功')
                return True
            else:
                self.get_logger().error(f'{node_name}: configure 超时/失败')
                return False
        else:
            # activate: 异步发送，轮询状态直到变为 active 或超时
            self.get_logger().info(f'{node_name}: 发送 activate 请求 (异步模式)...')
            future = client.call_async(req)

            # 短暂等待，看 service call 是否能立即完成
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'{node_name}: activate 快速成功')
                    return True
                else:
                    self.get_logger().warn(f'{node_name}: activate service 返回失败，轮询状态...')
            else:
                self.get_logger().info(f'{node_name}: activate service call 未立即返回，切换到轮询模式...')

            # 轮询状态直到达到目标
            self.get_logger().info(f'{node_name}: 轮询等待进入 "{target_state}" 状态 (最多 {STATE_TIMEOUT}s)...')
            deadline = time.time() + STATE_TIMEOUT
            while time.time() < deadline:
                state = self.get_state(node_name)
                if state is not None:
                    self.get_logger().info(f'{node_name}: 当前状态 = {state}', throttle_duration_sec=3.0)
                    if state == target_state:
                        self.get_logger().info(f'{node_name}: 已达到 {target_state} 状态!')
                        return True
                time.sleep(POLL_INTERVAL)

            self.get_logger().error(f'{node_name}: 等待 {target_state} 状态超时 ({STATE_TIMEOUT}s)')
            return False

    def get_state(self, node_name):
        """查询节点的当前生命周期状态"""
        service_name = f'{node_name}/get_state'
        client = self.create_client(GetState, service_name)

        if not client.wait_for_service(timeout_sec=3.0):
            return None

        req = GetState.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result().current_state.label
        return None

    def wait_for_node(self, node_name, timeout=SERVICE_TIMEOUT):
        """等待节点的 get_state 服务可用"""
        service_name = f'{node_name}/get_state'
        client = self.create_client(GetState, service_name)
        self.get_logger().info(f'等待 {node_name} 就绪 (timeout={timeout}s)...')
        result = client.wait_for_service(timeout_sec=timeout)
        if result:
            self.get_logger().info(f'{node_name} 已就绪')
        else:
            self.get_logger().error(f'{node_name} 未能就绪(超时 {timeout}s)')
        return result

    def wait_for_action_server(self, action_name, timeout=15.0):
        """等待 action server 就绪"""
        import subprocess
        self.get_logger().info(f'等待 action server {action_name} 就绪...')
        deadline = time.time() + timeout
        while time.time() < deadline:
            result = subprocess.run(
                ['ros2', 'action', 'list'],
                capture_output=True, text=True, timeout=5.0
            )
            if action_name in result.stdout:
                self.get_logger().info(f'action server {action_name} 已就绪')
                return True
            time.sleep(1.0)
        self.get_logger().warn(f'action server {action_name} 等待超时')
        return False

    def configure_nodes(self, nodes):
        """配置所有节点"""
        success = True
        for node in nodes:
            self.get_logger().info(f'配置 {node}...')
            if not self.call_change_state(node, Transition.TRANSITION_CONFIGURE):
                self.get_logger().warn(f'{node}: configure 失败，继续...')
                success = False
        return success

    def activate_nodes(self, nodes):
        """异步激活所有节点（带轮询）"""
        success = True
        for node in nodes:
            self.get_logger().info(f'激活 {node}...')
            if not self.call_change_state(node, Transition.TRANSITION_ACTIVATE):
                self.get_logger().warn(f'{node}: activate 失败，继续...')
                success = False
        return success


def main():
    rclpy.init(args=sys.argv)

    manager = ManualLifecycleManager()
    manager.get_logger().info('=== 手动生命周期管理器已启动 ===')

    nav_nodes = [
        '/map_server',
        '/controller_server',
        '/planner_server',
        '/bt_navigator',
        '/behavior_server',
        '/smoother_server',
        '/velocity_smoother',
        '/waypoint_follower',
        # 注意：global_costmap/global_costmap 和 local_costmap/local_costmap
        # 由其父节点（planner_server / controller_server）自动管理生命周期，
        # 此处手动管理会导致 "No transition matching" 警告，故不列出。
    ]

    # =====================================================
    # 第0步：等待所有节点就绪（轮询，最多等 SERVICE_TIMEOUT 秒）
    # =====================================================
    manager.get_logger().info('>>> 步骤0: 等待所有 nav2 节点就绪...')
    all_ready = True
    for node in nav_nodes:
        if manager.wait_for_node(node, timeout=SERVICE_TIMEOUT):
            pass
        else:
            manager.get_logger().error(f'{node} 未能就绪(超时)')
            all_ready = False

    if not all_ready:
        manager.get_logger().warn('部分节点未就绪，再等10秒重试...')
        time.sleep(10.0)
        for node in nav_nodes:
            state = manager.get_state(node)
            if state is None:
                manager.wait_for_node(node, timeout=10.0)

    time.sleep(CONFIGURE_GAP)

    # =====================================================
    # 第1步：configure 所有节点
    # =====================================================
    manager.get_logger().info('>>> 步骤1: configure 所有节点')
    manager.configure_nodes(nav_nodes)

    time.sleep(CONFIGURE_GAP)

    # 检查 configure 结果
    for node in nav_nodes:
        state = manager.get_state(node)
        manager.get_logger().info(f'{node} 状态: {state}')

    # =====================================================
    # 第2步：activate 除 bt_navigator 外的所有节点（异步+轮询模式）
    # =====================================================
    manager.get_logger().info('>>> 步骤2: activate 其他节点（异步+轮询，可容忍慢激活）')
    non_bt_nodes = [n for n in nav_nodes if n != '/bt_navigator']
    manager.activate_nodes(non_bt_nodes)

    time.sleep(ACTIVATE_GAP)

    # =====================================================
    # 第3步：等 controller_server 的 FollowPath action 就绪后，再激活 bt_navigator
    # =====================================================
    manager.get_logger().info(f'>>> 步骤3: 等待 FollowPath action server...')
    manager.wait_for_action_server('/follow_path', timeout=20.0)

    manager.get_logger().info(f'再等待 {BT_WAIT}s 确保所有服务就绪...')
    time.sleep(BT_WAIT)

    manager.get_logger().info('>>> 步骤4: activate bt_navigator (异步+轮询)')
    manager.call_change_state('/bt_navigator', Transition.TRANSITION_ACTIVATE)

    # 最终状态检查
    time.sleep(2.0)
    manager.get_logger().info('=== 最终状态 ===')
    for node in nav_nodes:
        state = manager.get_state(node)
        status_icon = '✅' if state == 'active' else ('⚠️' if state == 'inactive' else '❌')
        manager.get_logger().info(f'  {status_icon} {node}: {state}')

    manager.get_logger().info('=== 生命周期管理完成 ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
