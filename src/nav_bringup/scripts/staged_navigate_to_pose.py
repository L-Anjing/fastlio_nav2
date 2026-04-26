#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    half_yaw = yaw * 0.5
    q = Quaternion()
    q.w = math.cos(half_yaw)
    q.z = math.sin(half_yaw)
    return q


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_action_name(name):
    return '/' + str(name).strip('/')


class StagedNavigateToPose(Node):
    def __init__(self):
        super().__init__('staged_navigate_to_pose')
        self._cb_group = ReentrantCallbackGroup()

        self.declare_parameter('public_action_name', 'navigate_to_pose')
        self.declare_parameter('raw_action_name', 'navigate_to_pose_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('start_yaw_enabled', True)
        self.declare_parameter('start_yaw_tolerance', 0.20)
        self.declare_parameter('start_yaw_timeout', 8.0)
        self.declare_parameter('start_yaw_kp', 1.6)
        self.declare_parameter('start_yaw_min_vel', 0.20)
        self.declare_parameter('start_yaw_max_vel', 0.90)
        self.declare_parameter('start_yaw_rate', 20.0)
        self.declare_parameter('final_yaw_enabled', True)
        self.declare_parameter('final_yaw_tolerance', 0.25)
        self.declare_parameter('final_yaw_timeout', 8.0)
        self.declare_parameter('final_yaw_kp', 1.4)
        self.declare_parameter('final_yaw_min_vel', 0.12)
        self.declare_parameter('final_yaw_max_vel', 0.70)
        self.declare_parameter('final_yaw_rate', 20.0)

        self._public_action_name = normalize_action_name(self.get_parameter('public_action_name').value)
        self._raw_action_name = normalize_action_name(self.get_parameter('raw_action_name').value)
        self._cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._global_frame = self.get_parameter('global_frame').value
        self._base_frame = self.get_parameter('base_frame').value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._raw_client = ActionClient(
            self, NavigateToPose, self._raw_action_name, callback_group=self._cb_group)
        self._server = ActionServer(
            self,
            NavigateToPose,
            self._public_action_name,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._cb_group)

        self.get_logger().info(
            f'Staged NavigateToPose proxy: {self._public_action_name} -> {self._raw_action_name}, '
            f'final yaw via {self._cmd_vel_topic}')

    def goal_callback(self, goal_request):
        self.get_logger().info(
            'Accepted staged goal: '
            f'({goal_request.pose.pose.position.x:.2f}, {goal_request.pose.pose.position.y:.2f})')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel requested for staged goal')
        return CancelResponse.ACCEPT

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())

    def _wait_future(self, future, goal_handle=None, timeout=None):
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False
            if timeout is not None and time.monotonic() - start > timeout:
                return False
            time.sleep(0.02)
        return future.done()

    def _current_yaw(self):
        tf = self._tf_buffer.lookup_transform(
            self._global_frame, self._base_frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.2))
        return yaw_from_quaternion(tf.transform.rotation)

    def _current_pose2d(self):
        tf = self._tf_buffer.lookup_transform(
            self._global_frame, self._base_frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.2))
        return (
            tf.transform.translation.x,
            tf.transform.translation.y,
            yaw_from_quaternion(tf.transform.rotation),
        )

    def _path_yaw_to_goal(self, target_pose, fallback_yaw):
        try:
            current_x, current_y, current_yaw = self._current_pose2d()
        except TransformException as exc:
            self.get_logger().warn(f'Path yaw uses fallback yaw because TF lookup failed: {exc}')
            return fallback_yaw

        dx = target_pose.position.x - current_x
        dy = target_pose.position.y - current_y
        if math.hypot(dx, dy) < 0.05:
            return current_yaw
        return math.atan2(dy, dx)

    def _rotate_to_yaw(self, goal_handle, target_yaw, stage_name):
        prefix = f'{stage_name}_yaw'
        if not self.get_parameter(f'{prefix}_enabled').value:
            return True

        tolerance = float(self.get_parameter(f'{prefix}_tolerance').value)
        timeout = float(self.get_parameter(f'{prefix}_timeout').value)
        kp = float(self.get_parameter(f'{prefix}_kp').value)
        min_vel = float(self.get_parameter(f'{prefix}_min_vel').value)
        max_vel = float(self.get_parameter(f'{prefix}_max_vel').value)
        rate_hz = float(self.get_parameter(f'{prefix}_rate').value)
        period = 1.0 / max(rate_hz, 1.0)
        start = time.monotonic()

        self.get_logger().info(f'Starting independent {stage_name} yaw alignment')
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._publish_stop()
                return False
            if time.monotonic() - start > timeout:
                self._publish_stop()
                self.get_logger().warn(f'{stage_name.capitalize()} yaw alignment timed out; continuing')
                return True

            try:
                current_yaw = self._current_yaw()
            except TransformException as exc:
                self._publish_stop()
                self.get_logger().warn(f'{stage_name.capitalize()} yaw TF lookup failed: {exc}')
                return True

            error = normalize_angle(target_yaw - current_yaw)
            if abs(error) <= tolerance:
                self._publish_stop()
                self.get_logger().info(f'{stage_name.capitalize()} yaw aligned, error={error:.3f} rad')
                return True

            angular = max(min(abs(error) * kp, max_vel), min_vel)
            cmd = Twist()
            cmd.angular.z = math.copysign(angular, error)
            self._cmd_pub.publish(cmd)
            time.sleep(period)

    def _align_final_yaw(self, goal_handle, target_yaw):
        return self._rotate_to_yaw(goal_handle, target_yaw, 'final')

    def execute_callback(self, goal_handle):
        target_yaw = yaw_from_quaternion(goal_handle.request.pose.pose.orientation)

        if not self._raw_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Raw Nav2 action {self._raw_action_name} not available')
            goal_handle.abort()
            return NavigateToPose.Result()

        raw_goal = NavigateToPose.Goal()
        raw_goal.pose = goal_handle.request.pose
        path_yaw = self._path_yaw_to_goal(raw_goal.pose.pose, target_yaw)
        if not self._rotate_to_yaw(goal_handle, path_yaw, 'start'):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return NavigateToPose.Result()
        raw_goal.pose.pose.orientation = quaternion_from_yaw(path_yaw)
        raw_goal.behavior_tree = goal_handle.request.behavior_tree

        def feedback_callback(feedback_msg):
            goal_handle.publish_feedback(feedback_msg.feedback)

        send_future = self._raw_client.send_goal_async(raw_goal, feedback_callback=feedback_callback)
        if not self._wait_future(send_future, goal_handle=goal_handle, timeout=15.0):
            self._publish_stop()
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                self.get_logger().error('Timed out while sending raw Nav2 goal')
                goal_handle.abort()
            return NavigateToPose.Result()

        raw_goal_handle = send_future.result()
        if raw_goal_handle is None or not raw_goal_handle.accepted:
            self.get_logger().error('Raw Nav2 goal was rejected')
            goal_handle.abort()
            return NavigateToPose.Result()

        result_future = raw_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                raw_goal_handle.cancel_goal_async()
                self._publish_stop()
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.05)

        raw_result = result_future.result()
        if raw_result is None or raw_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f'Raw Nav2 goal failed with status {getattr(raw_result, "status", None)}')
            goal_handle.abort()
            return NavigateToPose.Result()

        if not self._align_final_yaw(goal_handle, target_yaw):
            goal_handle.canceled()
            return NavigateToPose.Result()

        goal_handle.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    node = StagedNavigateToPose()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
