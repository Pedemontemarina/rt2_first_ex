import rclpy
import time
import threading
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from custom_interfaces.action import XPosition


class PositionActionServer(Node):

    def __init__(self):
        super().__init__('position_action_server')

        self._cb_group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._current_x = 0.0

        self._current_goal_handle = None
        self._preempt_requested_for = None

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10,
                                 callback_group=self._cb_group)

        self._action_server = ActionServer(
            self,
            XPosition,
            'xposition',
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            goal_callback=self.goal_callback,
            callback_group=self._cb_group)

    def odom_callback(self, msg):
        self._current_x = msg.pose.pose.position.x

    def cancel_callback(self, goal_handle):
        self.get_logger().warn('Received cancel request from client')
        return CancelResponse.ACCEPT

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received goal request: x_target = {goal_request.x_target}')
        with self._lock:
            if self._current_goal_handle is not None:
                self.get_logger().warn('Preempting previous goal')
                self._preempt_requested_for = self._current_goal_handle
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal: x_target = {goal_handle.request.x_target}')

        with self._lock:
            self._current_goal_handle = goal_handle

        target_x = goal_handle.request.x_target
        threshold = 0.1
        feedback_msg = XPosition.Feedback()
        vel_msg = Twist()

        while True:

            # 1) cancel richiesto dal client
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Cancel requested, stopping')
                vel_msg.linear.x = 0.0
                self._cmd_vel_pub.publish(vel_msg)
                goal_handle.canceled()
                result = XPosition.Result()
                result.success = False
                with self._lock:
                    if self._current_goal_handle is goal_handle:
                        self._current_goal_handle = None
                return result

            # 2) preemption da nuovo goal
            with self._lock:
                preempt_id = (
                    self._preempt_requested_for.goal_id
                    if self._preempt_requested_for is not None
                    else None
                )

            preempt_me = (preempt_id is not None and goal_handle.goal_id == preempt_id)

            if preempt_me:
                self.get_logger().warn('Preempted by newer goal, aborting')
                vel_msg.linear.x = 0.0
                self._cmd_vel_pub.publish(vel_msg)
                goal_handle.abort()
                result = XPosition.Result()
                result.success = False
                with self._lock:
                    if self._current_goal_handle is goal_handle:
                        self._current_goal_handle = None
                    self._preempt_requested_for = None
                return result

            error = target_x - self._current_x

            # 3) arrivato
            if abs(error) < threshold:
                vel_msg.linear.x = 0.0
                self._cmd_vel_pub.publish(vel_msg)
                goal_handle.succeed()
                result = XPosition.Result()
                result.success = True
                with self._lock:
                    if self._current_goal_handle is goal_handle:
                        self._current_goal_handle = None
                return result

            # 4) muovi il robot
            vel_msg.linear.x = max(min(error * 0.5, 0.5), -0.5)
            self._cmd_vel_pub.publish(vel_msg)

            # 5) manda feedback
            feedback_msg.current_x = self._current_x
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = PositionActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()