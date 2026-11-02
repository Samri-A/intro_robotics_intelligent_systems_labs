import math

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lab_interfaces.action import DriveDistance
from my_robot_interfaces.action import Rotate


class SquareDriver(Node):

    def __init__(self):
        super().__init__("square_driver")
        self.declare_parameter("side_length", 0.5)

        self.cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.drive_client = ActionClient(self, DriveDistance, "drive_distance")
        self.rotate_client = ActionClient(self, Rotate, "rotate")

        self.has_odom = False
        self.create_subscription(Odometry, "odom", self.on_odom, 10)

    def on_odom(self, _msg):
        self.has_odom = True

    def wait_for_ready(self):
        self.get_logger().info("Waiting for action servers...")
        self.drive_client.wait_for_server()
        self.rotate_client.wait_for_server()
        self.get_logger().info("Action servers connected. Waiting for odometry...")
        while rclpy.ok() and not self.has_odom:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Odometry received. Ready to drive square.")

    def on_drive_feedback(self, feedback_msg):
        self.get_logger().info(f"Travelled {feedback_msg.feedback.travelled:.2f} m")

    def on_rotate_feedback(self, feedback_msg):
        self.get_logger().info(f"Turned {feedback_msg.feedback.turned:.2f} rad")

    def drive(self, distance):
        goal = DriveDistance.Goal()
        goal.distance = float(distance)
        send_future = self.drive_client.send_goal_async(goal, feedback_callback=self.on_drive_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("drive_distance goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    def rotate(self, angle):
        goal = Rotate.Goal()
        goal.angle = float(angle)
        send_future = self.rotate_client.send_goal_async(goal, feedback_callback=self.on_rotate_feedback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("rotate goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED

    def stop(self):
        self.cmd_vel.publish(Twist())

    def run(self):
        self.wait_for_ready()

        side_length = float(self.get_parameter("side_length").value)
        quarter_turn = math.pi / 2.0  # 90 degrees left

        self.get_logger().info(f"Starting square of side length {side_length:.2f} m")

        for side in range(1, 5):
            self.get_logger().info(f"Side {side}/4: driving {side_length:.2f} m")
            success = self.drive(side_length)
            if not success:
                self.stop()
                self.get_logger().error(f"Failed to drive forward on side {side}")
                return

            self.get_logger().info(f"Side {side}/4: turning left 90 degrees")
            success = self.rotate(quarter_turn)
            if not success:
                self.stop()
                self.get_logger().error(f"Failed to rotate on side {side}")
                return

        self.get_logger().info("Completed driving square!")


def main():
    rclpy.init()
    node = SquareDriver()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
