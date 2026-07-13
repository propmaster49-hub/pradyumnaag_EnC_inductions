#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from kratos_pradyumnaag_msgs.msg import RoverStatus


class RoverStatusMsgPublisher(Node):
    """
    Publishes a RoverStatus custom message at 2 Hz.
    """

    def __init__(self):
        super().__init__("rover_status_msg_publisher")

        self.publisher_ = self.create_publisher(
            RoverStatus,
            "rover_status",
            10
        )

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        """
        Creates and publishes a RoverStatus message.
        """
        msg = RoverStatus()

        msg.battery_percentage = 85.0
        msg.velocity = 3.0
        msg.emergency_stop = False
        msg.mode = "AUTONOMOUS"

        self.publisher_.publish(msg)

        self.get_logger().info(
            f"Battery: {msg.battery_percentage:.1f}% | "
            f"Velocity: {msg.velocity:.2f} m/s | "
            f"E-Stop: {msg.emergency_stop} | "
            f"Mode: {msg.mode}"
        )


def main(args=None):
    """
    Initializes ROS2, starts the publisher node, and shuts down cleanly.
    """
    rclpy.init(args=args)

    node = RoverStatusMsgPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()