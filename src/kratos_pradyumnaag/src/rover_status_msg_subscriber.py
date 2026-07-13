#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from kratos_pradyumnaag_msgs.msg import RoverStatus


class RoverStatusMsgSubscriber(Node):
    """
    Subscribes to the RoverStatus custom message and prints its contents.
    """

    def __init__(self):
        super().__init__("rover_status_msg_subscriber")

        self.subscription = self.create_subscription(
            RoverStatus,
            "rover_status",
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """
        Receives and displays a RoverStatus message.
        """
        self.get_logger().info(
            f"Battery: {msg.battery_percentage:.1f}% | "
            f"Velocity: {msg.velocity:.2f} m/s | "
            f"E-Stop: {msg.emergency_stop} | "
            f"Mode: {msg.mode}"
        )


def main(args=None):
    """
    Initializes ROS2, starts the subscriber node, and shuts down cleanly.
    """
    rclpy.init(args=args)

    node = RoverStatusMsgSubscriber()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()