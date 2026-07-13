#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class IKController(Node):
    """
    ROS2 node that computes inverse kinematics for a simple robotic arm
    and publishes joint angles to /joint_states.
    """

    def __init__(self):
        super().__init__("ik_controller")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        # Link lengths
        self.L1 = 0.35
        self.L2 = 0.35
        self.base_height = 0.15

        # Current end effector position
        self.x = 0.70
        self.y = 0.0
        self.z = 0.15

    def inverse_kinematics(self, x, y, z):
        """
        Computes inverse kinematics for the arm.

        Args:
            x (float): Target x.
            y (float): Target y.
            z (float): Target z.

        Returns:
            tuple:
                (base, shoulder, elbow)

            Returns None if unreachable.
        """

        # Base rotation
        base = math.atan2(y, x)

        # Horizontal distance from base
        r = math.sqrt(x**2 + y**2)

        # Height relative to shoulder
        z_rel = z - self.base_height

        d = math.sqrt(r**2 + z_rel**2)

        if d > (self.L1 + self.L2):
            return None

        if d < abs(self.L1 - self.L2):
            return None

        cos_elbow = (
            d**2
            - self.L1**2
            - self.L2**2
        ) / (2 * self.L1 * self.L2)

        cos_elbow = max(-1.0, min(1.0, cos_elbow))

        elbow = math.acos(cos_elbow)

        shoulder = (
            math.atan2(z_rel, r)
            -
            math.atan2(
                self.L2 * math.sin(elbow),
                self.L1 + self.L2 * math.cos(elbow)
            )
        )

        return base, shoulder, elbow

    def publish_joint_state(self, base, shoulder, elbow):
        """
        Publishes joint angles to /joint_states.
        """

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.name = [
            "base_yaw_joint",
            "shoulder_joint",
            "elbow_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
            "gripper_servo_joint",
        ]

        msg.position = [
            base,
            shoulder,
            elbow,
            0.0,
            0.0,
            0.0,
        ]

        msg.velocity = []
        msg.effort = []

        self.publisher.publish(msg)

    def run(self):
        """
        Reads user input continuously.
        """

        while rclpy.ok():

            print("\nCurrent Position")
            print(f"x = {self.x:.3f}")
            print(f"y = {self.y:.3f}")
            print(f"z = {self.z:.3f}")

            axis = input("Axis (x/y/z): ").lower()

            if axis not in ["x", "y", "z"]:
                print("Invalid axis.")
                continue

            try:
                displacement = float(input("Displacement (m): "))
            except ValueError:
                print("Invalid number.")
                continue

            target_x = self.x
            target_y = self.y
            target_z = self.z

            if axis == "x":
                target_x += displacement

            elif axis == "y":
                target_y += displacement

            else:
                target_z += displacement

            angles = self.inverse_kinematics(
                target_x,
                target_y,
                target_z
            )

            if angles is None:
                print("Target outside reachable workspace.")
                continue

            base, shoulder, elbow = angles

            self.publish_joint_state(
                base,
                shoulder,
                elbow
            )

            self.x = target_x
            self.y = target_y
            self.z = target_z

            print("Motion successful.")


def main(args=None):

    rclpy.init(args=args)

    node = IKController()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
