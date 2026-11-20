#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class GripperCommander(Node):
    JOINT_NAMES = ("fr3_finger_joint1", "fr3_finger_joint2")

    def __init__(self, command):
        super().__init__("ex5_gripper_command")

        self.target_position = 0.04 if command == "open" else 0.0

        self.current_positions = {}
        self.state_samples_after_command = 0
        self.command_sent = False

        self.publisher = self.create_publisher(
            Float64MultiArray,
            "/gripper_position_controller/commands",
            10,
        )

        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
        )

    def _joint_state_callback(self, msg):
        for joint_name in self.JOINT_NAMES:
            try:
                index = msg.name.index(joint_name)
            except ValueError:
                continue

            if index < len(msg.position):
                self.current_positions[joint_name] = msg.position[index]

        if (
            self.command_sent
            and all(name in self.current_positions for name in self.JOINT_NAMES)
        ):
            self.state_samples_after_command += 1

    def run(self):
        deadline = time.monotonic() + 10.0
        last_publish = 0.0
        publish_count = 0

        command = Float64MultiArray(
            data=[self.target_position, self.target_position]
        )

        initial_positions = None

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            now = time.monotonic()

            if (
                all(name in self.current_positions for name in self.JOINT_NAMES)
                and self.publisher.get_subscription_count() > 0
                and now - last_publish >= 0.1
            ):
                if initial_positions is None:
                    initial_positions = tuple(
                        self.current_positions[name]
                        for name in self.JOINT_NAMES
                    )

                self.command_sent = True
                self.publisher.publish(command)
                publish_count += 1
                last_publish = now

            positions_available = all(
                name in self.current_positions
                for name in self.JOINT_NAMES
            )

            movement_detected = False

            if positions_available and initial_positions is not None:
                movement_detected = any(
                    abs(self.current_positions[name] - initial_positions[i]) > 0.001
                    for i, name in enumerate(self.JOINT_NAMES)
                )

            if (
                publish_count > 0
                and self.state_samples_after_command > 0
                and movement_detected
            ):
                self.get_logger().info(
                    f"Gripper movement detected after "
                    f"{publish_count} command publications."
                )
                return 0

        missing_joints = [
            name for name in self.JOINT_NAMES
            if name not in self.current_positions
        ]

        if missing_joints:
            self.get_logger().error(
                f"No /joint_states position for "
                f"{', '.join(missing_joints)}. "
                "Check that joint_state_broadcaster is active."
            )
        elif self.publisher.get_subscription_count() == 0:
            self.get_logger().error(
                "No controller is subscribed to "
                "/gripper_position_controller/commands. "
                "Check that gripper_position_controller is active."
            )
        else:
            measured = ", ".join(
                f"{name}={self.current_positions[name]:.4f} m"
                for name in self.JOINT_NAMES
            )

            self.get_logger().error(
                "No gripper movement detected. "
                f"Measured positions: [{measured}] after "
                f"{publish_count} published commands."
            )

        return 1


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Open or close the ex5 Franka gripper."
    )
    parser.add_argument("command", choices=("open", "close"))
    parsed_args = parser.parse_args(args)

    rclpy.init(args=[])

    node = GripperCommander(parsed_args.command)

    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
