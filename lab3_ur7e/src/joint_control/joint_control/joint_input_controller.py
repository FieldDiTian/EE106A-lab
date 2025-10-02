#!/usr/bin/env python3
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointInputController(Node):
    def __init__(self):
        super().__init__('ur7e_joint_input_controller')

        self.joint_names = [
            'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
            'wrist_2_joint', 'wrist_3_joint'
        ]
        self.joint_positions = [0.0] * 6
        self.got_joint_states = False

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/scaled_joint_trajectory_controller/joint_trajectory',
            10,
        )

        self.running = True
        self.input_thread = threading.Thread(target=self.prompt_loop, daemon=True)
        self.input_thread.start()

    def joint_state_callback(self, msg: JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
        self.got_joint_states = True

    def prompt_loop(self):
        print("Joint input controller running.")
        print(
            "Enter six joint angles in radians separated by spaces for joints: "
            "shoulder_lift, elbow, wrist_1, wrist_2, wrist_3, shoulder_pan."
        )
        print("Type 'exit' to quit.")

        waiting_reported = False
        while self.running:
            if not self.got_joint_states:
                if not waiting_reported:
                    print("Waiting for joint states...")
                    waiting_reported = True
                time.sleep(0.1)
                continue

            try:
                raw = input("target angles [rad]: ")
            except EOFError:
                self.running = False
                break
            except KeyboardInterrupt:
                self.running = False
                print()
                break

            if not raw.strip():
                continue

            if raw.strip().lower() in {'exit', 'quit'}:
                self.running = False
                break

            parts = raw.replace(',', ' ').split()
            if len(parts) != 6:
                print("Please enter exactly six values.")
                continue

            try:
                target_positions = [float(p) for p in parts]
            except ValueError:
                print("Invalid number format. Use radians, e.g., 0.0 1.57 ...")
                continue

            self.send_trajectory(target_positions)

        print("Input loop stopped.")

    def send_trajectory(self, positions):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.velocities = [0.0] * 6
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.publisher.publish(traj)
        self.joint_positions = positions
        print(
            "Commanded joints: "
            + " ".join(f"{pos:.3f}" for pos in positions)
        )

    def stop(self):
        self.running = False


def main(args=None):
        rclpy.init(args=args)
        node = JointInputController()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.stop()
            print("\nExiting joint input controller...")
        finally:
            node.stop()
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
