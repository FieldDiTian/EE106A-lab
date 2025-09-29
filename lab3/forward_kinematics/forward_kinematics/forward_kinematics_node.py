#!/usr/bin/env python3
"""
Forward Kinematics ROS2 Node for UR7e Robot

This node subscribes to joint states and computes forward kinematics,
printing the 4x4 homogeneous transformation matrix to the terminal.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

# Import the forward kinematics functions
try:
    from .forward_kinematics import ur7e_forward_kinematics_from_joint_state
except ImportError:
    # Fallback for direct execution
    from forward_kinematics import ur7e_forward_kinematics_from_joint_state


class ForwardKinematicsNode(Node):
    """
    ROS2 Node that subscribes to joint states and computes forward kinematics.
    """
    
    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        # Set numpy print options for clean output
        np.set_printoptions(precision=4, suppress=True)
        
        # Declare parameters
        self.declare_parameter('joint_topic', '/joint_states')
        self.declare_parameter('joint_names', [
            'shoulder_pan_joint',
            'shoulder_lift_joint', 
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ])
        
        # Get parameters
        joint_topic = self.get_parameter('joint_topic').get_parameter_value().string_value
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        
        # Create subscriber
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            joint_topic,
            self.joint_state_callback,
            10  # QoS depth
        )
        
        self.get_logger().info(f'Forward Kinematics Node started!')
        self.get_logger().info(f'Subscribing to: {joint_topic}')
        self.get_logger().info(f'Expected joint names: {self.joint_names}')
        
        # Counter for received messages
        self.msg_count = 0
    
    def joint_state_callback(self, msg):
        """
        Callback function for joint state messages.
        
        Args:
            msg (sensor_msgs.msg.JointState): Joint state message
        """
        try:
            self.msg_count += 1
            
            # Log joint names on first message for debugging
            if self.msg_count == 1:
                self.get_logger().info(f'Received joint names: {msg.name}')
            
            # Compute forward kinematics
            T = ur7e_forward_kinematics_from_joint_state(msg)
            
            # Print the transformation matrix with nice formatting
            print(f"\n--- Forward Kinematics Result (Message #{self.msg_count}) ---")
            print("T =")
            print(f"[[{T[0,0]:8.4f} {T[0,1]:8.4f} {T[0,2]:8.4f} {T[0,3]:8.4f}]")
            print(f" [{T[1,0]:8.4f} {T[1,1]:8.4f} {T[1,2]:8.4f} {T[1,3]:8.4f}]")
            print(f" [{T[2,0]:8.4f} {T[2,1]:8.4f} {T[2,2]:8.4f} {T[2,3]:8.4f}]")
            print(f" [{T[3,0]:8.4f} {T[3,1]:8.4f} {T[3,2]:8.4f} {T[3,3]:8.4f}]]")
            
            # Also print joint angles for reference (optional)
            angles = []
            for joint_name in self.joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    angles.append(msg.position[idx])
            
            if len(angles) == 6:
                print(f"Joint angles (rad): {np.array(angles)}")
            print("-" * 55)
            
        except Exception as e:
            self.get_logger().error(f'Error computing forward kinematics: {str(e)}')
            self.get_logger().error(f'Available joints: {msg.name}')
            self.get_logger().error(f'Expected joints: {self.joint_names}')


def main(args=None):
    """
    Main function to initialize and spin the ROS2 node.
    
    Args:
        args: Command line arguments (optional)
    """
    rclpy.init(args=args)
    
    try:
        # Create and spin the node
        node = ForwardKinematicsNode()
        
        print("\n" + "="*60)
        print("  Forward Kinematics Node for UR7e Robot")
        print("  Listening for joint states...")
        print("  Press Ctrl+C to exit")
        print("="*60 + "\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down forward kinematics node...")
    except Exception as e:
        print(f"Error in forward kinematics node: {e}")
    finally:
        # Clean up
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()