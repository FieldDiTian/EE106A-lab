#!/usr/bin/env python3
"""
tf_echo.py - Task 3: tf listener node that duplicates tf2_echo functionality

Equivalent to: ros2 run tf2_ros tf2_echo <target_frame> <source_frame>
Usage: ros2 run forward_kinematics tf_echo <target_frame> <source_frame>
"""

import sys
import argparse
import rclpy
from rclpy.node import Node
import tf2_ros


class TFEchoNode(Node):
    def __init__(self, target_frame, source_frame):
        super().__init__('tf_echo')
        self.target = target_frame
        self.source = source_frame
        
        # tf2 Buffer and TransformListener (自动订阅 /tf)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        
        # Timer to periodically lookup transforms (10Hz like tf2_echo)
        self.timer = self.create_timer(0.1, self.on_timer)
        
        self.get_logger().info(f'tf_echo: Waiting for transform from {source_frame} to {target_frame}')

    def on_timer(self):
        try:
            # Lookup transform: source_frame in target_frame coordinates
            trans = self.buffer.lookup_transform(
                self.target, 
                self.source, 
                rclpy.time.Time()
            )
            
            # Extract translation and rotation
            t = trans.transform.translation
            q = trans.transform.rotation
            
            # Print in tf2_echo compatible format
            stamp = trans.header.stamp
            print(f"At time {stamp.sec}.{stamp.nanosec:09d}")
            print(f"- Translation: [{t.x:.3f}, {t.y:.3f}, {t.z:.3f}]")
            print(f"- Rotation: in Quaternion [{q.x:.3f}, {q.y:.3f}, {q.z:.3f}, {q.w:.3f}]")
            print()
            
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'Lookup exception: {e}')
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f'Connectivity exception: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'Extrapolation exception: {e}')


def main():
    """
    Main function that parses command line arguments and runs tf_echo node.
    """
    parser = argparse.ArgumentParser(description='tf_echo - tf listener node')
    parser.add_argument('target_frame', help='Target frame')
    parser.add_argument('source_frame', help='Source frame')
    
    # Parse known args to separate ROS args from our args
    args, ros_args = parser.parse_known_args()
    
    rclpy.init(args=ros_args)
    
    try:
        node = TFEchoNode(args.target_frame, args.source_frame)
        print(f"tf_echo: Listening for transform from '{args.source_frame}' to '{args.target_frame}'")
        print("Press Ctrl+C to exit")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down tf_echo...")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()