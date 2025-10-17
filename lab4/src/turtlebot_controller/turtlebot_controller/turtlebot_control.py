#!/usr/bin/env python3
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException

class TurtleBotController(Node):
    def __init__(self, frame1, frame2):
        super().__init__('turtlebot_controller')

        self.turtle_frame = frame1
        self.ar_frame = frame2

        # 优化的控制参数
        self.K_linear = 0.8   # 线速度增益
        self.K_angular = 3.0  # 角速度增益
        
        # 控制参数
        self.goal_distance = 0.5    # 目标距离（米）
        self.stop_distance = 0.15   # 停止距离（米）
        self.safety_distance = 0.05 # 安全距离（5厘米）- 防止撞击
        self.angle_tolerance = 0.05 # 角度容忍度（弧度）
        
        # 速度限制
        self.max_linear_vel = 0.4   # 最大线速度
        self.min_linear_vel = 0.05  # 最小线速度
        self.max_angular_vel = 1.2  # 最大角速度
        
        # 控制状态
        self.last_error = 0.0
        self.integral_error = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info(
            f"TurtleBotController: robot='{self.turtle_frame}', target='{self.ar_frame}' | "
            f"Goal distance={self.goal_distance}m, Stop distance={self.stop_distance}m, Safety distance={self.safety_distance}m"
        )

    def loop(self):
        try:
            # 获取从机器msg.name = ["joint1", "joint2"]
msg.position = [0.1, 0.2]
msg.velocity = [0.0, 0.0]
msg.effort = [0.0, 0.0]人坐标系到目标坐标系的变换
            tf = self.tf_buffer.lookup_transform(self.turtle_frame, self.ar_frame, Time())

            # 提取位置信息
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            
            # 计算距离和角度
            distance = np.sqrt(x**2 + y**2)
            angle_to_target = np.arctan2(y, x)
            
            # 创建控制命令
            control_cmd = Twist()
            
            # 距离控制逻辑
            if distance <= self.safety_distance:
                # 安全距离内 - 立即停止防止撞击
                control_cmd.linear.x = 0.0
                control_cmd.angular.z = 0.0
                self.get_logger().info(f"🛑 SAFETY STOP! Distance too close: {distance:.3f}m (< {self.safety_distance}m)")
                
            elif distance <= self.stop_distance:
                # 停止区域 - 已到达目标
                control_cmd.linear.x = 0.0
                control_cmd.angular.z = 0.0
                self.get_logger().info(f"🎯 Target reached! Distance: {distance:.3f}m")
                
            elif distance <= self.goal_distance:
                # 在目标区域内，慢速精确靠近
                if abs(angle_to_target) > self.angle_tolerance:
                    # 需要调整角度
                    control_cmd.linear.x = self.min_linear_vel
                    control_cmd.angular.z = self.K_angular * angle_to_target * 0.8
                else:
                    # 角度已对准，慢速前进，但要确保不会过于接近
                    remaining_distance = distance - self.safety_distance
                    if remaining_distance > 0.02:  # 至少保留2厘米缓冲
                        linear_vel = max(self.min_linear_vel, 
                                       self.K_linear * remaining_distance * 0.3)
                        control_cmd.linear.x = min(linear_vel, 0.08)  # 非常慢的速度
                    else:
                        control_cmd.linear.x = 0.0  # 太接近了，停止
                    control_cmd.angular.z = self.K_angular * angle_to_target * 0.3
                    
            else:
                # 距离较远，正常导航
                if abs(angle_to_target) > 0.3:
                    # 角度偏差大，主要转向
                    control_cmd.linear.x = 0.1
                    control_cmd.angular.z = self.K_angular * angle_to_target
                elif abs(angle_to_target) > 0.1:
                    # 角度偏差中等，边转边进
                    control_cmd.linear.x = self.K_linear * distance * 0.7
                    control_cmd.angular.z = self.K_angular * angle_to_target * 0.8
                else:
                    # msg.name = ["joint1", "joint2"]
msg.position = [0.1, 0.2]
msg.velocity = [0.0, 0.0]
msg.effort = [0.0, 0.0]角度基本对准，主要前进
                    control_cmd.linear.x = self.K_linear * distance
                    control_cmd.angular.z = self.K_angular * angle_to_target * 0.5
            
            # 应用速度限制和平滑处理
            control_cmd.linear.x = np.clip(control_cmd.linear.x, 
                                         0.0, self.max_linear_vel)
            control_cmd.angular.z = np.clip(control_cmd.angular.z, 
                                          -self.max_angular_vel, self.max_angular_vel)
            
            # 速度平滑处理（避免突变）
            if hasattr(self, 'last_linear_vel'):
                max_accel = 0.1  # 最大加速度
                vel_diff = control_cmd.linear.x - self.last_linear_vel
                if abs(vel_diff) > max_accel:
                    control_cmd.linear.x = self.last_linear_vel + np.sign(vel_diff) * max_accel
            
            self.last_linear_vel = control_cmd.linear.x
            
            # 发布控制命令
            self.pub.publish(control_cmd)
            
            # Status display
            status = ""
            if distance <= self.safety_distance:
                status = "🛑 Safety Stop"
            elif distance <= self.stop_distance:
                status = "🎯 Target Reached"
            elif distance <= self.goal_distance:
                status = "🔍 Precise Approach"
            elif abs(angle_to_target) > 0.3:
                status = "🔄 Turning"
            else:
                status = "➡️ Moving Forward"
            
            self.get_logger().info(
                f"{status} | Distance: {distance:.3f}m, Angle: {angle_to_target:.3f}rad, "
                f"Linear vel: {control_cmd.linear.x:.3f}, Angular vel: {control_cmd.angular.z:.3f}"
            )

        except (TransformException, LookupException, ConnectivityException, ExtrapolationException) as e:
            # 如果无法获取变换，发布零速度命令
            self.get_logger().warn(f"⚠️ Transform lookup failed: {e}")
            self.pub.publish(Twist())
    
    def destroy_node(self):
        try:
            self.pub.publish(Twist())
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print("Usage: python3 turtlebot_control.py frame1 frame2")
        rclpy.shutdown()
        return

    frame1 = sys.argv[1]
    frame2 = sys.argv[2]

    node = TurtleBotController(frame1, frame2)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
