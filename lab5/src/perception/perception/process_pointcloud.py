import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header

class RealSensePCSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_pc_subscriber')

        # Plane coefficients and max distance (meters)
        self.declare_parameter('plane.a', 0.0)
        self.declare_parameter('plane.b', 0.0)
        self.declare_parameter('plane.c', 0.0)
        self.declare_parameter('plane.d', 0.0)
        self.declare_parameter('max_distance', 0.6)

        self.a = self.get_parameter('plane.a').value
        self.b = self.get_parameter('plane.b').value
        self.c = self.get_parameter('plane.c').value
        self.d = self.get_parameter('plane.d').value
        self.max_distance = self.get_parameter('max_distance').value

        # Subscribers
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.pointcloud_callback,
            10
        )

        # Publishers
        self.cube_pose_pub = self.create_publisher(PointStamped, '/cube_pose', 1)
        self.filtered_points_pub = self.create_publisher(PointCloud2, '/filtered_points', 1)

        self.get_logger().info("Subscribed to PointCloud2 topic and marker publisher ready")

    def pointcloud_callback(self, msg: PointCloud2):
        # Convert PointCloud2 to Nx3 array
        points = []
        for p in pc2.read_points(msg, field_names=('x','y','z'), skip_nans=True):
            points.append([p[0], p[1], p[2]])

        points = np.array(points)
        # ------------------------
        #TODO: Add your code here!
        # ------------------------

        # Apply max distance filter
        z = points[:, 2]
        y = points[:, 1]
        distance_mask = (z > 0) & (z <= self.max_distance)
       
        # Apply other filtering relative to plane (keep points on the +y side of the reference plane)
        # For plane f(x,y,z) = a*x + b*y + c*z + d, the +y side is defined by the sign of b:
        # - If b >= 0, keep f > 0
        # - If b < 0, keep f < 0
        f = self.a * points[:, 0] + self.b * points[:, 1] + self.c * points[:, 2] + self.d
        plane_eps = 1e-3
        if self.b >= 0:
            # Flip: treat y-positive side as f < 0 (exclude plane by epsilon)
            plane_ypos_mask = f < -plane_eps
        else:
            plane_ypos_mask = f > plane_eps

        # Combine masks: within z range and on +y side of the plane
        mask = distance_mask & plane_ypos_mask
        filtered_points = points[mask]

        # Compute position of the cube via remaining points
        if len(filtered_points) > 0:
            cube_x = float(np.mean(filtered_points[:, 0]))
            cube_y = float(np.mean(filtered_points[:, 1]))
            cube_z = float(np.mean(filtered_points[:, 2]))
        else:
            cube_x = 0.0
            cube_y = 0.0
            cube_z = 0.0

        # Minimal debug to help verify masks
        self.get_logger().info(
            f"z_ok: {int(np.sum(distance_mask))}, ypos_ok: {int(np.sum(plane_ypos_mask))}, both: {int(np.sum(mask))}"
        )
        self.get_logger().info(f"Filtered points: {len(filtered_points)}")

        cube_pose = PointStamped()
        # Fill in message
        cube_pose.header = msg.header
        cube_pose.point.x = cube_x
        cube_pose.point.y = cube_y
        cube_pose.point.z = cube_z

        self.cube_pose_pub.publish(cube_pose)

        self.publish_filtered_points(filtered_points, msg.header)

    def publish_filtered_points(self, filtered_points: np.ndarray, header: Header):
        # Create PointCloud2 message from filtered Nx3 array
        filtered_msg = pc2.create_cloud_xyz32(header, filtered_points.tolist())
        self.filtered_points_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealSensePCSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()