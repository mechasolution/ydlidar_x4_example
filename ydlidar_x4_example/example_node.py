import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32

from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

import math


class YdlidarX4Example(Node):
    def __init__(self):
        super().__init__("ydlidar_x4_example")
        self.get_logger().info("ydlidar_x4_example Start")

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.scan_points_pub_handler = self.create_publisher(
            PointCloud, "scan_points", 10
        )

        self.scan_sub_handler = self.create_subscription(
            LaserScan, "scan", self.scan_sub_callback, qos_profile
        )

        self.points_sub_handler = self.create_subscription(
            PointCloud, "scan_points", self.points_sub_callback, 10
        )

    def scan_sub_callback(self, data):
        pointcloud_msg = PointCloud()
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        pointcloud_msg.header.frame_id = "laser_frame"

        angle_increment = data.angle_increment
        angle = data.angle_min

        # 각도, 거리 -> 좌표 (m단위)
        for point in data.ranges:
            coordinate_x = math.cos(angle) * point
            coordinate_y = math.sin(angle) * point
            angle += angle_increment
            if abs(coordinate_x) == 0 and abs(coordinate_y) == 0:
                continue
            point_msg = Point32()
            point_msg.x = coordinate_x
            point_msg.y = coordinate_y
            point_msg.z = float(0)

            pointcloud_msg.points.append(point_msg)

        self.scan_points_pub_handler.publish(pointcloud_msg)

    def points_sub_callback(self, data):
        self.get_logger().info(
            "points[0] = x:{0}, y:{1}".format(data.points[0].x, data.points[0].y)
        )


def main(args=None):
    rclpy.init(args=args)

    ydlidar_x4_example = YdlidarX4Example()

    rclpy.spin(ydlidar_x4_example)
    ydlidar_x4_example.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
