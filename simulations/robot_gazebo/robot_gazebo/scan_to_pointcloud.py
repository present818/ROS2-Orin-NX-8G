import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class ScanToPointCloud(Node):
    def __init__(self):
        super().__init__('scan_to_pointcloud')

        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan/points')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.publisher = self.create_publisher(PointCloud2, output_topic, 10)
        self.subscription = self.create_subscription(
            LaserScan,
            input_topic,
            self.scan_callback,
            scan_qos,
        )

        self._logged_first_cloud = False
        self.get_logger().info(
            f'Projecting LaserScan {input_topic} to PointCloud2 {output_topic}'
        )

    def scan_callback(self, scan_msg):
        points = []
        angle = scan_msg.angle_min
        min_range = scan_msg.range_min
        max_range = scan_msg.range_max

        for distance in scan_msg.ranges:
            if math.isfinite(distance) and min_range <= distance <= max_range:
                points.append((
                    distance * math.cos(angle),
                    distance * math.sin(angle),
                    0.0,
                ))
            angle += scan_msg.angle_increment

        cloud_msg = point_cloud2.create_cloud_xyz32(scan_msg.header, points)
        self.publisher.publish(cloud_msg)

        if not self._logged_first_cloud:
            self.get_logger().info(
                f'Published first point cloud with {len(points)} points'
            )
            self._logged_first_cloud = True


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointCloud()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
