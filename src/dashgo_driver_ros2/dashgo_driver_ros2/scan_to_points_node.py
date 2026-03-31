import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data


class ScanToPointsNode(Node):
    def __init__(self):
        super().__init__("scan_to_points")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("points_topic", "/points_raw")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("laser_height", 0.18)
        self.declare_parameter("x_offset", 0.0)
        self.declare_parameter("y_offset", 0.0)

        scan_topic = self.get_parameter("scan_topic").value
        points_topic = self.get_parameter("points_topic").value

        self.output_frame = self.get_parameter("output_frame").value
        self.laser_height = float(self.get_parameter("laser_height").value)
        self.x_offset = float(self.get_parameter("x_offset").value)
        self.y_offset = float(self.get_parameter("y_offset").value)

        self.points_pub = self.create_publisher(PointCloud2, points_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Bridging {scan_topic} -> {points_topic} in frame {self.output_frame}"
        )

    def scan_callback(self, msg):
        points = []
        angle = msg.angle_min

        range_min = msg.range_min if msg.range_min > 0.0 else 0.0
        range_max = msg.range_max if msg.range_max > 0.0 else float("inf")

        for distance in msg.ranges:
            if math.isfinite(distance) and range_min <= distance <= range_max:
                x = distance * math.cos(angle) + self.x_offset
                y = distance * math.sin(angle) + self.y_offset
                points.append((x, y, self.laser_height))
            angle += msg.angle_increment

        cloud = point_cloud2.create_cloud_xyz32(
            msg.header,
            points,
        )
        cloud.header.frame_id = self.output_frame
        self.points_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ScanToPointsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
