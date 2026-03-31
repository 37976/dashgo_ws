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
        self.declare_parameter("filtered_scan_topic", "/scan_filtered")
        self.declare_parameter("points_topic", "/points_raw")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("laser_height", 0.18)
        self.declare_parameter("x_offset", 0.0)
        self.declare_parameter("y_offset", 0.0)
        self.declare_parameter("min_valid_range", 0.30)

        scan_topic = self.get_parameter("scan_topic").value
        filtered_scan_topic = self.get_parameter("filtered_scan_topic").value
        points_topic = self.get_parameter("points_topic").value

        self.output_frame = self.get_parameter("output_frame").value
        self.laser_height = float(self.get_parameter("laser_height").value)
        self.x_offset = float(self.get_parameter("x_offset").value)
        self.y_offset = float(self.get_parameter("y_offset").value)
        self.min_valid_range = float(self.get_parameter("min_valid_range").value)

        self.filtered_scan_pub = self.create_publisher(LaserScan, filtered_scan_topic, 10)
        self.points_pub = self.create_publisher(PointCloud2, points_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Bridging {scan_topic} -> {filtered_scan_topic} and {points_topic} "
            f"with min valid range {self.min_valid_range:.2f} m"
        )

    def scan_callback(self, msg):
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = max(msg.range_min, self.min_valid_range)
        filtered_scan.range_max = msg.range_max
        filtered_scan.intensities = list(msg.intensities)

        points = []
        angle = msg.angle_min

        range_min = max(msg.range_min, self.min_valid_range)
        range_max = msg.range_max if msg.range_max > 0.0 else float("inf")
        filtered_ranges = []

        for distance in msg.ranges:
            if math.isfinite(distance) and range_min <= distance <= range_max:
                filtered_ranges.append(distance)
                x = distance * math.cos(angle) + self.x_offset
                y = distance * math.sin(angle) + self.y_offset
                points.append((x, y, self.laser_height))
            else:
                filtered_ranges.append(float("inf"))
            angle += msg.angle_increment

        filtered_scan.ranges = filtered_ranges
        self.filtered_scan_pub.publish(filtered_scan)

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
