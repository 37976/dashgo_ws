#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomToMapTransformer(Node):
    def __init__(self):
        super().__init__('odom_to_map_transformer')
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transform()
        self.get_logger().info('Publishing static identity transform: map -> odom')

    def publish_static_transform(self):
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'map'
        static_transform.child_frame_id = 'odom'
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(static_transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToMapTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
