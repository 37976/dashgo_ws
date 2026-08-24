#!/usr/bin/env python3
"""
slam_controller: Bridge between slam_toolbox and the DashGo navigation stack.

- Subscribes to /slam_map (from slam_toolbox) with transient_local QoS
- Subscribes to /control_mode to know when mapping mode is active
- When mode == "mapping": relays /slam_map to /combined_grid
- Publishes /mapping_status for web UI
- Provides services to save the current SLAM map as PGM+YAML
"""
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from std_srvs.srv import Trigger

from nav_slam.map_saver import save_occupancy_grid


class SlamController(Node):
    """Bridge between slam_toolbox and DashGo navigation/web stack."""

    def __init__(self):
        super().__init__("slam_controller")

        # --- Parameters ---
        self.declare_parameter("slam_map_topic", "/slam_map")
        self.declare_parameter("combined_grid_topic", "/combined_grid")
        self.declare_parameter("map_save_dir", "")
        self.declare_parameter("relay_rate_hz", 2.0)
        self.declare_parameter("map_frame_id", "map")

        self.slam_map_topic = str(self.get_parameter("slam_map_topic").value)
        self.combined_grid_topic = str(self.get_parameter("combined_grid_topic").value)
        self.map_save_dir = str(self.get_parameter("map_save_dir").value)
        self.relay_rate_hz = float(self.get_parameter("relay_rate_hz").value)
        self.map_frame_id = str(self.get_parameter("map_frame_id").value)

        # Default save directory: nav_slam/map/ under workspace
        if not self.map_save_dir:
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.map_save_dir = os.path.join(pkg_dir, "map")

        # --- State ---
        self.latest_slam_map: OccupancyGrid | None = None
        self.control_mode = "nav"
        self.is_mapping = False
        self.save_filename = "dashgo_slam_map"
        self.relay_timer = None

        # --- Subscriptions ---
        # transient_local: get the latest map even if published before we subscribed
        slam_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, self.slam_map_topic, self.slam_map_callback, slam_qos
        )
        self.create_subscription(String, "/control_mode", self.mode_callback, 10)

        # --- Publications ---
        self.combined_grid_pub = self.create_publisher(
            OccupancyGrid, self.combined_grid_topic, 10
        )
        self.status_pub = self.create_publisher(String, "/mapping_status", 10)

        # --- Services ---
        self.save_srv = self.create_service(Trigger, "~/save_map", self.save_map_callback)
        self.set_filename_srv = self.create_service(
            Trigger, "~/set_map_filename", self.set_filename_callback
        )

        self.get_logger().info("slam_controller started")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def slam_map_callback(self, msg: OccupancyGrid):
        """Cache the latest SLAM map."""
        self.latest_slam_map = msg

    def mode_callback(self, msg: String):
        """React to control_mode changes."""
        new_mode = msg.data.strip().lower()
        if new_mode == self.control_mode:
            return
        self.control_mode = new_mode

        if new_mode == "mapping" and not self.is_mapping:
            self.start_mapping()
        elif new_mode != "mapping" and self.is_mapping:
            self.stop_mapping()

    # ------------------------------------------------------------------
    # Mapping lifecycle
    # ------------------------------------------------------------------

    def start_mapping(self):
        """Begin relaying SLAM map to /combined_grid."""
        self.is_mapping = True
        period = 1.0 / max(self.relay_rate_hz, 0.5)
        self.relay_timer = self.create_timer(period, self.relay_callback)
        self.publish_status("mapping")
        self.get_logger().info("Mapping started — relaying /slam_map to /combined_grid")

    def stop_mapping(self):
        """Stop relaying."""
        self.is_mapping = False
        if self.relay_timer is not None:
            self.relay_timer.cancel()
            self.relay_timer = None
        self.publish_status("idle")
        self.get_logger().info("Mapping stopped")

    def relay_callback(self):
        """Publish the latest SLAM map to /combined_grid."""
        if self.latest_slam_map is None:
            return
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.latest_slam_map.header.frame_id or self.map_frame_id
        grid.info = self.latest_slam_map.info
        grid.data = self.latest_slam_map.data
        self.combined_grid_pub.publish(grid)

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def save_map_callback(self, request, response):
        """Save current SLAM map to PGM+YAML files."""
        if self.latest_slam_map is None:
            response.success = False
            response.message = "No SLAM map data available yet"
            return response

        previous_status = "mapping" if self.is_mapping else "idle"
        self.publish_status("saving")

        try:
            pgm_path, yaml_path = save_occupancy_grid(
                self.latest_slam_map, self.save_filename, self.map_save_dir
            )
            response.success = True
            response.message = f"Map saved: {pgm_path}"
            self.get_logger().info(f"Map saved to {pgm_path}")
        except Exception as e:
            response.success = False
            response.message = f"Save failed: {e}"
            self.get_logger().error(f"Map save failed: {e}")
        finally:
            self.publish_status(previous_status)

        return response

    def set_filename_callback(self, request, response):
        """Set the filename for the next map save (Trigger, message field as filename)."""
        filename = request.message.strip() if request.message else ""
        if filename:
            self.save_filename = filename
            self.get_logger().info(f"Save filename set to: {filename}")
            response.success = True
            response.message = f"Filename set: {filename}"
        else:
            response.success = False
            response.message = "Empty filename"
        return response

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def publish_status(self, status: str):
        """Publish mapping status for web UI consumption."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
