import glob
import os

from ament_index_python.packages import get_package_share_directory
from dashgo_driver_ros2.device_resolver import resolve_serial_ports
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def resolve_qr_popup_script():
    matches = glob.glob(
        os.path.join(
            get_package_share_directory("dashgo_web_control"),
            "..",
            "..",
            "lib",
            "python*",
            "site-packages",
            "dashgo_web_control",
            "show_web_qr_popup.py",
        )
    )
    source_path = os.path.expanduser(
        "~/project/dashgo_ws/src/dashgo_web_control/dashgo_web_control/show_web_qr_popup.py"
    )

    if matches:
        return matches[0]

    if os.path.exists(source_path):
        return source_path

    raise FileNotFoundError("Unable to locate show_web_qr_popup.py")


def resolve_hotspot_script():
    matches = glob.glob(
        os.path.join(
            get_package_share_directory("dashgo_web_control"),
            "..",
            "..",
            "lib",
            "python*",
            "site-packages",
            "dashgo_web_control",
            "hotspot_manager.py",
        )
    )
    source_path = os.path.expanduser(
        "~/project/dashgo_ws/src/dashgo_web_control/dashgo_web_control/hotspot_manager.py"
    )

    if matches:
        return matches[0]

    if os.path.exists(source_path):
        return source_path

    raise FileNotFoundError("Unable to locate hotspot_manager.py")


def generate_launch_description():
    resolved_ports = resolve_serial_ports()
    driver_share = get_package_share_directory("dashgo_driver_ros2")
    nav_share_dir = get_package_share_directory("nav_slam")
    default_nav_map = os.path.join(nav_share_dir, "map", "dashgo_slam_map.yaml")              # 静态地图读取接口
    default_nav_rviz = os.path.join(nav_share_dir, "config", "rviz.rviz")
    default_robot_urdf = os.path.join(driver_share, "urdf", "dashgo_visual.urdf")

    robot_launch = os.path.join(driver_share, "launch", "dashgo_robot.launch.py")
    with open(default_robot_urdf, "r", encoding="utf-8") as urdf_file:
        robot_description = urdf_file.read()

    start_robot = LaunchConfiguration("start_robot")
    start_nav = LaunchConfiguration("start_nav")
    start_nav_rviz = LaunchConfiguration("start_nav_rviz")
    start_lidar = LaunchConfiguration("start_lidar")
    start_d435 = LaunchConfiguration("start_d435")
    start_web_ui = LaunchConfiguration("start_web_ui")
    start_hotspot = LaunchConfiguration("start_hotspot")
    publish_robot_model = LaunchConfiguration("publish_robot_model")
    driver_port = LaunchConfiguration("driver_port")
    lidar_port = LaunchConfiguration("lidar_port")
    nav_map_yaml = LaunchConfiguration("nav_map_yaml")
    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")
    laser_z = LaunchConfiguration("laser_z")
    points_frame = LaunchConfiguration("points_frame")
    scan_min_range = LaunchConfiguration("scan_min_range")
    web_host = LaunchConfiguration("web_host")
    web_port = LaunchConfiguration("web_port")
    web_image_topic = LaunchConfiguration("web_image_topic")
    web_map_publish_hz = LaunchConfiguration("web_map_publish_hz")
    web_camera_publish_hz = LaunchConfiguration("web_camera_publish_hz")
    web_camera_max_width = LaunchConfiguration("web_camera_max_width")
    hotspot_connection_name = LaunchConfiguration("hotspot_connection_name")
    hotspot_ssid = LaunchConfiguration("hotspot_ssid")
    hotspot_password = LaunchConfiguration("hotspot_password")
    hotspot_ifname = LaunchConfiguration("hotspot_ifname")
    map_odom_topic = LaunchConfiguration("map_odom_topic")
    control_odom_topic = LaunchConfiguration("control_odom_topic")

    # 新增：全局定位相关参数
    use_static_map = LaunchConfiguration("use_static_map")
    use_global_localize = LaunchConfiguration("use_global_localize")
    use_pointcloud_obstacles = LaunchConfiguration("use_pointcloud_obstacles")
    use_dynamic_obstacle_points = LaunchConfiguration("use_dynamic_obstacle_points")
    use_slam = LaunchConfiguration("use_slam")
    use_continuous_orb = LaunchConfiguration("use_continuous_orb")
    goal_relocalization_enabled = LaunchConfiguration("goal_relocalization_enabled")
    orb_match_period_sec = LaunchConfiguration("orb_match_period_sec")
    orb_max_iterations = LaunchConfiguration("orb_max_iterations")
    orb_min_f1_score = LaunchConfiguration("orb_min_f1_score")
    orb_required_consistent_matches = LaunchConfiguration("orb_required_consistent_matches")
    orb_consistent_translation_m = LaunchConfiguration("orb_consistent_translation_m")
    orb_consistent_yaw_deg = LaunchConfiguration("orb_consistent_yaw_deg")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_robot", default_value="true"),
            DeclareLaunchArgument("start_nav", default_value="true"),
            DeclareLaunchArgument("start_nav_rviz", default_value="true"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("start_d435", default_value="true"),
            DeclareLaunchArgument("start_web_ui", default_value="true"),
            DeclareLaunchArgument("start_hotspot", default_value="false"),
            DeclareLaunchArgument("publish_robot_model", default_value="true"),
            DeclareLaunchArgument("driver_port", default_value=resolved_ports["driver_port"]),
            DeclareLaunchArgument(
                "lidar_port",
                default_value=resolved_ports["lidar_port"],
            ),
            DeclareLaunchArgument("nav_map_yaml", default_value=default_nav_map),
            DeclareLaunchArgument("base_frame", default_value="base_footprint"),
            DeclareLaunchArgument("laser_frame", default_value="laser"),
            DeclareLaunchArgument("laser_z", default_value="0.52"),
            DeclareLaunchArgument("points_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_min_range", default_value="0.25"),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8080"),
            DeclareLaunchArgument("web_image_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument("web_map_publish_hz", default_value="2.0"),
            DeclareLaunchArgument("web_camera_publish_hz", default_value="12.0"),
            DeclareLaunchArgument("web_camera_max_width", default_value="320"),
            DeclareLaunchArgument("hotspot_connection_name", default_value="dashgo-hotspot"),
            DeclareLaunchArgument("hotspot_ssid", default_value="Dashgo-Robot"),
            DeclareLaunchArgument("hotspot_password", default_value="dashgo12345"),
            DeclareLaunchArgument("hotspot_ifname", default_value=""),
            DeclareLaunchArgument("map_odom_topic", default_value="/odom"),
            DeclareLaunchArgument("control_odom_topic", default_value="/odom_in_map"),
            # 新增：全局定位相关参数
            DeclareLaunchArgument("use_static_map", default_value="true"),
            DeclareLaunchArgument("use_global_localize", default_value="true"),
            DeclareLaunchArgument("use_pointcloud_obstacles", default_value="true"),
            DeclareLaunchArgument("use_dynamic_obstacle_points", default_value="false"),
            DeclareLaunchArgument("use_slam", default_value="false"),
            DeclareLaunchArgument("use_continuous_orb", default_value="true"),
            DeclareLaunchArgument("goal_relocalization_enabled", default_value="false"),
            DeclareLaunchArgument("orb_match_period_sec", default_value="2.0"),
            DeclareLaunchArgument("orb_max_iterations", default_value="50"),
            DeclareLaunchArgument("orb_min_f1_score", default_value="35.0"),
            DeclareLaunchArgument("orb_required_consistent_matches", default_value="2"),
            DeclareLaunchArgument("orb_consistent_translation_m", default_value="0.30"),
            DeclareLaunchArgument("orb_consistent_yaw_deg", default_value="5.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(robot_launch),
                condition=IfCondition(start_robot),
                launch_arguments={
                    "start_lidar": start_lidar,
                    "start_d435": start_d435,
                    "start_t265": "false",
                    "driver_port": driver_port,
                    "driver_baud": "115200",
                    "lidar_port": lidar_port,
                    "lidar_baud": "1000000",
                    "base_frame": base_frame,
                    "laser_frame": laser_frame,
                    "laser_z": laser_z,
                    "publish_laser_tf": "false",
                    "publish_sonar_tf": "false",
                    "publish_odom_tf": "false",
                }.items(),
            ),
            LogInfo(msg=f"Dashgo auto-detected base port default: {resolved_ports['driver_port']}"),
            LogInfo(msg=f"Dashgo auto-detected lidar port default: {resolved_ports['lidar_port']}"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                condition=IfCondition(publish_robot_model),
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "publish_frequency": 30.0,
                    }
                ],
            ),
            # ---- 激光扫描转点云 (base_link 帧) ----
            Node(
                package="dashgo_driver_ros2",
                executable="scan_to_points_node",
                name="scan_to_points",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[
                    {
                        "scan_topic": "/scan",
                        "filtered_scan_topic": "/scan_filtered",
                        "points_topic": "/points_raw",
                        "output_frame": points_frame,
                        "laser_height": laser_z,
                        "x_offset": 0.0,
                        "y_offset": 0.0,
                        "min_valid_range": scan_min_range,
                    }
                ],
            ),
            # ---- 全局定位：静态地图服务器 (SLAM模式时跳过) ----
            Node(
                package="nav_slam",
                executable="static_map_server",
                name="static_map_server",
                condition=IfCondition(
                    PythonExpression([
                        "'", start_nav, "' == 'true' and '", use_slam, "' == 'false'"
                    ])
                ),
                output="screen",
                parameters=[{
                    "map_yaml_path": nav_map_yaml,
                    "publish_period_sec": 1.0,
                    "frame_id": "map",
                }],
            ),
            # ---- 全局定位：一次性地图转发 (SLAM模式时跳过) ----
            Node(
                package="nav_slam",
                executable="map_once_relay",
                name="map_once_relay",
                condition=IfCondition(
                    PythonExpression([
                        "'", start_nav, "' == 'true' and '", use_slam, "' == 'false'"
                    ])
                ),
                output="screen",
            ),
            # ---- map→odom 唯一校正器：融合初始定位和持续 ORB 观测 ----
            Node(
                package="nav_slam",
                executable="map_odom_corrector",
                name="map_odom_corrector",
                condition=IfCondition(
                    PythonExpression([
                        "'", start_nav, "' == 'true' and '", use_slam,
                        "' == 'false' and '", use_global_localize, "' == 'true'"
                    ])
                ),
                output="screen",
                parameters=[{
                    "odom_topic": map_odom_topic,
                    "match_pose_topic": "/orb/match_pose",
                    "initial_pose_topic": "/lidar_global/match_pose",
                    "state_history_sec": 30.0,
                    "required_consistent_matches": orb_required_consistent_matches,
                    "consistent_translation_m": orb_consistent_translation_m,
                    "consistent_yaw_deg": orb_consistent_yaw_deg,
                    "max_correction_linear_mps": 0.20,
                    "max_correction_angular_degps": 12.0,
                }],
            ),
            # ---- 全局定位：激光 ORB 初始观测 ----
            Node(
                package="nav_slam",
                executable="lidar_global_localize",
                name="lidar_global_localize",
                condition=IfCondition(
                    PythonExpression([
                        "'", start_nav, "' == 'true' and '", use_slam,
                        "' == 'false' and '", use_global_localize, "' == 'true'"
                    ])
                ),
                output="screen",
                parameters=[{
                    "map_yaml_path": nav_map_yaml,
                    "scan_topic": "/scan_filtered",
                }],
            ),
            # ---- 激光扫描直接转 map 帧点云 ----
            Node(
                package="nav_slam",
                executable="laser_scan_to_points",
                name="laser_scan_to_points",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[{
                    "scan_topic": "/scan_filtered",
                    "output_topic": "/mapokk",
                    "target_frame": "map",
                }],
            ),
            # ---- Voronoi 骨架规划器 ----
            Node(
                package="nav2_voronoi_planner",
                executable="voronoi_node",
                name="voronoi",
                condition=IfCondition(start_nav),
                output="screen",
                remappings=[("/odom", "/odom_in_map")],
                parameters=[
                    {
                        "robot_radius": 0.20,
                        "occ_threshold": 15,
                        "trunk_safety_penalty_scale": 0.06,
                        "direct_connect_distance": 0.60,
                    }
                ],
            ),
            # ---- 代价地图发布 (SLAM模式时跳过，由slam_controller接管) ----
            Node(
                package="nav_slam",
                executable="map_pub",
                name="map_pub",
                condition=IfCondition(
                    PythonExpression([
                        "'", start_nav, "' == 'true' and '", use_slam, "' == 'false'"
                    ])
                ),
                output="screen",
                parameters=[
                    {
                        "use_static_map": use_static_map,
                        "static_map_yaml": nav_map_yaml,
                        "grid_width": 40.0,
                        "grid_height": 40.0,
                        "resolution": 0.05,
                        "dynamic_obstacle_timeout": 0.6,
                        "accumulate_pointcloud_obstacles": False,
                        "min_height": 0.0,
                        "max_height": 1.0,
                        "obstacle_radius": 0.08,
                        "clear_radius": 0.20,
                        "projection_gap_fill_cells": 0,
                        "odom_topic": control_odom_topic,
                        "use_pointcloud_obstacles": use_pointcloud_obstacles,
                        "use_dynamic_obstacle_points": use_dynamic_obstacle_points,
                    }
                ],
            ),
            # ---- SLAM 在线建图 (use_slam:=true 时启用) ----
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav_share_dir, "launch", "slam_mapping.launch.py")
                ),
                condition=IfCondition(use_slam),
                launch_arguments={}.items(),
            ),
            # ---- odom → base_footprint TF 桥接 ----
            Node(
                package="nav_slam",
                executable="odom_tf_bridge",
                name="odom_tf_bridge",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[{"odom_topic": map_odom_topic}],
            ),
            # ---- odom → map 坐标中转 ----
            Node(
                package="nav_slam",
                executable="odom_to_map_relay",
                name="odom_to_map_relay",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[{
                    "odom_topic": map_odom_topic,
                    "output_topic": control_odom_topic,
                }],
            ),
            # ---- 持续 ORB 地图匹配 ----
            Node(
                package="nav_slam",
                executable="orb_map_matcher",
                name="orb_map_matcher",
                condition=IfCondition(
                    PythonExpression([
                        "'", start_nav, "' == 'true' and '", use_slam,
                        "' == 'false' and '", use_global_localize,
                        "' == 'true' and '", use_continuous_orb, "' == 'true'"
                    ])
                ),
                output="screen",
                parameters=[{
                    "map_yaml_path": nav_map_yaml,
                    "scan_topic": "/scan_filtered",
                    "odom_topic": control_odom_topic,
                    "match_pose_topic": "/orb/match_pose",
                    "base_frame": base_frame,
                    "match_period_sec": orb_match_period_sec,
                    "lidar_max_range": 8.0,
                    "map_resolution": 0.05,
                    "max_iterations": orb_max_iterations,
                    "min_f1_score": orb_min_f1_score,
                    "local_search_radius_m": 1.0,
                }],
            ),
            # ---- 到达目标后触发一次全局重定位 ----
            Node(
                package="nav_slam",
                executable="nav_goal_relocalizer",
                name="nav_goal_relocalizer",
                condition=IfCondition(
                    PythonExpression([
                        "'", start_nav, "' == 'true' and '", use_slam,
                        "' == 'false' and '", use_global_localize,
                        "' == 'true' and '", goal_relocalization_enabled, "' == 'true'"
                    ])
                ),
                output="screen",
                parameters=[{
                    "goal_relocalization_enabled": goal_relocalization_enabled,
                    "min_relocalize_interval_sec": 10.0,
                    "orb_disable_duration_sec": 30.0,
                }],
            ),
            # ---- map→odom fallback 静态 TF (SLAM/全局定位时跳过) ----
            Node(
                package="nav_slam",
                executable="odom_map_tf",
                name="odom_map_tf",
                condition=IfCondition(
                    PythonExpression([
                        "'", start_nav, "' == 'true' and '", use_slam,
                        "' == 'false' and '", use_global_localize, "' == 'false'"
                    ])
                ),
                output="screen",
            ),
            # ---- 点云坐标系变换 (已禁用: laser_scan_to_points 通过TF变换代替，避免坐标系bug) ----
            # Node(
            #     package="nav_slam",
            #     executable="points_pub_map",
            #     name="points_pub_map",
            #     condition=IfCondition(start_nav),
            #     output="screen",
            #     parameters=[{"frame_id": "map"}, {"odom_topic": map_odom_topic}],
            # ),
            # ---- 路径跟踪控制器 ----
            Node(
                package="nav_slam",
                executable="start_nav",
                name="start_nav",
                condition=IfCondition(start_nav),
                output="screen",
                parameters=[{"odom_topic": control_odom_topic}],
            ),
            # ---- RViz ----
            Node(
                package="rviz2",
                executable="rviz2",
                name="nav_rviz2",
                condition=IfCondition(start_nav_rviz),
                output="screen",
                arguments=["-d", default_nav_rviz],
            ),
            # ---- Web 控制 ----
            Node(
                package="dashgo_web_control",
                executable="web_control_node",
                name="dashgo_web_control",
                condition=IfCondition(start_web_ui),
                output="screen",
                parameters=[
                    {
                        "host": web_host,
                        "port": web_port,
                        "odom_topic": control_odom_topic,
                        "image_topic": web_image_topic,
                        "map_publish_hz": web_map_publish_hz,
                        "camera_publish_hz": web_camera_publish_hz,
                        "camera_max_width": web_camera_max_width,
                        "robot_radius": 0.20,
                    }
                ],
            ),
            TimerAction(
                period=0.3,
                condition=IfCondition(start_hotspot),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            resolve_hotspot_script(),
                            "--connection-name",
                            hotspot_connection_name,
                            "--ssid",
                            hotspot_ssid,
                            "--password",
                            hotspot_password,
                            "--ifname",
                            hotspot_ifname,
                        ],
                        output="screen",
                    ),
                ],
            ),
            TimerAction(
                period=2.3,
                condition=IfCondition(start_web_ui),
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "python3",
                            resolve_qr_popup_script(),
                        ],
                        additional_env={
                            "DASHGO_WEB_UI_HOST": web_host,
                            "DASHGO_WEB_UI_PORT": web_port,
                            "DASHGO_HOTSPOT_ENABLED": start_hotspot,
                            "DASHGO_HOTSPOT_SSID": hotspot_ssid,
                            "DASHGO_HOTSPOT_PASSWORD": hotspot_password,
                        },
                        output="screen",
                    ),
                ],
            ),
        ]
    )
