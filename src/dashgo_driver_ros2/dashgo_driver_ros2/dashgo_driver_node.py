import math
import sys
import threading
import time
import traceback

import rclpy
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import Imu
from serial import Serial
from serial.serialutil import SerialException
from std_msgs.msg import Float32, Int16, Int16MultiArray
from tf2_ros import TransformBroadcaster


ODOM_POSE_COVARIANCE = [
    1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e3,
]
ODOM_POSE_COVARIANCE2 = [
    1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-9,
]
ODOM_TWIST_COVARIANCE = [
    1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e3,
]
ODOM_TWIST_COVARIANCE2 = [
    1e-9, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1e-3, 1e-9, 0.0, 0.0, 0.0,
    0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1e-9,
]
IMU_ANGULAR_VELOCITY_COVARIANCE_UNKNOWN = [
    -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
]
IMU_LINEAR_ACCELERATION_COVARIANCE_UNKNOWN = [
    -1.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
]


def normalize_angle_radians(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def normalize_angle_degrees(angle):
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle


class Arduino:
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12

    def __init__(self, port="/dev/dashgo", baudrate=57600, timeout=0.5):
        self.PID_RATE = 30
        self.PID_INTERVAL = 1000 / 30

        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = timeout
        self.inter_char_timeout = timeout / 30.0
        self.encoder_count = 0

        self.port = None
        self.mutex = threading.Lock()
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

    def connect(self):
        try:
            self.port = Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.write_timeout,
            )
            time.sleep(1.0)
            test = self.get_baud()
            if test != self.baudrate:
                time.sleep(1.0)
                test = self.get_baud()
                if test != self.baudrate:
                    raise SerialException("Unexpected baud rate reply from controller")
        except SerialException:
            traceback.print_exc(file=sys.stdout)
            raise

    def close(self):
        if self.port and self.port.is_open:
            self.port.close()

    def _flush_input(self):
        if not self.port:
            return
        if hasattr(self.port, "reset_input_buffer"):
            self.port.reset_input_buffer()
        else:
            self.port.flushInput()

    def _write_line(self, command):
        if not self.port:
            raise SerialException("Serial port is not connected")
        self.port.write((command + "\r").encode("ascii"))

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        if not self.port:
            raise SerialException("Serial port is not connected")

        value = bytearray()
        attempts = 0
        while True:
            char = self.port.read(1)
            if char == b"\r":
                break
            if char:
                value.extend(char)
            attempts += 1
            if attempts * self.inter_char_timeout > timeout:
                return None

        return value.decode("ascii", errors="ignore").strip("\r")

    def recv_array(self):
        try:
            data = self.recv(self.timeout * self.N_ANALOG_PORTS)
            if not data:
                return []
            return [int(item) for item in data.split()]
        except Exception:
            return []

    def execute(self, command):
        with self.mutex:
            self._flush_input()
            attempts = 0
            value = None
            while attempts < 2:
                try:
                    self._write_line(command)
                    value = self.recv(self.timeout)
                    if value not in ("", "Invalid Command", None):
                        return int(value)
                except Exception:
                    pass
                attempts += 1
            raise SerialException(f"Failed to execute command: {command}")

    def execute_array(self, command):
        with self.mutex:
            self._flush_input()
            attempts = 0
            while attempts < 2:
                try:
                    self._write_line(command)
                    values = self.recv_array()
                    if values:
                        return values
                except Exception:
                    pass
                attempts += 1
            raise SerialException(f"Failed to execute array command: {command}")

    def execute_ack(self, command):
        with self.mutex:
            self._flush_input()
            attempts = 0
            while attempts < 2:
                try:
                    self._write_line(command)
                    ack = self.recv(self.timeout)
                    if ack not in ("", "Invalid Command", None):
                        return ack == "OK"
                except Exception:
                    pass
                attempts += 1
            raise SerialException(f"Failed to execute ack command: {command}")

    def execute_raw(self, command, timeout=None):
        with self.mutex:
            self._flush_input()
            attempts = 0
            recv_timeout = self.timeout if timeout is None else timeout
            while attempts < 2:
                try:
                    self._write_line(command)
                    value = self.recv(recv_timeout)
                    if value not in ("", None):
                        return value
                except Exception:
                    pass
                attempts += 1
            raise SerialException(f"Failed to execute raw command: {command}")

    def update_pid(self, kp, kd, ki, ko):
        command = f"u {kp}:{kd}:{ki}:{ko}"
        self.execute_ack(command)

    def get_baud(self):
        return int(self.execute("b"))

    def get_encoder_counts(self):
        values = self.execute_array("e")
        if len(values) != 2:
            raise SerialException("Encoder count reply did not contain 2 values")
        return values

    def reset_encoders(self):
        return self.execute_ack("r")

    def drive(self, right, left):
        return self.execute_ack(f"m {int(right)} {int(left)}")

    def stop(self):
        self.drive(0, 0)

    def get_pidin(self):
        values = self.execute_array("i")
        if len(values) != 2:
            raise SerialException("PID input reply did not contain 2 values")
        return values

    def get_pidout(self):
        values = self.execute_array("f")
        if len(values) != 2:
            raise SerialException("PID output reply did not contain 2 values")
        return values

    def get_imu_angles(self, command="angle"):
        values = self.execute_array(command)
        if len(values) != 2:
            raise SerialException(
                f"IMU angle reply did not contain 2 values for command: {command}"
            )
        return values


class BaseController:
    def __init__(self, node, arduino, base_frame):
        self.node = node
        self.arduino = arduino
        self.base_frame = base_frame
        self.odom_frame = node.get_parameter("odom_frame").value
        self.cmd_vel_topic = node.get_parameter("cmd_vel_topic").value
        self.odom_topic = node.get_parameter("odom_topic").value
        self.publish_odom_tf = bool(node.get_parameter("publish_odom_tf").value)
        self.motors_reversed = bool(node.get_parameter("motors_reversed").value)
        self.use_imu = bool(node.get_parameter("useImu").value)
        self.imu_command = str(node.get_parameter("imu_command").value)
        self.imu_angle_topic = str(node.get_parameter("imu_angle_topic").value)
        self.imu_raw_topic = str(node.get_parameter("imu_raw_topic").value)
        self.imu_topic = str(node.get_parameter("imu_topic").value)
        self.imu_frame = str(node.get_parameter("imu_frame").value) or self.base_frame
        self.imu_yaw_covariance = float(node.get_parameter("imu_yaw_covariance").value)
        self.imu_yaw_scale = float(node.get_parameter("imu_yaw_scale").value)
        self.use_imu_heading_in_odom = bool(
            node.get_parameter("use_imu_heading_in_odom").value
        )

        self.rate = float(node.get_parameter("base_controller_rate").value)
        self.timeout = float(node.get_parameter("base_controller_timeout").value)
        self.stopped = False

        self.wheel_diameter = float(node.get_parameter("wheel_diameter").value)
        self.wheel_track = float(node.get_parameter("wheel_track").value)
        self.encoder_resolution = int(node.get_parameter("encoder_resolution").value)
        self.gear_reduction = float(node.get_parameter("gear_reduction").value)
        self.kp = int(node.get_parameter("Kp").value)
        self.kd = int(node.get_parameter("Kd").value)
        self.ki = int(node.get_parameter("Ki").value)
        self.ko = int(node.get_parameter("Ko").value)
        self.accel_limit = float(node.get_parameter("accel_limit").value)

        self.arduino.update_pid(self.kp, self.kd, self.ki, self.ko)

        self.ticks_per_meter = (
            self.encoder_resolution * self.gear_reduction /
            (self.wheel_diameter * math.pi)
        )
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate
        self.bad_encoder_count = 0

        self.encoder_min = int(node.get_parameter("encoder_min").value)
        self.encoder_max = int(node.get_parameter("encoder_max").value)
        self.encoder_low_wrap = int(
            self.encoder_min + (self.encoder_max - self.encoder_min) * 0.3
        )
        self.encoder_high_wrap = int(
            self.encoder_min + (self.encoder_max - self.encoder_min) * 0.7
        )
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0

        now = self.node.get_clock().now()
        self.then = now
        self.t_delta = Duration(seconds=1.0 / self.rate)
        self.t_next = now + self.t_delta

        self.enc_left = None
        self.enc_right = None
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v_left = 0.0
        self.v_right = 0.0
        self.v_des_left = 0.0
        self.v_des_right = 0.0
        self.last_cmd_vel = now
        self.latest_imu_yaw = None
        self.latest_imu_yaw_wrapped = None
        self.latest_imu_primary = None
        self.latest_imu_stamp = None
        self.imu_heading_offset = 0.0
        self.imu_heading_initialized = False
        self.imu_yaw_delta = 0.0
        self.imu_angular_velocity_z = 0.0

        self.cmd_vel_sub = node.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.odom_pub = node.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(node)
        self.left_encoder_pub = node.create_publisher(Int16, "Lencoder", 10)
        self.right_encoder_pub = node.create_publisher(Int16, "Rencoder", 10)
        self.left_pidin_pub = node.create_publisher(Int16, "Lpidin", 10)
        self.right_pidin_pub = node.create_publisher(Int16, "Rpidin", 10)
        self.left_pidout_pub = node.create_publisher(Int16, "Lpidout", 10)
        self.right_pidout_pub = node.create_publisher(Int16, "Rpidout", 10)
        self.left_vel_pub = node.create_publisher(Int16, "Lvel", 10)
        self.right_vel_pub = node.create_publisher(Int16, "Rvel", 10)
        self.imu_angle_pub = None
        self.imu_raw_pub = None
        self.imu_pub = None
        if self.use_imu:
            self.imu_angle_pub = node.create_publisher(Float32, self.imu_angle_topic, 10)
            self.imu_raw_pub = node.create_publisher(Int16MultiArray, self.imu_raw_topic, 10)
            self.imu_pub = node.create_publisher(Imu, self.imu_topic, 10)

        self.arduino.reset_encoders()
        self.node.get_logger().info(
            f"Started base controller for base width {self.wheel_track} m "
            f"with {self.encoder_resolution} ticks per rev"
        )
        if self.use_imu:
            self.node.get_logger().info(
                f"IMU angle polling enabled via '{self.imu_command}' "
                f"(yaw topic: {self.imu_angle_topic}, raw topic: {self.imu_raw_topic}, "
                f"imu topic: {self.imu_topic}, frame: {self.imu_frame})"
            )
            self.node.get_logger().info(
                f"IMU yaw calibration scale: {self.imu_yaw_scale:.4f}"
            )
            if self.use_imu_heading_in_odom:
                self.node.get_logger().info(
                    "Using IMU yaw to override /odom heading for navigation."
                )

    def poll(self):
        now = self.node.get_clock().now()
        if now.nanoseconds <= self.t_next.nanoseconds:
            return

        try:
            left_pidin, right_pidin = self.arduino.get_pidin()
            self.left_pidin_pub.publish(Int16(data=int(left_pidin)))
            self.right_pidin_pub.publish(Int16(data=int(right_pidin)))

            left_pidout, right_pidout = self.arduino.get_pidout()
            self.left_pidout_pub.publish(Int16(data=int(left_pidout)))
            self.right_pidout_pub.publish(Int16(data=int(right_pidout)))

            left_enc, right_enc = self.arduino.get_encoder_counts()
            self.left_encoder_pub.publish(Int16(data=int(left_enc)))
            self.right_encoder_pub.publish(Int16(data=int(right_enc)))
        except Exception as exc:
            self.bad_encoder_count += 1
            self.node.get_logger().error(
                f"Encoder/PID polling failed ({self.bad_encoder_count}): {exc}"
            )
            return

        if self.use_imu:
            self._poll_imu()

        dt = (now - self.then).nanoseconds / 1e9
        if dt <= 0.0:
            return
        self.then = now

        if self.enc_left is None:
            dleft = 0.0
            dright = 0.0
        else:
            if left_enc < self.encoder_low_wrap and self.enc_left > self.encoder_high_wrap:
                self.l_wheel_mult += 1
            elif left_enc > self.encoder_high_wrap and self.enc_left < self.encoder_low_wrap:
                self.l_wheel_mult -= 1

            if right_enc < self.encoder_low_wrap and self.enc_right > self.encoder_high_wrap:
                self.r_wheel_mult += 1
            elif right_enc > self.encoder_high_wrap and self.enc_right < self.encoder_low_wrap:
                self.r_wheel_mult -= 1

            wrap_span = self.encoder_max - self.encoder_min
            dleft = (
                left_enc + self.l_wheel_mult * wrap_span - self.enc_left
            ) / self.ticks_per_meter
            dright = (
                right_enc + self.r_wheel_mult * wrap_span - self.enc_right
            ) / self.ticks_per_meter

        self.enc_left = left_enc
        self.enc_right = right_enc

        dxy_ave = (dright + dleft) / 2.0
        dth = (dright - dleft) / self.wheel_track
        vxy = dxy_ave / dt
        vth = dth / dt
        heading_for_integration = self.th

        if dxy_ave != 0.0:
            dx = math.cos(dth) * dxy_ave
            dy = -math.sin(dth) * dxy_ave
            self.x += math.cos(heading_for_integration) * dx - math.sin(heading_for_integration) * dy
            self.y += math.sin(heading_for_integration) * dx + math.cos(heading_for_integration) * dy

        if (
            self.use_imu and
            self.use_imu_heading_in_odom and
            self.latest_imu_yaw is not None
        ):
            self.th += self.imu_yaw_delta
            vth = self.imu_angular_velocity_z
        elif dth != 0.0:
            self.th += dth

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(self.th / 2.0)
        quaternion.w = math.cos(self.th / 2.0)

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = quaternion
        if self.publish_odom_tf:
            self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = vxy
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth

        if self.v_des_left == 0.0 and self.v_des_right == 0.0:
            odom.pose.covariance = ODOM_POSE_COVARIANCE2
            odom.twist.covariance = ODOM_TWIST_COVARIANCE2
        else:
            odom.pose.covariance = ODOM_POSE_COVARIANCE
            odom.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_pub.publish(odom)

        if now.nanoseconds > (self.last_cmd_vel + Duration(seconds=self.timeout)).nanoseconds:
            self.v_des_left = 0.0
            self.v_des_right = 0.0

        self.v_left = self._ramp_velocity(self.v_left, self.v_des_left)
        self.v_right = self._ramp_velocity(self.v_right, self.v_des_right)

        self.left_vel_pub.publish(Int16(data=int(self.v_left)))
        self.right_vel_pub.publish(Int16(data=int(self.v_right)))

        if not self.stopped:
            drive_left = int(self.v_left)
            drive_right = int(self.v_right)
            if self.motors_reversed:
                drive_left *= -1
                drive_right *= -1
            self.arduino.drive(drive_left, drive_right)

        self.t_next = now + self.t_delta

    def _poll_imu(self):
        try:
            primary_angle, yaw_angle = self.arduino.get_imu_angles(self.imu_command)
        except Exception as exc:
            self.node.get_logger().warn(
                f"IMU polling failed via '{self.imu_command}': {exc}",
                throttle_duration_sec=5.0,
            )
            return

        if self.imu_raw_pub is not None:
            self.imu_raw_pub.publish(
                Int16MultiArray(data=[int(primary_angle), int(yaw_angle)])
            )

        now = self.node.get_clock().now()
        calibrated_yaw_degrees = normalize_angle_degrees(
            float(yaw_angle) * self.imu_yaw_scale
        )
        wrapped_yaw_radians = normalize_angle_radians(
            math.radians(calibrated_yaw_degrees)
        )
        if (
            self.latest_imu_yaw is None or
            self.latest_imu_yaw_wrapped is None or
            self.latest_imu_stamp is None
        ):
            yaw_radians = wrapped_yaw_radians
            yaw_delta = 0.0
        else:
            yaw_delta = normalize_angle_radians(
                wrapped_yaw_radians - self.latest_imu_yaw_wrapped
            )
            yaw_radians = self.latest_imu_yaw + yaw_delta

        if self.latest_imu_yaw is not None and self.latest_imu_stamp is not None:
            imu_dt = (now - self.latest_imu_stamp).nanoseconds / 1e9
            if imu_dt > 0.0:
                self.imu_angular_velocity_z = yaw_delta / imu_dt
        self.imu_yaw_delta = yaw_delta
        self.latest_imu_primary = int(primary_angle)
        self.latest_imu_yaw = yaw_radians
        self.latest_imu_yaw_wrapped = wrapped_yaw_radians
        self.latest_imu_stamp = now

        if self.imu_angle_pub is not None:
            self.imu_angle_pub.publish(Float32(data=float(calibrated_yaw_degrees)))

        if self.imu_pub is not None:
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = math.sin(yaw_radians / 2.0)
            quaternion.w = math.cos(yaw_radians / 2.0)

            imu_msg = Imu()
            imu_msg.header.stamp = self.node.get_clock().now().to_msg()
            imu_msg.header.frame_id = self.imu_frame
            imu_msg.orientation = quaternion
            imu_msg.orientation_covariance = [
                1e6, 0.0, 0.0,
                0.0, 1e6, 0.0,
                0.0, 0.0, self.imu_yaw_covariance,
            ]
            imu_msg.angular_velocity.z = float(self.imu_angular_velocity_z)
            imu_msg.angular_velocity_covariance = IMU_ANGULAR_VELOCITY_COVARIANCE_UNKNOWN
            imu_msg.linear_acceleration_covariance = IMU_LINEAR_ACCELERATION_COVARIANCE_UNKNOWN
            self.imu_pub.publish(imu_msg)

    def _ramp_velocity(self, current, target):
        if current < target:
            current += self.max_accel
            if current > target:
                current = target
        else:
            current -= self.max_accel
            if current < target:
                current = target
        return current

    def stop(self):
        self.stopped = True
        try:
            self.arduino.drive(0, 0)
        except Exception:
            pass

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = self.node.get_clock().now()

        x = msg.linear.x
        th = msg.angular.z

        if x == 0.0:
            right = th * self.wheel_track * self.gear_reduction / 2.0
            left = -right
        elif th == 0.0:
            left = x
            right = x
        else:
            left = x - th * self.wheel_track * self.gear_reduction / 2.0
            right = x + th * self.wheel_track * self.gear_reduction / 2.0

        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)


class DashgoDriverNode(Node):
    def __init__(self):
        super().__init__("dashgo_driver")

        self._declare_parameters()

        self.serial_lock = threading.Lock()
        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.timeout = float(self.get_parameter("timeout").value)
        self.rate = int(self.get_parameter("rate").value)
        self.sensorstate_rate = int(self.get_parameter("sensorstate_rate").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.use_base_controller = bool(self.get_parameter("use_base_controller").value)
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.controller = Arduino(self.port, self.baud, self.timeout)
        self.controller.connect()
        self.get_logger().info(
            f"Connected to Arduino on port {self.port} at {self.baud} baud"
        )

        self.base_controller = None
        if self.use_base_controller:
            self.base_controller = BaseController(self, self.controller, self.base_frame)

        timer_period = 1.0 / max(self.rate, 1)
        self.poll_timer = self.create_timer(timer_period, self._poll_once)
        self.get_logger().info(
            "Dashgo driver ready "
            f"(cmd_vel: {self.cmd_vel_topic}, base_frame: {self.base_frame})"
        )

    def _declare_parameters(self):
        common = ParameterDescriptor(
            description="Dashgo base driver runtime parameter."
        )
        defaults = [
            ("port", "/dev/ttyUSB1"),
            ("baud", 57600),
            ("timeout", 0.5),
            ("rate", 50),
            ("sensorstate_rate", 10),
            ("cmd_vel_topic", "cmd_vel"),
            ("odom_topic", "odom"),
            ("odom_frame", "odom"),
            ("publish_odom_tf", True),
            ("useSonar", False),
            ("useImu", False),
            ("imu_command", "angle"),
            ("imu_angle_topic", "imu_angle"),
            ("imu_raw_topic", "imu_angles_raw"),
            ("imu_topic", "imu/data"),
            ("imu_frame", ""),
            ("imu_yaw_covariance", 0.05),
            ("imu_yaw_scale", 1.0),
            ("use_imu_heading_in_odom", True),
            ("use_base_controller", True),
            ("base_controller_rate", 10.0),
            ("base_controller_timeout", 1.0),
            ("base_frame", "base_link"),
            ("sonar_height", 0.115),
            ("sonar0_offset_yaw", 0.524),
            ("sonar0_offset_x", 0.18),
            ("sonar0_offset_y", 0.10),
            ("sonar1_offset_yaw", 0.0),
            ("sonar1_offset_x", 0.20),
            ("sonar1_offset_y", 0.0),
            ("sonar2_offset_yaw", -0.524),
            ("sonar2_offset_x", 0.18),
            ("sonar2_offset_y", -0.10),
            ("sonar3_offset_yaw", 3.14),
            ("sonar3_offset_x", -0.20),
            ("sonar3_offset_y", 0.0),
            ("wheel_diameter", 0.1280),
            ("wheel_track", 0.341),
            ("encoder_resolution", 1200),
            ("gear_reduction", 1.0),
            ("motors_reversed", False),
            ("Kp", 50),
            ("Kd", 20),
            ("Ki", 0),
            ("Ko", 50),
            ("accel_limit", 1.0),
            ("encoder_min", -32768),
            ("encoder_max", 32768),
        ]
        for name, value in defaults:
            self.declare_parameter(name, value, common)

    def _poll_once(self):
        if not self.base_controller:
            return

        with self.serial_lock:
            self.base_controller.poll()

    def stop_robot(self):
        zero = Twist()
        self.cmd_vel_pub.publish(zero)
        if self.base_controller:
            self.base_controller.stop()
        try:
            self.controller.stop()
        except Exception:
            pass

    def destroy_node(self):
        self.stop_robot()
        self.controller.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DashgoDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SerialException as exc:
        print(f"Serial exception: {exc}", file=sys.stderr)
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
