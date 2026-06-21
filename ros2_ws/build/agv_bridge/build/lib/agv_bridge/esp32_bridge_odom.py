#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

import serial


class ESP32BridgeOdom(Node):
    def __init__(self):
        super().__init__('esp32_bridge_odom')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('wheel_base', 0.170)
        self.declare_parameter('cmd_wheel_base', 0.0)
        self.declare_parameter('odom_wheel_base', 0.0)
        self.declare_parameter('ticks_per_rev', 1975.0)

        # Calibration knobs. Use these before changing firmware PID.
        self.declare_parameter('left_tick_sign', 1.0)
        self.declare_parameter('right_tick_sign', 1.0)
        self.declare_parameter('left_distance_scale', 1.0)
        self.declare_parameter('right_distance_scale', 1.0)

        self.declare_parameter('left_cmd_sign', 1.0)
        self.declare_parameter('right_cmd_sign', 1.0)
        self.declare_parameter('cmd_send_rate_hz', 20.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        # Keep the old bridge behavior by default: convert /cmd_vel to
        # WHEEL_VEL,left_mps,right_mps before sending to ESP32.
        self.declare_parameter('send_cmd_vel_direct', False)
        self.declare_parameter('force_zero_linear_on_spin', True)
        self.declare_parameter('spin_linear_deadband', 0.02)
        self.declare_parameter('spin_angular_deadband', 0.05)
        self.declare_parameter('debug_log', False)

        port = self.get_parameter('port').value
        baudrate = int(self.get_parameter('baudrate').value)

        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        cmd_wheel_base = float(self.get_parameter('cmd_wheel_base').value)
        odom_wheel_base = float(self.get_parameter('odom_wheel_base').value)
        self.cmd_wheel_base = cmd_wheel_base if cmd_wheel_base > 0.0 else self.wheel_base
        self.odom_wheel_base = odom_wheel_base if odom_wheel_base > 0.0 else self.wheel_base
        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)

        self.left_tick_sign = float(self.get_parameter('left_tick_sign').value)
        self.right_tick_sign = float(self.get_parameter('right_tick_sign').value)
        self.left_distance_scale = float(self.get_parameter('left_distance_scale').value)
        self.right_distance_scale = float(self.get_parameter('right_distance_scale').value)

        self.left_cmd_sign = float(self.get_parameter('left_cmd_sign').value)
        self.right_cmd_sign = float(self.get_parameter('right_cmd_sign').value)

        self.cmd_timeout_sec = float(self.get_parameter('cmd_timeout_sec').value)
        self.send_cmd_vel_direct = bool(self.get_parameter('send_cmd_vel_direct').value)
        self.force_zero_linear_on_spin = bool(
            self.get_parameter('force_zero_linear_on_spin').value
        )
        self.spin_linear_deadband = float(self.get_parameter('spin_linear_deadband').value)
        self.spin_angular_deadband = float(self.get_parameter('spin_angular_deadband').value)
        self.debug_log = bool(self.get_parameter('debug_log').value)

        cmd_send_rate_hz = float(self.get_parameter('cmd_send_rate_hz').value)
        cmd_period = 1.0 / cmd_send_rate_hz

        base_mpt = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        self.left_mpt = base_mpt * self.left_distance_scale
        self.right_mpt = base_mpt * self.right_distance_scale

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            20
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.serial_lock = threading.Lock()
        self.ser = serial.Serial(port, baudrate, timeout=0.05)

        self.prev_left_ticks = None
        self.prev_right_ticks = None
        self.prev_wall_time = None
        self.last_enc_time = time.monotonic()
        self.enc_count = 0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.latest_linear_x = 0.0
        self.latest_angular_z = 0.0
        self.last_cmd_time = time.monotonic()
        self.last_odom_debug_time = time.monotonic()

        self.running = True
        self.reader_thread = threading.Thread(
            target=self.serial_reader_loop,
            daemon=True
        )
        self.reader_thread.start()

        self.cmd_timer = self.create_timer(cmd_period, self.send_latest_cmd)
        self.health_timer = self.create_timer(1.0, self.report_health)
        self.add_on_set_parameters_callback(self.on_param_update)

        self.get_logger().info(
            f'ESP32 bridge started on {port} @ {baudrate}. '
            f'left_mpt={self.left_mpt:.8f}, right_mpt={self.right_mpt:.8f}, '
            f'cmd_wheel_base={self.cmd_wheel_base:.4f}, '
            f'odom_wheel_base={self.odom_wheel_base:.4f}, '
            f'force_zero_linear_on_spin={self.force_zero_linear_on_spin}, '
            f'send_cmd_vel_direct={self.send_cmd_vel_direct}'
        )

    def on_param_update(self, params):
        for param in params:
            name = param.name
            value = param.value

            if name == 'cmd_wheel_base':
                value = float(value)
                self.cmd_wheel_base = value if value > 0.0 else self.wheel_base
            elif name == 'odom_wheel_base':
                value = float(value)
                self.odom_wheel_base = value if value > 0.0 else self.wheel_base
            elif name == 'left_distance_scale':
                self.left_distance_scale = float(value)
            elif name == 'right_distance_scale':
                self.right_distance_scale = float(value)
            elif name == 'force_zero_linear_on_spin':
                self.force_zero_linear_on_spin = bool(value)
            elif name == 'spin_linear_deadband':
                self.spin_linear_deadband = float(value)
            elif name == 'spin_angular_deadband':
                self.spin_angular_deadband = float(value)
            elif name == 'debug_log':
                self.debug_log = bool(value)
            else:
                continue

            base_mpt = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
            self.left_mpt = base_mpt * self.left_distance_scale
            self.right_mpt = base_mpt * self.right_distance_scale

        return SetParametersResult(successful=True)

    def report_health(self):
        if time.monotonic() - self.last_enc_time > 1.0:
            self.get_logger().warn(
                'No ENC lines received from ESP32 for >1s. '
                'No ENC means /odom cannot be published.'
            )

    def yaw_to_quaternion(self, yaw):
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def cmd_vel_callback(self, msg):
        self.latest_linear_x = float(msg.linear.x)
        self.latest_angular_z = float(msg.angular.z)
        self.last_cmd_time = time.monotonic()

        if self.debug_log:
            self.get_logger().info(
                f'RX /cmd_vel: v={self.latest_linear_x:.4f}, '
                f'w={self.latest_angular_z:.4f}'
            )

    def send_latest_cmd(self):
        now = time.monotonic()

        if now - self.last_cmd_time > self.cmd_timeout_sec:
            v = 0.0
            w = 0.0
        else:
            v = self.latest_linear_x
            w = self.latest_angular_z

        try:
            if self.send_cmd_vel_direct:
                line = f'CMD_VEL,{v:.4f},{w:.4f}\n'
            else:
                v_left = v - (w * self.cmd_wheel_base * 0.5)
                v_right = v + (w * self.cmd_wheel_base * 0.5)
                v_left *= self.left_cmd_sign
                v_right *= self.right_cmd_sign
                line = f'WHEEL_VEL,{v_left:.4f},{v_right:.4f}\n'

            with self.serial_lock:
                self.ser.write(line.encode('utf-8'))

            if self.debug_log:
                self.get_logger().info(f'SEND_SERIAL: {line.strip()}')

        except Exception as e:
            self.get_logger().warn(f'Failed to write command to ESP32: {e}')

    def serial_reader_loop(self):
        while self.running and rclpy.ok():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode('utf-8', errors='ignore').strip()
                if line.startswith('ENC,'):
                    self.handle_encoder_line(line)

            except Exception as e:
                self.get_logger().warn(f'Serial read error: {e}')

    def handle_encoder_line(self, line):
        parts = line.split(',')
        if len(parts) != 3:
            return

        try:
            left_raw = int(parts[1])
            right_raw = int(parts[2])
        except ValueError:
            return

        self.last_enc_time = time.monotonic()
        self.enc_count += 1

        if self.debug_log and self.enc_count <= 5:
            self.get_logger().info(f'RX_SERIAL: {line}')

        left_ticks = self.left_tick_sign * left_raw
        right_ticks = self.right_tick_sign * right_raw

        stamp = self.get_clock().now()
        wall_now = time.monotonic()

        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            self.prev_wall_time = wall_now
            return

        dt = wall_now - self.prev_wall_time
        if dt <= 0.0:
            return

        d_left_ticks = left_ticks - self.prev_left_ticks
        d_right_ticks = right_ticks - self.prev_right_ticks

        self.update_odometry(stamp, d_left_ticks, d_right_ticks, dt)

        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        self.prev_wall_time = wall_now

    def update_odometry(self, stamp, d_left_ticks, d_right_ticks, dt):
        ds_left = d_left_ticks * self.left_mpt
        ds_right = d_right_ticks * self.right_mpt

        ds = 0.5 * (ds_left + ds_right)
        dtheta = (ds_right - ds_left) / self.odom_wheel_base
        is_spin_cmd = (
            self.force_zero_linear_on_spin and
            abs(self.latest_linear_x) <= self.spin_linear_deadband and
            abs(self.latest_angular_z) >= self.spin_angular_deadband and
            ds_left * ds_right < 0.0
        )

        if is_spin_cmd:
            ds = 0.0

        if abs(dtheta) < 1e-9:
            dx = ds * math.cos(self.theta)
            dy = ds * math.sin(self.theta)
        else:
            radius = ds / dtheta
            next_theta = self.theta + dtheta
            dx = radius * (math.sin(next_theta) - math.sin(self.theta))
            dy = -radius * (math.cos(next_theta) - math.cos(self.theta))

        self.x += dx
        self.y += dy
        self.theta = self.normalize_angle(self.theta + dtheta)

        vx = ds / dt
        wz = dtheta / dt

        self.maybe_log_odom_debug(
            d_left_ticks,
            d_right_ticks,
            ds_left,
            ds_right,
            ds,
            dtheta,
            is_spin_cmd
        )

        self.publish_odom(stamp, vx, wz)
        self.publish_tf(stamp)

    def maybe_log_odom_debug(
        self,
        d_left_ticks,
        d_right_ticks,
        ds_left,
        ds_right,
        ds,
        dtheta,
        is_spin_cmd
    ):
        if not self.debug_log:
            return

        now = time.monotonic()
        if now - self.last_odom_debug_time < 0.5:
            return

        self.last_odom_debug_time = now
        self.get_logger().info(
            f'ODOM_STEP: d_ticks=({d_left_ticks:.1f},{d_right_ticks:.1f}) '
            f'ds_lr=({ds_left:.5f},{ds_right:.5f}) ds={ds:.5f} '
            f'dtheta={math.degrees(dtheta):.2f}deg '
            f'odom_wheel_base={self.odom_wheel_base:.4f} '
            f'spin_fix={is_spin_cmd}'
        )

    def publish_odom(self, stamp, vx, wz):
        qx, qy, qz, qw = self.yaw_to_quaternion(self.theta)

        msg = Odometry()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz

        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[35] = 0.05

        self.odom_pub.publish(msg)

    def publish_tf(self, stamp):
        qx, qy, qz, qw = self.yaw_to_quaternion(self.theta)

        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def send_stop_cmd(self):
        try:
            if self.send_cmd_vel_direct:
                line = b'CMD_VEL,0.0000,0.0000\n'
            else:
                line = b'WHEEL_VEL,0.0000,0.0000\n'

            with self.serial_lock:
                self.ser.write(line)
        except Exception:
            pass

    def destroy_node(self):
        self.running = False
        self.send_stop_cmd()

        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32BridgeOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
