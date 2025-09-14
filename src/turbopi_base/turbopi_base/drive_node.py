#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

try:
    from HiwonderSDK import mecanum
    from HiwonderSDK import Board
except Exception:
    mecanum = None
    Board = None

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

class DriveNode(Node):
    def __init__(self):
        super().__init__('turbopi_drive')
        # limits and deadband
        self.declare_parameter('max_linear_mps', 0.6)
        self.declare_parameter('max_angular_rps', 1.5)
        self.declare_parameter('deadband', 0.02)

        self.max_lin = float(self.get_parameter('max_linear_mps').value)
        self.max_yaw = float(self.get_parameter('max_angular_rps').value)
        self.deadband = float(self.get_parameter('deadband').value)

        if mecanum is None or Board is None:
            self.get_logger().warn('HiwonderSDK not available here. Run on the Pi.')
            self.chassis = None
            self.board_available = False
        else:
            # Use calibrated chassis if available, otherwise use standard chassis
            if calibrated_available:
                self.chassis = CalibratedMecanumChassis()
                self.get_logger().info('Using calibrated mecanum chassis')
            else:
                self.chassis = mecanum.MecanumChassis()
                self.get_logger().info('Using standard mecanum chassis')
            self.board_available = True

        # Camera servo positions (servo IDs: pan=1, tilt=2)
        self.pan_angle = 0.0    # Current pan angle
        self.tilt_angle = 0.0   # Current tilt angle

        # Servo limits (in degrees, roughly)
        self.pan_min, self.pan_max = -90.0, 90.0
        self.tilt_min, self.tilt_max = -45.0, 45.0

        # Subscriptions
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.pan_sub = self.create_subscription(Float64, 'camera_pan', self.on_camera_pan, 10)
        self.tilt_sub = self.create_subscription(Float64, 'camera_tilt', self.on_camera_tilt, 10)

        self.get_logger().info('turbopi_drive ready. Subscribing to /cmd_vel, /camera_pan, /camera_tilt.')

    def on_cmd_vel(self, msg: Twist):
        if self.chassis is None:
            return

        vx = clamp(msg.linear.x, -self.max_lin, self.max_lin)
        vy = clamp(msg.linear.y, -self.max_lin, self.max_lin)
        wz = clamp(msg.angular.z, -self.max_yaw, self.max_yaw)

        # deadband
        if abs(vx) < self.deadband: vx = 0.0
        if abs(vy) < self.deadband: vy = 0.0
        if abs(wz) < self.deadband: wz = 0.0

        # convert to SDK units
        vx_mm = vx * 1000.0
        vy_mm = vy * 1000.0
        velocity_mm = math.hypot(vx_mm, vy_mm)

        if velocity_mm == 0.0:
            direction_deg = 90.0
        else:
            # rotate +90 deg so ROS +x forward maps to SDK forward
            direction_rad = math.atan2(vy_mm, vx_mm) + math.pi/2.0
            direction_deg = math.degrees(direction_rad) % 360.0

        try:
            self.chassis.set_velocity(velocity_mm, direction_deg, wz)
        except Exception as e:
            self.get_logger().error(f'set_velocity failed: {e}')

    def on_camera_pan(self, msg: Float64):
        """Handle camera pan commands"""
        if not self.board_available:
            return

        # Clamp angle to limits
        angle = clamp(msg.data, self.pan_min, self.pan_max)
        self.pan_angle = angle

        # Convert angle to servo pulse (roughly 500-2500 microseconds)
        # Center position (0°) = 1500μs, ±90° = ±500μs
        pulse = int(1500 + (angle / 90.0) * 500)
        pulse = clamp(pulse, 500, 2500)

        try:
            # Convert angle to pulse (1500 = center, ±500 for ±90°)
            pulse = int(1500 + (angle / 90.0) * 500)
            pulse = clamp(pulse, 500, 2500)
            Board.setPWMServoPulse(1, pulse, 200)  # Servo 1 = pan, 200ms duration
            self.get_logger().info(f'Pan servo: {angle:.1f}° (pulse: {pulse})')
        except Exception as e:
            self.get_logger().error(f'Pan servo failed: {e}')

    def on_camera_tilt(self, msg: Float64):
        """Handle camera tilt commands"""
        if not self.board_available:
            return

        # Clamp angle to limits
        angle = clamp(msg.data, self.tilt_min, self.tilt_max)
        self.tilt_angle = angle

        # Convert angle to servo pulse
        pulse = int(1500 + (angle / 45.0) * 500)  # Tilt has smaller range
        pulse = clamp(pulse, 500, 2500)

        try:
            # Convert angle to pulse (1500 = center, ±500 for ±45°)
            pulse = int(1500 + (angle / 45.0) * 500)
            pulse = clamp(pulse, 500, 2500)
            Board.setPWMServoPulse(2, pulse, 200)  # Servo 2 = tilt, 200ms duration
            self.get_logger().info(f'Tilt servo: {angle:.1f}° (pulse: {pulse})')
        except Exception as e:
            self.get_logger().error(f'Tilt servo failed: {e}')

def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    finally:
        # stop motors and center camera on shutdown
        if node.chassis is not None:
            try:
                node.chassis.reset_motors()
            except Exception:
                pass

        if node.board_available:
            try:
                # Center camera servos
                Board.setPWMServoPulse(1, 1500, 500)  # Pan to center
                Board.setPWMServoPulse(2, 1500, 500)  # Tilt to center
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
