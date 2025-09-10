#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from HiwonderSDK import mecanum
except Exception:
    mecanum = None

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

        if mecanum is None:
            self.get_logger().warn('HiwonderSDK not available here. Run on the Pi.')
            self.chassis = None
        else:
            self.chassis = mecanum.MecanumChassis()

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.get_logger().info('turbopi_drive ready. Subscribing to /cmd_vel.')

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

def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    finally:
        # stop motors on shutdown
        if node.chassis is not None:
            try:
                node.chassis.reset_motors()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
