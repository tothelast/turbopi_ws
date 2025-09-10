#!/usr/bin/env python3
import sys
import termios
import select
import signal
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
Keyboard teleop, hold a key to move.
 w/s: forward/back
 a/d: rotate left/right
 j/l: strafe left/right
 k  : stop
 q  : quit

Speed scales:
 u: linear up    m: linear down
 i: angular up   ,: angular down
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('turbopi_keyboard_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.declare_parameter('lin', 0.20)   # m/s
        self.declare_parameter('ang', 1.00)   # rad/s
        self.lin = float(self.get_parameter('lin').value)
        self.ang = float(self.get_parameter('ang').value)

        # terminal state
        self.fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(self.fd)

        # SIGINT handling: do NOT let rclpy auto-shutdown before we can send stop
        self._sigint = False
        signal.signal(signal.SIGINT, self._on_sigint)

        self.get_logger().info(HELP.strip())
        self.get_logger().info(f"lin={self.lin:.2f} m/s, ang={self.ang:.2f} rad/s")
        self.get_logger().info("Press q or Ctrl-C to quit.")

    def _on_sigint(self, signum, frame):
        # mark for clean exit; we will publish stop from the main loop
        self._sigint = True

    def _kbmode(self, enable: bool):
        if enable:
            new = termios.tcgetattr(self.fd)
            # disable canonical + echo; KEEP ISIG so signals are delivered
            new[3] = (new[3] & ~(termios.ICANON | termios.ECHO)) | termios.ISIG
            termios.tcsetattr(self.fd, termios.TCSADRAIN, new)
        else:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._saved)

    def publish_stop(self, repeats: int = 10, rate_hz: float = 30.0):
        msg = Twist()
        period = 1.0 / rate_hz
        for _ in range(repeats):
            try:
                self.pub.publish(msg)
                # give ROS a tick to flush
                rclpy.spin_once(self, timeout_sec=0.0)
            except Exception:
                # if context is already going down, just stop attempting
                break
            time.sleep(period)

    def loop(self):
        self._kbmode(True)
        try:
            while rclpy.ok():
                # if Ctrl-C pressed, exit cleanly
                if self._sigint:
                    self.publish_stop()
                    return

                # poll keyboard without blocking
                rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not rlist:
                    continue
                ch = sys.stdin.read(1)

                if ch == 'q':
                    self.publish_stop()
                    return

                # speed scaling
                if ch == 'u':
                    self.lin *= 1.1; continue
                if ch == 'm':
                    self.lin *= 0.9; continue
                if ch == 'i':
                    self.ang *= 1.1; continue
                if ch == ',':
                    self.ang *= 0.9; continue

                msg = Twist()
                # motions
                if ch == 'w':
                    msg.linear.x = self.lin
                elif ch == 's':
                    msg.linear.x = -self.lin
                elif ch == 'j':
                    msg.linear.y = self.lin
                elif ch == 'l':
                    msg.linear.y = -self.lin
                elif ch == 'a':
                    msg.angular.z = +self.ang
                elif ch == 'd':
                    msg.angular.z = -self.ang
                elif ch == 'k':
                    pass  # zero twist stop
                else:
                    continue

                self.pub.publish(msg)
        finally:
            self._kbmode(False)

def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.loop()
    finally:
        # only shut down if still running
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
