#!/usr/bin/env python3
import sys
import termios
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

HELP = """
Keyboard teleop (hold-to-move, auto-brake on release)

 Movement:
   w/s : forward/back
   a/d : strafe left/right
   z/c : rotate left/right
   k   : stop (zero twist)
   q   : quit (sends stop burst)

 Speed scales:
   u/m : linear up/downz
   i/, : angular up/down
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('turbopi_keyboard_teleop')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # parameters
        self.declare_parameter('lin', 0.20)        # m/s
        self.declare_parameter('ang', 1.00)        # rad/s
        self.declare_parameter('rate_hz', 30.0)    # publish rate
        self.declare_parameter('idle_timeout', 0.20)  # seconds with no key before braking
        self.declare_parameter('hold_extend', 0.6)   # seconds to sustain motion after a key press
        self.hold_extend = float(self.get_parameter('hold_extend').value)
        self.hold_deadline = 0.0


        self.lin = float(self.get_parameter('lin').value)
        self.ang = float(self.get_parameter('ang').value)
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.idle_timeout = float(self.get_parameter('idle_timeout').value)

        # current desired command and last input time
        self.desired = Twist()
        self.last_input = time.time()

        # configure terminal to non-canonical, signals enabled
        self.fd = sys.stdin.fileno()
        self._saved = termios.tcgetattr(self.fd)
        new = termios.tcgetattr(self.fd)
        new[3] = (new[3] & ~(termios.ICANON | termios.ECHO)) | termios.ISIG
        termios.tcsetattr(self.fd, termios.TCSADRAIN, new)

        # timers: keyboard poll and periodic publisher
        self.poll_timer = self.create_timer(0.01, self._poll_keyboard)  # 100 Hz poll
        self.pub_timer = self.create_timer(1.0 / self.rate_hz, self._publish_cmd)  # periodic publish

        self.get_logger().info(HELP.strip())
        self.get_logger().info(f"lin={self.lin:.2f} m/s, ang={self.ang:.2f} rad/s")

    def destroy_node(self):
        # restore terminal
        try:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._saved)
        except Exception:
            pass
        super().destroy_node()

    def _poll_keyboard(self):
        rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not rlist:
            return
        ch = sys.stdin.read(1)
        t = time.time()

        # speed scaling
        if ch == 'u':
            self.lin *= 1.1; return
        if ch == 'm':
            self.lin *= 0.9; return
        if ch == 'i':
            self.ang *= 1.1; return
        if ch == ',':
            self.ang *= 0.9; return

        cmd = Twist()

        # movement mapping, ROS REP 103: +x forward, +y left, +z up
        if ch == 'w':
            cmd.linear.x = self.lin
        elif ch == 's':
            cmd.linear.x = -self.lin
        elif ch == 'a':
            cmd.linear.y = +self.lin   # left strafe
        elif ch == 'd':
            cmd.linear.y = -self.lin   # right strafe
        elif ch == 'c':
            cmd.angular.z = +self.ang  # rotate left (CCW)
        elif ch == 'z':
            cmd.angular.z = -self.ang  # rotate right (CW)
        elif ch == 'k':
            self.desired = Twist()
            self.last_input = t
            self.hold_deadline = t
            return
        elif ch == 'q':
            self._stop_burst()
            rclpy.shutdown()
            return
        else:
            # ignore unknown keys
            return

        # accept this new command and mark input time
        self.desired = cmd
        self.last_input = t
        self.hold_deadline = t + self.hold_extend


    def _publish_cmd(self):
        now = time.time()
        if now <= self.hold_deadline:
            out = self.desired
        elif now - self.last_input > self.idle_timeout:
            out = Twist()  # auto-brake
        else:
            out = self.desired
        self.pub.publish(out)


    def _stop_burst(self, repeats: int = 10, rate_hz: float = 30.0):
        msg = Twist()
        dt = 1.0 / rate_hz
        for _ in range(repeats):
            try:
                self.pub.publish(msg)
            except Exception:
                break
            time.sleep(dt)

def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._stop_burst()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
