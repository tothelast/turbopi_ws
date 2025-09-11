#!/usr/bin/python3
# coding=utf8
import os
import sys
import time
import signal
import threading
import numpy as np

# Make sure the SDK root is on sys.path even if run from Functions/
SDK_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if SDK_ROOT not in sys.path:
    sys.path.insert(0, SDK_ROOT)

import cv2
import yaml_handle
import pandas as pd
import HiwonderSDK.Sonar as Sonar
import HiwonderSDK.Board as Board
import HiwonderSDK.mecanum as mecanum

# Environment hint so Qt does not try X11 on headless
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

# -------------- Parameters --------------
FWD_SPEED = 35            # 0..100
TURN_YAW = -0.6           # -2..2, negative left
THRESHOLD_CM = 30.0       # turn when obstacle is closer than this
TURN_TIME_S = 0.5         # turn burst duration
LOOP_HZ = 20              # control loop frequency
DIST_WINDOW = 5           # moving average window for distance
TEXT_COLOR = (0, 255, 255)

# -------------- State --------------
car = mecanum.MecanumChassis()
sonar = None
__running = False

# -------------- Setup helpers --------------
def init_servos():
    """Set camera servos to positions from servo_config.yaml."""
    try:
        servo_data = yaml_handle.get_yaml_data(yaml_handle.servo_file_path)
        servo1 = int(servo_data.get("servo1", 1500))
        servo2 = int(servo_data.get("servo2", 1500))
        Board.setPWMServoPulse(1, servo1, 1000)
        Board.setPWMServoPulse(2, servo2, 1000)
    except Exception as e:
        print(f"[warn] servo init skipped or failed: {e}")

def leds_off_safe():
    try:
        Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
        Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
        Board.RGB.show()
    except Exception:
        pass

# -------------- Camera wrapper --------------
class Cam:
    def __init__(self):
        self._cam = None
        self.frame = None
        self._th = None
        self._lock = threading.Lock()
        self._alive = False

    def open(self, correction=False):
        # Use the SDK camera so you keep the vendor pipeline
        import Camera
        self._cam = Camera.Camera()
        self._cam.camera_open(correction=correction)
        self._alive = True
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()

    def _loop(self):
        while self._alive:
            frm = self._cam.frame
            if frm is not None:
                with self._lock:
                    self.frame = frm.copy()
            time.sleep(0.01)

    def get(self):
        with self._lock:
            return None if self.frame is None else self.frame.copy()

    def close(self):
        self._alive = False
        try:
            self._cam.camera_close()
        except Exception:
            pass

# -------------- Control loop --------------
def avoidance_loop():
    """Simple forward, avoid by turning left when too close."""
    global __running

    # distance filter
    dist_buf = []

    # camera is optional, still open so overlays work if a display is attached
    cam = Cam()
    try:
        cam.open(correction=True)
    except Exception as e:
        print(f"[warn] camera open failed, continuing headless: {e}")

    t_period = 1.0 / LOOP_HZ
    last_turn_time = 0.0
    turning = False

    print("Avoidance loop started")
    while __running:
        t0 = time.time()

        # distance read, cm
        try:
            d_cm = sonar.getDistance() / 10.0
        except Exception as e:
            print(f"[warn] sonar read failed: {e}")
            d_cm = 0.0

        # moving average to smooth spikes
        dist_buf.append(max(0.0, float(d_cm)))
        if len(dist_buf) > DIST_WINDOW:
            dist_buf.pop(0)
        d_avg = float(np.mean(dist_buf)) if dist_buf else 999.0

        # state machine
        now = time.time()
        if turning:
            # keep turning for TURN_TIME_S then try forward again
            if now - last_turn_time >= TURN_TIME_S:
                turning = False
        else:
            if d_avg <= THRESHOLD_CM and d_avg > 0.0:
                # enter turn burst
                turning = True
                last_turn_time = now

        # command motors
        if turning:
            car.set_velocity(0, 90, TURN_YAW)
            action = f"TURN yaw={TURN_YAW:+.2f}"
        else:
            car.set_velocity(FWD_SPEED, 90, 0)
            action = f"FWD {FWD_SPEED}"

        # optional overlay if somebody has a display attached
        frm = cam.get()
        if frm is not None:
            try:
                txt = f"Dist:{d_avg:4.1f} cm  Act:{action}"
                cv2.putText(frm, txt, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, TEXT_COLOR, 2)
                # Do not call imshow in headless; if you want, save a throttled preview
                # cv2.imshow('Avoidance', cv2.resize(frm, (320, 240)))
                # cv2.waitKey(1)
            except Exception:
                pass

        # console log for debugging
        print(f"d={d_avg:5.1f} cm -> {action}")

        # pace loop
        dt = time.time() - t0
        if dt < t_period:
            time.sleep(t_period - dt)

    # stop on exit
    car.set_velocity(0, 90, 0)
    cam.close()
    leds_off_safe()
    try:
        cv2.destroyAllWindows()
    except Exception:
        pass
    print("Avoidance loop stopped")

# -------------- Lifecycle --------------
def start():
    global sonar, __running
    print("Avoidance Init")
    init_servos()
    leds_off_safe()
    sonar = Sonar.Sonar()
    __running = True
    avoidance_loop()

def manual_stop(signum=None, frame=None):
    global __running
    print("Closing...")
    __running = False

if __name__ == "__main__":
    signal.signal(signal.SIGINT, manual_stop)
    try:
        start()
    finally:
        manual_stop()
