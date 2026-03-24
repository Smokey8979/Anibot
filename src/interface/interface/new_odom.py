#!/usr/bin/env python3
"""
serial_odom_bridge.py  —  Anibot differential-drive odometry node

Responsibilities
────────────────
• Reads encoder ticks from Arduino over serial  (format: "E,m1,m2,ms\\n")
• Publishes /odom  (nav_msgs/Odometry) and TF odom→base_footprint at publish_rate Hz
• Subscribes /cmd_vel and translates to "M,pwm1,pwm2\\n" motor commands
• Auto-resets Arduino on startup so encoder counters start from zero
• Software velocity ramp prevents jerky starts and stops
• swap_motors corrects M1/M2 channel vs physical left/right assignment

Motor channel assignment  (swap_motors)
────────────────────────────────────────
  swap_motors = True  →  M1 = physical RIGHT wheel,  M2 = physical LEFT wheel
  swap_motors = False →  M1 = physical LEFT  wheel,  M2 = physical RIGHT wheel

Getting swap_motors wrong causes THREE simultaneous problems:
  1. left_scale/right_scale PWM corrections go to the WRONG motor → robot veers
  2. Encoder L/R assignment is mirrored → odom reports turns BACKWARDS
  3. AMCL gets contradictory lidar vs odom data → robot wobbles uncontrollably

Encoder invert flags
─────────────────────
  invert_left / invert_right should be True ONLY if that encoder physically
  counts DOWNWARD when its wheel rolls forward (a wiring / signal polarity issue).
  Do NOT use these flags to fix motor spin direction — do that at the Arduino
  or wiring level instead.

Scan TF / "scan moves with robot" explanation
──────────────────────────────────────────────
  The LaserScan is published in frame "laser".
  Full TF chain needed for the scan to appear world-fixed:
      map → odom            (AMCL — only published after you set 2D Pose Estimate!)
      odom → base_footprint (this node, 20 Hz)
      base_footprint → … → laser  (robot_state_publisher from URDF)

  If the scan moves with the robot, check in order:
    ① RViz fixed frame must be "map" (not "odom" — odom frame always moves with robot)
    ② Set 2D Pose Estimate in RViz so AMCL publishes map→odom
    ③ swap_motors must be correct so odom is accurate enough for AMCL to localise
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial
import threading
import math
import time


class SerialOdomBridge(Node):
    def __init__(self):
        super().__init__('serial_odom_bridge')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('serial_port',      '/dev/ttyACM0')
        self.declare_parameter('baudrate',         115200)
        self.declare_parameter('encoder_ppr',      28)
        self.declare_parameter('gear_ratio',       19.2)
        self.declare_parameter('wheel_radius',     0.041)
        self.declare_parameter('wheel_base',       0.355)
        self.declare_parameter('publish_rate',     20.0)
        self.declare_parameter('max_linear_speed', 0.20)
        self.declare_parameter('max_pwm',          150)
        self.declare_parameter('cmd_timeout_ms',   500)
        self.declare_parameter('invert_left',      False)
        self.declare_parameter('invert_right',     False)
        self.declare_parameter('left_scale',       0.82)
        self.declare_parameter('right_scale',      1.0)
        self.declare_parameter('ramp_step',        10)
        self.declare_parameter('ramp_step_stop',   20)
        self.declare_parameter('swap_motors',      True)

        self.serial_port      = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate         = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.encoder_ppr      = float(self.get_parameter('encoder_ppr').get_parameter_value().integer_value)
        self.gear_ratio       = float(self.get_parameter('gear_ratio').get_parameter_value().double_value)
        self.wheel_radius     = float(self.get_parameter('wheel_radius').get_parameter_value().double_value)
        self.wheel_base       = float(self.get_parameter('wheel_base').get_parameter_value().double_value)
        self.publish_rate     = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').get_parameter_value().double_value)
        self.max_pwm          = int(self.get_parameter('max_pwm').get_parameter_value().integer_value)
        self.cmd_timeout_ms   = int(self.get_parameter('cmd_timeout_ms').get_parameter_value().integer_value)
        self.invert_left      = bool(self.get_parameter('invert_left').get_parameter_value().bool_value)
        self.invert_right     = bool(self.get_parameter('invert_right').get_parameter_value().bool_value)
        self.left_scale       = float(self.get_parameter('left_scale').get_parameter_value().double_value)
        self.right_scale      = float(self.get_parameter('right_scale').get_parameter_value().double_value)
        self.ramp_step        = int(self.get_parameter('ramp_step').get_parameter_value().integer_value)
        self.ramp_step_stop   = int(self.get_parameter('ramp_step_stop').get_parameter_value().integer_value)
        self.swap_motors      = bool(self.get_parameter('swap_motors').get_parameter_value().bool_value)

        self.ticks_per_wheel_rev = self.encoder_ppr * self.gear_ratio
        self.ticks_per_meter     = self.ticks_per_wheel_rev / (2.0 * math.pi * self.wheel_radius)

        # ── Startup summary ───────────────────────────────────────────────
        self.get_logger().info(
            f"serial={self.serial_port}@{self.baudrate}  "
            f"ppr={self.encoder_ppr}  gear={self.gear_ratio}"
        )
        self.get_logger().info(
            f"wheel_r={self.wheel_radius}m  wheel_base={self.wheel_base}m  "
            f"ticks/m={self.ticks_per_meter:.1f}"
        )
        self.get_logger().info(
            f"swap_motors={self.swap_motors}  "
            f"left_scale={self.left_scale}  right_scale={self.right_scale}  "
            f"invert_left={self.invert_left}  invert_right={self.invert_right}  "
            f"max_pwm={self.max_pwm}  max_speed={self.max_linear_speed}m/s"
        )
        if not self.swap_motors:
            self.get_logger().warn(
                "swap_motors=False → M1 treated as LEFT motor, M2 as RIGHT motor. "
                "If your robot veers right when driving straight, or odom reports "
                "turns in the wrong direction, set swap_motors=True in bringup.launch.py"
            )

        # ── State ─────────────────────────────────────────────────────────
        self._m1_raw            = 0
        self._m2_raw            = 0
        self._serial_lock       = threading.Lock()
        self._serial            = None
        self._last_serial_time  = None
        self._first_move_logged = False   # for the one-time direction check

        self.x          = 0.0
        self.y          = 0.0
        self.th         = 0.0
        self.last_left  = None
        self.last_right = None
        self.last_time  = None
        self.last_cmd_time = 0.0

        self._target_pwm_l  = 0
        self._target_pwm_r  = 0
        self._current_pwm_l = 0
        self._current_pwm_r = 0
        self._pwm_lock = threading.Lock()

        # ── ROS interfaces ────────────────────────────────────────────────
        self.odom_pub       = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist,  'cmd_vel',     self.cmd_vel_cb,    10)
        self.create_subscription(Twist,  'bot_cmd_vel', self.cmd_vel_cb,    10)
        self.create_subscription(String, 'serial_cmd',  self.serial_cmd_cb, 10)

        # ── Serial open + Arduino reset ───────────────────────────────────
        try:
            self._serial = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"Opened serial {self.serial_port}")
            time.sleep(2.5)
            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
                for _ in range(3):
                    self._serial.write(b"reset\n")
                    self._serial.flush()
                    time.sleep(0.3)
                self._serial.reset_input_buffer()
                self.get_logger().info("Arduino reset ×3 — encoders zeroed, odom clean")
            except Exception as e:
                self.get_logger().warn(f"Could not send reset: {e}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            self._serial = None

        self._stop_reader   = threading.Event()
        self._reader_thread = threading.Thread(target=self._serial_reader, daemon=True)
        self._reader_thread.start()

        self._timer = self.create_timer(1.0 / self.publish_rate, self._timer_cb)

    # ── Serial reader thread ──────────────────────────────────────────────

    def _serial_reader(self):
        """Parse 'E,m1,m2,ms' lines from Arduino."""
        buf = ""
        while not self._stop_reader.is_set():
            if self._serial is None:
                time.sleep(0.5)
                continue
            try:
                data = self._serial.read(self._serial.in_waiting or 1)
                if not data:
                    time.sleep(0.001)
                    continue
                buf += data.decode('ascii', errors='ignore')
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    parts = line.split(',')
                    if len(parts) >= 4 and parts[0] == 'E':
                        try:
                            m1 = int(parts[1])
                            m2 = int(parts[2])
                        except Exception:
                            continue
                        with self._serial_lock:
                            self._m1_raw           = m1
                            self._m2_raw           = m2
                            self._last_serial_time = time.time()
                    else:
                        self.get_logger().debug(f"serial: {line}")
            except Exception as e:
                self.get_logger().warn(f"serial read error: {e}")
                time.sleep(0.2)

    def _write_serial(self, s: str) -> bool:
        if self._serial is None:
            return False
        try:
            with self._serial_lock:
                self._serial.write(
                    (s if s.endswith('\n') else s + '\n').encode('ascii', errors='ignore')
                )
            return True
        except Exception as e:
            self.get_logger().error(f"serial write error: {e}")
            return False

    def serial_cmd_cb(self, msg: String):
        cmd = msg.data.strip()
        if cmd:
            self._write_serial(cmd)

    # ── cmd_vel ───────────────────────────────────────────────────────────

    def _vel_to_pwm(self, v_wheel: float, scale: float) -> int:
        """Wheel speed (m/s) → signed PWM with motor deadband compensation."""
        clipped = max(-self.max_linear_speed, min(self.max_linear_speed, v_wheel))
        if abs(clipped) < 1e-6:
            return 0
        PWM_MIN = 70  # empirical stall-avoidance minimum (~60 is hardware deadband)
        ratio   = abs(clipped) / self.max_linear_speed
        pwm_mag = int(PWM_MIN + ratio * (self.max_pwm - PWM_MIN))
        pwm_mag = int(round(pwm_mag * scale))
        pwm_mag = min(pwm_mag, self.max_pwm)
        return pwm_mag if clipped > 0 else -pwm_mag

    def cmd_vel_cb(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z
        # Standard diff-drive kinematics → physical wheel speeds
        v_left  = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)
        with self._pwm_lock:
            self._target_pwm_l = self._vel_to_pwm(v_left,  self.left_scale)
            self._target_pwm_r = self._vel_to_pwm(v_right, self.right_scale)
        self.last_cmd_time = time.time()

    def _ramp(self, current: int, target: int) -> int:
        if current == target:
            return current
        step = self.ramp_step_stop if target == 0 else self.ramp_step
        diff = target - current
        if abs(diff) <= step:
            return target
        return current + step if diff > 0 else current - step

    # ── Timer callback ────────────────────────────────────────────────────

    def _timer_cb(self):
        # ── 1. Snapshot raw encoder counts ─────────────────────────────────
        with self._serial_lock:
            m1_raw = self._m1_raw
            m2_raw = self._m2_raw

        # ── 2. Assign M1/M2 to physical LEFT/RIGHT ──────────────────────────
        #
        #   swap_motors=True  → M1=physical_RIGHT encoder,  M2=physical_LEFT encoder
        #   swap_motors=False → M1=physical_LEFT  encoder,  M2=physical_RIGHT encoder
        #
        #   This assignment MUST be consistent with the M command write below,
        #   and it MUST match the physical hardware wiring.
        if self.swap_motors:
            l_raw = m2_raw   # physical LEFT  ← Arduino M2
            r_raw = m1_raw   # physical RIGHT ← Arduino M1
        else:
            l_raw = m1_raw   # physical LEFT  ← Arduino M1
            r_raw = m2_raw   # physical RIGHT ← Arduino M2

        # Negate only if that encoder counts down for forward rotation
        if self.invert_left:
            l_raw = -l_raw
        if self.invert_right:
            r_raw = -r_raw

        # ── 3. Odometry ─────────────────────────────────────────────────────
        now_t = time.time()

        if self.last_left is None:
            self.last_left  = l_raw
            self.last_right = r_raw
            self.last_time  = now_t
        else:
            dt = now_t - self.last_time
            if dt > 0.0:
                dl_ticks = l_raw - self.last_left
                dr_ticks = r_raw - self.last_right

                # One-time forward-direction sanity check
                if not self._first_move_logged and (abs(dl_ticks) > 5 or abs(dr_ticks) > 5):
                    self._first_move_logged = True
                    sign_l = "+" if dl_ticks >= 0 else "-"
                    sign_r = "+" if dr_ticks >= 0 else "-"
                    ok = (dl_ticks > 0 and dr_ticks > 0)
                    self.get_logger().info(
                        f"FIRST ENCODER MOVEMENT: L={sign_l}{abs(dl_ticks):d} "
                        f"R={sign_r}{abs(dr_ticks):d}  "
                        f"{'OK - both positive = robot moving forward' if ok else 'Check encoder direction'}"
                    )

                # Ticks → metres
                dl_m = (dl_ticks / self.ticks_per_wheel_rev) * (2.0 * math.pi * self.wheel_radius)
                dr_m = (dr_ticks / self.ticks_per_wheel_rev) * (2.0 * math.pi * self.wheel_radius)

                # Diff-drive pose integration
                d      = (dr_m + dl_m) / 2.0
                dtheta = (dr_m - dl_m) / self.wheel_base

                if abs(dtheta) < 1e-6:
                    dx_body = d
                    dy_body = 0.0
                else:
                    R       = d / dtheta
                    dx_body = R * math.sin(dtheta)
                    dy_body = R * (1.0 - math.cos(dtheta))

                self.x  += math.cos(self.th) * dx_body - math.sin(self.th) * dy_body
                self.y  += math.sin(self.th) * dx_body + math.cos(self.th) * dy_body
                self.th  = (self.th + dtheta + math.pi) % (2.0 * math.pi) - math.pi

                vx  = d      / dt
                vth = dtheta / dt

                qz = math.sin(self.th * 0.5)
                qw = math.cos(self.th * 0.5)

                # ── Odometry message ────────────────────────────────────────
                odom                         = Odometry()
                odom.header.stamp            = self.get_clock().now().to_msg()
                odom.header.frame_id         = 'odom'
                odom.child_frame_id          = 'base_footprint'
                odom.pose.pose.position.x    = self.x
                odom.pose.pose.position.y    = self.y
                odom.pose.pose.position.z    = 0.0
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                odom.twist.twist.linear.x    = vx
                odom.twist.twist.angular.z   = vth

                # Pose covariance — 6×6 row-major [x, y, z, roll, pitch, yaw]
                # z/roll/pitch are NOT measured by a 2D diff-drive robot.
                # Setting them to 0 (the default) tells the filter we know them
                # perfectly, which is false and can cause numerical instability.
                # Setting to 1e6 tells the filter "I have no idea" — correct.
                odom.pose.covariance[0]  = 0.10   # x     — measured, modest uncertainty
                odom.pose.covariance[7]  = 0.10   # y
                odom.pose.covariance[14] = 1e6    # z     — NOT measured
                odom.pose.covariance[21] = 1e6    # roll  — NOT measured
                odom.pose.covariance[28] = 1e6    # pitch — NOT measured
                odom.pose.covariance[35] = 0.20   # yaw   — measured but has wheel-imbalance error

                odom.twist.covariance[0]  = 0.10
                odom.twist.covariance[7]  = 0.10
                odom.twist.covariance[14] = 1e6
                odom.twist.covariance[21] = 1e6
                odom.twist.covariance[28] = 1e6
                odom.twist.covariance[35] = 0.20

                self.odom_pub.publish(odom)

                # ── odom → base_footprint TF ────────────────────────────────
                t                         = TransformStamped()
                t.header.stamp            = odom.header.stamp
                t.header.frame_id         = 'odom'
                t.child_frame_id          = 'base_footprint'
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation.z    = qz
                t.transform.rotation.w    = qw
                self.tf_broadcaster.sendTransform(t)

            self.last_left  = l_raw
            self.last_right = r_raw
            self.last_time  = now_t

        # ── 4. cmd_vel watchdog ─────────────────────────────────────────────
        if (time.time() - self.last_cmd_time) * 1000.0 > self.cmd_timeout_ms:
            with self._pwm_lock:
                self._target_pwm_l = 0
                self._target_pwm_r = 0

        # ── 5. Ramp PWM and send ────────────────────────────────────────────
        with self._pwm_lock:
            new_l = self._ramp(self._current_pwm_l, self._target_pwm_l)
            new_r = self._ramp(self._current_pwm_r, self._target_pwm_r)
            changed              = (new_l != self._current_pwm_l or
                                    new_r != self._current_pwm_r)
            self._current_pwm_l = new_l
            self._current_pwm_r = new_r

        if changed:
            # MUST mirror the encoder assignment above:
            #   swap_motors=True  → M1=physical_right, M2=physical_left
            #   swap_motors=False → M1=physical_left,  M2=physical_right
            if self.swap_motors:
                serial_cmd = f"M,{new_r},{new_l}"   # M1 gets right PWM, M2 gets left PWM
            else:
                serial_cmd = f"M,{new_l},{new_r}"   # M1 gets left PWM,  M2 gets right PWM
            self._write_serial(serial_cmd)
            self.get_logger().debug(f"PWM→ {serial_cmd}")

    # ── Cleanup ───────────────────────────────────────────────────────────

    def destroy_node(self):
        self._stop_reader.set()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=0.5)
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
