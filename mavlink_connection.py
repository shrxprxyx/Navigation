"""
mavlink_connection.py
----------------------
Thin wrapper around pymavlink for the specific calls corridor navigation needs:
connect, arm, takeoff, guided-mode velocity commands, and reading telemetry
(position, distance sensors) back from SITL / the real flight controller.

This is deliberately NOT a full autopilot abstraction -- just enough to run
the corridor navigator against Mission Planner's SITL and, later, against
real hardware with minimal changes.
"""

import time
from pymavlink import mavutil
import config


class Vehicle:
    def __init__(self, connection_string=None):
        self.connection_string = connection_string or config.MAVLINK_CONNECTION_STRING
        self.master = None

    # ---------------- connection ----------------
    def connect(self, timeout_s=30):
        print(f"[mavlink] Connecting to {self.connection_string} ...")
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.master.wait_heartbeat(timeout=timeout_s)
        print(f"[mavlink] Heartbeat received (sysid={self.master.target_system}, "
              f"compid={self.master.target_component})")
        self._request_data_streams()
        return self.master

    def _request_data_streams(self, rate_hz=10):
        """
        ArduPilot only pushes telemetry (position, altitude, etc.) to a GCS
        that has explicitly asked for it -- Mission Planner does this
        automatically on connect, but our script is a separate client and
        needs to request it too, or things like get_relative_altitude()
        will just hang / return None.
        """
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,  # start streaming
        )
        # give the autopilot a moment to start pushing messages
        time.sleep(0.5)

    # ---------------- arming / mode ----------------
    def set_mode(self, mode_name):
        mode_id = self.master.mode_mapping()[mode_name]
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        self._wait_for_mode(mode_name)

    def _wait_for_mode(self, mode_name, timeout_s=10):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
            if msg is None:
                continue
            current_mode = mavutil.mode_string_v10(msg)
            if current_mode == mode_name:
                print(f"[mavlink] Mode set to {mode_name}")
                return True
        print(f"[mavlink] WARNING: mode {mode_name} not confirmed within timeout")
        return False

    def arm(self, wait_armed=True, timeout_s=45, retry_interval_s=3):
        """
        Sends the arm command and retries periodically until armed or timeout.
        SITL commonly refuses to arm for the first ~10-20s after starting
        (PreArm: Need Position Estimate) while the EKF settles, even if GPS
        already shows a fix -- so a single arm attempt often isn't enough.
        """
        print("[mavlink] Arming (will retry until armed or timeout)...")
        t0 = time.time()
        last_attempt = 0

        while time.time() - t0 < timeout_s:
            if time.time() - last_attempt >= retry_interval_s:
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0, 1, 0, 0, 0, 0, 0, 0,
                )
                last_attempt = time.time()

            if not wait_armed:
                return True

            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                print("[mavlink] Armed.")
                return True

            # surface any PreArm/failure reason text so it's not a silent wait
            status = self.master.recv_match(type="STATUSTEXT", blocking=False)
            if status and status.text:
                print(f"[mavlink] STATUSTEXT: {status.text}")

        print(f"[mavlink] WARNING: arm not confirmed within {timeout_s}s timeout")
        return False

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        print("[mavlink] Disarm command sent.")

    # ---------------- takeoff ----------------
    def takeoff(self, altitude_m):
        print(f"[mavlink] Taking off to {altitude_m} m ...")
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude_m,
        )
        self._wait_for_altitude(altitude_m)

    def _wait_for_altitude(self, target_altitude_m, tolerance_m=0.3, timeout_s=30):
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            alt = self.get_relative_altitude()
            if alt is not None and abs(alt - target_altitude_m) <= tolerance_m:
                print(f"[mavlink] Reached altitude {alt:.2f} m")
                return True
            time.sleep(0.2)
        print("[mavlink] WARNING: target altitude not confirmed within timeout")
        return False

    # ---------------- telemetry ----------------
    def get_relative_altitude(self):
        msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if msg is None:
            return None
        return msg.relative_alt / 1000.0  # mm -> m

    def get_local_position(self):
        """Returns (x, y, z) in local NED frame (meters), or None."""
        msg = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg is None:
            return None
        return (msg.x, msg.y, msg.z)

    def get_yaw(self):
        """Returns current yaw in radians (NED, 0 = North), or None."""
        msg = self.master.recv_match(type="ATTITUDE", blocking=True, timeout=2)
        if msg is None:
            return None
        return msg.yaw

    # ---------------- guided velocity control ----------------
    def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
        """
        Send velocity setpoint in BODY frame (forward, right, down), m/s.
        This is what the corridor navigator calls every control-loop tick.
        """
        type_mask = (
            0b0000_0111_1100_0111  # ignore position & accel, use velocity + yaw_rate
        )
        self.master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (ignored)
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            0, 0, 0,        # x, y, z position (ignored)
            vx, vy, vz,     # velocity
            0, 0, 0,        # accel (ignored)
            0, yaw_rate,    # yaw, yaw_rate
        )

    def hold_position(self):
        self.send_velocity_body(0, 0, 0, 0)

    # ---------------- fake distance sensor injection (for sim option A) ----------------
    def send_fake_distance_sensor(self, distance_cm, orientation, sensor_id):
        """
        orientation: MAV_SENSOR_ROTATION_* (e.g. NONE=front, YAW_90=right, YAW_270=left)
        Used only when simulating without real hardware / without Gazebo range plugins.
        """
        self.master.mav.distance_sensor_send(
            0,                      # time_boot_ms
            10,                     # min_distance (cm)
            800,                    # max_distance (cm)
            int(distance_cm),
            mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
            sensor_id,
            orientation,
            0,                      # covariance
        )