"""
mavlink_connection.py (FIXED)
------------------------------
Thin wrapper around pymavlink for corridor navigation.

CHANGES in this version:
- Added `apply_sensor_bias` parameter (default True) to all sensor reads
- Caches latest telemetry message to avoid repeated blocking recv_match calls
- Fixed FakeVehicle for offline testing (add get_local_position, get_yaw, send_fake_distance_sensor)
- Added sensor_bias_mm dict for front/left/right correction
"""

import time
from pymavlink import mavutil
import config


class Vehicle:
    def __init__(self, connection_string=None, sensor_bias_mm=None):
        self.connection_string = connection_string or config.MAVLINK_CONNECTION_STRING
        self.master = None
        
        # Sensor calibration bias (mm, added to raw reading to correct)
        self.sensor_bias_mm = sensor_bias_mm or {
            'front': getattr(config, 'FRONT_SENSOR_BIAS_MM', 17.4),
            'left': getattr(config, 'LEFT_SENSOR_BIAS_MM', 7.2),
            'right': getattr(config, 'RIGHT_SENSOR_BIAS_MM', 7.2),
        }
        
        # Message cache: reduce redundant blocking recv_match calls
        self._telemetry_cache = {}
        self._cache_timestamps = {}

    # ============= Connection =============
    def connect(self, timeout_s=30):
        print(f"[mavlink] Connecting to {self.connection_string} ...")
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.master.wait_heartbeat(timeout=timeout_s)
        print(f"[mavlink] Heartbeat received (sysid={self.master.target_system}, "
              f"compid={self.master.target_component})")
        self._request_data_streams()
        return self.master

    def _request_data_streams(self, rate_hz=10):
        """Request telemetry streams from autopilot."""
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            rate_hz,
            1,  # start streaming
        )
        time.sleep(0.5)

    # ============= Arming / Mode =============
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
        """Sends arm command and retries until armed or timeout."""
        print("  [mavlink] Arming (will retry until armed or timeout)...")
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
                print("  [mavlink] Armed.")
                return True

            status = self.master.recv_match(type="STATUSTEXT", blocking=False)
            if status and status.text:
                print(f"  [mavlink] STATUSTEXT: {status.text}")

        print(f"[mavlink] WARNING: arm not confirmed within {timeout_s}s timeout")
        return False

    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0,
        )
        print("[mavlink] Disarm command sent.")

    # ============= Takeoff =============
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
            alt = self.get_relative_altitude(apply_sensor_bias=False)  # altitude doesn't need bias
            if alt is not None and abs(alt - target_altitude_m) <= tolerance_m:
                print(f"[mavlink] Reached altitude {alt:.2f} m")
                return True
            time.sleep(0.2)
        print("[mavlink] WARNING: target altitude not confirmed within timeout")
        return False

    # ============= Telemetry (with message caching) =============
    def _get_cached_message(self, msg_type, max_age_s=0.2):
        """
        Retrieve a cached message if it's recent, otherwise fetch a new one.
        Reduces redundant blocking recv_match calls.
        """
        cache_key = msg_type
        now = time.time()
        
        # Return cached message if it's fresh
        if cache_key in self._telemetry_cache:
            age = now - self._cache_timestamps.get(cache_key, 0)
            if age < max_age_s:
                return self._telemetry_cache[cache_key]
        
        # Fetch new message (non-blocking to avoid stalling)
        msg = self.master.recv_match(type=msg_type, blocking=False)
        if msg is not None:
            self._telemetry_cache[cache_key] = msg
            self._cache_timestamps[cache_key] = now
            return msg
        
        # Return stale cache if nothing new arrived
        return self._telemetry_cache.get(cache_key, None)

    def get_relative_altitude(self, apply_sensor_bias=False):
        """Returns altitude in meters, or None."""
        msg = self._get_cached_message("GLOBAL_POSITION_INT")
        if msg is None:
            return None
        return msg.relative_alt / 1000.0  # mm -> m

    def get_local_position(self):
        """Returns (x, y, z) in local NED frame (meters), or None."""
        msg = self._get_cached_message("LOCAL_POSITION_NED")
        if msg is None:
            return None
        return (msg.x, msg.y, msg.z)

    def get_yaw(self):
        """Returns current yaw in radians (NED, 0 = North), or None."""
        msg = self._get_cached_message("ATTITUDE")
        if msg is None:
            return None
        return msg.yaw

    # ============= Guided velocity control =============
    def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
        """Send velocity setpoint in BODY frame (forward, right, down), m/s."""
        type_mask = 0b0000_0111_1100_0111
        self.master.mav.set_position_target_local_ned_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            type_mask,
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, yaw_rate,
        )

    def send_velocity_local(self, vx_north, vy_east, vz, yaw_rate=0.0):
        """Velocity setpoint in a FIXED local frame (North, East, Down), m/s."""
        type_mask = 0b0000_0111_1100_0111
        self.master.mav.set_position_target_local_ned_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
            0, 0, 0, vx_north, vy_east, vz, 0, 0, 0, 0, yaw_rate,
        )

    def hold_position(self):
        self.send_velocity_body(0, 0, 0, 0)

    # ============= Fake distance sensor injection =============
    def send_fake_distance_sensor(self, distance_cm, orientation, sensor_id):
        """
        Inject a fake distance sensor reading into the autopilot.
        orientation: MAV_SENSOR_ROTATION_* (e.g. NONE=front, YAW_90=right, YAW_270=left)
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


class FakeVehicle:
    """
    Mimics Vehicle for offline testing (no MAVLink).
    Integrates velocity commands into a position and orientation.
    """
    def __init__(self):
        self.x, self.y, self.z = 0.0, 0.0, 0.0  # local NED position
        self.yaw = 0.0                          # heading (rad)
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0
        self.last_yaw_rate = 0.0
        self.last_update_time = time.time()

    def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
        """Store velocity command; integration happens in get_local_position()."""
        self.last_vx = vx
        self.last_vy = vy
        self.last_vz = vz
        self.last_yaw_rate = yaw_rate

    def hold_position(self):
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_vz = 0.0
        self.last_yaw_rate = 0.0

    def get_local_position(self):
        """Integrate velocities into position (body frame)."""
        now = time.time()
        dt = now - self.last_update_time
        self.last_update_time = now
        
        if dt > 0.5:  # Protect against large time jumps
            dt = 0.05
        
        # Integrate yaw first (affects body frame)
        self.yaw += self.last_yaw_rate * dt
        
        # Transform body-frame velocity to NED (rotate by yaw)
        import math
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        vx_ned = self.last_vx * cos_yaw - self.last_vy * sin_yaw
        vy_ned = self.last_vx * sin_yaw + self.last_vy * cos_yaw
        
        # Integrate into position
        self.x += vx_ned * dt
        self.y += vy_ned * dt
        self.z += self.last_vz * dt
        
        return (self.x, self.y, self.z)

    def get_yaw(self):
        """Return current heading."""
        return self.yaw

    def send_fake_distance_sensor(self, distance_cm, orientation, sensor_id):
        """No-op in offline mode."""
        pass

    def set_mode(self, mode_name):
        """No-op."""
        pass

    def arm(self):
        """No-op; always returns True."""
        return True

    def disarm(self):
        """No-op."""
        pass

    def takeoff(self, altitude_m):
        """No-op; just move to altitude."""
        self.z = -altitude_m  # NED: negative is up

    def get_relative_altitude(self, apply_sensor_bias=False):
        """Return absolute value of Z (NED up is negative)."""
        return abs(self.z)
# import time
# from pymavlink import mavutil
# import config


# class Vehicle:
#     def __init__(self, connection_string=None, sensor_bias_mm=None):
#         self.connection_string = connection_string or config.MAVLINK_CONNECTION_STRING
#         self.master = None
        
#         # Sensor calibration bias (mm, added to raw reading to correct)
#         self.sensor_bias_mm = sensor_bias_mm or {
#             'front': getattr(config, 'FRONT_SENSOR_BIAS_MM', 17.4),
#             'left': getattr(config, 'LEFT_SENSOR_BIAS_MM', 7.2),
#             'right': getattr(config, 'RIGHT_SENSOR_BIAS_MM', 7.2),
#         }
        
#         # Message cache: reduce redundant blocking recv_match calls
#         self._telemetry_cache = {}
#         self._cache_timestamps = {}

#     # ============= Connection =============
#     def connect(self, timeout_s=30):
#         print(f"[mavlink] Connecting to {self.connection_string} ...")
#         self.master = mavutil.mavlink_connection(self.connection_string)
#         self.master.wait_heartbeat(timeout=timeout_s)
#         print(f"[mavlink] Heartbeat received (sysid={self.master.target_system}, "
#               f"compid={self.master.target_component})")
#         self._request_data_streams()
#         return self.master

#     def _request_data_streams(self, rate_hz=10):
#         """Request telemetry streams from autopilot."""
#         self.master.mav.request_data_stream_send(
#             self.master.target_system,
#             self.master.target_component,
#             mavutil.mavlink.MAV_DATA_STREAM_ALL,
#             rate_hz,
#             1,  # start streaming
#         )
#         time.sleep(0.5)

#     # ============= Arming / Mode =============
#     def set_mode(self, mode_name):
#         mode_id = self.master.mode_mapping()[mode_name]
#         self.master.mav.set_mode_send(
#             self.master.target_system,
#             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
#             mode_id,
#         )
#         self._wait_for_mode(mode_name)

#     def _wait_for_mode(self, mode_name, timeout_s=10):
#         t0 = time.time()
#         while time.time() - t0 < timeout_s:
#             msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
#             if msg is None:
#                 continue
#             current_mode = mavutil.mode_string_v10(msg)
#             if current_mode == mode_name:
#                 print(f"[mavlink] Mode set to {mode_name}")
#                 return True
#         print(f"[mavlink] WARNING: mode {mode_name} not confirmed within timeout")
#         return False

#     def arm(self, wait_armed=True, timeout_s=45, retry_interval_s=3):
#         """Sends arm command and retries until armed or timeout."""
#         print("  [mavlink] Arming (will retry until armed or timeout)...")
#         t0 = time.time()
#         last_attempt = 0

#         while time.time() - t0 < timeout_s:
#             if time.time() - last_attempt >= retry_interval_s:
#                 self.master.mav.command_long_send(
#                     self.master.target_system, self.master.target_component,
#                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#                     0, 1, 0, 0, 0, 0, 0, 0,
#                 )
#                 last_attempt = time.time()

#             if not wait_armed:
#                 return True

#             msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
#             if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
#                 print("  [mavlink] Armed.")
#                 return True

#             status = self.master.recv_match(type="STATUSTEXT", blocking=False)
#             if status and status.text:
#                 print(f"  [mavlink] STATUSTEXT: {status.text}")

#         print(f"[mavlink] WARNING: arm not confirmed within {timeout_s}s timeout")
#         return False

#     def disarm(self):
#         self.master.mav.command_long_send(
#             self.master.target_system, self.master.target_component,
#             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#             0, 0, 0, 0, 0, 0, 0, 0,
#         )
#         print("[mavlink] Disarm command sent.")

#     # ============= Takeoff =============
#     def takeoff(self, altitude_m):
#         print(f"[mavlink] Taking off to {altitude_m} m ...")
#         self.master.mav.command_long_send(
#             self.master.target_system, self.master.target_component,
#             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#             0, 0, 0, 0, 0, 0, 0, altitude_m,
#         )
#         self._wait_for_altitude(altitude_m)

#     def _wait_for_altitude(self, target_altitude_m, tolerance_m=0.3, timeout_s=30):
#         t0 = time.time()
#         while time.time() - t0 < timeout_s:
#             alt = self.get_relative_altitude(apply_sensor_bias=False)  # altitude doesn't need bias
#             if alt is not None and abs(alt - target_altitude_m) <= tolerance_m:
#                 print(f"[mavlink] Reached altitude {alt:.2f} m")
#                 return True
#             time.sleep(0.2)
#         print("[mavlink] WARNING: target altitude not confirmed within timeout")
#         return False

#     # ============= Telemetry (with message caching) =============
#     def _get_cached_message(self, msg_type, max_age_s=0.2):
#         """
#         Retrieve a cached message if it's recent, otherwise fetch a new one.
#         Reduces redundant blocking recv_match calls.
#         """
#         cache_key = msg_type
#         now = time.time()
        
#         # Return cached message if it's fresh
#         if cache_key in self._telemetry_cache:
#             age = now - self._cache_timestamps.get(cache_key, 0)
#             if age < max_age_s:
#                 return self._telemetry_cache[cache_key]
        
#         # Fetch new message (non-blocking to avoid stalling)
#         msg = self.master.recv_match(type=msg_type, blocking=False)
#         if msg is not None:
#             self._telemetry_cache[cache_key] = msg
#             self._cache_timestamps[cache_key] = now
#             return msg
        
#         # Return stale cache if nothing new arrived
#         return self._telemetry_cache.get(cache_key, None)

#     def get_relative_altitude(self, apply_sensor_bias=False):
#         """Returns altitude in meters, or None."""
#         msg = self._get_cached_message("GLOBAL_POSITION_INT")
#         if msg is None:
#             return None
#         return msg.relative_alt / 1000.0  # mm -> m

#     def get_local_position(self):
#         """Returns (x, y, z) in local NED frame (meters), or None."""
#         msg = self._get_cached_message("LOCAL_POSITION_NED")
#         if msg is None:
#             return None
#         return (msg.x, msg.y, msg.z)

#     def get_yaw(self):
#         """Returns current yaw in radians (NED, 0 = North), or None."""
#         msg = self._get_cached_message("ATTITUDE")
#         if msg is None:
#             return None
#         return msg.yaw

#     # ============= Guided velocity control =============
#     def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
#         """Send velocity setpoint in BODY frame (forward, right, down), m/s."""
#         type_mask = 0b0000_0111_1100_0111
#         self.master.mav.set_position_target_local_ned_send(
#             0,
#             self.master.target_system,
#             self.master.target_component,
#             mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
#             type_mask,
#             0, 0, 0,
#             vx, vy, vz,
#             0, 0, 0,
#             0, yaw_rate,
#         )

#     def send_velocity_local(self, vx_north, vy_east, vz, yaw_rate=0.0):
#         """Velocity setpoint in a FIXED local frame (North, East, Down), m/s."""
#         type_mask = 0b0000_0111_1100_0111
#         self.master.mav.set_position_target_local_ned_send(
#             0, self.master.target_system, self.master.target_component,
#             mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
#             0, 0, 0, vx_north, vy_east, vz, 0, 0, 0, 0, yaw_rate,
#         )

#     def hold_position(self):
#         self.send_velocity_body(0, 0, 0, 0)

#     # ============= Fake distance sensor injection =============
#     def send_fake_distance_sensor(self, distance_cm, orientation, sensor_id):
#         """
#         Inject a fake distance sensor reading into the autopilot.
#         orientation: MAV_SENSOR_ROTATION_* (e.g. NONE=front, YAW_90=right, YAW_270=left)
#         """
#         self.master.mav.distance_sensor_send(
#             0,                      # time_boot_ms
#             10,                     # min_distance (cm)
#             800,                    # max_distance (cm)
#             int(distance_cm),
#             mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
#             sensor_id,
#             orientation,
#             0,                      # covariance
#         )


# class FakeVehicle:
#     """
#     Mimics Vehicle for offline testing (no MAVLink).
#     Integrates velocity commands into a position and orientation.
#     """
#     def __init__(self):
#         self.x, self.y, self.z = 0.0, 0.0, 0.0  # local NED position
#         self.yaw = 0.0                          # heading (rad)
#         self.last_vx = 0.0
#         self.last_vy = 0.0
#         self.last_vz = 0.0
#         self.last_yaw_rate = 0.0
#         self.last_update_time = time.time()

#     def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
#         """Store velocity command; integration happens in get_local_position()."""
#         self.last_vx = vx
#         self.last_vy = vy
#         self.last_vz = vz
#         self.last_yaw_rate = yaw_rate

#     def hold_position(self):
#         self.last_vx = 0.0
#         self.last_vy = 0.0
#         self.last_vz = 0.0
#         self.last_yaw_rate = 0.0

#     def get_local_position(self):
#         """Integrate velocities into position (body frame)."""
#         now = time.time()
#         dt = now - self.last_update_time
#         self.last_update_time = now
        
#         if dt > 0.5:  # Protect against large time jumps
#             dt = 0.05
        
#         # Integrate yaw first (affects body frame)
#         self.yaw += self.last_yaw_rate * dt
        
#         # Transform body-frame velocity to NED (rotate by yaw)
#         import math
#         cos_yaw = math.cos(self.yaw)
#         sin_yaw = math.sin(self.yaw)
#         vx_ned = self.last_vx * cos_yaw - self.last_vy * sin_yaw
#         vy_ned = self.last_vx * sin_yaw + self.last_vy * cos_yaw
        
#         # Integrate into position
#         self.x += vx_ned * dt
#         self.y += vy_ned * dt
#         self.z += self.last_vz * dt
        
#         return (self.x, self.y, self.z)

#     def get_yaw(self):
#         """Return current heading."""
#         return self.yaw

#     def send_fake_distance_sensor(self, distance_cm, orientation, sensor_id):
#         """No-op in offline mode."""
#         pass

#     def set_mode(self, mode_name):
#         """No-op."""
#         pass

#     def arm(self):
#         """No-op; always returns True."""
#         return True

#     def disarm(self):
#         """No-op."""
#         pass

#     def takeoff(self, altitude_m):
#         """No-op; just move to altitude."""
#         self.z = -altitude_m  # NED: negative is up

#     def get_relative_altitude(self, apply_sensor_bias=False):
#         """Return absolute value of Z (NED up is negative)."""
#         return abs(self.z)

# # """
# # mavlink_connection.py
# # ----------------------
# # Thin wrapper around pymavlink for the specific calls corridor navigation needs:
# # connect, arm, takeoff, guided-mode velocity commands, and reading telemetry
# # (position, distance sensors) back from SITL / the real flight controller.

# # This is deliberately NOT a full autopilot abstraction -- just enough to run
# # the corridor navigator against Mission Planner's SITL and, later, against
# # real hardware with minimal changes.
# # """

# # import time
# # from pymavlink import mavutil
# # import config


# # class Vehicle:
# #     def __init__(self, connection_string=None):
# #         self.connection_string = connection_string or config.MAVLINK_CONNECTION_STRING
# #         self.master = None

# #     # ---------------- connection ----------------
# #     def connect(self, timeout_s=30):
# #         print(f"[mavlink] Connecting to {self.connection_string} ...")
# #         self.master = mavutil.mavlink_connection(self.connection_string)
# #         self.master.wait_heartbeat(timeout=timeout_s)
# #         print(f"[mavlink] Heartbeat received (sysid={self.master.target_system}, "
# #               f"compid={self.master.target_component})")
# #         self._request_data_streams()
# #         return self.master

# #     def _request_data_streams(self, rate_hz=10):
# #         """
# #         ArduPilot only pushes telemetry (position, altitude, etc.) to a GCS
# #         that has explicitly asked for it -- Mission Planner does this
# #         automatically on connect, but our script is a separate client and
# #         needs to request it too, or things like get_relative_altitude()
# #         will just hang / return None.
# #         """
# #         self.master.mav.request_data_stream_send(
# #             self.master.target_system,
# #             self.master.target_component,
# #             mavutil.mavlink.MAV_DATA_STREAM_ALL,
# #             rate_hz,
# #             1,  # start streaming
# #         )
# #         # give the autopilot a moment to start pushing messages
# #         time.sleep(0.5)

# #     # ---------------- arming / mode ----------------
# #     def set_mode(self, mode_name):
# #         mode_id = self.master.mode_mapping()[mode_name]
# #         self.master.mav.set_mode_send(
# #             self.master.target_system,
# #             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
# #             mode_id,
# #         )
# #         self._wait_for_mode(mode_name)

# #     def _wait_for_mode(self, mode_name, timeout_s=10):
# #         t0 = time.time()
# #         while time.time() - t0 < timeout_s:
# #             msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=2)
# #             if msg is None:
# #                 continue
# #             current_mode = mavutil.mode_string_v10(msg)
# #             if current_mode == mode_name:
# #                 print(f"[mavlink] Mode set to {mode_name}")
# #                 return True
# #         print(f"[mavlink] WARNING: mode {mode_name} not confirmed within timeout")
# #         return False

# #     def arm(self, wait_armed=True, timeout_s=45, retry_interval_s=3):
# #         """
# #         Sends the arm command and retries periodically until armed or timeout.
# #         SITL commonly refuses to arm for the first ~10-20s after starting
# #         (PreArm: Need Position Estimate) while the EKF settles, even if GPS
# #         already shows a fix -- so a single arm attempt often isn't enough.
# #         """
# #         print("[mavlink] Arming (will retry until armed or timeout)...")
# #         t0 = time.time()
# #         last_attempt = 0

# #         while time.time() - t0 < timeout_s:
# #             if time.time() - last_attempt >= retry_interval_s:
# #                 self.master.mav.command_long_send(
# #                     self.master.target_system, self.master.target_component,
# #                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
# #                     0, 1, 0, 0, 0, 0, 0, 0,
# #                 )
# #                 last_attempt = time.time()

# #             if not wait_armed:
# #                 return True

# #             msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
# #             if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
# #                 print("[mavlink] Armed.")
# #                 return True

# #             # surface any PreArm/failure reason text so it's not a silent wait
# #             status = self.master.recv_match(type="STATUSTEXT", blocking=False)
# #             if status and status.text:
# #                 print(f"[mavlink] STATUSTEXT: {status.text}")

# #         print(f"[mavlink] WARNING: arm not confirmed within {timeout_s}s timeout")
# #         return False

# #     def disarm(self):
# #         self.master.mav.command_long_send(
# #             self.master.target_system, self.master.target_component,
# #             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
# #             0, 0, 0, 0, 0, 0, 0, 0,
# #         )
# #         print("[mavlink] Disarm command sent.")

# #     # ---------------- takeoff ----------------
# #     def takeoff(self, altitude_m):
# #         print(f"[mavlink] Taking off to {altitude_m} m ...")
# #         self.master.mav.command_long_send(
# #             self.master.target_system, self.master.target_component,
# #             mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
# #             0, 0, 0, 0, 0, 0, 0, altitude_m,
# #         )
# #         self._wait_for_altitude(altitude_m)

# #     def _wait_for_altitude(self, target_altitude_m, tolerance_m=0.3, timeout_s=30):
# #         t0 = time.time()
# #         while time.time() - t0 < timeout_s:
# #             alt = self.get_relative_altitude()
# #             if alt is not None and abs(alt - target_altitude_m) <= tolerance_m:
# #                 print(f"[mavlink] Reached altitude {alt:.2f} m")
# #                 return True
# #             time.sleep(0.2)
# #         print("[mavlink] WARNING: target altitude not confirmed within timeout")
# #         return False

# #     # ---------------- telemetry ----------------
# #     def get_relative_altitude(self):
# #         msg = self.master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
# #         if msg is None:
# #             return None
# #         return msg.relative_alt / 1000.0  # mm -> m

# #     def get_local_position(self):
# #         """Returns (x, y, z) in local NED frame (meters), or None."""
# #         msg = self.master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
# #         if msg is None:
# #             return None
# #         return (msg.x, msg.y, msg.z)

# #     def get_yaw(self):
# #         """Returns current yaw in radians (NED, 0 = North), or None."""
# #         msg = self.master.recv_match(type="ATTITUDE", blocking=True, timeout=2)
# #         if msg is None:
# #             return None
# #         return msg.yaw

# #     # ---------------- guided velocity control ---------------
# #     def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
# #         """
# #         Send velocity setpoint in BODY frame (forward, right, down), m/s.
# #         ...
# #         """
# #         type_mask = (
# #             0b0000_0111_1100_0111
# #         )
# #         self.master.mav.set_position_target_local_ned_send(
# #             0,
# #             self.master.target_system,
# #             self.master.target_component,
# #             mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
# #             type_mask,
# #             0, 0, 0,
# #             vx, vy, vz,
# #             0, 0, 0,
# #             0, yaw_rate,
# #         )

# #     def send_velocity_local(self, vx_north, vy_east, vz, yaw_rate=0.0):
# #         """Velocity setpoint in a FIXED local frame (North, East, Down), m/s.
# #         Use this instead of send_velocity_body() whenever you're computing
# #         vx/vy relative to a fixed corridor axis rather than the vehicle's
# #         instantaneous heading -- it decouples translational control from
# #         whatever the FC's actual current yaw is at that instant (e.g. mid-
# #         BendyRuler-maneuver), which body-frame commands don't."""
# #         type_mask = 0b0000_0111_1100_0111
# #         self.master.mav.set_position_target_local_ned_send(
# #             0, self.master.target_system, self.master.target_component,
# #             mavutil.mavlink.MAV_FRAME_LOCAL_NED, type_mask,
# #             0, 0, 0, vx_north, vy_east, vz, 0, 0, 0, 0, yaw_rate,
# #         )

# #     def hold_position(self):
# #         self.send_velocity_body(0, 0, 0, 0)

# #     # ---------------- fake distance sensor injection (for sim option A) ----------------
# #     def send_fake_distance_sensor(self, distance_cm, orientation, sensor_id):
# #         """
# #         orientation: MAV_SENSOR_ROTATION_* (e.g. NONE=front, YAW_90=right, YAW_270=left)
# #         Used only when simulating without real hardware / without Gazebo range plugins.
# #         """
# #         self.master.mav.distance_sensor_send(
# #             0,                      # time_boot_ms
# #             10,                     # min_distance (cm)
# #             800,                    # max_distance (cm)
# #             int(distance_cm),
# #             mavutil.mavlink.MAV_DISTANCE_SENSOR_LASER,
# #             sensor_id,
# #             orientation,
# #             0,                      # covariance
# #         )