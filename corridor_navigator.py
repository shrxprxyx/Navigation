"""
corridor_navigator.py
-----------------------
The actual corridor-navigation control loop:
  - reads left/right/front distances every tick
  - computes a lateral centering correction (PID on left-right error)
  - computes a forward speed (slows/stops based on front obstacle distance)
  - sends body-frame velocity setpoints to the flight controller
  - exits when it has traveled the corridor length, or aborts on timeout /
    unsafe centering error

This module is intentionally sensor-source-agnostic: pass it anything with
a .read(along_corridor_m, lateral_offset_m) -> {"left_m","right_m","front_m"}
method (sensor_sim.SimulatedCorridorSensors for now, real_sensors.RealCorridorSensors
later).
"""

import time
from pymavlink import mavutil
import config
from pid_controller import PID


class CorridorResult:
    SUCCESS = "success"
    TIMEOUT = "timeout"
    UNSAFE_ABORT = "unsafe_abort"


class CorridorNavigator:
    def __init__(self, vehicle, sensors, logger=None):
        self.vehicle = vehicle
        self.sensors = sensors
        self.logger = logger

        self.lateral_pid = PID(
            kp=config.LATERAL_KP,
            ki=config.LATERAL_KI,
            kd=config.LATERAL_KD,
            output_limit=config.LATERAL_MAX_MPS,
        )

        # Internal estimate of how far we've traveled into the corridor.
        # In sim this is tracked from commanded velocity * dt (dead reckoning).
        # On real hardware, replace with vision/optical-flow or local position deltas.
        self._along_corridor_m = 0.0
        self._lateral_offset_m = 0.0
        self._avoid_direction = None  # locked +1 (right) / -1 (left) while sidestepping an obstacle
        self._clear_streak = 0        # consecutive ticks with front distance clear of obstacle

    def run(self, real_time=True):
        """
        Blocking call: navigates the full corridor length. Returns CorridorResult.

        real_time=True  -> sleeps to match CONTROL_LOOP_HZ (use for SITL/real hardware,
                            where the vehicle's actual motion is happening in wall-clock time).
        real_time=False -> runs as fast as possible, using a virtual clock (use for the
                            offline/no-MAVLink logic test, where nothing physically moves).
        """
        self.lateral_pid.reset()
        period = 1.0 / config.CONTROL_LOOP_HZ
        start_time = time.time()
        virtual_time = 0.0

        print("[corridor] Starting corridor navigation...")

        # Track progress from REAL telemetry, not dead-reckoning from our own
        # commanded velocity. This matters specifically because BendyRuler is
        # now primary: if the FC steers the real vehicle around an obstacle,
        # that has to be reflected here, or this loop has no way of ever
        # knowing the obstacle was cleared -- it'll sit at vx=0 forever and
        # hit the timeout below every single time.
        start_pos = self.vehicle.get_local_position()
        for _ in range(10):
            if start_pos is not None:
                break
            time.sleep(0.2)
            start_pos = self.vehicle.get_local_position()
        if start_pos is None:
            print("[corridor] WARNING: couldn't read starting position, falling back to dead-reckoning.")
        start_x, start_y = (start_pos[0], start_pos[1]) if start_pos else (0.0, 0.0)

        while True:
            loop_start = time.time()
            elapsed_time = (time.time() - start_time) if real_time else virtual_time

            if elapsed_time > config.CORRIDOR_TIMEOUT_S:
                print("[corridor] TIMEOUT -- aborting, holding position.")
                self.vehicle.hold_position()
                return CorridorResult.TIMEOUT

            # Pull real position every tick. NED local frame: x = forward
            # (assumes corridor runs along the takeoff heading -- fine for a
            # straight corridor aligned with SITL's default heading; revisit
            # with a yaw-rotation if the corridor isn't oriented that way).
            # Falls back to the last dead-reckoned estimate on a dropped read
            # so a single missed telemetry packet doesn't stall the loop.
            pos = self.vehicle.get_local_position()
            if pos is not None:
                self._along_corridor_m = pos[0] - start_x
                self._lateral_offset_m = pos[1] - start_y

            reading = self.sensors.read(self._along_corridor_m, self._lateral_offset_m)
            left, right, front = reading["left_m"], reading["right_m"], reading["front_m"]
            obstacle_ahead = reading.get("obstacle_ahead", front < config.OBSTACLE_SLOW_DISTANCE_M)

            # PRIMARY avoidance input: feed the flight controller's own BendyRuler
            # the same readings our Python logic sees, every tick. This is what
            # was missing before -- the FC had nothing to react to.
            if config.FEED_DISTANCE_SENSOR_TO_FC:
                self.vehicle.send_fake_distance_sensor(
                    front * 100, mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, sensor_id=0)
                self.vehicle.send_fake_distance_sensor(
                    left * 100, mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_270, sensor_id=1)
                self.vehicle.send_fake_distance_sensor(
                    right * 100, mavutil.mavlink.MAV_SENSOR_ROTATION_YAW_90, sensor_id=2)

            centering_error = left - right  # +ve => closer to right wall => drift left needed... see sign note below
            # Sign convention: vy > 0 means move RIGHT (body frame).
            # If left > right, drone is closer to the right wall, so it should
            # move LEFT (vy negative) to recenter. error = left - right is
            # positive in that case, so we want vy = -Kp*error ... but PID
            # already returns kp*error, so we negate when sending.
            #
            # Safety check is based on actual wall clearance, not the left/right
            # DIFFERENCE -- a large difference is expected and fine while
            # deliberately sidestepping an obstacle, as long as we're not
            # actually close to hitting a wall.
            min_clearance = min(left, right)
            if min_clearance < config.MIN_WALL_CLEARANCE_M:
                print(f"[corridor] UNSAFE wall clearance {min_clearance:.2f} m -- aborting.")
                self.vehicle.hold_position()
                return CorridorResult.UNSAFE_ABORT

            lateral_correction = self.lateral_pid.update(centering_error)
            vy = -lateral_correction  # negate per sign convention above

            vx = self._compute_forward_speed(front, obstacle_ahead)

            # --- reactive obstacle sidestep (BACKUP steering, off by default) ---
            # If a REAL obstacle (not just the corridor exit) is forcing us to
            # slow/stop, nudge laterally toward whichever side currently has
            # more clearance, so we can go around obstacles that aren't
            # dead-center in the corridor.
            #
            # Gated behind CUSTOM_SIDESTEP_STEERING_ENABLED: with BendyRuler
            # now primary (see FEED_DISTANCE_SENSOR_TO_FC above), letting this
            # ALSO actively steer means two independent systems could each
            # choose a different dodge direction and fight each other. Flip
            # this back on only if you disable FEED_DISTANCE_SENSOR_TO_FC and
            # want pure Python-side avoidance again (e.g. testing without
            # relying on ArduPilot's OA_TYPE params at all).
            if config.CUSTOM_SIDESTEP_STEERING_ENABLED and obstacle_ahead and front < config.OBSTACLE_SLOW_DISTANCE_M:
                self._clear_streak = 0
                # Lock the avoidance direction the FIRST time we detect an obstacle,
                # based on which side currently has more clearance. Hold that choice
                # until we've cleared the obstacle, so sensor noise can't cause the
                # side-to-side flip-flopping that stalls forward progress.
                if self._avoid_direction is None:
                    self._avoid_direction = 1 if (right - left) > 0 else -1
                avoid_vy = config.LATERAL_MAX_MPS * self._avoid_direction

                # blend: closer the obstacle, the more we prioritize avoidance over centering
                closeness = max(0.0, min(1.0,
                    (config.OBSTACLE_SLOW_DISTANCE_M - front) / config.OBSTACLE_SLOW_DISTANCE_M))
                vy = (1 - closeness) * vy + closeness * avoid_vy
                # never allow the nudge to push us within MIN_WALL_CLEARANCE_M
                # (+ a small buffer, since this is a velocity command that will
                # keep moving us for one more control period before we re-check)
                buffer = config.MIN_WALL_CLEARANCE_M + 0.15
                if right - buffer < 0 and vy > 0:
                    vy = 0.0
                if left - buffer < 0 and vy < 0:
                    vy = 0.0
                # keep a small forward creep instead of a hard 0 while sidestepping,
                # as long as we're not on top of the obstacle
                if front > config.OBSTACLE_STOP_DISTANCE_M:
                    vx = max(vx, config.FORWARD_SPEED_MIN_MPS)
            else:
                # Clear of any obstacle -- but only release the direction lock
                # after several CONSECUTIVE clear readings, not just one. A
                # single clear tick can happen from noise (or from briefly
                # exiting the detection band mid-maneuver) and releasing too
                # early causes rapid re-lock/re-trigger cycling (the "sawtooth"
                # pattern) instead of a clean, committed pass around the obstacle.
                self._clear_streak += 1
                if self._clear_streak >= config.CLEAR_STREAK_TO_RELEASE:
                    self._avoid_direction = None
                    self._clear_streak = 0

            self.vehicle.send_velocity_body(vx=vx, vy=vy, vz=0.0, yaw_rate=0.0)

            # Fallback only: if this tick's telemetry read failed (pos is None,
            # checked above), keep the loop moving using the old dead-reckoning
            # method rather than freezing at a stale position. When telemetry
            # is healthy, next tick's real position read overwrites this anyway.
            if pos is None:
                self._along_corridor_m += vx * period
                self._lateral_offset_m += vy * period

            if self.logger:
                self.logger.log(
                    t=elapsed_time,
                    along=self._along_corridor_m,
                    lateral=self._lateral_offset_m,
                    left=left, right=right, front=front,
                    vx=vx, vy=vy,
                )

            if self._along_corridor_m >= config.CORRIDOR_LENGTH_M:
                print("[corridor] Reached corridor end.")
                self.vehicle.hold_position()
                return CorridorResult.SUCCESS

            # maintain loop rate (real hardware/SITL) or advance virtual clock (offline test)
            if real_time:
                elapsed = time.time() - loop_start
                sleep_time = max(0.0, period - elapsed)
                time.sleep(sleep_time)
            else:
                virtual_time += period

    def _compute_forward_speed(self, front_distance_m, obstacle_ahead=False):
        # Only a REAL obstacle should bring us to a full stop. The corridor
        # exit shrinking below the stop threshold is not a reason to halt --
        # that would mean the drone freezes just short of the finish line.
        if obstacle_ahead and front_distance_m <= config.OBSTACLE_STOP_DISTANCE_M:
            return 0.0
        if front_distance_m >= config.OBSTACLE_SLOW_DISTANCE_M:
            return config.FORWARD_SPEED_MAX_MPS
        # linear ramp between stop distance and slow distance
        span = config.OBSTACLE_SLOW_DISTANCE_M - config.OBSTACLE_STOP_DISTANCE_M
        frac = (front_distance_m - config.OBSTACLE_STOP_DISTANCE_M) / span
        speed = config.FORWARD_SPEED_MIN_MPS + frac * (
            config.FORWARD_SPEED_MAX_MPS - config.FORWARD_SPEED_MIN_MPS
        )
        return max(config.FORWARD_SPEED_MIN_MPS, min(config.FORWARD_SPEED_MAX_MPS, speed))