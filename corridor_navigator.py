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

        while True:
            loop_start = time.time()
            elapsed_time = (time.time() - start_time) if real_time else virtual_time

            if elapsed_time > config.CORRIDOR_TIMEOUT_S:
                print("[corridor] TIMEOUT -- aborting, holding position.")
                self.vehicle.hold_position()
                return CorridorResult.TIMEOUT

            reading = self.sensors.read(self._along_corridor_m, self._lateral_offset_m)
            left, right, front = reading["left_m"], reading["right_m"], reading["front_m"]

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

            vx = self._compute_forward_speed(front)

            # --- reactive obstacle sidestep ---
            # If something ahead is forcing us to slow/stop, nudge laterally
            # toward whichever side currently has more clearance, so we can
            # go around obstacles that aren't dead-center in the corridor.
            # This overrides pure wall-centering only while an obstacle is close.
            if front < config.OBSTACLE_SLOW_DISTANCE_M:
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
                # clear of any obstacle -- release the direction lock so the next
                # obstacle can be evaluated fresh
                self._avoid_direction = None

            self.vehicle.send_velocity_body(vx=vx, vy=vy, vz=0.0, yaw_rate=0.0)

            # Dead-reckon our progress (replace with real odometry on hardware)
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

    def _compute_forward_speed(self, front_distance_m):
        if front_distance_m <= config.OBSTACLE_STOP_DISTANCE_M:
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