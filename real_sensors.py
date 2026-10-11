"""real_sensors.py

Drop-in replacement for SimulatedCorridorSensors / LoggedFrontLRSensor on real hardware.
Same interface: read(along_corridor_m, lateral_offset_m) -> {left_m, right_m, front_m, obstacle_ahead}

  left / right : 2x VL53L1X on the Pi (I2C, separate addresses via XSHUT)
  front        : lidar wired to the FC, read from its DISTANCE_SENSOR MAVLink messages
All values are CENTER-referenced (sensor reading + mount offset), same as sensor_sim.
"""
import config
from tof_interface import RangeSensor, SensorConfig, make_vl53l1x_pair

DISTANCE_SENSOR_MSG_ID = 132


class RealCorridorSensors:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        left_raw, right_raw = make_vl53l1x_pair(
            config.TOF_LEFT_XSHUT_GPIO, config.TOF_RIGHT_XSHUT_GPIO,
            config.TOF_LEFT_ADDR, config.TOF_RIGHT_ADDR, config.TOF_I2C_BUS)
        self.left = RangeSensor(left_raw, SensorConfig(
            offset_mm=config.LEFT_SENSOR_BIAS_MM, min_mm=30, max_mm=4000))
        self.right = RangeSensor(right_raw, SensorConfig(
            offset_mm=config.RIGHT_SENSOR_BIAS_MM, min_mm=30, max_mm=4000))
        # Front: max-range ("nothing ahead") must pass through as a valid clear reading,
        # and big jumps are legitimate (obstacle appears / corridor exit), so loosen both.
        self.front = RangeSensor(self._front_raw, SensorConfig(
            offset_mm=config.FRONT_SENSOR_BIAS_MM, min_mm=100,
            max_mm=1_000_000, max_jump_mm=1_000_000))
        vehicle.request_message_interval(DISTANCE_SENSOR_MSG_ID, 10)

    def _front_raw(self):
        return self.vehicle.get_distance_sensor_mm(config.FRONT_LIDAR_ORIENTATION)

    def read(self, along_corridor_m, lateral_offset_m):
        l, r, f = self.left.read(), self.right.read(), self.front.read()

        # Side sensor lost (after the 0.5 s hold) -> report 0 so the navigator's
        # MIN_WALL_CLEARANCE check aborts and holds position (fail-safe).
        left_m = (l.range_mm / 1000.0 + config.SENSOR_OFFSET_LEFT_M) if l.valid else 0.0
        right_m = (r.range_mm / 1000.0 + config.SENSOR_OFFSET_RIGHT_M) if r.valid else 0.0

        if f.valid:
            front_m = min(f.range_mm / 1000.0, config.FRONT_LIDAR_MAX_RANGE_M)
        else:
            # Lidar silent: don't assume clear. Report just above the stop distance so the
            # speed ramp drops to crawl speed instead of cruising blind (or freezing forever).
            front_m = config.OBSTACLE_STOP_DISTANCE_M + 0.01

        # Only treat it as an obstacle if it's meaningfully closer than the corridor exit
        # (the sim knows this from geometry; here we use dead-reckoned distance to the exit).
        exit_dist = config.CORRIDOR_LENGTH_M - along_corridor_m
        obstacle_ahead = (f.valid and front_m < config.FRONT_LIDAR_MAX_RANGE_M
                          and front_m < exit_dist - 0.5)

        return {
            "left_m": round(left_m, 3),
            "right_m": round(right_m, 3),
            "front_m": round(front_m, 3),
            "obstacle_ahead": obstacle_ahead,
        }