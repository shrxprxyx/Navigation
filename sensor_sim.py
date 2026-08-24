"""
sensor_sim.py
-------------
Fakes what your TF-Luna (front) and VL53L1X x2 (left/right) sensors WOULD
report, given the drone's true position inside a virtual corridor.

This lets you test corridor_navigator.py's control logic against SITL's
actual flight dynamics, without needing Gazebo or real hardware yet.

Swap this module out for real_sensors.py (reading actual serial/I2C data)
once you move to hardware -- corridor_navigator.py doesn't need to change,
it just calls get_left(), get_right(), get_front().
"""

import random
import config


class SimulatedCorridorSensors:
    def __init__(self, corridor_width_m=None, obstacles=None):
        self.width = corridor_width_m or config.CORRIDOR_WIDTH_M
        self.obstacles = obstacles or config.SIM_OBSTACLES
        self.noise_std = config.SIM_WALL_NOISE_STD_M

    def _noisy(self, value):
        return max(0.0, value + random.gauss(0, self.noise_std))

    def read(self, along_corridor_m, lateral_offset_m):
        """
        along_corridor_m: distance traveled into the corridor (0 = entrance)
        lateral_offset_m: drone's lateral position relative to corridor
                           centerline (+ = right of center, - = left)

        Returns dict: {left_m, right_m, front_m}
        """
        half_width = self.width / 2.0

        # Distance from drone center to each wall
        left_dist = self._noisy(half_width + lateral_offset_m)
        right_dist = self._noisy(half_width - lateral_offset_m)

        # Front distance: nearest obstacle ahead, or corridor exit distance
        front_dist = config.CORRIDOR_LENGTH_M - along_corridor_m
        for obs_along, obs_lateral, obs_radius in self.obstacles:
            if obs_along <= along_corridor_m:
                continue  # already passed it
            # crude check: if obstacle roughly in our lateral path, consider it
            lateral_gap = abs(obs_lateral - lateral_offset_m)
            if lateral_gap < obs_radius + 0.25:  # +0.25 ~ drone half-width margin
                dist_to_obs = obs_along - along_corridor_m
                front_dist = min(front_dist, dist_to_obs)

        front_dist = self._noisy(min(front_dist, config.FRONT_LIDAR_MAX_RANGE_M))

        return {
            "left_m": round(left_dist, 3),
            "right_m": round(right_dist, 3),
            "front_m": round(front_dist, 3),
        }