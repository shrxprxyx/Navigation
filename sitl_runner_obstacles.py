"""
Scenario-based SITL corridor runner for AeroTHON SkyScan.

TWO-LAYER OBSTACLE AVOIDANCE:
  1. PRIMARY: ArduPilot's BendyRuler, running on the flight controller.
     Fed live DISTANCE_SENSOR readings every step (same as before) — it
     handles the actual lateral steering around obstacles on its own.
  2. BACKUP: ObstacleAvoider (navigation/obstacle_avoid.py) runs on the
     companion computer side and gates forward speed independently of
     whatever BendyRuler is doing. If BendyRuler is misconfigured, has a
     sensor gap, or just doesn't react fast enough, this forces a STOP
     based on our own direct reading of the front sensors — belt and
     suspenders, not fighting BendyRuler's steering.

This was NOT how the previous version worked — ObstacleAvoider existed
in the codebase but was never actually called in the live loop. That's
fixed here.

SCENARIOS — run each one separately against Mission Planner SITL to
exercise different cases without needing real hardware:
    none          — empty corridor, sanity check
    centered      — single pillar dead-center (original test case)
    left_wall     — pillar hugging the left wall
    right_wall    — pillar hugging the right wall
    narrow_squeeze— large obstacle, tight-but-passable gap on both sides
    double        — two pillars in sequence, offset to opposite sides
    blocking      — obstacle spans the full width — should STOP and
                    hold, never push through (there is no backtrack/
                    replan logic — this is a known limitation, see the
                    note at the bottom of this file)

Run:
    python sitl_runner_obstacles.py <scenario_name>
    python sitl_runner_obstacles.py centered
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "navigation"))

from comms.mavlink_bridge import MavlinkBridge
from real_sensors import RealSensors
from wall_follow import WallFollower
from yaw_align import YawAligner
from altitude import AltitudeHold
from obstacle_avoid import ObstacleAvoider
from config import FORWARD_SPEED_MS, SLOW_SPEED_MS

# ── corridor geometry (mm) ────────────────────────────────────────
CORR_WIDTH = 3500
CORR_HEIGHT = 3000
CORR_LENGTH = 10000

START_X = 0
START_Y = 1750
START_Z = 1500

TOF_MAX = 4000
LIDAR_MAX = 5000

ORIENT_FORWARD = 0
ORIENT_RIGHT = 2
ORIENT_LEFT = 6
ORIENT_FRONT_RIGHT = 10
ORIENT_FRONT_LEFT = 14
ORIENT_DOWN = 25

# ── scenario library: each is a list of {x, y, radius} obstacles ──
SCENARIOS = {
    "none": [],
    "centered": [
        {"x": 5000, "y": 1750, "radius": 200},
    ],
    "left_wall": [
        {"x": 5000, "y": 600, "radius": 200},
    ],
    "right_wall": [
        {"x": 5000, "y": 2900, "radius": 200},
    ],
    "narrow_squeeze": [
        {"x": 5000, "y": 1750, "radius": 900},  # ~850mm gap each side
    ],
    "double": [
        {"x": 3500, "y": 1200, "radius": 200},
        {"x": 7000, "y": 2200, "radius": 200},
    ],
    "blocking": [
        {"x": 5000, "y": 1750, "radius": 1800},  # spans nearly full width
    ],
}


def dist_to_nearest_obstacle(drone_x, drone_y, obstacles):
    """Distance (mm) from drone to the surface of the closest obstacle. Returns None if no obstacles."""
    if not obstacles:
        return None
    best = None
    for obs in obstacles:
        d = math.sqrt((drone_x - obs["x"]) ** 2 + (drone_y - obs["y"]) ** 2) - obs["radius"]
        if best is None or d < best:
            best = d
    return best


def compute_sensors(drone_x, drone_y, drone_z, yaw_deg, obstacles):
    yaw_rad = math.radians(yaw_deg)

    left_mm = min(drone_y, TOF_MAX)
    right_mm = min(CORR_WIDTH - drone_y, TOF_MAX)
    down_mm = min(drone_z, TOF_MAX)

    fwd_wall = CORR_LENGTH - drone_x

    # nearest obstacle that's actually ahead of the drone, within the forward sensor cone
    fwd_obs = LIDAR_MAX
    for obs in obstacles:
        dx = obs["x"] - drone_x
        dy = obs["y"] - drone_y
        if dx > 0:
            lateral_offset = abs(dy)
            if lateral_offset < obs["radius"] + 150:
                candidate = max(dx - obs["radius"], 50)
                fwd_obs = min(fwd_obs, candidate)

    fwd_mm = min(fwd_wall, fwd_obs, LIDAR_MAX)

    nearest_obs_dist = dist_to_nearest_obstacle(drone_x, drone_y, obstacles)
    obs_component = max(nearest_obs_dist, 50) if nearest_obs_dist is not None else TOF_MAX

    fl_angle = math.radians(45) + yaw_rad
    fl_dist = left_mm / max(abs(math.sin(fl_angle)), 0.01)
    fl_mm = min(fl_dist, obs_component, TOF_MAX)

    fr_angle = math.radians(45) - yaw_rad
    fr_dist = right_mm / max(abs(math.sin(fr_angle)), 0.01)
    fr_mm = min(fr_dist, obs_component, TOF_MAX)

    return {
        ORIENT_LEFT: round(left_mm),
        ORIENT_RIGHT: round(right_mm),
        ORIENT_DOWN: round(down_mm),
        ORIENT_FORWARD: round(fwd_mm),
        ORIENT_FRONT_LEFT: round(fl_mm),
        ORIENT_FRONT_RIGHT: round(fr_mm),
    }


def send_proximity(mav, readings):
    ts = int(time.time() * 1000) & 0xFFFFFFFF
    for orientation, dist_mm in readings.items():
        mav.distance_sensor_send(ts, 20, 4000, max(dist_mm // 10, 2), 0, 0, orientation, 0)


def run_scenario(scenario_name):
    if scenario_name not in SCENARIOS:
        print(f"Unknown scenario '{scenario_name}'. Options: {list(SCENARIOS.keys())}")
        return

    obstacles = SCENARIOS[scenario_name]

    conn = "tcp:127.0.0.1:5763"
    print(f"connecting to Mission Planner SITL at {conn}")
    bridge = MavlinkBridge(conn)
    sensors = RealSensors()

    wall = WallFollower()
    yaw = YawAligner()
    alt = AltitudeHold()
    avoider = ObstacleAvoider()  # BACKUP layer — was previously unused

    bridge.arm_and_takeoff(alt=5.0)
    if not bridge.vehicle.armed:
        print("failed to arm — aborting scenario, not pretending to fly")
        bridge.close()
        return

    print(f"airborne — running scenario: {scenario_name}")
    print(f"obstacles: {obstacles if obstacles else 'none'}\n")

    dt = 0.05
    step = 0
    max_steps = 600
    stop_streak = 0
    STOP_STREAK_ABORT = int(5.0 / dt)  # 5 seconds stuck at STOP = give up, don't hang forever

    drone_x, drone_y, drone_z, drone_yaw = float(START_X), float(START_Y), float(START_Z), 0.0

    print(f"{'step':>5}  {'x_mm':>6}  {'y_mm':>6}  {'fwd':>6}  "
          f"{'vx':>6}  {'vy':>8}  {'obstacle':>10}")

    while step < max_steps:
        readings = compute_sensors(drone_x, drone_y, drone_z, drone_yaw, obstacles)
        for orientation, dist_mm in readings.items():
            sensors._r[orientation] = dist_mm

        send_proximity(bridge.vehicle._master.mav, readings)  # feeds BendyRuler (primary)

        left, right = sensors.read_left(), sensors.read_right()
        fl, fr = sensors.read_front_left(), sensors.read_front_right()
        alt_r = sensors.read_down()
        fwd = readings[ORIENT_FORWARD]
        lidar_scan = [(0, fwd / 1000.0)]  # matches RealSensors.read_lidar() format

        vy = wall.compute(left, right, dt)
        yaw_rt = yaw.compute(fl, fr)
        vz = alt.compute(alt_r)

        # BACKUP layer: independent forward-speed gate, regardless of what BendyRuler decides
        obstacle_state = avoider.check(fl, fr, lidar_scan)
        if obstacle_state == "STOP":
            vx = 0.0
            stop_streak += 1
        elif obstacle_state == "SLOW":
            vx = SLOW_SPEED_MS
            stop_streak = 0
        else:
            vx = FORWARD_SPEED_MS
            stop_streak = 0

        if stop_streak > STOP_STREAK_ABORT:
            print(f"\n  !! stuck at STOP for {STOP_STREAK_ABORT * dt:.0f}s — aborting scenario "
                  f"(no backtrack/replan logic exists yet, see file header)")
            break

        bridge.send_velocity_yaw(vx, vy, vz, yaw_rt)

        drone_x += vx * 1000 * dt
        drone_y += vy * 1000 * dt
        drone_z += (-vz) * 1000 * dt
        drone_yaw += yaw_rt * dt
        drone_y = max(100, min(CORR_WIDTH - 100, drone_y))
        drone_z = max(200, min(CORR_HEIGHT - 200, drone_z))

        print(f"{step:>5}  {drone_x:>6.0f}  {drone_y:>6.0f}  {fwd:>6}  "
              f"{vx:>6.2f}  {vy:>8.4f}  {obstacle_state:>10}")

        if drone_x >= CORR_LENGTH - 200:
            print(f"\n  \u2713 corridor complete at x={drone_x:.0f}mm — scenario '{scenario_name}' passed")
            break

        step += 1
        time.sleep(dt)
    else:
        print(f"\n  !! max steps reached without finishing — scenario '{scenario_name}' did not complete")

    bridge.land()
    bridge.close()
    print("done")


if __name__ == "__main__":
    scenario = sys.argv[1] if len(sys.argv) > 1 else "centered"
    run_scenario(scenario)


# ── KNOWN LIMITATIONS ────────────────────────────────────────────
# 1. No backtrack/replan: if STOP holds for too long (see STOP_STREAK_ABORT),
#    the scenario just gives up rather than trying an alternate path. Real
#    missions would need a "reverse and try again" or "abort to hover"
#    strategy for the 'blocking' scenario case.
# 2. Drone modeled as a single point, not an actual physical footprint —
#    'narrow_squeeze' doesn't account for propeller/frame width, only the
#    sensor readings. A real drone needs real clearance margin added on
#    top of what this sim considers "passable."
# 3. BendyRuler's actual steering behavior depends on ArduPilot's own
#    OA_BR_* parameters (lookahead distance, margin, etc.) — this script
#    only feeds it sensor data, it doesn't control HOW BendyRuler reacts.
#    Tune those in Mission Planner's full parameter list if avoidance
#    looks too aggressive or too timid.