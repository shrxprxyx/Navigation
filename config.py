"""
config.py
---------
All tunable parameters for corridor navigation live here.
Edit values in this file instead of hunting through the other scripts.
"""

# ---------------- MAVLink connection ----------------
# Mission Planner's SITL instance already listens/broadcasts on TCP 5760
# (that's the port Mission Planner itself connects to -- see the top-right
# of its window). ArduPilot SITL automatically opens TWO EXTRA TCP ports
# for additional clients: 5762 and 5763. We connect our companion-computer
# script to one of those, so it can run alongside Mission Planner without
# stealing its connection or needing any extra --out param.
# If tcp:5762 is already taken (e.g. by another script), try tcp:5763.
MAVLINK_CONNECTION_STRING = "tcp:127.0.0.1:5762"

# ---------------- Corridor geometry ----------------
# From your mission doc: corridor is 3.5 m wide, 10 m long, flown at ~3 m
# altitude (10 ft) after descending from the 5 m QR-scan altitude.
CORRIDOR_WIDTH_M = 3.5
CORRIDOR_LENGTH_M = 10.0
CORRIDOR_ALTITUDE_M = 3.0
SCAN_ALTITUDE_M = 5.0
DELIVERY_ZONE_ALTITUDE_M = 10.0

# ---------------- Sensor mounting ----------------
# Distance from drone center to each side sensor's mounting point (m).
# Used to convert raw sensor range -> distance from drone CENTER to wall.
SENSOR_OFFSET_LEFT_M = 0.15
SENSOR_OFFSET_RIGHT_M = 0.15

# Front lidar (TF-Luna) max reliable range (m) and obstacle trigger distance
FRONT_LIDAR_MAX_RANGE_M = 8.0
OBSTACLE_STOP_DISTANCE_M = 1.0      # stop/hold if obstacle closer than this
OBSTACLE_SLOW_DISTANCE_M = 2.5      # start slowing down within this range
CLEAR_STREAK_TO_RELEASE = 8         # consecutive clear ticks required before releasing avoidance lock (debounce)

# ---------------- Avoidance architecture ----------------
# PRIMARY: feed live DISTANCE_SENSOR data to the flight controller every
# tick, so ArduPilot's BendyRuler (if enabled via OA_TYPE in Mission
# Planner's param list) does the actual lateral steering around obstacles.
FEED_DISTANCE_SENSOR_TO_FC = True

# BACKUP: the companion-computer-side sidestep steering in
# corridor_navigator.py (the avoid_direction lock/blend logic). Left OFF
# by default now that BendyRuler is primary -- running both at once means
# two independent systems can each pick a different dodge direction and
# fight each other. Speed ramp-down and the min-wall-clearance hard abort
# stay ACTIVE regardless of this flag -- those don't steer, so they can't
# conflict with BendyRuler, and they're exactly the kind of safety net
# you want even when the FC is doing the steering.
CUSTOM_SIDESTEP_STEERING_ENABLED = False

# ---------------- Control loop ----------------
CONTROL_LOOP_HZ = 10.0              # how often we send velocity setpoints
FORWARD_SPEED_MAX_MPS = 0.5         # cruise speed inside corridor (slow!)
FORWARD_SPEED_MIN_MPS = 0.1         # crawl speed when near obstacle
LATERAL_KP = 0.6                    # proportional gain, centering error -> vy
LATERAL_KI = 0.02
LATERAL_KD = 0.15
LATERAL_MAX_MPS = 0.3               # clamp lateral correction speed
YAW_LOCKED = True                   # keep yaw fixed, only move fwd/lateral

# ---------------- Safety ----------------
MIN_WALL_CLEARANCE_M = 0.25         # abort if drone gets closer than this to either wall
CORRIDOR_TIMEOUT_S = 90             # abort corridor nav if it takes longer than this

# ---------------- Simulated sensors (for Option A: logic-only sim) ----------------
# Only used by sensor_sim.py when running WITHOUT Gazebo, to fake a
# corridor + static obstacles so you can test your control logic.
SIM_OBSTACLES = [
    # (distance_along_corridor_m, lateral_offset_from_center_m, radius_m)
    (4.0, 0.5, 0.3),
    (7.0, -0.6, 0.3),
]
SIM_WALL_NOISE_STD_M = 0.03   # sensor noise to make sim realistic