"""
config.py
---------
All tunable parameters for corridor navigation live here.
Edit values in this file instead of hunting through the other scripts.
"""

# ---------------- MAVLink connection ----------------
# Mission Planner's SITL instance already listens/broadcasts on 14550.
# We connect a SECOND client (our "companion computer" script) on 14551,
# which SITL also outputs to by default when using --out in sim_vehicle.py,
# or you can add an extra output in Mission Planner:
#   Simulation tab -> "Extra Params" -> add "--out=udp:127.0.0.1:14551"
# If you're only using Mission Planner's built-in simulator (no manual
# sim_vehicle.py), connect directly on 14550 instead -- just make sure
# only ONE app is the "primary" GCS to avoid mode-change conflicts.
MAVLINK_CONNECTION_STRING = "udp:127.0.0.1:14551"

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