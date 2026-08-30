"""
main.py
-------
Entry point. Run this with Mission Planner's SITL already running.

    python main.py

Sequence:
  1. Connect to SITL over MAVLink
  2. Set GUIDED mode, arm, take off to corridor altitude
  3. Run the corridor navigator until it clears the corridor (or aborts)
  4. Land

This currently uses the SIMULATED sensors (sensor_sim.py) so you can test
your control logic's behavior against real ArduCopter flight dynamics in
SITL before wiring up real TF-Luna / VL53L1X hardware.
"""

import sys
import time
import csv
import threading
from pymavlink import mavutil
import config
from mavlink_connection import Vehicle
from sensor_sim import SimulatedCorridorSensors
from corridor_navigator import CorridorNavigator, CorridorResult
from flight_logger import FlightLogger


def _send_clear_distance_burst(vehicle, times=5, interval=0.2):
    """One-off burst, used just before arming to clear the initial PreArm check.
    Front sensor only -- see corridor_navigator.py for why left/right are
    deliberately excluded from the BendyRuler feed."""
    clear_cm = 800
    for _ in range(times):
        vehicle.send_fake_distance_sensor(clear_cm, mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, sensor_id=0)
        time.sleep(interval)


def _keep_proximity_alive(vehicle, stop_event, interval=0.5):
    """
    Runs in a background thread from right after arming until the corridor
    phase starts sending real readings. Without this, the one-off burst
    above goes stale during takeoff/climb (which can easily take several
    seconds) and ArduPilot flags it as "Bad Proximity" mid-flight -- not
    a prearm block this time, but still means BendyRuler has nothing
    trustworthy to react to during that window. Front sensor only, same
    reasoning as corridor_navigator.py.
    """
    clear_cm = 800
    while not stop_event.is_set():
        vehicle.send_fake_distance_sensor(clear_cm, mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, sensor_id=0)
        stop_event.wait(interval)


def main():
    vehicle = Vehicle()
    vehicle.connect()

    vehicle.set_mode("GUIDED")

    if config.FEED_DISTANCE_SENSOR_TO_FC:
        print("[main] Sending placeholder proximity data to clear PRX1 PreArm check...")
        _send_clear_distance_burst(vehicle)

    armed = vehicle.arm()
    if not armed:
        print("[main] Failed to arm -- check PreArm messages above (often just "
              "needs more time after SITL startup for EKF/GPS to settle). Aborting.")
        sys.exit(1)

    vehicle.takeoff(config.SCAN_ALTITUDE_M)

    # Keep proximity data "fresh" in the FC's eyes throughout takeoff, QR
    # scan placeholder, and descent -- all of which happen before
    # corridor_navigator.py starts sending real readings. Stopped right
    # before navigator.run() so real data takes over cleanly, no two
    # sources writing at once.
    keepalive_stop = threading.Event()
    keepalive_thread = None
    if config.FEED_DISTANCE_SENSOR_TO_FC:
        keepalive_thread = threading.Thread(
            target=_keep_proximity_alive, args=(vehicle, keepalive_stop), daemon=True)
        keepalive_thread.start()

    # --- placeholder: QR scan + banner alignment would happen here ---
    print("[main] (QR scan / banner alignment step goes here)")

    # Descend to corridor altitude before entering
    print(f"[main] Descending to corridor altitude {config.CORRIDOR_ALTITUDE_M} m")
    
    # Descend using velocity command
    descent_rate = 0.5  # m/s downward
    altitude_to_lose = config.SCAN_ALTITUDE_M - config.CORRIDOR_ALTITUDE_M  # 5 - 3 = 2m
    descent_time = altitude_to_lose / descent_rate  # ~4 seconds

    for _ in range(int(descent_time * 10)):  # 10 Hz control loop
        current_alt = vehicle.get_relative_altitude()
        if current_alt is not None and current_alt <= config.CORRIDOR_ALTITUDE_M + 0.3:
            break
        vehicle.send_velocity_body(vx=0, vy=0, vz=0.5, yaw_rate=0.0)
        time.sleep(0.1)

    vehicle.hold_position()
    time.sleep(1)

    if keepalive_thread is not None:
        keepalive_stop.set()
        keepalive_thread.join(timeout=1)

    sensors = SimulatedCorridorSensors()
    logger = FlightLogger("corridor_run_log.csv")
    navigator = CorridorNavigator(vehicle, sensors, logger=logger)

    result = navigator.run()
    logger.close()

    if result != CorridorResult.SUCCESS:
        print(f"[main] Corridor navigation ended with: {result}. Holding / landing.")
        vehicle.disarm()
        sys.exit(1)

    print("[main] Corridor cleared successfully. Continuing mission...")
    
    # === Plot the flight path ===
    import matplotlib.pyplot as plt
    
    # Read the log file
    along_corridor = []
    lateral_offset = []
    
    try:
        with open("corridor_run_log.csv", "r") as f:
            reader = csv.DictReader(f)
            for row in reader:
                along_corridor.append(float(row["along_m"]))
                lateral_offset.append(float(row["lateral_m"]))
    except Exception as e:
        print(f"[main] Error reading log file: {e}")
    
    # Create the plot
    plt.figure(figsize=(12, 6))
    plt.plot(along_corridor, lateral_offset, 'b-', linewidth=2, label="Drone path")
    plt.axhline(y=-config.CORRIDOR_WIDTH_M/2, color='r', linestyle='--', label="Left wall")
    plt.axhline(y=config.CORRIDOR_WIDTH_M/2, color='r', linestyle='--', label="Right wall")
    plt.axhline(y=0, color='gray', linestyle=':', alpha=0.5, label="Center")
    
    plt.xlabel("Distance along corridor (m)", fontsize=12)
    plt.ylabel("Lateral offset from center (m)", fontsize=12)
    plt.title("Corridor Navigation Flight Path", fontsize=14)
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    
    # Save as PNG
    output_file = "corridor_flight_path.png"
    plt.savefig(output_file, dpi=150, bbox_inches='tight')
    print(f"[main] Flight path saved to {output_file}")
    plt.close()
    
    # --- placeholder: ascend to delivery altitude, target ID, payload drop ---
    
    print("[main] Landing...")
    vehicle.disarm()


if __name__ == "__main__":
    main()
# """
# main.py
# -------
# Entry point. Run this with Mission Planner's SITL already running.

#     python main.py

# Sequence:
#   1. Connect to SITL over MAVLink
#   2. Set GUIDED mode, arm, take off to corridor altitude
#   3. Run the corridor navigator until it clears the corridor (or aborts)
#   4. Land

# This currently uses the SIMULATED sensors (sensor_sim.py) so you can test
# your control logic's behavior against real ArduCopter flight dynamics in
# SITL before wiring up real TF-Luna / VL53L1X hardware.
# """

# import sys
# import config
# from mavlink_connection import Vehicle
# from sensor_sim import SimulatedCorridorSensors
# from corridor_navigator import CorridorNavigator, CorridorResult
# from flight_logger import FlightLogger


# def main():
#     vehicle = Vehicle()
#     vehicle.connect()

#     vehicle.set_mode("GUIDED")
#     vehicle.arm()
#     vehicle.takeoff(config.SCAN_ALTITUDE_M)

#     # --- placeholder: QR scan + banner alignment would happen here ---
#     print("[main] (QR scan / banner alignment step goes here)")

#     # Descend to corridor altitude before entering
#     print(f"[main] Descending to corridor altitude {config.CORRIDOR_ALTITUDE_M} m")
#     vehicle.takeoff(config.CORRIDOR_ALTITUDE_M)  # takeoff() also works for descending targets

#     sensors = SimulatedCorridorSensors()
#     logger = FlightLogger("corridor_run_log.csv")
#     navigator = CorridorNavigator(vehicle, sensors, logger=logger)

#     result = navigator.run()
#     logger.close()

#     if result != CorridorResult.SUCCESS:
#         print(f"[main] Corridor navigation ended with: {result}. Holding / landing.")
#         vehicle.set_mode("LAND")
#         sys.exit(1)

#     print("[main] Corridor cleared successfully. Continuing mission...")
#     # --- placeholder: ascend to delivery altitude, target ID, payload drop ---
#     vehicle.disarm()
#     # vehicle.set_mode("LAND")


# if __name__ == "__main__":
#     main()