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
import config
from mavlink_connection import Vehicle
from sensor_sim import SimulatedCorridorSensors
from corridor_navigator import CorridorNavigator, CorridorResult
from flight_logger import FlightLogger


def main():
    vehicle = Vehicle()
    vehicle.connect()

    vehicle.set_mode("GUIDED")
    vehicle.arm()
    vehicle.takeoff(config.SCAN_ALTITUDE_M)

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