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
import config
from mavlink_connection import Vehicle
from sensor_sim import SimulatedCorridorSensors
from corridor_navigator import CorridorNavigator, CorridorResult
from flight_logger import FlightLogger


def main():
    vehicle = Vehicle()
    vehicle.connect()

    vehicle.set_mode("GUIDED")

    armed = vehicle.arm()
    if not armed:
        print("[main] Failed to arm -- check PreArm messages above (often just "
              "needs more time after SITL startup for EKF/GPS to settle). Aborting.")
        sys.exit(1)

    vehicle.takeoff(config.SCAN_ALTITUDE_M)

    # --- placeholder: QR scan + banner alignment would happen here ---
    print("[main] (QR scan / banner alignment step goes here)")

    # Descend to corridor altitude before entering
    print(f"[main] Descending to corridor altitude {config.CORRIDOR_ALTITUDE_M} m")
    vehicle.takeoff(config.CORRIDOR_ALTITUDE_M)  # takeoff() also works for descending targets

    sensors = SimulatedCorridorSensors()
    logger = FlightLogger("corridor_run_log.csv")
    navigator = CorridorNavigator(vehicle, sensors, logger=logger)

    result = navigator.run()
    logger.close()

    if result != CorridorResult.SUCCESS:
        print(f"[main] Corridor navigation ended with: {result}. Holding / landing.")
        vehicle.set_mode("LAND")
        sys.exit(1)

    print("[main] Corridor cleared successfully. Continuing mission...")
    # --- placeholder: ascend to delivery altitude, target ID, payload drop ---

    vehicle.set_mode("LAND")


if __name__ == "__main__":
    main()