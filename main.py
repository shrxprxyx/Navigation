"""
main.py
-------
Entry point.

    python main.py

Sequence:
  1. Connect to the FC over MAVLink (serial on the real drone, tcp for SITL)
  2. Build the sensors and (real drone) check that all three read sensibly BEFORE arming
  3. Wait for ENTER (real drone), set GUIDED, arm, take off, descend to corridor altitude
  4. Run the corridor navigator until it clears the corridor (or aborts)
  5. Land -- unless the pilot took over (FC left GUIDED) or you pressed Ctrl+C, in which case the
     script just stops commanding and leaves the aircraft to the pilot.

FIRST RUN ON HARDWARE: props OFF.
"""

import sys
import time
import csv
import threading
import traceback
from pymavlink import mavutil
import config
from mavlink_connection import Vehicle
from corridor_navigator import CorridorNavigator, CorridorResult
from flight_logger import FlightLogger


def _send_clear_distance_burst(vehicle, times=5, interval=0.2):
    """SITL only: one-off burst just before arming to clear the initial PreArm proximity check.
    Front sensor only -- see corridor_navigator.py for why left/right are excluded."""
    clear_cm = 800
    for _ in range(times):
        vehicle.send_fake_distance_sensor(clear_cm, mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, sensor_id=0)
        time.sleep(interval)


def _keep_proximity_alive(vehicle, stop_event, interval=0.5):
    """SITL only: keeps the fake proximity data fresh from arming until the corridor phase starts
    sending real readings, so ArduPilot doesn't flag 'Bad Proximity' during takeoff/descent."""
    clear_cm = 800
    while not stop_event.is_set():
        vehicle.send_fake_distance_sensor(clear_cm, mavutil.mavlink.MAV_SENSOR_ROTATION_NONE, sensor_id=0)
        stop_event.wait(interval)


def build_sensors(vehicle):
    """Pick the sensor source from config.SENSOR_SOURCE ("sim" | "logged" | "replay" | "obslog" | "real")."""
    src = config.SENSOR_SOURCE
    if src == "real":
        from real_sensors import RealCorridorSensors
        return RealCorridorSensors(vehicle)
    if src == "obslog":
        from side_obstacle_log import SideObstacleLogSensors
        return SideObstacleLogSensors()
    if src == "replay":
        from replay_sensors import ReplaySensors
        return ReplaySensors()
    if src == "sim":
        from sensor_sim import SimulatedCorridorSensors
        return SimulatedCorridorSensors()
    from logged_sensor import LoggedFrontLRSensor
    return LoggedFrontLRSensor()

def preflight_sensor_check(sensors, good_reads_needed=15, timeout_s=15.0):
    """Real drone: require N consecutive readings where at least one side sensor AND front lidar are all valid.
    Allows arming with single side sensor failure."""
    print("[main] Pre-flight sensor check ...")
    t0 = time.time()
    good = 0
    last = None
    while time.time() - t0 < timeout_s:
        r = sensors.read(0.0, 0.0)
        last = r
        # Require front sensor valid AND at least one side sensor working (> 0.0m)
        side_ok = r["left_m"] > 0.0 or r["right_m"] > 0.0
        ok = side_ok and r.get("front_valid", True)
        good = good + 1 if ok else 0
        if good >= good_reads_needed:
            print(f"[main] Sensors OK: left={r['left_m']:.2f} m  right={r['right_m']:.2f} m  "
                  f"front={r['front_m']:.2f} m")
            return True
        time.sleep(0.1)
    print(f"[main] SENSOR CHECK FAILED. Last reading: {last}")
    print("       With relaxed check: need front_valid=True AND (left_m>0.0 OR right_m>0.0)")
    print("       left/right = 0.0 means VL53L1X gave nothing; front_valid=False means no DISTANCE_SENSOR from FC")
    return False


def _plot_flight_path(csv_path, out_png="corridor_flight_path.png"):
    """Best-effort plot. A plotting problem (e.g. matplotlib missing) must never stop the landing."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        along, lateral = [], []
        with open(csv_path, "r") as f:
            for row in csv.DictReader(f):
                along.append(float(row["along_m"]))
                lateral.append(float(row["lateral_m"]))
        plt.figure(figsize=(12, 6))
        plt.plot(along, lateral, "b-", linewidth=2, label="Drone path")
        plt.axhline(y=-config.CORRIDOR_WIDTH_M / 2, color="r", linestyle="--", label="Left wall")
        plt.axhline(y=config.CORRIDOR_WIDTH_M / 2, color="r", linestyle="--", label="Right wall")
        plt.axhline(y=0, color="gray", linestyle=":", alpha=0.5, label="Center")
        plt.xlabel("Distance along corridor (m)")
        plt.ylabel("Lateral offset from center (m)")
        plt.title("Corridor Navigation Flight Path")
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.savefig(out_png, dpi=150, bbox_inches="tight")
        plt.close()
        print(f"[main] Flight path saved to {out_png}")
    except Exception as e:
        print(f"[main] (skipping flight-path plot: {e})")


def fly(vehicle):
    """The whole mission. Returns the process exit code."""
    vehicle.set_mode(config.GUIDED_MODE_NAME)

    # Build the sensors BEFORE arming: a missing log file, sensor library or dead I2C device
    # should fail here, on the ground, not after takeoff.
    sensors = build_sensors(vehicle)

    if config.PREFLIGHT_SENSOR_CHECK and not preflight_sensor_check(sensors):
        return 1

    if config.FEED_DISTANCE_SENSOR_TO_FC:
        print("[main] Sending placeholder proximity data to clear PRX1 PreArm check...")
        _send_clear_distance_burst(vehicle)

    if config.CONFIRM_BEFORE_ARM:
        input("[main] Area clear? Pilot ready with the RC mode switch? Press ENTER to ARM and take off... ")

    armed = vehicle.arm()
    if not armed:
        print("[main] Failed to arm -- check PreArm messages above (GPS/EKF, proximity, rangefinder, "
              "battery, safety switch). Aborting.")
        return 1

    # SITL only: keep proximity data "fresh" during takeoff/descent, stopped before the navigator
    # starts so two sources never write at once.
    keepalive_stop = threading.Event()
    keepalive_thread = None
    if config.FEED_DISTANCE_SENSOR_TO_FC:
        keepalive_thread = threading.Thread(
            target=_keep_proximity_alive, args=(vehicle, keepalive_stop), daemon=True)
        keepalive_thread.start()

    # Takeoff blocks until altitude is reached.
    vehicle.takeoff(config.SCAN_ALTITUDE_M)

    # --- placeholder: QR scan + banner alignment would happen here ---
    print("[main] (QR scan / banner alignment step goes here)")

    # Descend to corridor altitude before entering
    print(f"[main] Descending to corridor altitude {config.CORRIDOR_ALTITUDE_M} m")
    descent_rate = config.DESCENT_RATE_MPS                                   # m/s downward
    altitude_to_lose = config.SCAN_ALTITUDE_M - config.CORRIDOR_ALTITUDE_M   # 5 - 3 = 2 m
    descent_time = altitude_to_lose / descent_rate

    for _ in range(int(descent_time * 10 * 2)):          # 10 Hz loop, 2x margin in case the FC lags
        mode = vehicle.get_mode() if hasattr(vehicle, "get_mode") else None
        if config.PILOT_TAKEOVER_CHECK and mode is not None and mode != config.GUIDED_MODE_NAME:
            print(f"[main] FC mode is {mode} during descent -- pilot took over, script exits.")
            return 0
        current_alt = vehicle.get_relative_altitude()
        if current_alt is not None and current_alt <= config.CORRIDOR_ALTITUDE_M + 0.3:
            break
        vehicle.send_velocity_body(vx=0, vy=0, vz=descent_rate, yaw_rate=0.0)
        time.sleep(0.1)

    vehicle.hold_position()
    time.sleep(1)

    if keepalive_thread is not None:
        keepalive_stop.set()
        keepalive_thread.join(timeout=1)

    logger = FlightLogger("corridor_run_log.csv")
    navigator = CorridorNavigator(vehicle, sensors, logger=logger)

    result = navigator.run()
    logger.close()

    if result == CorridorResult.PILOT_TAKEOVER:
        print("[main] Pilot has control -- not landing. Script exits.")
        return 0

    if result != CorridorResult.SUCCESS:
        print(f"[main] Corridor navigation ended with: {result}. Landing.")
        vehicle.land()
        return 1

    print("[main] Corridor cleared successfully. Continuing mission...")
    _plot_flight_path("corridor_run_log.csv")

    # --- placeholder: ascend to delivery altitude, target ID, payload drop ---

    vehicle.land()
    return 0


def main():
    vehicle = Vehicle()
    vehicle.connect()
    try:
        code = fly(vehicle)
    except KeyboardInterrupt:
        print("\n[main] Ctrl+C -- holding position and leaving the aircraft to the pilot "
              "(take over with the RC mode switch; it is still in GUIDED).")
        try:
            vehicle.hold_position()
        except Exception:
            pass
        code = 130
    except Exception:
        traceback.print_exc()
        print("[main] Unexpected error -- trying to land.")
        try:
            vehicle.land()
        except Exception:
            pass
        code = 2
    sys.exit(code)


if __name__ == "__main__":
    main()