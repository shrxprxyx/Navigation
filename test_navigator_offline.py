"""
test_navigator_offline.py
---------------------------
Runs the corridor-centering + obstacle-avoidance LOGIC completely offline,
with a fake "vehicle" that just integrates velocity commands into a
position -- no MAVLink, no SITL required. Use this first to sanity check
your PID gains and obstacle thresholds before ever touching Mission Planner.

Run:
    python test_navigator_offline.py

It will print a summary and (if matplotlib is installed) plot the track.
"""

import config
from sensor_sim import SimulatedCorridorSensors
from corridor_navigator import CorridorNavigator, CorridorResult
from flight_logger import FlightLogger


class FakeVehicle:
    """Mimics mavlink_connection.Vehicle's interface used by CorridorNavigator."""
    def __init__(self):
        self.last_vx = 0.0
        self.last_vy = 0.0

    def send_velocity_body(self, vx, vy, vz, yaw_rate=0.0):
        self.last_vx = vx
        self.last_vy = vy

    def hold_position(self):
        self.last_vx = 0.0
        self.last_vy = 0.0


def main():
    vehicle = FakeVehicle()
    sensors = SimulatedCorridorSensors()
    logger = FlightLogger("offline_test_log.csv")

    navigator = CorridorNavigator(vehicle, sensors, logger=logger)
    result = navigator.run(real_time=False)
    logger.close()

    print(f"\n=== Result: {result} ===")
    print(f"Final along-corridor distance: {navigator._along_corridor_m:.2f} m")
    print(f"Final lateral offset: {navigator._lateral_offset_m:.2f} m")

    _maybe_plot("offline_test_log.csv")


def _maybe_plot(csv_path):
    try:
        import csv as csv_module
        import matplotlib.pyplot as plt
    except ImportError:
        print("(matplotlib not installed -- skipping plot; pip install matplotlib to enable)")
        return

    t, along, lateral, left, right, front = [], [], [], [], [], []
    with open(csv_path, newline="") as f:
        reader = csv_module.DictReader(f)
        for row in reader:
            t.append(float(row["t"]))
            along.append(float(row["along_m"]))
            lateral.append(float(row["lateral_m"]))
            left.append(float(row["left_m"]))
            right.append(float(row["right_m"]))
            front.append(float(row["front_m"]))

    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)

    axes[0].plot(along, lateral, label="lateral offset (m)")
    axes[0].axhline(config.CORRIDOR_WIDTH_M / 2, color="r", linestyle="--", label="wall")
    axes[0].axhline(-config.CORRIDOR_WIDTH_M / 2, color="r", linestyle="--")
    axes[0].set_ylabel("Lateral offset (m)")
    axes[0].set_title("Corridor track: lateral offset vs distance traveled")
    axes[0].legend()

    axes[1].plot(t, front, label="front distance (m)")
    axes[1].axhline(config.OBSTACLE_STOP_DISTANCE_M, color="orange", linestyle="--", label="stop threshold")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("Front distance (m)")
    axes[1].legend()

    plt.tight_layout()
    plt.savefig("offline_test_plot.png", dpi=150)
    print("Plot saved to offline_test_plot.png")


if __name__ == "__main__":
    main()