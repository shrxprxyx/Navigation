"""
sensor_check.py
---------------
Bench tool: prints the sensor readings the navigator would see. Run it with PROPS OFF.

    python sensor_check.py              # full check: VL53L1X pair on the Pi + front lidar via the FC
    python sensor_check.py --tof-only   # only the two VL53L1X (no flight controller needed)

Ctrl+C to stop.
"""
import sys
import time
import config


def tof_only():
    from tof_interface import make_adafruit_vl53l1x_pair, make_vl53l1x_pair
    if config.TOF_LIBRARY == "adafruit":
        left, right = make_adafruit_vl53l1x_pair(
            config.TOF_LEFT_XSHUT_GPIO, config.TOF_RIGHT_XSHUT_GPIO,
            config.TOF_LEFT_ADDR, config.TOF_RIGHT_ADDR,
            mode=config.TOF_DISTANCE_MODE, timing_ms=config.TOF_TIMING_BUDGET_MS,
            i2c_bus=config.TOF_I2C_BUS)
    else:
        left, right = make_vl53l1x_pair(
            config.TOF_LEFT_XSHUT_GPIO, config.TOF_RIGHT_XSHUT_GPIO,
            config.TOF_LEFT_ADDR, config.TOF_RIGHT_ADDR, config.TOF_I2C_BUS,
            mode=config.TOF_DISTANCE_MODE)
    print("Raw VL53L1X readings (mm, before bias/offset). Ctrl+C to stop.\n")
    while True:
        print(f"left={left()}  right={right()}")
        time.sleep(0.1)


def full():
    from mavlink_connection import Vehicle
    from real_sensors import RealCorridorSensors
    v = Vehicle()
    v.connect()
    s = RealCorridorSensors(v)
    print("Centre-referenced readings (bias + mount offset applied). Ctrl+C to stop.\n")
    while True:
        r = s.read(0.0, 0.0)
        raw = v.get_distance_sensor_mm(config.FRONT_LIDAR_ORIENTATION)
        print(f"left={r['left_m']:.3f}  right={r['right_m']:.3f}  front={r['front_m']:.3f}  "
              f"(front raw from FC: {raw} mm)  front_valid={r['front_valid']}  "
              f"mode={v.get_mode()}")
        time.sleep(0.1)


if __name__ == "__main__":
    try:
        tof_only() if "--tof-only" in sys.argv else full()
    except KeyboardInterrupt:
        print("\nstopped")  