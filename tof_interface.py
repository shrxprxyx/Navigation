"""tof_interface.py

Filtering wrapper + hardware readers for the VL53L1X ToF sensors on the Pi.
(The front lidar is wired to the FC and read through Vehicle.get_distance_sensor_mm.)

Pi setup:  pip install VL53L1X     (gpiozero ships with Raspberry Pi OS; enable I2C)

Each sensor = RangeSensor(raw_reader, SensorConfig). raw_reader returns mm or None.
"""
import random
import time
from collections import deque

_XSHUT_KEEPALIVE = []   # keep GPIO objects alive so XSHUT pins stay high


class SensorConfig:
    def __init__(self, offset_mm=0, min_mm=30, max_mm=4000,
                 median_window=5, stale_s=0.5, max_jump_mm=600):
        self.offset_mm = offset_mm        # added to raw (matches config.*_SENSOR_BIAS_MM)
        self.min_mm = min_mm
        self.max_mm = max_mm              # >= this is "no return"
        self.median_window = median_window
        self.stale_s = stale_s            # hold last valid this long, then invalid
        self.max_jump_mm = max_jump_mm    # reject single-sample jumps beyond this


class RangeReading:
    __slots__ = ("range_mm", "valid", "timestamp")

    def __init__(self, range_mm, valid, timestamp):
        self.range_mm, self.valid, self.timestamp = range_mm, valid, timestamp

    def __repr__(self):
        return f"RangeReading({self.range_mm}, valid={self.valid})"


class RangeSensor:
    def __init__(self, raw_reader, cfg=None):
        self._read_raw = raw_reader
        self._cfg = cfg or SensorConfig()
        self._buf = deque(maxlen=self._cfg.median_window)
        self._last = None
        self._last_t = 0.0

    def read(self):
        c, now = self._cfg, time.monotonic()
        raw = self._read_raw()

        ok = raw is not None and c.min_mm <= raw < c.max_mm
        if ok and self._last is not None:
            if abs(raw + c.offset_mm - self._last) > c.max_jump_mm:
                ok = False                # implausible jump -> spike

        if ok:
            self._buf.append(raw + c.offset_mm)
            self._last = sorted(self._buf)[len(self._buf) // 2]
            self._last_t = now
            return RangeReading(self._last, True, now)

        if self._last is not None and (now - self._last_t) < c.stale_s:
            return RangeReading(self._last, True, now)   # brief hold
        return RangeReading(None, False, now)


def make_vl53l1x_reader(i2c_bus=1, address=0x29, mode=2):
    """One VL53L1X via the `VL53L1X` package. mode: 1 short, 2 medium, 3 long. Returns mm."""
    import VL53L1X
    tof = VL53L1X.VL53L1X(i2c_bus=i2c_bus, i2c_address=address)
    tof.open()
    tof.start_ranging(mode)

    def read():
        try:
            d = tof.get_distance()        # mm; <=0 when invalid
            return d if d > 0 else None
        except Exception:
            return None
    return read


def make_vl53l1x_pair(xshut_left, xshut_right, addr_left=0x30, addr_right=0x31,
                      i2c_bus=1, mode=2):
    """Two VL53L1X share address 0x29, so boot them one at a time via their XSHUT pins
    and give each its own address. Wire XSHUT_L / XSHUT_R to two free Pi GPIOs (BCM numbers).
    Returns (left_reader, right_reader)."""
    import VL53L1X
    from gpiozero import DigitalOutputDevice
    pins = [DigitalOutputDevice(xshut_left, initial_value=False),
            DigitalOutputDevice(xshut_right, initial_value=False)]
    _XSHUT_KEEPALIVE.extend(pins)
    time.sleep(0.05)
    readers = []
    for pin, addr in zip(pins, (addr_left, addr_right)):
        pin.on()
        time.sleep(0.05)
        t = VL53L1X.VL53L1X(i2c_bus=i2c_bus, i2c_address=0x29)
        t.open()
        t.change_address(addr)
        t.close()
        readers.append(make_vl53l1x_reader(i2c_bus, addr, mode))
    return readers[0], readers[1]


def _adafruit_reader(sensor, stale_s=0.5):
    """Wrap one adafruit_vl53l1x sensor as a reader returning mm or None.
    The sensor only produces a new value every timing-budget (e.g. 50 ms), so between
    measurements we return the newest one instead of None. None comes back only when the
    sensor itself reported no target, or when it has produced nothing for `stale_s`."""
    state = {"mm": None, "t": time.monotonic()}

    def read():
        try:
            if sensor.data_ready:
                d = sensor.distance            # cm, or None when there is no valid target
                sensor.clear_interrupt()
                state["mm"] = None if d is None else d * 10.0
                state["t"] = time.monotonic()
        except Exception:
            pass
        if time.monotonic() - state["t"] > stale_s:
            return None                        # sensor stopped answering
        return state["mm"]
    return read


def make_adafruit_vl53l1x_pair(xshut_left, xshut_right, addr_left=0x30, addr_right=0x31,
                               mode=2, timing_ms=50):
    """Two VL53L1X with the Adafruit CircuitPython driver (distance in cm, None = no target).
    Both boot at 0x29, so they are started one at a time via their XSHUT pins and each is given
    its own address. mode: 1 = short, 2 = long. Returns (left_reader, right_reader) in mm."""
    import board
    import adafruit_vl53l1x
    from gpiozero import DigitalOutputDevice
    i2c = board.I2C()
    pins = [DigitalOutputDevice(xshut_left, initial_value=False),
            DigitalOutputDevice(xshut_right, initial_value=False)]
    _XSHUT_KEEPALIVE.extend(pins)
    time.sleep(0.05)
    readers = []
    for pin, addr in zip(pins, (addr_left, addr_right)):
        pin.on()
        time.sleep(0.05)
        sensor = adafruit_vl53l1x.VL53L1X(i2c)        # appears at the default 0x29
        sensor.set_address(addr)
        sensor.distance_mode = mode
        sensor.timing_budget = timing_ms
        sensor.start_ranging()
        readers.append(_adafruit_reader(sensor))
    return readers[0], readers[1]


class FakeRange:
    """Simulated raw reader (mm) for offline tests. Defaults mimic the bench noise."""
    def __init__(self, true_mm, bias_mm=-7, sigma_mm=2.0, dropout_p=0.0, spike_p=0.0):
        self._true, self._bias, self._sigma = true_mm, bias_mm, sigma_mm
        self._dropout_p, self._spike_p = dropout_p, spike_p

    def __call__(self):
        r = random.random()
        if r < self._dropout_p:
            return None
        true = self._true() if callable(self._true) else self._true
        if r < self._dropout_p + self._spike_p:
            return int(true + random.choice([-1, 1]) * random.randint(800, 1500))
        return int(round(true + self._bias + random.gauss(0, self._sigma)))