#!/usr/bin/env python3
"""
Raspberry Pi + ADS1115 firefly current logger with beacon power control.

The measured firefly is powered from its own 2xAA batteries through a shunt.
The control beacon is a separate firefly powered by a Pi-controlled switch for
the configured beacon window, usually 4 hours.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import signal
import sys
import time
from pathlib import Path

import board
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn


GAIN_MAP = {
    "2/3": 2 / 3,
    "1": 1,
    "2": 2,
    "4": 4,
    "8": 8,
    "16": 16,
}


class BeaconSwitch:
    def __init__(self, gpio: int | None, active_low: bool) -> None:
        self.device = None
        self.rpi_gpio = None
        self.gpio = gpio
        self.active_low = active_low

        if gpio is None:
            return

        try:
            from gpiozero import DigitalOutputDevice
            self.device = DigitalOutputDevice(
                gpio,
                active_high=not active_low,
                initial_value=False,
            )
            return
        except Exception:
            pass

        try:
            import RPi.GPIO as GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(gpio, GPIO.OUT, initial=self._physical_level(False, GPIO))
            self.rpi_gpio = GPIO
            return
        except Exception as exc:
            raise RuntimeError(
                "Beacon GPIO control requires gpiozero or RPi.GPIO. "
                "Install one of them, or run with --no-beacon-gpio."
            ) from exc

    def _physical_level(self, enabled: bool, gpio_module) -> int:
        active = enabled
        if self.active_low:
            active = not active
        return gpio_module.HIGH if active else gpio_module.LOW

    def set(self, enabled: bool) -> None:
        if self.device is not None:
            if enabled:
                self.device.on()
            else:
                self.device.off()
        elif self.rpi_gpio is not None and self.gpio is not None:
            self.rpi_gpio.output(
                self.gpio,
                self._physical_level(enabled, self.rpi_gpio),
            )

    def close(self) -> None:
        if self.device is not None:
            self.device.off()
            self.device.close()
        elif self.rpi_gpio is not None and self.gpio is not None:
            self.rpi_gpio.output(
                self.gpio,
                self._physical_level(False, self.rpi_gpio),
            )
            self.rpi_gpio.cleanup(self.gpio)


def parse_start_time(value: str, now: dt.datetime) -> dt.datetime | None:
    if value == "now":
        return now
    if value == "off":
        return None

    hour_str, minute_str = value.split(":", 1)
    start = now.replace(
        hour=int(hour_str),
        minute=int(minute_str),
        second=0,
        microsecond=0,
    )
    if start < now:
        start += dt.timedelta(days=1)
    return start


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Log firefly current with ADS1115 and control beacon power.",
    )
    parser.add_argument("--shunt-ohms", type=float, default=10.0)
    parser.add_argument(
        "--gain",
        choices=GAIN_MAP.keys(),
        default="4",
        help="ADS1115 PGA gain. Use 4 for 10 ohm shunt, 8 for more resolution if peaks stay below about 51 mA.",
    )
    parser.add_argument("--data-rate", type=int, default=860)
    parser.add_argument("--summary-interval", type=float, default=1.0)
    parser.add_argument("--duration-hours", type=float, default=24.0)
    parser.add_argument(
        "--output",
        default=None,
        help="CSV output path. Defaults to pi_ammeter/logs/firefly_YYYYmmdd_HHMMSS.csv",
    )
    parser.add_argument(
        "--beacon-start",
        default="now",
        help="'now', 'off', or HH:MM. HH:MM schedules the next occurrence.",
    )
    parser.add_argument("--beacon-on-hours", type=float, default=4.0)
    parser.add_argument("--beacon-gpio", type=int, default=17)
    parser.add_argument("--beacon-active-low", action="store_true")
    parser.add_argument(
        "--no-beacon-gpio",
        action="store_true",
        help="Measure only; do not control beacon power.",
    )
    parser.add_argument(
        "--measure-voltage",
        action="store_true",
        help="Also read firefly load voltage using A2=firefly VCC and A0=firefly GND.",
    )
    parser.add_argument(
        "--allow-negative",
        action="store_true",
        help="Do not clamp negative current to zero. Useful for debugging reversed inputs.",
    )
    return parser


def default_output_path() -> Path:
    base = Path(__file__).resolve().parent / "logs"
    base.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    return base / f"firefly_{stamp}.csv"


def main() -> int:
    args = build_parser().parse_args()
    output_path = Path(args.output) if args.output else default_output_path()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    now = dt.datetime.now()
    beacon_start = parse_start_time(args.beacon_start, now)
    beacon_end = (
        beacon_start + dt.timedelta(hours=args.beacon_on_hours)
        if beacon_start is not None
        else None
    )

    beacon = BeaconSwitch(
        None if args.no_beacon_gpio else args.beacon_gpio,
        active_low=args.beacon_active_low,
    )

    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    ads.gain = GAIN_MAP[args.gain]
    ads.data_rate = args.data_rate

    shunt = AnalogIn(ads, ADS.P0, ADS.P1)
    vcc = AnalogIn(ads, ADS.P2) if args.measure_voltage else None
    firefly_gnd = AnalogIn(ads, ADS.P0) if args.measure_voltage else None

    stop = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    start_monotonic = time.monotonic()
    end_monotonic = start_monotonic + args.duration_hours * 3600.0
    next_summary = start_monotonic + args.summary_interval
    last_sample = start_monotonic

    total_mAh = 0.0
    total_mWh = 0.0
    beacon_state = False

    print(f"Logging to {output_path}")
    if beacon_start is None:
        print("Beacon GPIO schedule: off")
    else:
        print(f"Beacon on: {beacon_start.isoformat(timespec='seconds')}")
        print(f"Beacon off: {beacon_end.isoformat(timespec='seconds')}")

    fieldnames = [
        "iso_time",
        "elapsed_s",
        "sample_count",
        "avg_mA",
        "min_mA",
        "max_mA",
        "avg_shunt_mV",
        "load_V",
        "interval_mAh",
        "total_mAh",
        "interval_mWh",
        "total_mWh",
        "beacon_on",
    ]

    try:
        with output_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            count = 0
            sum_mA = 0.0
            sum_shunt_mV = 0.0
            min_mA = math.inf
            max_mA = -math.inf
            interval_mAh = 0.0
            interval_mWh = 0.0

            while not stop and time.monotonic() < end_monotonic:
                wall_now = dt.datetime.now()
                should_beacon_on = (
                    beacon_start is not None
                    and beacon_end is not None
                    and beacon_start <= wall_now < beacon_end
                )
                if should_beacon_on != beacon_state:
                    beacon_state = should_beacon_on
                    beacon.set(beacon_state)
                    print(
                        f"{wall_now.isoformat(timespec='seconds')} beacon "
                        f"{'ON' if beacon_state else 'OFF'}"
                    )

                sample_now = time.monotonic()
                dt_s = sample_now - last_sample
                last_sample = sample_now

                shunt_v = shunt.voltage
                current_mA = (shunt_v / args.shunt_ohms) * 1000.0
                if not args.allow_negative and current_mA < 0.0:
                    current_mA = 0.0

                load_v = None
                if vcc is not None and firefly_gnd is not None:
                    load_v = max(0.0, vcc.voltage - firefly_gnd.voltage)

                sample_mAh = current_mA * dt_s / 3600.0
                total_mAh += sample_mAh
                interval_mAh += sample_mAh

                if load_v is not None:
                    sample_mWh = load_v * current_mA * dt_s / 3600.0
                    total_mWh += sample_mWh
                    interval_mWh += sample_mWh

                count += 1
                sum_mA += current_mA
                sum_shunt_mV += shunt_v * 1000.0
                min_mA = min(min_mA, current_mA)
                max_mA = max(max_mA, current_mA)

                if sample_now >= next_summary:
                    avg_mA = sum_mA / count if count else 0.0
                    avg_shunt_mV = sum_shunt_mV / count if count else 0.0
                    elapsed_s = sample_now - start_monotonic

                    writer.writerow(
                        {
                            "iso_time": wall_now.isoformat(timespec="seconds"),
                            "elapsed_s": f"{elapsed_s:.3f}",
                            "sample_count": count,
                            "avg_mA": f"{avg_mA:.6f}",
                            "min_mA": f"{min_mA:.6f}",
                            "max_mA": f"{max_mA:.6f}",
                            "avg_shunt_mV": f"{avg_shunt_mV:.6f}",
                            "load_V": "" if load_v is None else f"{load_v:.6f}",
                            "interval_mAh": f"{interval_mAh:.9f}",
                            "total_mAh": f"{total_mAh:.6f}",
                            "interval_mWh": "" if load_v is None else f"{interval_mWh:.9f}",
                            "total_mWh": "" if load_v is None else f"{total_mWh:.6f}",
                            "beacon_on": int(beacon_state),
                        }
                    )
                    f.flush()

                    print(
                        f"{elapsed_s:9.1f}s avg={avg_mA:8.4f} mA "
                        f"total={total_mAh:9.4f} mAh "
                        f"beacon={'ON' if beacon_state else 'off'}"
                    )

                    count = 0
                    sum_mA = 0.0
                    sum_shunt_mV = 0.0
                    min_mA = math.inf
                    max_mA = -math.inf
                    interval_mAh = 0.0
                    interval_mWh = 0.0
                    while next_summary <= sample_now:
                        next_summary += args.summary_interval

    finally:
        beacon.close()

    print(f"Done. Total current draw: {total_mAh:.4f} mAh")
    if total_mWh > 0.0:
        print(f"Done. Total energy draw: {total_mWh:.4f} mWh")
    return 0


if __name__ == "__main__":
    sys.exit(main())
