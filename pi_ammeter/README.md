# Raspberry Pi Firefly Ammeter

This setup logs the current of one measured firefly while the Pi also powers a
separate control beacon for the configured show window.

## Parts

- Raspberry Pi with I2C enabled
- ADS1115 I2C ADC+PGA module
- Shunt resistor, ideally 1-10 ohm
- Logic-level N-MOSFET module or relay module for switching the beacon power
- Separate 2xAA pack for the measured firefly
- Separate supply for the beacon firefly

## Measured firefly wiring

Low-side shunt measurement:

```text
2xAA +  --------------------------- measured firefly VCC

2xAA -  ----+---- Pi GND
            |
            +---- ADS1115 GND
            |
            +---- ADS1115 A1
            |
          SHUNT
            |
            +---- measured firefly GND
            |
            +---- ADS1115 A0
```

Optional voltage logging:

```text
ADS1115 A2 ---- measured firefly VCC
```

With 2xAA alkaline this is within the ADS1115 input range when the ADS1115 is
powered from 3.3 V. Do not connect A2 to voltages above the ADS1115 supply.

## ADS1115 to Raspberry Pi

```text
ADS1115 VDD ---- Pi 3V3
ADS1115 GND ---- Pi GND
ADS1115 SDA ---- Pi SDA / GPIO2 / physical pin 3
ADS1115 SCL ---- Pi SCL / GPIO3 / physical pin 5
```

Enable I2C on the Pi:

```bash
sudo raspi-config
```

Then enable `Interface Options -> I2C`.

## Beacon power switching

Do not power the beacon directly from a Pi GPIO pin. Use the GPIO only as a
control signal for a relay module or MOSFET switch.

Typical low-side N-MOSFET wiring:

```text
Beacon supply +  ------------------ beacon VCC

Beacon GND ------------------------ MOSFET drain
MOSFET source --------------------- beacon supply -
Beacon supply - ------------------- Pi GND
Pi GPIO17 ------------------------- MOSFET gate through ~100-330 ohm
MOSFET gate ----------------------- Pi GND through ~100k pulldown
```

If you use a ready-made relay module, wire:

```text
Pi GPIO17 ---- relay IN
Pi GND ------- relay GND
relay COM/NO - in series with beacon supply positive
```

Some relay modules are active-low. Use `--beacon-active-low` for those.

## Install Python dependencies

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install smbus2 RPi.GPIO
```

If you do not want to use a GPIO-controlled beacon switch yet, you can install
only:

```bash
pip install smbus2
```

## Run

Start logging now, power the beacon now, turn it off after 4 hours, and keep
logging for 24 hours:

```bash
python3 pi_ammeter/measure_firefly_day.py --shunt-ohms 10 --beacon-start now --duration-hours 24
```

Schedule the beacon for 20:00, off at 24:00, while logging for 24 hours:

```bash
python3 pi_ammeter/measure_firefly_day.py --shunt-ohms 10 --beacon-start 20:00 --beacon-on-hours 4 --duration-hours 24
```

Also log firefly voltage:

```bash
python3 pi_ammeter/measure_firefly_day.py --shunt-ohms 10 --measure-voltage
```

CSV files are written to `pi_ammeter/logs/`. The important column is
`total_mAh`. For a 30 day estimate:

```text
30_day_mAh = measured_24h_total_mAh * 30
```

Add a safety margin of at least 20-25%.
