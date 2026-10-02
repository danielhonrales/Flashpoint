#!/usr/bin/env python3
"""
Peltier open-loop test: no thermistor, no PID.

Each module in turn: HEAT at 100% for 5 s, rest 2 s, COOL at 100% for 5 s, off. Touch the skin
face briefly during each phase to feel whether it responds. If "HEAT" cools, that module's
heat_ph in PELTIERS below is the wrong way round: flip it (and in stimulus_editor.py).

At 100% the EN channel is fully on rather than pulsed, so there is no PWM involved at all.

Hardware assumed:
  PCA9685 ch14 -> DRV8876 #1 EN,  GPIO17 -> #1 PH,  GPIO27 -> #1 nSLEEP
  PCA9685 ch15 -> DRV8876 #2 EN,  GPIO22 -> #2 PH,  GPIO10 -> #2 nSLEEP
  PMODE -> GND (PH/EN mode).
  PCA9685 on I2C bus 1 (header pins 3 and 5), address 0x40.
  nSLEEP is held low (driver asleep, outputs off) except while a module is being tested.

SAFETY
  - The heat sink MUST be mounted on the outward face before running this.
  - Don't leave a module on skin; touch briefly.
  - 100% is ~1.6 A through a TEC1-12704 at 5 V, over the DRV8876's 1.3 A continuous rating.
    That's acceptable for 5 s bursts (the driver also has thermal shutdown), not for longer.
  - The modules run one at a time: both at 100% would draw ~3.2 A, past the 2.5 A fuse.
  - Power the drivers from the actuator bank, not the Pi's 5 V pin (see HARDWARE.md §2).
  - Ctrl+C at any time cuts all output: EN off over I2C, and nSLEEP low, which stops the driver
    even if I2C has failed.
  - stimulus_editor.py and pi_receiver.py drive these same Peltiers: close them first.

Usage: python3 peltier_test.py                (heat, rest, cool)
       python3 peltier_test.py heat           (heat only)
       python3 peltier_test.py cool --seconds 3
       python3 peltier_test.py --module 2     (just Peltier 2)
Needs only smbus2 and RPi.GPIO, both already on the Pi, plus pca9685_test.py in this folder.
"""

import argparse
import signal
import time

import RPi.GPIO as GPIO
from smbus2 import SMBus

from pca9685_test import ADDRESS, I2C_BUS, PCA9685, ensure_i2c_pins

# ---------------------------------------------------------------- config

PWM_FREQ = 150           # Hz. Global on the PCA9685 (set low for the ERMs); at 100% duty the
                         # Peltier channels don't pulse, so it doesn't affect this test.
DUTY = 1.0               # 100%: EN fully on
HEAT_S = 5               # seconds heating
REST_S = 2               # seconds off between heating and cooling (and between modules)
COOL_S = 5               # seconds cooling

PELTIERS = {
    # heat_ph: the PH level that heats the skin face (0: PH low heats; flip if hot/cold swap)
    1: {"channel": 14, "ph_pin": 17, "nsleep_pin": 27, "heat_ph": 0},
    2: {"channel": 15, "ph_pin": 22, "nsleep_pin": 10, "heat_ph": 0},
}
DRIVER_WAKE_S = 0.002    # the DRV8876 needs ~1 ms after nSLEEP goes high

# ---------------------------------------------------------------- hardware


class Peltiers:
    """The modules' EN channels on the PCA9685, and PH and nSLEEP pins on GPIO. PH is only set
    while EN is off, so a module is never reversed under load; nSLEEP keeps each driver asleep
    (outputs off) unless it is in use."""

    def __init__(self, pca: PCA9685):
        self.pca = pca
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for cfg in PELTIERS.values():
            GPIO.setup(cfg["nsleep_pin"], GPIO.OUT, initial=GPIO.LOW)     # asleep
            GPIO.setup(cfg["ph_pin"], GPIO.OUT, initial=GPIO.LOW)

    def wake(self, idx):
        GPIO.output(PELTIERS[idx]["nsleep_pin"], GPIO.HIGH)
        time.sleep(DRIVER_WAKE_S)

    def sleep(self, idx):
        GPIO.output(PELTIERS[idx]["nsleep_pin"], GPIO.LOW)

    def run(self, idx, heat, seconds):
        """Drive one module for `seconds`, heating or cooling the skin face, then off."""
        cfg = PELTIERS[idx]
        self.off(idx)
        GPIO.output(cfg["ph_pin"], cfg["heat_ph"] if heat else 1 - cfg["heat_ph"])
        self.pca.set_duty(cfg["channel"], DUTY)
        start = time.perf_counter()
        while (left := seconds - (time.perf_counter() - start)) > 0:
            print(f"\r  {'HEAT' if heat else 'COOL'} {DUTY:.0%}  {left:4.1f} s left ", end="",
                  flush=True)
            time.sleep(min(0.1, left))
        self.off(idx)
        print(f"\r  {'HEAT' if heat else 'COOL'} {DUTY:.0%}  done          ")

    def off(self, idx):
        self.pca.set_duty(PELTIERS[idx]["channel"], 0)

    def all_off(self):
        """Cut every Peltier output: drivers asleep first (that alone stops them), then EN off,
        retrying the I2C writes. False if EN couldn't be switched off."""
        ok = True
        for idx, cfg in PELTIERS.items():
            self.sleep(idx)
            for attempt in range(5):
                try:
                    self.off(idx)
                    break
                except OSError:
                    time.sleep(0.05)
            else:
                ok = False
            GPIO.output(cfg["ph_pin"], GPIO.LOW)
        return ok


# ---------------------------------------------------------------- main


def _on_sigterm(*args):
    raise KeyboardInterrupt     # clean up the same way as Ctrl+C


def main():
    parser = argparse.ArgumentParser(description="Heat and/or cool each Peltier at 100%.")
    parser.add_argument("mode", nargs="?", choices=["both", "heat", "cool"], default="both",
                        help="heat only, cool only, or both with a rest between (default both)")
    parser.add_argument("--seconds", type=float, default=None,
                        help=f"how long each phase runs (default {HEAT_S} s heat, {COOL_S} s cool)")
    parser.add_argument("--module", type=int, choices=list(PELTIERS),
                        help="test just this module (default: all, one after another)")
    args = parser.parse_args()
    if args.seconds is not None and args.seconds <= 0:
        parser.error("--seconds must be > 0")
    modules = [args.module] if args.module else list(PELTIERS)
    heat_s = args.seconds or HEAT_S
    cool_s = args.seconds or COOL_S
    phases = {"both": [("heat", heat_s), ("cool", cool_s)], "heat": [("heat", heat_s)],
              "cool": [("cool", cool_s)]}[args.mode]

    signal.signal(signal.SIGTERM, _on_sigterm)     # e.g. kill, systemctl stop
    ensure_i2c_pins()
    with SMBus(I2C_BUS) as bus:
        try:
            pca = PCA9685(bus, ADDRESS)
            pca.set_frequency(PWM_FREQ)
        except OSError as e:
            raise SystemExit(f"Can't reach the PCA9685 at 0x{ADDRESS:02x}: {e}\n"
                             "Check with: i2cdetect -y 1")
        peltiers = Peltiers(pca)
        try:
            peltiers.all_off()
            plan = f", rest {REST_S:g} s, ".join(f"{name.upper()} {secs:g} s" for name, secs in phases)
            print(f"Peltier test: {plan} at "
                  f"{DUTY:.0%}, on {'module ' if len(modules) == 1 else 'modules '}"
                  f"{', '.join(map(str, modules))}.\nHeat sinks mounted? Ctrl+C aborts at any time.")
            input("Press Enter to start ...")
            for n, idx in enumerate(modules):
                if n:
                    time.sleep(REST_S)
                cfg = PELTIERS[idx]
                print(f"\n--- Peltier {idx} (PCA9685 ch{cfg['channel']}, PH GPIO{cfg['ph_pin']}, "
                      f"nSLEEP GPIO{cfg['nsleep_pin']}) ---")
                peltiers.wake(idx)
                for k, (name, secs) in enumerate(phases):
                    if k:
                        print(f"  rest {REST_S} s")
                        time.sleep(REST_S)
                    peltiers.run(idx, heat=name == "heat", seconds=secs)
                peltiers.sleep(idx)
        except KeyboardInterrupt:
            print("\nAborted.")
        finally:
            if peltiers.all_off():
                print("All outputs off, drivers asleep.")
            else:
                print("\nCouldn't switch EN off over I2C, but the drivers are asleep (nSLEEP low),\n"
                      "which stops them. Check the PCA9685 before running again.")
            GPIO.cleanup([pin for cfg in PELTIERS.values()
                          for pin in (cfg["ph_pin"], cfg["nsleep_pin"])])


if __name__ == "__main__":
    main()
