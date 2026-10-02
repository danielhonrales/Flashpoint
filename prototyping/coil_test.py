"""
Coil test: drives a coil actuator through an H-bridge, flipping its polarity over and over.

The H-bridge's two inputs are on GPIO17 and GPIO27. One high and the other low drives current
one way through the coil, the other way round drives it back; both low lets the coil coast.
Between flips both inputs are held low for DEAD_TIME, so the bridge never sees both high.

Usage: python3 coil_test.py [--freq 20] [--duration 5] [--dead-time 0.0005]
    --freq      full back-and-forth cycles per second
    --duration  seconds to run; 0 runs until Ctrl+C

Both pins are left low when it exits. pi_receiver.py also drives GPIO17 and 27 (as ERMs 2 and 3),
so stop it before running this.
"""

import argparse
import time

import RPi.GPIO as GPIO

PIN_A = 20
PIN_B = 21
DEAD_TIME = 0.0005  # seconds both inputs are low between flips


def main():
    parser = argparse.ArgumentParser(description="Flip a coil's polarity through an H-bridge.")
    parser.add_argument("--freq", type=float, default=20.0, help="cycles per second (default 20)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="seconds to run; 0 for until Ctrl+C (default 5)")
    parser.add_argument("--dead-time", type=float, default=DEAD_TIME,
                        help=f"seconds both pins are low between flips (default {DEAD_TIME})")
    args = parser.parse_args()

    half_period = 1.0 / (2 * args.freq)
    if args.dead_time >= half_period:
        parser.error(f"--dead-time must be shorter than half a cycle ({half_period:.4f} s)")

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PIN_A, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(PIN_B, GPIO.OUT, initial=GPIO.LOW)

    print(f"Flipping GPIO{PIN_A}/GPIO{PIN_B} at {args.freq:g} Hz"
          + (f" for {args.duration:g} s" if args.duration else " until Ctrl+C"))
    start = next_flip = time.perf_counter()
    forward = True
    flips = 0
    try:
        while not args.duration or time.perf_counter() - start < args.duration:
            # Both low first, so the bridge's two sides are never on together.
            GPIO.output(PIN_A, GPIO.LOW)
            GPIO.output(PIN_B, GPIO.LOW)
            time.sleep(args.dead_time)
            GPIO.output(PIN_A if forward else PIN_B, GPIO.HIGH)
            forward = not forward
            flips += 1
            # Schedule from the start time rather than sleeping a fixed amount, so timing
            # errors don't pile up over a long run.
            next_flip += half_period
            time.sleep(max(0.0, next_flip - time.perf_counter()))
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.output(PIN_A, GPIO.LOW)
        GPIO.output(PIN_B, GPIO.LOW)
        # No GPIO.cleanup(): that leaves the pins floating, which the bridge may read as high.
    print(f"Stopped after {flips} flips ({flips / 2:g} cycles); both pins low.")


if __name__ == "__main__":
    main()
