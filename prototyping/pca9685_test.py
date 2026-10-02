"""
PCA9685 test: ramps PWM channels up and back down, one channel at a time, then pulses them all
together. By default it tests the eight ERMs on channels 0-7 (through the ULN2803).

The board is on I2C bus 1: SDA on header pin 3 (GPIO2), SCL on header pin 5 (GPIO3).
Check it's seen at 0x40 with:  i2cdetect -y 1

Usage: python3 pca9685_test.py                  (ERMs on channels 0-7, one ramp each, then all)
       python3 pca9685_test.py --channels 3 --cycles 3
       python3 pca9685_test.py --channels 0,2,4 --max 50
       python3 pca9685_test.py --hold 30         (all listed channels steady, e.g. to probe)
    --channels  which outputs: a range (0-7), a list (0,2,4) or one (3); default 0-7
    --freq      PWM frequency in Hz, 24-1526
    --ramp      seconds to go from off to --max, and the same back down
    --cycles    up-and-down ramps per channel; 0 runs the first channel until Ctrl+C
    --max       peak duty cycle, percent; default 75, HARDWARE.md's ERM cap
    --hold      instead of ramping, hold every listed channel at --max for this many seconds
                (0 = until Ctrl+C)

Every listed channel is switched fully off when it exits.

GPIO2 and GPIO3 are also ERM 0 and ERM 1 in stimulus_editor.py and pi_receiver.py, which turn
them into plain GPIO outputs; while that is so, I2C can't work. This script switches them back to
I2C, so close those programs first (and running them afterwards takes the pins back again).
"""

import argparse
import subprocess
import time

from smbus2 import SMBus

I2C_BUS = 1
ADDRESS = 0x40          # the default; solder jumpers A0-A5 on the board change it
SDA_GPIO, SCL_GPIO = 2, 3
OE_GPIO = 25            # the board's OE pin (HARDWARE.md): high disables every output
STEPS_PER_SECOND = 100  # duty-cycle updates per second during a ramp

# PCA9685 registers and bits.
MODE1, MODE2, PRESCALE = 0x00, 0x01, 0xFE
LED0_ON_L = 0x06        # each channel has 4 registers: ON_L, ON_H, OFF_L, OFF_H
RESTART, AUTO_INCREMENT, SLEEP = 0x80, 0x20, 0x10
OUTDRV = 0x04           # MODE2: totem-pole outputs (open-drain otherwise)
FULL = 0x10             # in ON_H / OFF_H: channel fully on / fully off
OSCILLATOR_HZ = 25_000_000


def ensure_i2c_pins():
    """Put GPIO2/3 back in I2C mode if something (e.g. the stimulus editor) made them GPIOs."""
    state = subprocess.run(["pinctrl", "get", f"{SDA_GPIO},{SCL_GPIO}"],
                           capture_output=True, text=True).stdout
    if state.count("= SDA1") + state.count("= SCL1") == 2:
        return
    print(f"GPIO{SDA_GPIO}/GPIO{SCL_GPIO} aren't in I2C mode:\n{state.rstrip()}\n"
          "Switching them back (stimulus_editor.py uses them as ERM 0 and 1; close it first).")
    subprocess.run(["pinctrl", "set", f"{SDA_GPIO},{SCL_GPIO}", "a0"], check=True)


def enable_outputs():
    """Drive OE low. The stimulus editor leaves it high on exit (every output disabled), so
    anything using the board afterwards has to switch the outputs back on."""
    subprocess.run(["pinctrl", "set", str(OE_GPIO), "op", "dl"], capture_output=True)


class PCA9685:
    def __init__(self, bus: SMBus, address: int = ADDRESS):
        self.bus = bus
        self.address = address
        enable_outputs()
        self.write(MODE2, OUTDRV)
        self.write(MODE1, AUTO_INCREMENT)   # awake, register auto-increment on
        time.sleep(0.0005)                  # the oscillator needs 500 us to start

    def write(self, register, value):
        self.bus.write_byte_data(self.address, register, value)

    def set_frequency(self, hz: float):
        prescale = round(OSCILLATOR_HZ / (4096 * hz)) - 1
        if not 3 <= prescale <= 255:
            raise ValueError(f"{hz} Hz is outside the PCA9685's 24-1526 Hz range")
        mode = self.bus.read_byte_data(self.address, MODE1)
        self.write(MODE1, (mode & ~RESTART) | SLEEP)   # the prescaler only takes writes asleep
        self.write(PRESCALE, prescale)
        self.write(MODE1, mode & ~SLEEP)
        time.sleep(0.0005)
        self.write(MODE1, (mode & ~SLEEP) | RESTART)   # resume the PWM channels
        return OSCILLATOR_HZ / (4096 * (prescale + 1))  # the frequency actually set

    def set_duty(self, channel: int, duty: float, phase: float = 0.0):
        """duty from 0 (fully off) to 1 (fully on). phase (0-1) delays where in each PWM period
        the channel goes high, e.g. so two loads don't draw their current pulses at once."""
        counts = round(max(0.0, min(1.0, duty)) * 4096)
        if counts <= 0:
            on, off = 0, FULL << 8          # full off
        elif counts >= 4096:
            on, off = FULL << 8, 0          # full on
        else:
            on = round(phase * 4096) % 4096     # high from count `on` for `counts` of each 4096
            off = (on + counts) % 4096
        self.bus.write_i2c_block_data(self.address, LED0_ON_L + 4 * channel,
                                      [on & 0xFF, on >> 8, off & 0xFF, off >> 8])


def parse_channels(text):
    """"0-5" -> [0..5], "0,2,4" -> [0, 2, 4], "3" -> [3]."""
    channels = []
    for part in text.split(","):
        first, _, last = part.strip().partition("-")
        channels += range(int(first), int(last or first) + 1)
    if not channels or not all(0 <= ch <= 15 for ch in channels):
        raise argparse.ArgumentTypeError(f"channels must be 0-15, got {text!r}")
    return list(dict.fromkeys(channels))


def spread(channels):
    """Each channel's PWM phase, spread evenly so their current pulses don't all coincide."""
    return {ch: i / len(channels) for i, ch in enumerate(channels)}


def ramp(pca, channel, peak, seconds, label):
    """0 -> peak -> 0 on one channel, timed from the start of each half so slow I2C writes
    don't stretch it."""
    steps = max(1, round(seconds * STEPS_PER_SECOND))
    for direction in ("up", "down"):
        start = time.perf_counter()
        for step in range(steps + 1):
            fraction = step / steps
            duty = peak * (fraction if direction == "up" else 1 - fraction)
            pca.set_duty(channel, duty)
            if step % (steps // 2 or 1) == 0:
                print(f"  {label} {direction:4} {duty * 100:5.1f}%")
            time.sleep(max(0.0, start + (step + 1) * seconds / steps - time.perf_counter()))


def hold(pca, channels, args, hz, peak):
    """Hold the channels steady at `peak`, e.g. to measure the outputs with a multimeter."""
    print(f"Channels {', '.join(map(str, channels))} held at {args.max:g}% ({hz:.0f} Hz) "
          + (f"for {args.hold:g} s" if args.hold else "until Ctrl+C") + ".\n"
          f"A DC meter on a channel's PWM pin should read about {3.3 * peak:.1f} V "
          "(the average, if the board's VCC is 3.3 V).")
    phases = spread(channels)
    for ch in channels:
        pca.set_duty(ch, peak, phases[ch])
    start = time.perf_counter()
    while not args.hold or time.perf_counter() - start < args.hold:
        time.sleep(0.1)


def main():
    parser = argparse.ArgumentParser(
        description="Ramp PCA9685 channels up and down one at a time, then pulse them together.")
    parser.add_argument("--channels", "--channel", type=parse_channels, default=parse_channels("0-7"),
                        help="a range (0-7), a list (0,2,4) or one channel (default 0-7)")
    parser.add_argument("--freq", type=float, default=150.0, help="PWM Hz (default 150)")
    parser.add_argument("--ramp", type=float, default=1.0,
                        help="seconds from off to peak, and back (default 1)")
    parser.add_argument("--cycles", type=int, default=1,
                        help="up-and-down ramps per channel (default 1); 0 for until Ctrl+C")
    parser.add_argument("--max", type=float, default=75.0,
                        help="peak duty cycle %% (default 75, the ERM cap)")
    parser.add_argument("--hold", type=float, default=None,
                        help="hold every channel at --max for this many seconds instead of "
                             "ramping (0 = until Ctrl+C)")
    parser.add_argument("--address", type=lambda s: int(s, 0), default=ADDRESS,
                        help=f"I2C address (default 0x{ADDRESS:02x})")
    args = parser.parse_args()
    if args.ramp <= 0 or not 0 < args.max <= 100:
        parser.error("need --ramp > 0 and 0 < --max <= 100")
    channels = args.channels

    ensure_i2c_pins()
    with SMBus(I2C_BUS) as bus:
        try:
            pca = PCA9685(bus, args.address)
        except OSError as e:
            raise SystemExit(f"No PCA9685 answering at 0x{args.address:02x} on I2C bus {I2C_BUS} "
                             f"({e}).\nCheck the wiring and power, and run: i2cdetect -y {I2C_BUS}\n"
                             "If the wiring is right, the bus may be too fast for it: set "
                             "i2c_arm_baudrate=100000 in /boot/firmware/config.txt and reboot.")
        hz = pca.set_frequency(args.freq)
        peak = args.max / 100
        try:
            for ch in channels:             # start from everything off
                pca.set_duty(ch, 0)
            if args.hold is not None:
                hold(pca, channels, args, hz, peak)
                return
            print(f"Channels {', '.join(map(str, channels))} at {hz:.0f} Hz, one at a time: "
                  f"0 -> {args.max:g}% -> 0 over {args.ramp:g} s each way, "
                  + (f"{args.cycles}x each" if args.cycles else "until Ctrl+C"))
            for ch in channels:
                print(f"\nchannel {ch}" + (f"  (ERM {ch + 1})" if ch <= 7 else ""))
                cycle = 0
                while not args.cycles or cycle < args.cycles:
                    cycle += 1
                    ramp(pca, ch, peak, args.ramp, f"ch{ch} ramp {cycle}")
                time.sleep(0.4)             # a gap, so each motor is felt separately
            if len(channels) > 1:
                print(f"\nall {len(channels)} together at {args.max:g}% for 2 s "
                      "(PWM pulses staggered across the cycle)")
                phases = spread(channels)
                for ch in channels:
                    pca.set_duty(ch, peak, phases[ch])
                time.sleep(2)
        except KeyboardInterrupt:
            pass
        finally:
            for ch in channels:
                pca.set_duty(ch, 0)
        print(f"\nDone; channels {', '.join(map(str, channels))} off.")


if __name__ == "__main__":
    main()
