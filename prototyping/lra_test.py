#!/usr/bin/env python3
"""
LRA test: plays test tones on the two LRAs through the PAM8406 amp.

Hardware (see HARDWARE.md):
  Pi 3.5 mm jack L / R -> PAM8406 L in / R in;  L+/L- -> LRA 1 (left),  R+/R- -> LRA 2 (right).
  GPIO24 -> PAM8406 SD (high = amp on, low = shutdown). This script turns the amp on while it
  plays and off when it exits.
  LRAs: Vybronics VG2230001H, resonant at ~70 Hz. Target ~2.0 Vrms at their terminals.

Tests (default: all, in this order):
  channels   left only, right only, both: checks each LRA and that L/R aren't swapped
  alternate  left/right/left/right: checks the two sides are independent (no crosstalk)
  ramp       both LRAs fading in and out: checks the level control
  sweep      40 -> 140 Hz on both: the frequency where it feels strongest is the resonance

Usage:
  python3 lra_test.py                     # all tests at 70 Hz, amplitude 0.3
  python3 lra_test.py sweep --amp 0.5
  python3 lra_test.py --hold 20           # steady 70 Hz on both, e.g. to measure Vrms with a meter

Start with a low --amp and raise it: amplitude 1 is the loudest the jack plays, and the amp's
gain on top of that can overdrive an LRA. Tones fade in and out, as abrupt starts click.

Close the stimulus editor first: it keeps the headphone jack open while it runs.
"""

import argparse
import subprocess
import threading
import time

import numpy as np
import RPi.GPIO as GPIO

AUDIO_DEVICE = "plughw:CARD=Headphones,DEV=0"
RATE = 48000            # samples per second
AMP_SD_PIN = 24         # PAM8406 SD: high = on
AMP_WAKE_S = 0.1        # the amp needs a moment after SD goes high
FADE_S = 0.02           # fade in/out on every tone, so they don't click
GAP_S = 0.6             # silence between tones
RESONANCE_HZ = 70.0


# ---------------------------------------------------------------- sound


def envelope(n, fade=FADE_S):
    """1 in the middle, easing from and to 0 over `fade` seconds (raised cosine)."""
    env = np.ones(n)
    k = min(int(fade * RATE), n // 2)
    if k:
        ramp = 0.5 - 0.5 * np.cos(np.linspace(0, np.pi, k))
        env[:k] = ramp
        env[-k:] = ramp[::-1]
    return env


def tone(freq, seconds, amp, left=True, right=True):
    """A stereo sine at `freq` Hz, on the chosen sides."""
    t = np.arange(int(seconds * RATE)) / RATE
    wave = amp * np.sin(2 * np.pi * freq * t) * envelope(len(t))
    return np.column_stack([wave * left, wave * right])


def sweep(f_start, f_end, seconds, amp):
    """Both sides gliding from f_start to f_end Hz (exponentially, so each octave takes the same
    time). Returns the samples and a function giving the frequency at a time."""
    n = int(seconds * RATE)
    t = np.arange(n) / RATE
    k = np.log(f_end / f_start) / seconds
    phase = 2 * np.pi * f_start * (np.exp(k * t) - 1) / k
    wave = amp * np.sin(phase) * envelope(n)
    return np.column_stack([wave, wave]), lambda s: f_start * np.exp(k * s)


def silence(seconds):
    return np.zeros((int(seconds * RATE), 2))


def _aplay():
    return subprocess.Popen(["aplay", "-q", "-D", AUDIO_DEVICE, "-t", "raw", "-f", "S16_LE",
                             "-r", str(RATE), "-c", "2"],
                            stdin=subprocess.PIPE, stderr=subprocess.PIPE)


def _to_bytes(samples):
    return (np.clip(samples, -1, 1) * 32767).astype("<i2").tobytes()


def _finish(process):
    if process.wait() != 0:
        error = process.stderr.read().decode(errors="replace").strip()
        raise SystemExit(f"aplay failed: {error}\n"
                         "Is the stimulus editor (or anything else) holding the headphone jack?")


def play(samples, marks=()):
    """Play stereo samples (-1..1), printing each (time, text) in `marks` as it's reached."""
    process = _aplay()

    def feed():
        try:
            process.stdin.write(_to_bytes(samples))
            process.stdin.close()
        except (BrokenPipeError, ValueError):
            pass

    feeder = threading.Thread(target=feed, daemon=True)
    start = time.perf_counter()
    feeder.start()
    for at, text in marks:              # roughly on time (the audio lags ~20-40 ms)
        time.sleep(max(0.0, start + at - time.perf_counter()))
        print(text, flush=True)
    feeder.join()
    _finish(process)


# ---------------------------------------------------------------- tests


def test_channels(freq, amp):
    print(f"\n--- channels: {freq:g} Hz at {amp:g}, 1 s each ---")
    for label, left, right in [("LEFT only  (LRA 1)", True, False),
                               ("RIGHT only (LRA 2)", False, True),
                               ("BOTH", True, True)]:
        play(np.concatenate([tone(freq, 1.0, amp, left, right), silence(GAP_S)]),
             [(0, f"  {label}")])


def test_alternate(freq, amp):
    print(f"\n--- alternate: left/right, 0.25 s each, 4 times ---")
    print("  each LRA should buzz only on its own turn")
    beats = []
    for _ in range(4):
        beats += [tone(freq, 0.25, amp, True, False), tone(freq, 0.25, amp, False, True)]
    play(np.concatenate(beats + [silence(GAP_S)]))


def test_ramp(freq, amp):
    print(f"\n--- ramp: both, 0 -> {amp:g} -> 0 over 4 s ---")
    n = int(4 * RATE)
    level = np.concatenate([np.linspace(0, 1, n // 2), np.linspace(1, 0, n - n // 2)])
    samples = tone(freq, 4, amp) * level[:, None]
    play(np.concatenate([samples, silence(GAP_S)]),
         [(0, "  rising ..."), (2, "  peak, falling ...")])


def test_sweep(amp, f_start=40.0, f_end=140.0, seconds=8.0):
    print(f"\n--- sweep: both, {f_start:g} -> {f_end:g} Hz over {seconds:g} s ---")
    print("  note the frequency where it feels strongest: that's the resonance")
    samples, freq_at = sweep(f_start, f_end, seconds, amp)
    marks = [(s, f"  {freq_at(s):5.0f} Hz") for s in np.arange(0, seconds, 0.5)]
    play(np.concatenate([samples, silence(GAP_S)]), marks)


def hold(freq, amp, seconds):
    """A steady tone on both LRAs, e.g. to measure the voltage across them with a meter."""
    print(f"\n--- hold: both at {freq:g} Hz, amplitude {amp:g}, "
          + (f"{seconds:g} s" if seconds else "until Ctrl+C") + " ---")
    print("  measure AC volts across each LRA's terminals (target ~2.0 Vrms), and adjust --amp\n"
          "  to hit it: that amplitude is then the LRAs' ceiling.")
    process = _aplay()
    step = 2 * np.pi * freq / RATE
    sample = 0

    def block(level_from, level_to):
        """1 s of the tone, carrying on from the last sample so the wave never jumps."""
        nonlocal sample
        n = np.arange(sample, sample + RATE)
        sample += RATE
        wave = amp * np.sin(step * n) * np.linspace(level_from, level_to, RATE)
        return _to_bytes(np.column_stack([wave, wave]))

    start = time.perf_counter()
    try:
        process.stdin.write(block(0, 1))                    # 1 s fade in
        while not seconds or time.perf_counter() - start < seconds:
            process.stdin.write(block(1, 1))                # blocks while aplay has enough
    except KeyboardInterrupt:
        pass
    finally:
        try:
            process.stdin.write(block(1, 0))                # 1 s fade out
            process.stdin.close()
        except (BrokenPipeError, ValueError):
            pass
    _finish(process)


# ---------------------------------------------------------------- main

TESTS = {"channels": test_channels, "alternate": test_alternate, "ramp": test_ramp,
         "sweep": test_sweep}


def mixer_level():
    """The headphone jack's PCM volume, as amixer reports it."""
    try:
        out = subprocess.run(["amixer", "-c", "Headphones", "sget", "PCM"], capture_output=True,
                             text=True).stdout
        return out.strip().splitlines()[-1].split(":", 1)[1].strip()
    except (OSError, IndexError):
        return "unknown"


def main():
    parser = argparse.ArgumentParser(description="Test the two LRAs through the PAM8406 amp.")
    parser.add_argument("tests", nargs="*", metavar="test",
                        help=f"any of: {', '.join(TESTS)} (default: all)")
    parser.add_argument("--freq", type=float, default=RESONANCE_HZ,
                        help=f"tone frequency in Hz (default {RESONANCE_HZ:g})")
    parser.add_argument("--amp", type=float, default=0.3,
                        help="amplitude, 0-1 of the jack's full scale (default 0.3)")
    parser.add_argument("--hold", type=float, default=None, metavar="SECONDS",
                        help="just play a steady tone on both LRAs (0 = until Ctrl+C)")
    args = parser.parse_args()
    if not 0 < args.amp <= 1 or not 0 < args.freq < RATE / 2:
        parser.error("need 0 < --amp <= 1 and a sensible --freq")
    unknown = [name for name in args.tests if name not in TESTS]
    if unknown:
        parser.error(f"unknown test {', '.join(unknown)}; choose from {', '.join(TESTS)}")

    print(f"Headphone jack volume: {mixer_level()}"
          "  (above 0 dB, loud tones clip: `amixer -c Headphones sset PCM 0dB`)")
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(AMP_SD_PIN, GPIO.OUT, initial=GPIO.HIGH)    # amp on
    time.sleep(AMP_WAKE_S)
    try:
        if args.hold is not None:
            hold(args.freq, args.amp, args.hold)
        else:
            for name in args.tests or TESTS:
                if name == "sweep":
                    test_sweep(args.amp)
                else:
                    TESTS[name](args.freq, args.amp)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        GPIO.output(AMP_SD_PIN, GPIO.LOW)                   # amp off
        GPIO.cleanup(AMP_SD_PIN)
    print("Done; amp shut down.")


if __name__ == "__main__":
    main()
