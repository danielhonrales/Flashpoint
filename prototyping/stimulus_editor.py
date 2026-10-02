"""
Stimulus editor: a GUI for timing the wearable's ERM vibration motors, LRAs and Peltiers from a
Raspberry Pi (see HARDWARE.md for the wiring).

A stimulus is a set of activation periods, all timed in seconds from the start of the stimulus:

  ERM periods     - which ERMs turn on, when, and for how long. One period can drive several ERMs,
                    and periods may overlap: an ERM stays on while any period containing it is
                    active, at the intensity (share of ERM_DUTY) of the one that started last.
                    By default the editor adds ERMs as a Tactile Brush stroke: one period per ERM,
                    in the order they were ticked, with onsets spaced for apparent motion.
  Peltier periods - which way a Peltier pumps heat (heating or cooling the skin face), how hard
                    (duty) and for how long. Each Peltier has its own periods. If periods on the
                    same Peltier overlap, the one that started most recently wins; when it ends
                    the Peltier falls back to whichever of its periods is still active, else off.
                    Periods starting at 0 can begin a lead-in earlier, as Peltiers are slow.
  LRA periods     - when an LRA vibrates, at what frequency and amplitude. The Bottom LRA (near
                    the elbow) and Top LRA (near the wrist) are driven by the PAM8406 amp from the
                    Pi's headphone jack (left and right channel), each playing a sine wave while on. Each LRA has its own periods;
                    overlapping ones resolve like Peltier periods, and with none active it's silent.

Stimuli are saved as JSON.   Usage: python3 stimulus_editor.py [stimulus.json]

Editing on the timeline: click a bar to select its period, Ctrl+click to add or remove one, or
drag a box around several. Drag a selection to move it, or a bar's left or right end to change
its start or duration (dragged edges snap to the grid and other periods' ends; hold Shift for
1 ms steps, press Esc to cancel). Ctrl+D disables the selected periods, or enables them again:
a disabled period stays in the stimulus, hatched on the timeline, but doesn't play. Ctrl+Z undoes
an edit, Ctrl+Y (or Ctrl+Shift+Z) redoes it.

Click an actuator's name on the timeline to mute it: its periods are left out whenever a stimulus
plays, until it is unmuted. Ctrl+click a name to solo that actuator, and again to unmute all.
Mutes belong to the editor session, not the stimulus, so they aren't saved. Right-click to move or copy the selection to another actuator,
with the same timing: an LRA or Peltier period made from a different kind takes its frequency and
amplitude, or duty and direction, from that actuator's tab.

To compare two stimuli, load one into each Compare slot on the toolbar (A and B) and play them
with their buttons or F6/F7. Either one stops whatever is playing and starts straight away. A slot
holding the stimulus open in the editor plays it as edited, saved or not; any other slot re-reads
its file each time, so saved changes are picked up.

Needs Tkinter, RPi.GPIO, smbus2 and numpy on the Pi, aplay (alsa-utils) for the LRAs, and
pca9685_test.py (the PCA9685 driver) in this folder:
    sudo apt install python3-tk python3-rpi.gpio python3-smbus2 python3-numpy alsa-utils

Safety: there's no thermistor cut-off yet, so Peltier duty is capped at PELTIER_MAX_DUTY and a
Peltier won't run longer without a break than PELTIER_MAX_HEAT_S when only heating, or
PELTIER_MAX_COOL_S when cooling at all. On exit every output is switched
off, the Peltier drivers are put to sleep and the PCA9685's outputs are disabled (OE high).
Any device that can't be opened (library missing, port not found, DRY_RUN set) is simulated
instead, and the editor shows a red SIMULATED banner naming it.
"""

import copy
import functools
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import tkinter as tk
from collections import defaultdict
from dataclasses import asdict, dataclass, field
from tkinter import filedialog, messagebox, ttk

import numpy as np

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

# PCA9685 PWM driver on I2C bus 1 (header pins 3 and 5). One frequency for all 16 channels.
PCA_ADDRESS = 0x40
PWM_FREQ = 150              # Hz: low, so the ERMs' PWM hum blends into their buzz (HARDWARE.md §4)
PCA_OE_PIN = 25             # BCM GPIO to the PCA9685's OE: high switches every PWM output off

# ERMs (bHaptics) on PCA9685 channels, switched low-side by the ULN2803.
ERM_CHANNELS = [0, 1, 2, 3, 4, 5, 6, 7]
# Where each ERM sits on the forearm (HARDWARE.md §10), with its distance from the wrist: an outer
# column (with the LRAs) and two inner columns, inner-left on the thumb side.
ERM_PLACES = ["outer 4cm", "outer 12cm", "in-L 0cm", "in-L 8cm", "in-L 16cm",
              "in-R 0cm", "in-R 8cm", "in-R 16cm"]

ERM_DUTY = 0.75             # duty while on: HARDWARE.md's cap, rated V / 5 V (verify for bHaptics)

# Peltiers (TEC1-12704), each through a DRV8876 in PH/EN mode: EN = duty on a PCA9685 channel,
# PH = direction and nSLEEP on BCM GPIOs. heat_ph is the PH level that heats the skin face
# (it depends on how the leads are wired; verify per module). A Peltier whose pins are None is
# simulated until they're filled in.
PELTIERS = [
    {"name": "Peltier 1", "channel": 14, "ph_pin": 17, "nsleep_pin": 27, "heat_ph": 0,
     "place": "outer 8cm"},
    {"name": "Peltier 2", "channel": 15, "ph_pin": 22, "nsleep_pin": 10, "heat_ph": 0,
     "place": "inner 8cm"},
]
PELTIER_MAX_DUTY = 1.0      # EN duty ceiling. Above ~0.66 the DRV8876 passes more than its 1.3 A
                            # continuous rating (and may hit thermal shutdown on long periods), and
                            # two Peltiers at once exceed the bus's 2.5 A fuse (HARDWARE.md §6).
# Longest a Peltier may run without a break, as there's no thermistor cut-off yet: while only
# heating, and while cooling at all (a run that mixes both gets the cooling limit).
PELTIER_MAX_HEAT_S = 20.0
PELTIER_MAX_COOL_S = 10.0


def peltier_limit(heat: bool) -> float:
    """The longest a Peltier may run without a break while heating (True) or cooling (False)."""
    return PELTIER_MAX_HEAT_S if heat else PELTIER_MAX_COOL_S
DRIVER_WAKE_S = 0.002       # the DRV8876 needs ~1 ms after nSLEEP goes high

# The LRAs' audio: the headphone jack's left and right channel, played through aplay.
AUDIO_DEVICE = "plughw:CARD=Headphones,DEV=0"   # an ALSA device, as `aplay -L` lists them
AUDIO_RATE = 48000          # samples per second
AUDIO_CHUNK = 240           # samples made at a time (5 ms)
AUDIO_BUFFER_TIME = 0.02    # seconds aplay buffers; shorter reacts sooner, longer risks dropouts
AUDIO_LATENCY = 0.04        # seconds from an LRA change to it being heard; stimuli make LRA changes
                            # this early. Measured on the Pi 4 jack with the settings above (±10 ms).
LRA_MAX_FREQUENCY = 1000.0  # Hz
LRA_RAMP_TIME = 0.002       # seconds an LRA's amplitude takes to change, so the wave never jumps (a click)
# Strength calibration per LRA (Bottom, Top): amplitudes are multiplied by this before playing.
# The Top LRA is 50% stronger than the Bottom one, so it's scaled to 1/1.5 to even them out.
LRA_GAINS = [1.0, 1 / 1.5]
AMP_SD_PIN = 24             # BCM GPIO to the PAM8406's SD: high = amp on, low = shut down
AMP_IDLE_OFF_S = 1.0        # the amp is shut down once the LRAs have been silent this long

DRY_RUN = False             # True: simulate every device, even on the Pi

# ----------------------------------------------------------------------------
# Hardware
# ----------------------------------------------------------------------------

class _NullDevice:
    """Stands in for a GPIO pin that is simulated."""

    def on(self): pass
    def off(self): pass
    def close(self): pass


class _GpioPin:
    """An RPi.GPIO output pin, driven low from the moment it is set up."""

    def __init__(self, pin: int):
        import RPi.GPIO as GPIO
        self._gpio = GPIO
        self.pin = pin
        GPIO.setwarnings(False)     # pins left as outputs by a previous run are expected
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)

    def on(self):
        self._gpio.output(self.pin, self._gpio.HIGH)

    def off(self):
        self._gpio.output(self.pin, self._gpio.LOW)

    def close(self):
        pass    # left as an output at its last level; see PwmBoard.close() for OE


class PwmBoard:
    """The PCA9685, shared by the ERMs and Peltiers. Its I2C writes are locked, as several
    playback threads use it at once."""

    def __init__(self, address: int = PCA_ADDRESS, frequency: float = PWM_FREQ,
                 oe_pin: int = PCA_OE_PIN):
        self.sim_reason = "DRY_RUN is set" if DRY_RUN else None
        self.frequency = frequency
        self._lock = threading.Lock()
        self._pca = self._bus = None
        self._oe = _NullDevice()
        if DRY_RUN:
            return
        try:
            from smbus2 import SMBus
            from pca9685_test import I2C_BUS, PCA9685, ensure_i2c_pins
            ensure_i2c_pins()           # a GPIO user may have left SDA/SCL as plain pins
            self._bus = SMBus(I2C_BUS)
            self._pca = PCA9685(self._bus, address)
            self.frequency = self._pca.set_frequency(frequency)
        except Exception as e:
            self.sim_reason = f"PCA9685: {type(e).__name__}: {e}"
            self._pca = None
            if self._bus is not None:
                self._bus.close()
            return
        try:
            self._oe = _GpioPin(oe_pin)     # low: outputs enabled
        except Exception:
            pass                            # OE isn't needed to drive the outputs

    def set_duty(self, channel: int, duty: float, phase: float = 0.0):
        if self._pca is not None:
            with self._lock:
                self._pca.set_duty(channel, duty, phase)

    def close(self):
        """Disable every output (OE high) and let go of the bus."""
        self._oe.on()
        if self._bus is not None:
            self._bus.close()


class ERM:
    """An ERM on a PCA9685 channel, switched low-side by the ULN2803: on = PWM at `duty`.
    `phase` offsets its pulses within the PWM cycle, so the ERMs don't all draw at once."""

    def __init__(self, board: PwmBoard, channel: int, duty: float = ERM_DUTY, phase: float = 0.0,
                 place: str = ""):
        self.board = board
        self.channel = channel
        self.place = place      # where it sits on the arm, for labels
        self.duty = duty        # duty at full intensity
        self.phase = phase
        self.level = 0.0        # present intensity, 0-1 of `duty`; 0 while off

    @property
    def sim_reason(self):
        return self.board.sim_reason

    @property
    def is_on(self):
        return self.level > 0

    def on(self, level: float = 1.0):
        """Run at `level` (0-1) of full intensity."""
        self.board.set_duty(self.channel, self.duty * level, self.phase)
        self.level = level

    def off(self):
        self.board.set_duty(self.channel, 0)
        self.level = 0.0

    def close(self):
        self.off()

    def __str__(self):
        return f"{self.place} · ch {self.channel}" if self.place else f"ch {self.channel}"


class Peltier:
    """A Peltier through a DRV8876 in PH/EN mode: EN (duty, i.e. power) on a PCA9685 channel,
    PH (direction) and nSLEEP on GPIO. The driver sleeps, outputs off, while the Peltier is off,
    and PH only changes while EN is off, so the module is never reversed under load."""

    def __init__(self, board: PwmBoard, name: str, channel: int, ph_pin: int | None,
                 nsleep_pin: int | None, heat_ph: int = 1, max_duty: float = PELTIER_MAX_DUTY,
                 phase: float = 0.0, place: str = ""):
        self.board = board
        self.name = name
        self.place = place      # where it sits on the arm, for labels
        self.channel = channel
        self.ph_pin, self.nsleep_pin = ph_pin, nsleep_pin
        self.heat_ph = heat_ph
        self.max_duty = max_duty
        self.phase = phase
        self.duty = 0.0         # present duty; 0 while off
        self.heat = None        # present direction while on: True heating, False cooling
        self._lock = threading.Lock()
        self._ph = self._nsleep = _NullDevice()
        self._reason = None
        if ph_pin is None or nsleep_pin is None:
            self._reason = "its PH / nSLEEP pins aren't set in PELTIERS"
        elif DRY_RUN:
            self._reason = "DRY_RUN is set"
        else:
            try:
                self._nsleep = _GpioPin(nsleep_pin)     # low: asleep
                self._ph = _GpioPin(ph_pin)
            except Exception as e:
                self._reason = f"{type(e).__name__}: {e}"

    @property
    def sim_reason(self):
        return self._reason or self.board.sim_reason

    @property
    def is_on(self):
        return self.duty > 0

    def check(self, duty: float, heat: bool):
        if not 0 < duty <= self.max_duty:
            raise ValueError(f"{self.name}: duty {duty:.0%} is outside 0-{self.max_duty:.0%}")
        if not isinstance(heat, bool):
            raise ValueError(f"{self.name}: heat must be true or false, got {heat!r}")

    def on(self, duty: float, heat: bool):
        """Heat (or cool) the skin face at `duty`, switching over from whatever it was doing."""
        self.check(duty, heat)
        with self._lock:
            if self._reason is None:    # a Peltier whose pins aren't known isn't driven at all
                if self.duty > 0 and heat != self.heat:
                    self.board.set_duty(self.channel, 0)    # off before reversing
                (self._ph.on if (self.heat_ph if heat else 1 - self.heat_ph) else self._ph.off)()
                if self.duty == 0:
                    self._nsleep.on()
                    time.sleep(DRIVER_WAKE_S)
                self.board.set_duty(self.channel, duty, self.phase)
            self.duty, self.heat = duty, heat

    def off(self):
        with self._lock:
            if self._reason is None:
                self.board.set_duty(self.channel, 0)
                self._nsleep.off()
                self._ph.off()
            self.duty, self.heat = 0.0, None

    def close(self):
        self.off()

    def __str__(self):
        return f"{self.place} · ch {self.channel}" if self.place else f"ch {self.channel}"


class AudioOutput:
    """A stereo stream of sine waves to an ALSA device, one per channel, each with its own
    frequency and amplitude (silent at amplitude 0). It plays from when it is opened until it is
    closed, through an aplay process fed by a background thread.

    Changes are heard about AUDIO_LATENCY seconds after they are made: the audio already queued
    in the pipe to aplay and in aplay's buffer plays out first.

    The amp (PAM8406 SD on AMP_SD_PIN) is switched on while any channel plays, or while a
    stimulus holds it on with hold(), and shut down once everything has been silent for
    AMP_IDLE_OFF_S.
    """

    CHANNELS = 2
    PIPE_SIZE = 4096    # bytes: the smallest pipe Linux allows, to keep the queue short

    def __init__(self, device: str = AUDIO_DEVICE, rate: int = AUDIO_RATE):
        self.rate = rate
        self.sim_reason = "DRY_RUN is set" if DRY_RUN else None
        self._settings = [(0.0, 0.0)] * self.CHANNELS    # (frequency, amplitude) per channel
        self._holds = 0             # hold() calls keeping the amp on
        self._amp = _NullDevice()
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._process = None
        self._thread = None
        if DRY_RUN:
            return
        try:
            self._process = subprocess.Popen(
                ["aplay", "-q", "-D", device, "-t", "raw", "-f", "S16_LE", "-r", str(rate),
                 "-c", str(self.CHANNELS), f"--buffer-time={round(AUDIO_BUFFER_TIME * 1e6)}"],
                stdin=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
            import fcntl
            fcntl.fcntl(self._process.stdin, fcntl.F_SETPIPE_SZ, self.PIPE_SIZE)
            self._process.wait(timeout=0.3)     # aplay quits at once if it can't open the device
            self.sim_reason = "aplay: " + self._process.stderr.read().decode(errors="replace").strip()
            self._process = None
        except subprocess.TimeoutExpired:
            try:
                self._amp = _GpioPin(AMP_SD_PIN)    # low: amp shut down
            except Exception:
                pass                                # an amp with SD tied on still plays
            self._thread = threading.Thread(target=self._stream, daemon=True)
            self._thread.start()
        except Exception as e:
            self.sim_reason = f"{type(e).__name__}: {e}"
            self._process = None

    def set(self, channel: int, frequency: float | None, amplitude: float):
        """Play `frequency` Hz at `amplitude` (0-1 of full scale) on a channel. A frequency of
        None keeps the present one, so the wave fades out smoothly when the amplitude drops to 0."""
        with self._lock:
            if frequency is None:
                frequency = self._settings[channel][0]
            self._settings[channel] = (frequency, amplitude)

    def hold(self, on: bool):
        """Keep the amp on (e.g. for a whole stimulus, so it's awake before the first LRA
        period), or release a previous hold."""
        with self._lock:
            self._holds += 1 if on else -1

    def _stream(self):
        n = AUDIO_CHUNK
        steps = np.arange(n)
        ramp = np.arange(1, n + 1) / (LRA_RAMP_TIME * self.rate)   # amplitude change by each sample
        phase = np.zeros(self.CHANNELS)
        amplitude = np.zeros(self.CHANNELS)
        samples = np.empty((n, self.CHANNELS))
        amp_on, last_sound = False, 0.0
        try:
            while not self._closed.is_set():
                with self._lock:
                    settings = list(self._settings)
                    held = self._holds > 0
                now = time.perf_counter()
                if held or amplitude.max() > 0 or any(target > 0 for _, target in settings):
                    last_sound = now
                    if not amp_on:
                        self._amp.on()
                        amp_on = True
                elif amp_on and now - last_sound > AMP_IDLE_OFF_S:
                    self._amp.off()
                    amp_on = False
                for ch, (frequency, target) in enumerate(settings):
                    # Phase carries on across chunks and frequency changes, so the wave is unbroken.
                    step = 2 * math.pi * frequency / self.rate
                    change = target - amplitude[ch]
                    envelope = amplitude[ch] + math.copysign(1, change) * np.minimum(ramp, abs(change))
                    samples[:, ch] = envelope * np.sin(phase[ch] + step * steps)
                    phase[ch] = (phase[ch] + step * n) % (2 * math.pi)
                    amplitude[ch] = envelope[-1]
                # Blocks while the pipe is full, which paces this loop to the sound card.
                self._process.stdin.write((samples * 32767).astype("<i2").tobytes())
        except (OSError, ValueError) as e:
            if not self._closed.is_set():
                print(f"Audio output stopped: {e}")

    def close(self):
        """Fade out and stop. Safe to call more than once."""
        if self._closed.is_set():
            return
        for channel in range(self.CHANNELS):
            self.set(channel, None, 0.0)
        if self._thread is not None:
            time.sleep(LRA_RAMP_TIME + AUDIO_CHUNK / self.rate)     # let the fade be made
        self._closed.set()
        self._amp.off()
        if self._process is not None:
            try:
                self._process.stdin.close()     # aplay plays what it has, then quits
            except OSError:
                pass
            if self._thread is not None:
                self._thread.join(timeout=1)
            try:
                self._process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self._process.kill()


class LRA:
    """A linear resonant actuator on one channel of an AudioOutput, through an audio amp. While on
    it plays a sine wave, which drives it hardest at its resonant frequency."""

    def __init__(self, output: AudioOutput, channel: int, name: str,
                 max_frequency: float = LRA_MAX_FREQUENCY, gain: float = 1.0):
        self.output = output
        self.channel = channel
        self.gain = gain        # strength calibration: amplitudes are scaled by this
        self.name = name
        self.max_frequency = max_frequency
        self.frequency = None   # while on, Hz; None while off
        self.amplitude = None   # while on, 0-1 of full scale; None while off

    @property
    def sim_reason(self):
        return self.output.sim_reason

    @property
    def is_on(self):
        return self.frequency is not None

    def check(self, frequency: float, amplitude: float):
        if not 0 < frequency <= self.max_frequency:
            raise ValueError(f"{self.name} LRA: {frequency} Hz is outside 0-{self.max_frequency} Hz")
        if not 0 < amplitude <= 1:
            raise ValueError(f"{self.name} LRA: amplitude {amplitude} is outside 0-1")

    def on(self, frequency: float, amplitude: float):
        """Play `frequency` Hz at `amplitude`, or switch to them if already on."""
        self.check(frequency, amplitude)
        self.output.set(self.channel, frequency, min(1.0, amplitude * self.gain))
        self.frequency, self.amplitude = frequency, amplitude

    def off(self):
        self.output.set(self.channel, None, 0.0)
        self.frequency = self.amplitude = None

    def close(self):
        self.off()
        self.output.close()

    def __str__(self):
        return f"jack {'L' if self.channel == 0 else 'R'}"


# ----------------------------------------------------------------------------
# Stimulus definition
# ----------------------------------------------------------------------------
# Times are rounded to the microsecond so back-to-back periods (e.g. one ending at 0.1 + 0.2
# and the next starting at 0.3) meet exactly instead of producing a spurious off/on.

def _check_period(start, duration):
    if start < 0 or duration <= 0:
        raise ValueError(f"need start >= 0 and duration > 0, got start={start}, duration={duration}")


@dataclass
class ErmActivation:
    erms: list[int]     # indices into the controller's ERM list
    start: float        # seconds from stimulus start
    duration: float     # seconds
    enabled: bool = True    # False: kept in the stimulus but not played
    intensity: float = 1.0  # 0-1 of full intensity (ERM_DUTY)

    def __post_init__(self):
        _check_period(self.start, self.duration)
        if not self.erms:
            raise ValueError("an ERM period needs at least one ERM")
        if not 0 < self.intensity <= 1:
            raise ValueError(f"ERM intensity must be 0-1, got {self.intensity}")
        self.start = round(self.start, 6)

    @property
    def end(self):
        return round(self.start + self.duration, 6)


@dataclass
class PeltierActivation:
    peltier: int        # index into the controller's Peltier list
    start: float        # seconds from stimulus start
    duration: float     # seconds
    duty: float         # EN duty, 0-PELTIER_MAX_DUTY: how hard it pumps
    heat: bool = True   # True heats the skin face, False cools it
    enabled: bool = True

    def __post_init__(self):
        _check_period(self.start, self.duration)
        self.start = round(self.start, 6)

    @property
    def end(self):
        return round(self.start + self.duration, 6)


@dataclass
class LraActivation:
    lra: int            # index into the controller's LRA list
    start: float        # seconds from stimulus start
    duration: float     # seconds
    frequency: float    # Hz
    amplitude: float    # 0-1 of full scale
    enabled: bool = True

    def __post_init__(self):
        _check_period(self.start, self.duration)
        self.start = round(self.start, 6)

    @property
    def end(self):
        return round(self.start + self.duration, 6)


@dataclass
class Stimulus:
    erm_activations: list[ErmActivation] = field(default_factory=list)
    peltier_activations: list[PeltierActivation] = field(default_factory=list)
    peltier_warmup: float = 0.0     # seconds: Peltier periods starting at 0 begin this long before it
    lra_activations: list[LraActivation] = field(default_factory=list)

    def __post_init__(self):
        if not self.peltier_warmup >= 0:
            raise ValueError(f"Peltier lead-in must be >= 0, got {self.peltier_warmup}")

    @property
    def duration(self):
        return max((p.end for p in self.periods), default=0.0)

    @property
    def periods(self):
        return [*self.erm_activations, *self.peltier_activations, *self.lra_activations]

    def enabled_only(self):
        """The stimulus as it plays: without its disabled periods."""
        return Stimulus([p for p in self.erm_activations if p.enabled],
                        [p for p in self.peltier_activations if p.enabled], self.peltier_warmup,
                        [p for p in self.lra_activations if p.enabled])

    def save(self, path):
        data = asdict(self)
        for key in ("erm_activations", "peltier_activations", "lra_activations"):
            for period in data[key]:
                if period["enabled"]:
                    del period["enabled"]   # only disabled periods say so, to keep files tidy
                if period.get("intensity") == 1.0:
                    del period["intensity"]     # likewise only ERM periods below full
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        if data.get("psu_activations"):
            raise ValueError("it was made for the old programmable power supplies "
                             "(psu_activations); its thermal periods need redoing as Peltier periods")
        # Refuse anything else (e.g. periods for an actuator this editor no longer has), rather
        # than drop it silently and lose it at the next save. Empty leftovers are fine.
        known = {"erm_activations", "peltier_activations", "peltier_warmup", "lra_activations"}
        unknown = [key for key, value in data.items()
                   if key not in known and value not in ([], None, 0)]
        if unknown:
            raise ValueError(f"it has {', '.join(sorted(unknown))}, which this editor doesn't support")
        return cls(erm_activations=[ErmActivation(**p) for p in data.get("erm_activations", [])],
                   peltier_activations=[PeltierActivation(**p)
                                        for p in data.get("peltier_activations", [])],
                   peltier_warmup=data.get("peltier_warmup", 0.0),
                   lra_activations=[LraActivation(**p) for p in data.get("lra_activations", [])])


# Tactile Brush (Israr & Poupyrev, CHI 2011): vibrating actuators one after another in bursts
# of duration d, with onsets SOA apart, feels like one continuous stroke rather than separate
# taps when SOA = 0.32 * d + 47.3 ms.
TB_SOA_SLOPE = 0.32
TB_SOA_INTERCEPT = 0.0473   # seconds


def tactile_brush_soa(duration: float) -> float:
    """Stimulus onset asynchrony (seconds) for apparent motion with bursts of `duration` seconds."""
    return TB_SOA_SLOPE * duration + TB_SOA_INTERCEPT


def tactile_brush_stroke(erms: list[int], start: float, duration: float,
                         intensity: float = 1.0) -> list[ErmActivation]:
    """One period per ERM, in the given order, each `duration` long and one SOA after the last."""
    soa = tactile_brush_soa(duration)
    return [ErmActivation([erm], start + k * soa, duration, intensity=intensity)
            for k, erm in enumerate(erms)]


def _erm_changes(periods: list[ErmActivation], n_erms: int):
    """Resolve overlapping ERM periods into [(time, erm_index, intensity or None), ...]. An ERM
    runs while any period containing it is active, at the intensity of the latest started."""
    changes = []
    for i in range(n_erms):
        spans = [(p.start, p.end, p.intensity) for p in periods if i in p.erms]
        changes += [(t, i, level) for t, level in _latest_wins(spans, None)]
    return sorted(changes, key=lambda change: change[0])


def _latest_wins(spans, idle):
    """Resolve overlapping (start, end, value) spans into [(time, value), ...]: the most recently
    started active span wins, and with none active the value is `idle`."""
    spans = sorted(spans, key=lambda span: span[0])     # stable: equal starts keep definition order
    changes = []
    value = idle
    for t in sorted({start for start, _, _ in spans} | {end for _, end, _ in spans}):
        active = [v for start, end, v in spans if start <= t < end]
        target = active[-1] if active else idle
        if target != value:
            changes.append((t, target))
            value = target
    return changes


def _shifted(periods, warmup):
    """(start, end) of each period, with those starting at 0 moved `warmup` seconds earlier."""
    return [(-warmup if p.start == 0 else p.start, p.end) for p in periods]


def _peltier_changes(periods: list[PeltierActivation], warmup: float = 0.0):
    """Resolve one Peltier's (possibly overlapping) periods into [(time, (duty, heat) or None), ...].
    Periods starting at 0 begin `warmup` seconds early, so times can be negative."""
    return _latest_wins([(start, end, (p.duty, p.heat))
                         for (start, end), p in zip(_shifted(periods, warmup), periods)], None)


def _on_runs(periods: list[PeltierActivation], warmup: float = 0.0):
    """[(seconds, heating only)]: each stretch the periods keep their Peltier on without a break,
    and whether it only ever heats in it."""
    runs, run_start, run_end, heat_only = [], None, None, True
    spans = sorted((start, end, p.heat) for (start, end), p in zip(_shifted(periods, warmup), periods))
    for start, end, heat in spans:
        if run_end is not None and start > run_end:
            runs.append((run_end - run_start, heat_only))
            run_end = None
        if run_end is None:
            run_start, run_end, heat_only = start, end, heat
        else:
            run_end, heat_only = max(run_end, end), heat_only and heat
    if run_end is not None:
        runs.append((run_end - run_start, heat_only))
    return runs


def _lra_changes(periods: list[LraActivation]):
    """Resolve one LRA's (possibly overlapping) periods into [(time, (frequency, amplitude) or None), ...]."""
    return _latest_wins([(p.start, p.end, (p.frequency, p.amplitude)) for p in periods], None)


# ----------------------------------------------------------------------------
# Playback
# ----------------------------------------------------------------------------

class StimulusController:
    """Owns the ERMs, Peltiers and LRAs and plays Stimulus objects on them."""

    def __init__(self, erms: list[ERM], peltiers: list[Peltier], lras: list[LRA] | None = None,
                 board: PwmBoard | None = None):
        self.erms = erms
        self.peltiers = peltiers
        self.lras = lras or []
        self.board = board
        self.t0 = None      # perf_counter() at time 0 of the running stimulus

    def reset(self):
        """Everything off (Peltier drivers asleep)."""
        for erm in self.erms:
            erm.off()
        for lra in self.lras:
            lra.off()
        for peltier in self.peltiers:
            peltier.off()

    def elapsed(self):
        """Seconds into the running stimulus (negative during the lead-in), or None when idle."""
        t0 = self.t0
        return None if t0 is None else time.perf_counter() - t0

    def check(self, stimulus: Stimulus):
        """Raise ValueError if the stimulus names a missing ERM/Peltier/LRA or an out-of-range
        duty, frequency or amplitude."""
        for period in stimulus.erm_activations:
            for i in period.erms:
                if not 0 <= i < len(self.erms):
                    raise ValueError(f"{period}: there is no ERM {i}")
        for period in stimulus.peltier_activations:
            if not 0 <= period.peltier < len(self.peltiers):
                raise ValueError(f"{period}: there is no Peltier {period.peltier}")
            self.peltiers[period.peltier].check(period.duty, period.heat)
        for period in stimulus.lra_activations:
            if not 0 <= period.lra < len(self.lras):
                raise ValueError(f"{period}: there is no LRA {period.lra}")
            self.lras[period.lra].check(period.frequency, period.amplitude)

    def check_safety(self, stimulus: Stimulus):
        """Raise ValueError if a Peltier would run without a break for longer than its limit:
        PELTIER_MAX_HEAT_S while only heating, PELTIER_MAX_COOL_S if it cools at all."""
        for i, peltier in enumerate(self.peltiers):
            periods = [p for p in stimulus.peltier_activations if p.peltier == i and p.enabled]
            for length, heat_only in _on_runs(periods, stimulus.peltier_warmup):
                limit = peltier_limit(heat_only)
                if length > limit:
                    raise ValueError(
                        f"{peltier.name} would run {length:.1f} s without a break; the limit is "
                        f"{limit:g} s {'heating' if heat_only else 'when cooling'} "
                        f"({'PELTIER_MAX_HEAT_S' if heat_only else 'PELTIER_MAX_COOL_S'}), as "
                        "there's no thermistor cut-off yet.")

    def run(self, stimulus: Stimulus, stop: threading.Event | None = None, verbose: bool = True,
            lead: float = 0.2):
        """Play the stimulus and block until it ends or `stop` is set. The devices are reset
        before it starts and again when it ends, fails or is stopped. Disabled periods are skipped.
        `lead` is the pause before time 0 (plus any Peltier lead-in); keep it above AUDIO_LATENCY,
        as LRA changes are made that early."""
        tracks = self._compile(stimulus.enabled_only())
        stop = stop or threading.Event()
        errors = []
        print_lock = threading.Lock()
        outputs = list({id(lra.output): lra.output for lra in self.lras}.values())

        self.reset()
        for output in outputs:
            output.hold(True)       # amp on now, so it's awake for the first LRA period
        # Give the devices a moment after reset(), then the Peltier lead-in runs before time 0.
        t0 = self.t0 = time.perf_counter() + max(lead, AUDIO_LATENCY) + stimulus.peltier_warmup

        def play(track):
            try:
                for t, description, action in track:
                    if stop.wait(max(0.0, t0 + t - time.perf_counter())):
                        return
                    action()
                    if verbose:
                        with print_lock:
                            print(f"{time.perf_counter() - t0:9.3f}s  {description}")
            except Exception as e:
                errors.append(e)
                stop.set()

        # One thread per track (ERMs, each Peltier, each LRA), so none delays the others.
        threads = [threading.Thread(target=play, args=(track,), daemon=True) for track in tracks]
        for thread in threads:
            thread.start()
        try:
            for thread in threads:
                while thread.is_alive():
                    thread.join(0.1)   # short joins keep Ctrl+C responsive
        finally:
            stop.set()
            for thread in threads:
                thread.join()
            self.t0 = None
            self.reset()
            for output in outputs:
                output.hold(False)
        if errors:
            raise errors[0]

    def _compile(self, stimulus: Stimulus):
        """Validate the stimulus and turn it into independent tracks (ERMs, then one per Peltier,
        then one per LRA), each a time-sorted list of (time, description, action)."""
        self.check(stimulus)
        self.check_safety(stimulus)
        erm_track = []
        for t, i, level in _erm_changes(stimulus.erm_activations, len(self.erms)):
            erm = self.erms[i]
            erm_track.append((t, f"ERM {i} ({erm}) {'off' if level is None else f'{level:.0%}'}",
                              erm.off if level is None else functools.partial(erm.on, level)))
        tracks = [erm_track]
        for i, peltier in enumerate(self.peltiers):
            periods = [p for p in stimulus.peltier_activations if p.peltier == i]
            tracks.append([(t, f"{peltier.name} " + ("off" if s is None else
                                                     f"{'heat' if s[1] else 'cool'} {s[0]:.0%}"),
                            peltier.off if s is None else functools.partial(peltier.on, *s))
                           for t, s in _peltier_changes(periods, stimulus.peltier_warmup)])
        for i, lra in enumerate(self.lras):
            periods = [p for p in stimulus.lra_activations if p.lra == i]
            # Made early by the audio latency, so they're felt on time alongside the ERMs.
            tracks.append([(t - AUDIO_LATENCY,
                            f"{lra.name} LRA {'off' if s is None else f'{s[0]:g} Hz at {s[1]:g}'}",
                            lra.off if s is None else functools.partial(lra.on, *s))
                           for t, s in _lra_changes(periods)])
        return [track for track in tracks if track]

    def close(self):
        try:
            self.reset()
        finally:
            for device in [*self.erms, *self.peltiers, *self.lras]:
                device.close()
            if self.board is not None:
                self.board.close()


def make_controller() -> StimulusController:
    """The wearable's devices as configured above (the editor and pi_receiver.py share this, so
    stimuli play the same in both)."""
    board = PwmBoard()
    erms = [ERM(board, ch, phase=n / len(ERM_CHANNELS), place=place)
            for n, (ch, place) in enumerate(zip(ERM_CHANNELS, ERM_PLACES))]
    peltiers = [Peltier(board, phase=n / len(PELTIERS), **cfg) for n, cfg in enumerate(PELTIERS)]
    audio = AudioOutput()
    # Headphone jack left channel -> the LRA near the elbow, right channel -> near the wrist.
    lras = [LRA(audio, 0, "Bottom", gain=LRA_GAINS[0]), LRA(audio, 1, "Top", gain=LRA_GAINS[1])]
    return StimulusController(erms, peltiers, lras, board)


# ----------------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------------

ERM_COLOR = "#ed7d31"
HEAT_COLOR, HEAT_TEXT_COLOR = "#f4a09a", "#b3261e"
COOL_COLOR, COOL_TEXT_COLOR = "#9cc6f0", "#1a5fa8"
LRA_COLOR = "#b39ddb"
LRA_TEXT_COLOR = "#4a2c8a"
SELECTED_COLOR = "#ffd54f"
MUTED_COLOR, MUTED_TEXT_COLOR = "#d6d6d6", "#9a9a9a"
LAMP_ON, LAMP_OFF = "#2ecc40", "#d0d0d0"
SIM_COLOR = "#c0392b"


def _fmt(x):
    """A number for display: up to microsecond precision, no trailing zeros."""
    return f"{x:.6f}".rstrip("0").rstrip(".")


def _parse_float(var, name):
    try:
        value = float(var.get())
    except ValueError:
        value = math.nan
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a number, got {var.get()!r}.")
    return value


def _tick_step(span, max_ticks):
    """A round tick spacing that puts at most max_ticks ticks across span seconds."""
    for step in (0.01, 0.02, 0.05, 0.1, 0.2, 0.5, 1, 2, 5, 10, 15, 30, 60, 120, 300, 600, 1800):
        if span / step <= max_ticks:
            return step
    return span / max_ticks


def _index_of(items, target):
    return next(i for i, item in enumerate(items) if item is target)


class Timeline(tk.Canvas):
    """One lane per ERM, Peltier and LRA showing the stimulus, live device state and a playhead.
    Bars can be selected, dragged and right-clicked; see the module docstring."""

    LABEL_W = 210
    AXIS_H = 26
    PAD = 10
    RIGHT_PAD = 24
    DRAG_THRESHOLD = 3      # pixels the mouse must move before a press becomes a drag
    EDGE = 5                # pixels from a bar's end that grab the end rather than the bar
    SNAP = 6                # pixels within which a dragged edge snaps to another period's end
    MIN_DURATION = 0.001    # seconds
    MIN_UNIT, MAX_UNIT = 14, 60     # pixels: the height of an ERM lane, which the others scale by

    def __init__(self, parent, editor):
        super().__init__(parent, background="white", highlightthickness=0, height=280)
        self.editor = editor
        self._periods = {}      # canvas item -> the period it draws
        self._lamps = []        # one per ERM, lit while it is on
        self._readouts = []     # one per Peltier, what it's doing now
        self._lra_lamps = []    # one per LRA, lit while it plays
        self._lra_readouts = []     # one per LRA, its present frequency and amplitude
        self._playhead = None
        self._playhead_span = (0, 0)
        self._to_x = None
        self._t_min = 0.0
        self._span = 1.0
        self._px_per_s = 1.0
        self._press = None      # the mouse press being handled, while the button is down
        self._frozen = None     # (t_min, span) held while dragging, so the axis doesn't rescale
        self._band = None       # the selection box being dragged out
        self._lane_rows = []    # (top, bottom, actuator) per lane, for clicks on the names
        self._lane_key = None   # while drawing, the actuator whose lane is being drawn
        self.bind("<Configure>", lambda event: self.redraw())
        self.bind("<Button-1>", self._on_press)
        self.bind("<Control-Button-1>", lambda event: self._on_press(event, toggle=True))
        self.bind("<B1-Motion>", self._on_drag)
        self.bind("<ButtonRelease-1>", self._on_release)
        self.bind("<Motion>", self._on_motion)
        self.bind("<Button-3>", self._on_right_click)

    def redraw(self):
        self.delete("all")
        self._periods.clear()
        self._lamps.clear()
        self._readouts.clear()
        self._lra_lamps.clear()
        self._lra_readouts.clear()
        self._playhead = None
        width, height = self.winfo_width(), self.winfo_height()
        if width < self.LABEL_W + 100:
            return      # not laid out yet
        controller, stimulus = self.editor.controller, self.editor.stimulus
        warmup = stimulus.peltier_warmup
        t_min = -warmup                     # the axis starts with the Peltier lead-in
        span = warmup + max(stimulus.duration, 1.0) * 1.05
        if self._frozen:
            t_min, span = self._frozen
        t_end = t_min + span
        left, right = self.LABEL_W, width - self.RIGHT_PAD
        x = self._to_x = lambda t: left + (right - left) * (t - t_min) / span
        self._t_min, self._span = t_min, span
        self._px_per_s = (right - left) / span
        unit = min(max((height - 2 * self.PAD - self.AXIS_H) / self._lane_units(),
                       self.MIN_UNIT), self.MAX_UNIT)
        lanes = [(self._draw_erm_lane, i, unit) for i in range(len(controller.erms))]
        lanes += [(self._draw_peltier_lane, k, 1.8 * unit) for k in range(len(controller.peltiers))]
        lanes += [(self._draw_lra_lane, j, 1.4 * unit) for j in range(len(controller.lras))]
        kinds = {self._draw_erm_lane: "erm", self._draw_peltier_lane: "peltier",
                 self._draw_lra_lane: "lra"}
        top = self.PAD
        bottom = top + sum(h for _, _, h in lanes)

        if warmup:
            self.create_rectangle(x(t_min), top, x(0), bottom, fill="#f2f2f2", outline="")
            self.create_line(x(0), top, x(0), bottom + 4, fill="#888", dash=(4, 3))
        step = _tick_step(span, max(2, (right - left) // 80))
        for n in range(math.ceil(t_min / step - 1e-9), int(t_end / step) + 1):
            tick_x = x(n * step)
            self.create_line(tick_x, top, tick_x, bottom + 4, fill="#e4e4e4")
            self.create_text(tick_x, bottom + 6, anchor="n", text=f"{_fmt(round(n * step, 6))} s",
                             fill="#555")
        self.create_line(left, bottom, right, bottom, fill="#888")

        y = top
        self._lane_rows = []
        for draw, index, h in lanes:
            self._lane_key = (kinds[draw], index)   # the actuator whose lane is being drawn
            self._lane_rows.append((y, y + h, self._lane_key))
            draw(index, y, h, x, t_end)
            y += h
            self.create_line(0, y, right, y, fill="#d4d4d4")
        if warmup:
            self.create_text((x(t_min) + x(0)) / 2, top + 2, anchor="n", fill="#777",
                             text="Peltier lead-in", tags=("overlay",))
        self.tag_raise("selected")
        self.tag_raise("overlay")
        self._playhead = self.create_line(0, top, 0, bottom, fill="#d62728", width=2, state="hidden")
        self._playhead_span = (top, bottom)
        self.update_live()

    def _lane_units(self):
        """The height of all the lanes, in ERM lanes."""
        controller = self.editor.controller
        return len(controller.erms) + 1.8 * len(controller.peltiers) + 1.4 * len(controller.lras)

    def height_for(self, unit):
        """The canvas height that shows every lane at `unit` pixels per ERM lane."""
        return math.ceil(2 * self.PAD + self.AXIS_H + 16 + self._lane_units() * unit)

    def _draw_erm_lane(self, i, y, h, x, t_end):
        erm = self.editor.controller.erms[i]
        mid = y + h / 2
        self._lamps.append(self.create_oval(8, mid - 6, 20, mid + 6, outline="#777", fill=LAMP_OFF))
        self._lane_label(mid, f"ERM {i}   {erm}", erm)
        base, ceiling = y + h - 3, y + 3        # bar height shows intensity
        for period in self.editor.stimulus.erm_activations:
            if i in period.erms:
                self._bar(period, x(period.start), base - (base - ceiling) * period.intensity,
                          x(period.end), base, ERM_COLOR)

    def _draw_peltier_lane(self, k, y, h, x, t_end):
        peltier = self.editor.controller.peltiers[k]
        self._lane_label(y + h / 2, f"{peltier.name}   {peltier}", peltier)
        self._readouts.append(self.create_text(self.LABEL_W - 10, y + h / 2, anchor="e"))
        base, ceiling = y + h - 4, y + 18    # bar height shows duty; room above for labels
        vy = lambda duty: base - (base - ceiling) * duty / peltier.max_duty
        warmup = self.editor.stimulus.peltier_warmup
        for period in sorted((p for p in self.editor.stimulus.peltier_activations
                              if p.peltier == k), key=lambda p: p.start):
            x0, x1 = x(period.start), x(period.end)
            color = HEAT_COLOR if period.heat else COOL_COLOR
            self._bar(period, x0, vy(period.duty), x1, base, color)
            if warmup and period.start == 0:
                self._bar(period, x(-warmup), vy(period.duty), x0, base, color, stipple="gray50")
            if x1 - x0 > 60:
                self.create_text(x0 + 3, vy(period.duty) - 1, anchor="sw", fill="#333",
                                 text=f"{'heat' if period.heat else 'cool'} {period.duty:.0%}",
                                 tags=("overlay",))

    def _draw_lra_lane(self, j, y, h, x, t_end):
        lra = self.editor.controller.lras[j]
        mid = y + h / 2
        self._lra_lamps.append(self.create_oval(8, mid - 6, 20, mid + 6, outline="#777",
                                                fill=LAMP_OFF))
        self._lane_label(mid, f"{lra.name} LRA   {lra}", lra)
        self._lra_readouts.append(self.create_text(self.LABEL_W - 10, mid, anchor="e",
                                                   fill=LRA_TEXT_COLOR))
        base, ceiling = y + h - 4, y + 16   # bar height shows amplitude; room above for labels
        for period in self.editor.stimulus.lra_activations:
            if period.lra == j:
                x0, x1 = x(period.start), x(period.end)
                top = base - (base - ceiling) * period.amplitude
                self._bar(period, x0, top, x1, base, LRA_COLOR)
                if x1 - x0 > 70:
                    self.create_text(x0 + 3, top - 1, anchor="sw", fill="#333", tags=("overlay",),
                                     text=f"{_fmt(period.frequency)} Hz · {_fmt(period.amplitude)}")

    def _lane_label(self, mid, text, device):
        simulated = device.sim_reason is not None
        muted = self.editor.is_muted(self._lane_key)
        if muted:       # just the name, so "muted" fits
            text = text.split("   ")[0] + "   muted"
        elif simulated:
            text += "   sim"
        self.create_text(28, mid, anchor="w", text=text,
                         fill=MUTED_TEXT_COLOR if muted else SIM_COLOR if simulated else "#222")

    def _bar(self, period, x0, y0, x1, y1, color, stipple=""):
        selected = self.editor.is_selected(period)
        disabled = not period.enabled       # drawn faint, with a dashed outline
        if self.editor.is_muted(self._lane_key):
            color = MUTED_COLOR
        item = self.create_rectangle(x0, y0, max(x1, x0 + 2), y1,
                                     stipple="gray25" if disabled else stipple,
                                     fill=SELECTED_COLOR if selected else color,
                                     outline="#000" if selected else "#555",
                                     width=2 if selected else 1, dash=(4, 3) if disabled else (),
                                     tags=("selected",) if selected else ())
        self._periods[item] = period

    # --- mouse -----------------------------------------------------------------

    def _hit(self, event):
        """The period under the mouse and which part of its bar: "start", "end" or "move"."""
        items = self.find_overlapping(event.x - 1, event.y - 1, event.x + 1, event.y + 1)
        period = next((self._periods[item] for item in reversed(items) if item in self._periods), None)
        if period is None:
            return None, None
        x0, x1 = self._to_x(period.start), self._to_x(period.end)
        if x1 - x0 >= 3 * self.EDGE:
            if abs(event.x - x1) <= self.EDGE:
                return period, "end"
            if abs(event.x - x0) <= self.EDGE:
                return period, "start"
        return period, "move"

    def _lane_at(self, event):
        """The actuator whose name was clicked, or None if the click wasn't on a name."""
        if event.x >= self.LABEL_W:
            return None
        return next((key for top, bottom, key in self._lane_rows
                     if top <= event.y < bottom), None)

    def _on_motion(self, event):
        if self._press is None and self._lane_at(event):
            self.configure(cursor="hand2")
        elif self._press is None:
            _, part = self._hit(event)
            self.configure(cursor={"start": "sb_h_double_arrow", "end": "sb_h_double_arrow",
                                   "move": "fleur"}.get(part, ""))

    def _on_press(self, event, toggle=False):
        editor = self.editor
        lane = self._lane_at(event)
        if lane is not None:
            editor.solo(lane) if toggle else editor.toggle_mute(lane)
            return
        period, part = self._hit(event)
        if period is None:
            self._press = {"x": event.x, "y": event.y, "part": "band", "add": toggle,
                           "dragging": False}
        elif toggle:
            editor.toggle_selected(period)
        else:
            # Pressing on one period of a selection keeps the selection, so it can be dragged;
            # a click without a drag narrows it down to that period on release.
            narrow = editor.is_selected(period) and len(editor.selection) > 1
            if not editor.is_selected(period):
                editor.select([period], show_tab=True)
            self._press = {"x": event.x, "y": event.y, "part": part, "period": period,
                           "narrow": narrow, "dragging": False}

    def _on_drag(self, event):
        press = self._press
        if press is None:
            return
        if not press["dragging"]:
            if max(abs(event.x - press["x"]), abs(event.y - press["y"])) < self.DRAG_THRESHOLD:
                return
            press["dragging"] = True
            if press["part"] == "band":
                self._band = self.create_rectangle(press["x"], press["y"], event.x, event.y,
                                                   outline="#333", dash=(3, 2))
            else:
                periods = list(self.editor.selection)
                press["periods"] = periods
                press["original"] = [(p.start, p.duration) for p in periods]
                press["edges"] = sorted({0.0} | {t for p in self.editor.stimulus.periods
                                                 if not any(p is q for q in periods)
                                                 for t in (p.start, p.end)})
                self._frozen = (self._t_min, self._span)
        if press["part"] == "band":
            self.coords(self._band, press["x"], press["y"], event.x, event.y)
        else:
            self._drag_periods(press, event)

    def _drag_periods(self, press, event):
        """Move the dragged periods, or their starts or ends, with the mouse."""
        periods, original = press["periods"], press["original"]
        start, duration = original[_index_of(periods, press["period"])]
        shift = (event.x - press["x"]) / self._px_per_s
        fine = event.state & 0x1        # Shift held
        grid = 0.001 if fine else _tick_step(self._span, self._px_per_s * self._span / 6)

        def snap_to_edge(t):
            if fine:
                return None
            nearest = min(press["edges"], key=lambda edge: abs(edge - t))
            return nearest if abs(nearest - t) * self._px_per_s <= self.SNAP else None

        def snap(t):
            edge = snap_to_edge(t)
            return edge if edge is not None else round(round(t / grid) * grid, 6)

        earliest = min(s for s, _ in original)
        shortest = min(d for _, d in original)
        part = press["part"]
        if part == "move":
            new_start = snap_to_edge(start + shift)
            if new_start is None:
                new_end = snap_to_edge(start + duration + shift)
                new_start = snap(start + shift) if new_end is None else new_end - duration
            delta = max(new_start - start, -earliest)
            for period, (s, d) in zip(periods, original):
                period.start = round(s + delta, 6)
        elif part == "end":
            delta = max(snap(start + duration + shift) - (start + duration),
                        self.MIN_DURATION - shortest)
            for period, (s, d) in zip(periods, original):
                period.duration = round(d + delta, 6)
        else:
            delta = min(max(snap(start + shift) - start, -earliest), shortest - self.MIN_DURATION)
            for period, (s, d) in zip(periods, original):
                period.start, period.duration = round(s + delta, 6), round(d - delta, 6)
        self.redraw()
        grabbed = press["period"]
        what = "" if len(periods) == 1 else f"{len(periods)} periods, dragged one: "
        self.editor._set_status(f"{what}start {_fmt(grabbed.start)} s,  duration "
                                f"{_fmt(grabbed.duration)} s,  end {_fmt(grabbed.end)} s"
                                "      Shift: 1 ms steps, no snapping.  Esc: cancel.")

    def _on_release(self, event):
        press, self._press = self._press, None
        if press is None:
            return
        editor = self.editor
        if press["part"] == "band":
            if press["dragging"]:
                items = self.find_overlapping(*self.coords(self._band))
                self.delete(self._band)
                self._band = None
                boxed = [self._periods[item] for item in items if item in self._periods]
                selection = list(editor.selection) if press["add"] else []
                for period in boxed:
                    if not any(period is p for p in selection):
                        selection.append(period)
                editor.select(selection)
            elif not press["add"]:
                editor.select([])
        elif press["dragging"]:
            self._frozen = None
            moved = any((p.start, p.duration) != o
                        for p, o in zip(press["periods"], press["original"]))
            if moved:
                editor.periods_edited()
            else:
                self.redraw()
        elif press["narrow"]:
            editor.select([press["period"]], show_tab=True)

    def cancel_drag(self):
        """Abandon a drag in progress, putting everything back. False if there wasn't one."""
        press, self._press = self._press, None
        if press is None or not press["dragging"]:
            self._press = press
            return False
        if press["part"] == "band":
            self.delete(self._band)
            self._band = None
        else:
            for period, (start, duration) in zip(press["periods"], press["original"]):
                period.start, period.duration = start, duration
            self._frozen = None
            self.redraw()
        self.editor._set_status("Drag cancelled.")
        return True

    def _on_right_click(self, event):
        period, _ = self._hit(event)
        if period is None or self._press is not None:
            return
        editor = self.editor
        if not editor.is_selected(period):
            editor.select([period], show_tab=True)
        count = len(editor.selection)
        what = "period" if count == 1 else f"{count} periods"
        menu = tk.Menu(self, tearoff=False)
        menu.add_cascade(label=f"Move {what} to", menu=editor.actuator_menu(menu, copy=False))
        menu.add_cascade(label=f"Copy {what} to", menu=editor.actuator_menu(menu, copy=True))
        menu.add_separator()
        verb = "Disable" if any(p.enabled for p in editor.selection) else "Enable"
        menu.add_command(label=f"{verb} {what}", accelerator="Ctrl+D", command=editor.toggle_enabled)
        menu.add_command(label=f"Delete {what}", command=lambda: editor.remove_periods(editor.selection))
        menu.tk_popup(event.x_root, event.y_root)

    def update_live(self):
        """Refresh the lamps, readouts and playhead from the devices."""
        controller = self.editor.controller
        for lamp, erm in zip(self._lamps, controller.erms):
            self.itemconfigure(lamp, fill=LAMP_ON if erm.is_on else LAMP_OFF)
        for readout, peltier in zip(self._readouts, controller.peltiers):
            heat = peltier.heat
            self.itemconfigure(readout, text="" if not peltier.is_on else
                               f"{'heat' if heat else 'cool'} {peltier.duty:.0%}",
                               fill=HEAT_TEXT_COLOR if heat else COOL_TEXT_COLOR)
        for lamp, readout, lra in zip(self._lra_lamps, self._lra_readouts, controller.lras):
            frequency, amplitude = lra.frequency, lra.amplitude
            self.itemconfigure(lamp, fill=LAMP_OFF if frequency is None else LAMP_ON)
            self.itemconfigure(readout, text="" if frequency is None
                               else f"{frequency:g} Hz · {amplitude:g}")
        if self._playhead is None:
            return
        elapsed = controller.elapsed()
        if elapsed is None:
            self.itemconfigure(self._playhead, state="hidden")
        else:
            playhead_x = self._to_x(max(self._t_min, elapsed))
            top, bottom = self._playhead_span
            self.coords(self._playhead, playhead_x, top, playhead_x, bottom)
            self.itemconfigure(self._playhead, state="normal")


class _PeriodPanel(ttk.Frame):
    """A table of periods plus a form to add, update and delete them."""

    columns = ()    # (heading, width) pairs; set by subclasses
    multi_update = False    # whether Update can set the values of several selected periods

    def __init__(self, parent, editor):
        super().__init__(parent, padding=8)
        self.editor = editor
        self._rows = {}     # tree item id -> period

        self.tree = ttk.Treeview(self, columns=[name for name, _ in self.columns], show="headings",
                                 height=7, selectmode="extended")
        for name, width in self.columns:
            self.tree.heading(name, text=name)
            self.tree.column(name, width=width, anchor="center")
        scrollbar = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.tree.bind("<<TreeviewSelect>>", self._on_tree_select)

        form = ttk.Frame(self, padding=(14, 0, 0, 0))
        form.grid(row=0, column=2, sticky="n")
        self.start_var = tk.StringVar(value="0")
        self.duration_var = tk.StringVar(value="1")
        for row, (label, var) in enumerate([("Start (s)", self.start_var),
                                            ("Duration (s)", self.duration_var)]):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(form, textvariable=var, width=10).grid(row=row, column=1, sticky="w", pady=2)
        self._build_fields(form, first_row=2)
        buttons = ttk.Frame(form)
        buttons.grid(row=99, column=0, columnspan=2, sticky="w", pady=(10, 0))
        ttk.Button(buttons, text="Add", command=self._add).pack(side=tk.LEFT)
        self.update_button = ttk.Button(buttons, text="Update", command=self._update)
        self.update_button.pack(side=tk.LEFT, padx=4)
        self.delete_button = ttk.Button(buttons, text="Delete", command=self._delete)
        self.delete_button.pack(side=tk.LEFT)
        self.move_button = ttk.Menubutton(buttons, text="Move/Copy ▾")
        menu = tk.Menu(self.move_button, tearoff=False)
        menu.add_cascade(label="Move selected to", menu=editor.actuator_menu(menu, copy=False))
        menu.add_cascade(label="Copy selected to", menu=editor.actuator_menu(menu, copy=True))
        self.move_button["menu"] = menu
        self.move_button.pack(side=tk.LEFT, padx=(4, 0))
        self.enable_button = ttk.Button(buttons, text="Disable", width=8,
                                        command=editor.toggle_enabled)
        self.enable_button.pack(side=tk.LEFT, padx=(4, 0))
        self._bind_return(form)
        self._bind_return(self.tree)

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.refresh()

    # Subclasses provide these.
    def periods(self): raise NotImplementedError
    def _row(self, period): raise NotImplementedError
    def _build_fields(self, form, first_row): raise NotImplementedError
    def _fill_fields(self, period): raise NotImplementedError
    def _make_periods(self, start, duration): raise NotImplementedError   # -> list of periods
    def with_form_values(self, period): raise NotImplementedError  # period's timing, form's values

    def refresh(self):
        self.tree.delete(*self.tree.get_children())
        self._rows.clear()
        self.tree.tag_configure("disabled", foreground="#999")
        for period in self.periods():
            values = list(self._row(period))
            if not period.enabled:
                values[0] = f"{values[0]}  (off)"
            item = self.tree.insert("", tk.END, values=values,
                                    tags=() if period.enabled else ("disabled",))
            self._rows[item] = period
        self.show_selection()

    def show_selection(self):
        """Select the rows of the editor's selected periods, and fill the form if there is one."""
        iids = [iid for iid, p in self._rows.items() if self.editor.is_selected(p)]
        if set(self.tree.selection()) != set(iids):
            self.tree.selection_set(iids)
        if iids:
            self.tree.see(iids[0])
        if len(iids) == 1:
            period = self._rows[iids[0]]
            self.start_var.set(_fmt(period.start))
            self.duration_var.set(_fmt(period.duration))
            self._fill_fields(period)
        self._update_buttons()

    def selected_periods(self):
        return [self._rows[iid] for iid in self.tree.selection() if iid in self._rows]

    def selected_period(self):
        """The selected period, if exactly one is."""
        periods = self.selected_periods()
        return periods[0] if len(periods) == 1 else None

    def _on_tree_select(self, event):
        # This also fires after show_selection() has matched the rows to the editor's selection;
        # only a change the user made is passed on.
        periods = self.selected_periods()
        rows = self._rows.values()
        mine = [p for p in self.editor.selection if any(p is q for q in rows)]
        if {id(p) for p in periods} != {id(p) for p in mine}:
            self.editor.select(periods)
        self._update_buttons()

    def _bind_return(self, widget):
        """Enter updates the selected period (or adds one if none is selected); Shift+Enter adds."""
        widget.bind("<Return>", self._on_return)
        widget.bind("<KP_Enter>", self._on_return)
        for child in widget.winfo_children():
            self._bind_return(child)

    def _on_return(self, event):
        if event.state & 0x1 or not self.selected_periods():    # 0x1: Shift held
            self._add()
        elif self._can_update():
            self._update()
        return "break"

    def _can_update(self):
        count = len(self.selected_periods())
        return count == 1 or (count > 1 and self.multi_update)

    def _update_buttons(self):
        self.update_button.state(["!disabled"] if self._can_update() else ["disabled"])
        any_selected = ["!disabled"] if self.selected_periods() else ["disabled"]
        self.delete_button.state(any_selected)
        self.move_button.state(any_selected)
        self.enable_button.state(any_selected)
        periods = self.editor.selection
        self.enable_button.configure(
            text="Enable" if periods and not any(p.enabled for p in periods) else "Disable")

    def _read_form(self):
        try:
            start = _parse_float(self.start_var, "Start")
            duration = _parse_float(self.duration_var, "Duration")
            return self._make_periods(start, duration)
        except ValueError as e:
            messagebox.showerror("Invalid period", str(e), parent=self)
            return None

    def _add(self):
        periods = self._read_form()
        if periods is not None:
            self.editor.add_periods(periods)

    def _update(self):
        """One period selected: replace it with the form. Several: give them all the form's
        values (not its timing), where the panel allows that."""
        old = self.selected_periods()
        if len(old) == 1:
            new = self._read_form()
            if new is not None:
                self.editor.replace_periods([(old[0], new)])
        elif len(old) > 1 and self.multi_update:
            try:
                self.editor.replace_periods([(p, [self.with_form_values(p)]) for p in old])
            except ValueError as e:
                messagebox.showerror("Invalid values", str(e), parent=self)

    def _delete(self):
        periods = self.selected_periods()
        if periods:
            self.editor.remove_periods(periods)


class ErmPanel(_PeriodPanel):
    columns = (("ERMs", 120), ("Level (%)", 70), ("Start (s)", 80), ("Duration (s)", 80),
               ("End (s)", 80))

    def periods(self):
        return sorted(self.editor.stimulus.erm_activations, key=lambda p: p.start)

    def _row(self, period):
        return (", ".join(str(i) for i in sorted(set(period.erms))),
                _fmt(round(period.intensity * 100, 4)),
                _fmt(period.start), _fmt(period.duration), _fmt(period.end))

    def form_intensity(self):
        """The intensity in the form, 0-1."""
        intensity = _parse_float(self.intensity_var, "ERM level") / 100
        if not 0 < intensity <= 1:
            raise ValueError(f"ERM level must be 1-100%, got {self.intensity_var.get()}%.")
        return round(intensity, 6)

    def _build_fields(self, form, first_row):
        self.duration_var.set("0.1")    # Tactile Brush strokes are built from short bursts
        self.intensity_var = tk.StringVar(value="100")
        ttk.Label(form, text="Level (%)").grid(row=first_row, column=0, sticky="w", pady=2)
        ttk.Entry(form, textvariable=self.intensity_var, width=10).grid(
            row=first_row, column=1, sticky="w", pady=2)
        first_row += 1
        self.brush_var = tk.BooleanVar(value=True)
        timing = ttk.Frame(form)
        timing.grid(row=first_row, column=0, columnspan=2, sticky="w", pady=(6, 0))
        ttk.Label(timing, text="Timing").pack(side=tk.LEFT, padx=(0, 6))
        for text, value in [("Tactile Brush", True), ("Simultaneous", False)]:
            ttk.Radiobutton(timing, text=text, variable=self.brush_var, value=value,
                            command=self._update_info).pack(side=tk.LEFT, padx=(0, 6))

        box = ttk.LabelFrame(form, text="ERMs", padding=(6, 2))
        box.grid(row=first_row + 1, column=0, columnspan=2, sticky="we", pady=(6, 0))
        self.erm_vars = []
        self.order = []     # ticked ERMs in the order they were ticked: a stroke's direction
        for i, erm in enumerate(self.editor.controller.erms):
            var = tk.BooleanVar()
            ttk.Checkbutton(box, text=f"{i}: {erm}", variable=var,
                            command=functools.partial(self._on_tick, i)).grid(
                row=i % 4, column=i // 4, sticky="w", padx=(0, 10))
            self.erm_vars.append(var)

        self.info = ttk.Label(form, foreground="#666", justify=tk.LEFT, wraplength=420)
        self.info.grid(row=first_row + 2, column=0, columnspan=2, sticky="w", pady=(4, 0))
        self.duration_var.trace_add("write", lambda *args: self._update_info())
        self._update_info()

    def _on_tick(self, i):
        if self.erm_vars[i].get():
            self.order.append(i)
        else:
            self.order.remove(i)
        self._update_info()

    def _update_info(self):
        if not self.brush_var.get():
            text = "All ticked ERMs run together as one period."
        elif not self.order:
            text = "Tick ERMs in the order the stroke should cross them."
        else:
            text = "Order " + " → ".join(map(str, self.order))
            try:
                duration = _parse_float(self.duration_var, "Duration")
            except ValueError:
                duration = None
            if duration is not None and duration > 0:
                soa = tactile_brush_soa(duration)
                total = (len(self.order) - 1) * soa + duration
                text += (f"\nEach ERM on {_fmt(round(duration * 1000, 1))} ms, "
                         f"SOA {_fmt(round(soa * 1000, 1))} ms, stroke {_fmt(round(total, 4))} s")
        self.info.configure(text=text)

    def _fill_fields(self, period):
        self.intensity_var.set(_fmt(round(period.intensity * 100, 4)))
        for i, var in enumerate(self.erm_vars):
            var.set(i in period.erms)
        self.order = list(dict.fromkeys(period.erms))
        if len(self.order) > 1:
            self.brush_var.set(False)   # so Update keeps a multi-ERM period as it is
        self._update_info()

    def _make_periods(self, start, duration):
        if not self.order:
            raise ValueError("Tick at least one ERM.")
        intensity = self.form_intensity()
        if self.brush_var.get():
            return tactile_brush_stroke(self.order, start, duration, intensity)
        return [ErmActivation(list(self.order), start, duration, intensity=intensity)]


class PeltierPanel(_PeriodPanel):
    columns = (("Mode", 80), ("Duty (%)", 80), ("Start (s)", 80), ("Duration (s)", 80),
               ("End (s)", 80))
    multi_update = True

    def __init__(self, parent, editor, index):
        self.index = index
        self.peltier = editor.controller.peltiers[index]
        super().__init__(parent, editor)

    def periods(self):
        return sorted((p for p in self.editor.stimulus.peltier_activations
                       if p.peltier == self.index), key=lambda p: p.start)

    def _row(self, period):
        return ("heat" if period.heat else "cool", _fmt(round(period.duty * 100, 4)),
                _fmt(period.start), _fmt(period.duration), _fmt(period.end))

    def _build_fields(self, form, first_row):
        self.duty_var = tk.StringVar(value="30")
        self.heat_var = tk.BooleanVar(value=True)
        ttk.Label(form, text="Duty (%)").grid(row=first_row, column=0, sticky="w", pady=2)
        ttk.Entry(form, textvariable=self.duty_var, width=10).grid(
            row=first_row, column=1, sticky="w", pady=2)
        mode = ttk.Frame(form)
        mode.grid(row=first_row + 1, column=0, columnspan=2, sticky="w", pady=2)
        for text, value in [("Heat", True), ("Cool", False)]:
            ttk.Radiobutton(mode, text=text, variable=self.heat_var, value=value).pack(
                side=tk.LEFT, padx=(0, 8))
        ttk.Label(form, foreground="#666", justify=tk.LEFT,
                  text=f"Duty up to {self.peltier.max_duty:.0%}; on without a break at most "
                       f"{PELTIER_MAX_HEAT_S:g} s heating,\n{PELTIER_MAX_COOL_S:g} s cooling. "
                       "Heat / cool is for the skin face.").grid(
            row=first_row + 2, column=0, columnspan=2, sticky="w")

    def _fill_fields(self, period):
        self.duty_var.set(_fmt(round(period.duty * 100, 4)))
        self.heat_var.set(period.heat)

    def _make_periods(self, start, duration):
        duty = _parse_float(self.duty_var, f"{self.peltier.name} duty") / 100
        heat = bool(self.heat_var.get())
        self.peltier.check(duty, heat)
        return [PeltierActivation(self.index, start, duration, round(duty, 6), heat)]

    def with_form_values(self, period):
        return self._make_periods(period.start, period.duration)[0]


class LraPanel(_PeriodPanel):
    columns = (("Frequency (Hz)", 110), ("Amplitude", 80), ("Start (s)", 80), ("Duration (s)", 80),
               ("End (s)", 80))
    multi_update = True

    def __init__(self, parent, editor, index):
        self.index = index
        self.lra = editor.controller.lras[index]
        super().__init__(parent, editor)

    def periods(self):
        return sorted((p for p in self.editor.stimulus.lra_activations if p.lra == self.index),
                      key=lambda p: p.start)

    def _row(self, period):
        return (_fmt(period.frequency), _fmt(period.amplitude), _fmt(period.start),
                _fmt(period.duration), _fmt(period.end))

    def _build_fields(self, form, first_row):
        self.frequency_var = tk.StringVar(value="70")
        self.amplitude_var = tk.StringVar(value="0.5")
        for row, (label, var) in enumerate([("Frequency (Hz)", self.frequency_var),
                                            ("Amplitude (0–1)", self.amplitude_var)], first_row):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w", pady=2)
            ttk.Entry(form, textvariable=var, width=10).grid(row=row, column=1, sticky="w", pady=2)
        ttk.Label(form, foreground="#666", justify=tk.LEFT,
                  text="Strongest at the LRA's resonant frequency.\n"
                       "Amplitude 1 is the loudest the jack plays.").grid(
            row=first_row + 2, column=0, columnspan=2, sticky="w")

    def _fill_fields(self, period):
        self.frequency_var.set(_fmt(period.frequency))
        self.amplitude_var.set(_fmt(period.amplitude))

    def _make_periods(self, start, duration):
        frequency = _parse_float(self.frequency_var, "Frequency")
        amplitude = _parse_float(self.amplitude_var, "Amplitude")
        self.lra.check(frequency, amplitude)
        return [LraActivation(self.index, start, duration, frequency, amplitude)]

    def with_form_values(self, period):
        return self._make_periods(period.start, period.duration)[0]


class ManualPanel(ttk.Frame):
    """Direct control of each ERM, Peltier and LRA, e.g. for checking the wiring. Locked while a
    stimulus runs. A Peltier switches itself off after its limit (PELTIER_MAX_HEAT_S heating,
    PELTIER_MAX_COOL_S once it has cooled), counted from when it went on."""

    def __init__(self, parent, editor):
        super().__init__(parent, padding=8)
        self.editor = editor
        controller = editor.controller
        self._widgets = []      # everything to lock while a stimulus runs

        erm_box = ttk.LabelFrame(self, text="ERMs", padding=6)
        erm_box.grid(row=0, column=0, rowspan=2, sticky="nw")
        self.erm_vars = []
        for i, erm in enumerate(controller.erms):
            var = tk.BooleanVar()
            button = ttk.Checkbutton(erm_box, text=f"ERM {i} ({erm})", variable=var,
                                     command=functools.partial(self._toggle, i))
            button.grid(row=i % 4, column=i // 4, sticky="w", padx=(0, 10), pady=1)
            self.erm_vars.append(var)
            self._widgets.append(button)

        peltier_box = ttk.LabelFrame(self, text="Peltiers", padding=6)
        peltier_box.grid(row=0, column=1, sticky="nw", padx=(14, 0))
        self.duty_vars = []
        self.readouts = []
        self._auto_off = {}     # Peltier index -> pending after() id that switches it off
        self._on_since = {}     # Peltier index -> when it went on, and whether it only heated
        for k, peltier in enumerate(controller.peltiers):
            var = tk.StringVar(value="30")
            ttk.Label(peltier_box, text=peltier.name).grid(row=k, column=0, sticky="w", padx=(0, 6))
            entry = ttk.Entry(peltier_box, textvariable=var, width=5)
            entry.grid(row=k, column=1, pady=2)
            ttk.Label(peltier_box, text="%").grid(row=k, column=2, padx=(2, 6))
            buttons = []
            for column, (text, heat) in enumerate([("Heat", True), ("Cool", False)], start=3):
                button = ttk.Button(peltier_box, text=text, width=5,
                                    command=functools.partial(self._peltier_on, k, heat))
                button.grid(row=k, column=column, padx=(0, 4))
                buttons.append(button)
            off = ttk.Button(peltier_box, text="Off", width=4,
                             command=functools.partial(self._peltier_off, k))
            off.grid(row=k, column=5)
            readout = ttk.Label(peltier_box, width=12)
            readout.grid(row=k, column=6, padx=(8, 0))
            self.duty_vars.append(var)
            self.readouts.append(readout)
            self._widgets += [entry, *buttons, off]

        lra_box = ttk.LabelFrame(self, text="LRAs", padding=6)
        lra_box.grid(row=1, column=1, sticky="nw", padx=(14, 0), pady=(8, 0))
        self.lra_vars = []      # (frequency, amplitude) per LRA
        self.lra_readouts = []
        for j, lra in enumerate(controller.lras):
            frequency, amplitude = tk.StringVar(value="175"), tk.StringVar(value="0.5")
            ttk.Label(lra_box, text=lra.name).grid(row=j, column=0, sticky="w", padx=(0, 6))
            entries = []
            for column, (var, unit) in enumerate([(frequency, "Hz"), (amplitude, "amp")]):
                entry = ttk.Entry(lra_box, textvariable=var, width=6)
                entry.grid(row=j, column=1 + 2 * column, pady=2)
                entry.bind("<Return>", lambda event, j=j: self._lra_on(j))
                ttk.Label(lra_box, text=unit).grid(row=j, column=2 + 2 * column, padx=(2, 6))
                entries.append(entry)
            on = ttk.Button(lra_box, text="On", width=4, command=functools.partial(self._lra_on, j))
            on.grid(row=j, column=5)
            off = ttk.Button(lra_box, text="Off", width=4, command=functools.partial(self._lra_off, j))
            off.grid(row=j, column=6, padx=(4, 0))
            readout = ttk.Label(lra_box, width=16, foreground=LRA_TEXT_COLOR)
            readout.grid(row=j, column=7, padx=(8, 0))
            self.lra_vars.append((frequency, amplitude))
            self.lra_readouts.append(readout)
            self._widgets += [*entries, on, off]

        reset = ttk.Button(self, text="All actuators off",
                           command=self.reset_devices)
        reset.grid(row=2, column=0, columnspan=2, sticky="w", pady=(12, 0))
        self._widgets.append(reset)

    def _toggle(self, i):
        erm = self.editor.controller.erms[i]
        if self.erm_vars[i].get():
            erm.on()
        else:
            erm.off()

    def _peltier_on(self, k, heat):
        if self.editor.running:
            return
        peltier = self.editor.controller.peltiers[k]
        try:
            was_on = peltier.is_on
            peltier.on(_parse_float(self.duty_vars[k], "Duty") / 100, heat)
        except Exception as e:
            messagebox.showerror(peltier.name, str(e), parent=self)
            return
        # The limit counts from when it went on; once it has cooled, the cooling limit applies.
        since, heat_only = self._on_since.get(k, (time.monotonic(), True)) if was_on \
            else (time.monotonic(), True)
        heat_only = heat_only and heat
        self._on_since[k] = (since, heat_only)
        pending = self._auto_off.pop(k, None)
        if pending is not None:
            self.after_cancel(pending)
        left = peltier_limit(heat_only) - (time.monotonic() - since)
        self._auto_off[k] = self.after(max(0, int(left * 1000)),
                                       functools.partial(self._peltier_off, k, True))

    def _peltier_off(self, k, automatic=False):
        pending = self._auto_off.pop(k, None)
        if pending is not None and not automatic:
            self.after_cancel(pending)
        if not self.editor.running:
            peltier = self.editor.controller.peltiers[k]
            if automatic and peltier.is_on:
                heat_only = self._on_since.get(k, (0, True))[1]
                self.editor._set_status(
                    f"{peltier.name} switched off after {peltier_limit(heat_only):g} s "
                    f"({'PELTIER_MAX_HEAT_S' if heat_only else 'PELTIER_MAX_COOL_S'}).")
            peltier.off()
            self._on_since.pop(k, None)

    def _lra_on(self, j):
        if self.editor.running:
            return
        lra = self.editor.controller.lras[j]
        frequency, amplitude = self.lra_vars[j]
        try:
            lra.on(_parse_float(frequency, "Frequency"), _parse_float(amplitude, "Amplitude"))
        except Exception as e:
            messagebox.showerror(f"{lra.name} LRA", str(e), parent=self)

    def _lra_off(self, j):
        if not self.editor.running:
            self.editor.controller.lras[j].off()

    def reset_devices(self):
        for pending in self._auto_off.values():
            self.after_cancel(pending)
        self._auto_off.clear()
        self._on_since.clear()
        try:
            self.editor.controller.reset()
        except Exception as e:
            messagebox.showerror("Reset failed", str(e), parent=self)

    def set_running(self, running):
        for widget in self._widgets:
            widget.state(["disabled"] if running else ["!disabled"])

    def update_live(self):
        controller = self.editor.controller
        for var, erm in zip(self.erm_vars, controller.erms):
            if var.get() != erm.is_on:
                var.set(erm.is_on)
        for label, peltier in zip(self.readouts, controller.peltiers):
            heat = peltier.heat
            label.configure(text="off" if not peltier.is_on else
                            f"now {'heat' if heat else 'cool'} {peltier.duty:.0%}",
                            foreground=HEAT_TEXT_COLOR if heat else COOL_TEXT_COLOR)
        for label, lra in zip(self.lra_readouts, controller.lras):
            label.configure(text="off" if lra.frequency is None
                            else f"now {lra.frequency:g} Hz · {lra.amplitude:g}")


class StimulusEditor(tk.Tk):
    POLL_MS = 50
    UNDO_LIMIT = 200    # edits kept for undo

    def __init__(self, controller: StimulusController, path: str | None = None):
        super().__init__()
        self.controller = controller
        self.stimulus = Stimulus()
        self.path = None
        self.selection = []         # the periods highlighted in the timeline and their tables
        self.muted = set()          # actuators left out of playback: ("erm", 3), ("lra", 0), ...
        # Undo history. Each state is a (stimulus, selection) pair, the selection given as
        # (list name, index) references into the stimulus; stimuli in it are never modified.
        self._saved = Stimulus()            # the stimulus as last opened or saved
        self._committed = Stimulus()        # the stimulus as of the last completed edit
        self._committed_selection = []
        self._undo = []
        self._redo = []
        self._run_thread = None
        self._run_stop = None
        self._run_error = None
        self._run_duration = 0.0
        self._stopped_by_user = False
        self._run_label = ""        # what is playing, for the status bar
        self.slots = {"A": None, "B": None}     # Compare slot -> path of the stimulus it plays

        ttk.Style(self).theme_use("clam")
        self._build_toolbar()
        self.status = ttk.Label(self, anchor="w", padding=(8, 3))
        self.status.pack(side=tk.BOTTOM, fill=tk.X)
        panes = ttk.PanedWindow(self, orient=tk.VERTICAL)
        panes.pack(fill=tk.BOTH, expand=True, padx=6)
        self.timeline = Timeline(panes, self)
        self.tabs = ttk.Notebook(panes)
        panes.add(self.timeline, weight=1)
        panes.add(self.tabs, weight=0)
        self.erm_panel = ErmPanel(self.tabs, self)
        self.tabs.add(self.erm_panel, text="ERM periods")
        self.peltier_panels = []
        for k, peltier in enumerate(controller.peltiers):
            panel = PeltierPanel(self.tabs, self, k)
            self.tabs.add(panel, text=f"{peltier.name} periods")
            self.peltier_panels.append(panel)
        self.lra_panels = []
        for j, lra in enumerate(controller.lras):
            panel = LraPanel(self.tabs, self, j)
            self.tabs.add(panel, text=f"{lra.name} LRA periods")
            self.lra_panels.append(panel)
        self.manual = ManualPanel(self.tabs, self)
        self.tabs.add(self.manual, text="Manual control")

        self.bind("<Control-n>", lambda event: self.new_stimulus())
        self.bind("<Control-o>", lambda event: self.open_stimulus())
        self.bind("<Control-s>", lambda event: self.save())
        self.bind("<F5>", lambda event: self.run())
        self.bind("<Escape>", lambda event: self.timeline.cancel_drag() or self.stop())
        self.bind("<F6>", lambda event: self.play_slot("A"))
        self.bind("<F7>", lambda event: self.play_slot("B"))
        self.bind("<Delete>", self._on_delete_key)
        self.bind("<BackSpace>", self._on_delete_key)
        self.bind("<Control-a>", self._on_select_all)
        self.bind("<Control-d>", self._on_toggle_key)
        self.bind("<Control-z>", lambda event: self.undo())
        self.bind("<Control-y>", lambda event: self.redo())
        self.bind("<Control-Z>", lambda event: self.redo())     # Ctrl+Shift+Z
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self._set_status("Ready.  F5 runs, F6/F7 play Compare A/B, Esc stops.  Drag bars to move or resize, "
                         "Ctrl+click or drag a box to select several, Ctrl+D disables, Ctrl+Z undoes, right-click for more.  "
                         "Click an actuator's name to mute it, Ctrl+click to solo.")
        self._refresh()
        self._fit_to_screen()
        if path:
            self.open_stimulus(path)
        self.after_idle(self.manual.reset_devices)   # start from a known state
        self._poll()

    # Room left on the screen for taskbars and window decorations, in pixels.
    SCREEN_MARGIN_W, SCREEN_MARGIN_H = 40, 100
    COMFORTABLE_UNIT = 24   # ERM lane height the window is sized for, if the screen has room

    def _fit_to_screen(self):
        """Size the window to show the whole toolbar, every tab and every timeline lane,
        as far as the screen allows, and centre it."""
        self.update_idletasks()     # so the widgets' requested sizes are known
        max_w = self.winfo_screenwidth() - self.SCREEN_MARGIN_W
        max_h = self.winfo_screenheight() - self.SCREEN_MARGIN_H
        # Everything but the timeline: toolbar, status bar, tabs and the pane divider.
        fixed_h = (self.toolbar.winfo_reqheight() + self.status.winfo_reqheight()
                   + self.tabs.winfo_reqheight() + 12)
        want_w = max(self.toolbar.winfo_reqwidth(), self.tabs.winfo_reqwidth() + 12,
                     self.timeline.LABEL_W + 500)
        want_h = fixed_h + self.timeline.height_for(self.COMFORTABLE_UNIT)
        need_h = fixed_h + self.timeline.height_for(self.timeline.MIN_UNIT)
        width, height = min(want_w, max_w), min(want_h, max_h)
        self.timeline.configure(height=height - fixed_h)
        self.minsize(min(want_w, max_w), min(need_h, max_h))
        x = (self.winfo_screenwidth() - width) // 2
        y = max(0, (self.winfo_screenheight() - height) // 2 - self.SCREEN_MARGIN_H // 4)
        self.geometry(f"{width}x{height}+{x}+{y}")

    def _build_toolbar(self):
        bar = self.toolbar = ttk.Frame(self, padding=6)
        bar.pack(side=tk.TOP, fill=tk.X)
        for text, command in [("New", self.new_stimulus), ("Open…", self.open_stimulus),
                              ("Save", self.save), ("Save as…", self.save_as)]:
            ttk.Button(bar, text=text, command=command).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        self.run_button = ttk.Button(bar, text="▶ Run (F5)", command=self.run)
        self.run_button.pack(side=tk.LEFT, padx=(0, 4))
        self.stop_button = ttk.Button(bar, text="■ Stop (Esc)", command=self.stop, state="disabled")
        self.stop_button.pack(side=tk.LEFT)
        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Label(bar, text="Peltier lead-in (s)").pack(side=tk.LEFT, padx=(0, 4))
        self.warmup_var = tk.StringVar(value=_fmt(self.stimulus.peltier_warmup))
        warmup = ttk.Entry(bar, textvariable=self.warmup_var, width=6)
        warmup.pack(side=tk.LEFT)
        warmup.bind("<Return>", lambda event: self._apply_warmup())
        warmup.bind("<FocusOut>", lambda event: self._apply_warmup())

        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        ttk.Label(bar, text="Compare").pack(side=tk.LEFT, padx=(0, 4))
        self.slot_buttons = {}
        for slot, key in [("A", "F6"), ("B", "F7")]:
            button = ttk.Button(bar, command=functools.partial(self.play_slot, slot))
            button.pack(side=tk.LEFT)
            menu_button = ttk.Menubutton(bar, text="▾", width=2)
            menu = tk.Menu(menu_button, tearoff=False)
            menu.add_command(label=f"Load a file into {slot}…",
                             command=functools.partial(self.load_slot, slot))
            menu.add_command(label=f"Put the open stimulus in {slot}",
                             command=functools.partial(self.load_slot, slot, current=True))
            menu.add_command(label=f"Empty {slot}", command=functools.partial(self._set_slot, slot, None))
            menu_button["menu"] = menu
            menu_button.pack(side=tk.LEFT, padx=(0, 6))
            self.slot_buttons[slot] = (button, key)

        devices = ([(f"ERM {i}", erm) for i, erm in enumerate(self.controller.erms)]
                   + [(peltier.name, peltier) for peltier in self.controller.peltiers]
                   + [(f"{lra.name} LRA", lra) for lra in self.controller.lras])
        simulated = defaultdict(list)       # reason -> device names
        for name, device in devices:
            if device.sim_reason:
                simulated[device.sim_reason].append(name)
        if simulated:
            names = [name for group in simulated.values() for name in group]
            text = "SIMULATED: all devices" if len(names) == len(devices) else f"SIMULATED: {', '.join(names)}"
            details = "\n\n".join(f"{', '.join(group)}\n    {reason}" for reason, group in simulated.items())
            banner = tk.Label(bar, text=f"{text}  (details)", bg=SIM_COLOR, fg="white", padx=8,
                              cursor="hand2")
            banner.pack(side=tk.RIGHT)
            banner.bind("<Button-1>", lambda event: messagebox.showwarning(
                "Simulated devices", "These devices are not driving hardware:\n\n" + details, parent=self))

    # --- editing -------------------------------------------------------------

    def _panels(self):
        return [self.erm_panel, *self.peltier_panels, *self.lra_panels]

    def _panel_for(self, period):
        if isinstance(period, ErmActivation):
            return self.erm_panel
        if isinstance(period, LraActivation):
            return self.lra_panels[period.lra]
        return self.peltier_panels[period.peltier]

    def _list_for(self, period):
        if isinstance(period, ErmActivation):
            return self.stimulus.erm_activations
        if isinstance(period, LraActivation):
            return self.stimulus.lra_activations
        return self.stimulus.peltier_activations

    def is_selected(self, period):
        return any(p is period for p in self.selection)

    def select(self, periods, show_tab=False):
        """Select these periods; show_tab brings up the tab of the last one."""
        self.selection = list(periods)
        for panel in self._panels():
            panel.show_selection()
        if show_tab and self.selection:
            self.tabs.select(self._panel_for(self.selection[-1]))
        if self.stimulus == self._committed:    # not mid-drag: undo can bring this selection back
            self._committed_selection = self._selection_refs()
        self.timeline.redraw()

    def toggle_selected(self, period):
        if self.is_selected(period):
            self.select([p for p in self.selection if p is not period])
        else:
            self.select([*self.selection, period], show_tab=True)

    def _on_select_all(self, event):
        if not isinstance(event.widget, (tk.Entry, ttk.Entry)):
            self.select(self.stimulus.periods)

    def add_periods(self, new):
        self._list_for(new[0]).extend(new)
        self._changed(select=new)

    def replace_periods(self, replacements):
        """Put each (old, [new, ...]) period's replacements where it was."""
        selection = []
        for old, new in replacements:
            for period in new:
                period.enabled = old.enabled
            periods = self._list_for(old)
            i = _index_of(periods, old)
            periods[i:i + 1] = new
            selection += new
        self._changed(select=selection)

    def remove_periods(self, periods):
        for period in list(periods):
            in_list = self._list_for(period)
            del in_list[_index_of(in_list, period)]
        self._changed(select=[])

    def toggle_enabled(self):
        """Disable the selected periods, or enable them if they are all disabled already."""
        if not self.selection:
            return
        enable = not any(p.enabled for p in self.selection)
        for period in self.selection:
            period.enabled = enable
        self._changed(select=self.selection)

    def _on_toggle_key(self, event):
        if not isinstance(event.widget, (tk.Entry, ttk.Entry)):
            self.toggle_enabled()

    def periods_edited(self):
        """The selected periods were changed in place (e.g. dragged)."""
        self._changed(select=self.selection)

    def actuator_menu(self, parent, copy):
        """A menu of every actuator; picking one moves (or copies) the selection to it."""
        menu = tk.Menu(parent, tearoff=False)
        for i, erm in enumerate(self.controller.erms):
            menu.add_command(label=f"ERM {i}  ({erm})",
                             command=functools.partial(self.move_selection, ("erm", i), copy))
        menu.add_separator()
        for k, peltier in enumerate(self.controller.peltiers):
            menu.add_command(label=peltier.name,
                             command=functools.partial(self.move_selection, ("peltier", k), copy))
        menu.add_separator()
        for j, lra in enumerate(self.controller.lras):
            menu.add_command(label=f"{lra.name} LRA",
                             command=functools.partial(self.move_selection, ("lra", j), copy))
        return menu

    def move_selection(self, actuator, copy=False):
        """Move (or copy) the selected periods to another actuator, keeping their timing.
        Periods changing kind take the other values from that actuator's tab."""
        kind, index = actuator
        try:
            new = [self._period_on(period, kind, index) for period in self.selection]
        except ValueError as e:
            messagebox.showerror("Can't move periods", str(e), parent=self)
            return
        if not new:
            return
        for old, period in zip(self.selection, new):
            period.enabled = old.enabled
        if not copy:
            for period in self.selection:
                in_list = self._list_for(period)
                del in_list[_index_of(in_list, period)]
        for period in new:
            self._list_for(period).append(period)
        self._changed(select=new)
        self.tabs.select(self._panel_for(new[-1]))

    def _period_on(self, period, kind, index):
        """A period with the timing of `period` on the given actuator."""
        if kind == "erm":
            intensity = (period.intensity if isinstance(period, ErmActivation)
                         else self.erm_panel.form_intensity())
            return ErmActivation([index], period.start, period.duration, intensity=intensity)
        if kind == "peltier":
            if isinstance(period, PeltierActivation):
                self.controller.peltiers[index].check(period.duty, period.heat)
                return PeltierActivation(index, period.start, period.duration, period.duty,
                                         period.heat)
            return self.peltier_panels[index].with_form_values(period)
        if isinstance(period, LraActivation):
            self.controller.lras[index].check(period.frequency, period.amplitude)
            return LraActivation(index, period.start, period.duration, period.frequency,
                                 period.amplitude)
        return self.lra_panels[index].with_form_values(period)

    # --- muting --------------------------------------------------------------

    def _actuators(self):
        """Every actuator's key and name."""
        c = self.controller
        return ([(("erm", i), f"ERM {i}") for i in range(len(c.erms))]
                + [(("peltier", k), peltier.name) for k, peltier in enumerate(c.peltiers)]
                + [(("lra", j), f"{lra.name} LRA") for j, lra in enumerate(c.lras)])

    def is_muted(self, actuator):
        return actuator in self.muted

    def toggle_mute(self, actuator):
        self.muted ^= {actuator}
        self._mutes_changed()

    def solo(self, actuator):
        """Mute every other actuator, or unmute all if this one is soloed already."""
        others = {key for key, _ in self._actuators()} - {actuator}
        self.muted = set() if self.muted == others else others
        self._mutes_changed()

    def _mute_summary(self):
        """"Left LRA soloed", "ERM 1, Right LRA muted", or "" with nothing muted."""
        names = [name for key, name in self._actuators() if key in self.muted]
        playing = [name for key, name in self._actuators() if key not in self.muted]
        if len(playing) == 1 and len(names) > 1:
            return f"{playing[0]} soloed"
        return f"{', '.join(names)} muted" if names else ""

    def _mutes_changed(self):
        self.timeline.redraw()
        summary = self._mute_summary()
        self._set_status(f"{summary}." if summary else "Nothing muted.")

    def _without_muted(self, stimulus):
        """The stimulus with the muted actuators' periods left out."""
        muted = self.muted
        erm_periods = []
        for p in stimulus.erm_activations:
            erms = [i for i in p.erms if ("erm", i) not in muted]
            if erms:
                erm_periods.append(ErmActivation(erms, p.start, p.duration, p.enabled, p.intensity))
        return Stimulus(erm_periods,
                        [p for p in stimulus.peltier_activations
                         if ("peltier", p.peltier) not in muted],
                        stimulus.peltier_warmup,
                        [p for p in stimulus.lra_activations if ("lra", p.lra) not in muted])

    def _on_delete_key(self, event):
        # In a text box these keys edit the text instead.
        if isinstance(event.widget, (tk.Entry, ttk.Entry)) or not self.selection:
            return
        self.remove_periods(self.selection)

    def _changed(self, select):
        self.selection = list(select)
        self._commit()
        self._refresh()

    # --- undo ----------------------------------------------------------------

    @property
    def dirty(self):
        """True if the stimulus differs from the file it was opened from or saved to."""
        return self.stimulus != self._saved

    def _selection_refs(self):
        refs = []
        for period in self.selection:
            key = {ErmActivation: "erm_activations", PeltierActivation: "peltier_activations",
                   LraActivation: "lra_activations"}[type(period)]
            refs.append((key, _index_of(getattr(self.stimulus, key), period)))
        return refs

    def _commit(self):
        """Record a completed edit, so it can be undone."""
        if self.stimulus != self._committed:
            self._undo.append((self._committed, self._committed_selection))
            del self._undo[:-self.UNDO_LIMIT]
            self._redo.clear()
            self._committed = copy.deepcopy(self.stimulus)
        self._committed_selection = self._selection_refs()

    def undo(self):
        self._step(self._undo, self._redo, "Undone", "Nothing to undo.")

    def redo(self):
        self._step(self._redo, self._undo, "Redone", "Nothing to redo.")

    def _step(self, source, target, done, nothing):
        """Go back (or forward) to the state on top of `source`, putting the present one on `target`."""
        self.timeline.cancel_drag()
        if not source:
            self._set_status(nothing)
            return
        target.append((self._committed, self._committed_selection))
        state, refs = source.pop()
        self._committed, self._committed_selection = state, refs
        self.stimulus = copy.deepcopy(state)
        self.selection = [getattr(self.stimulus, key)[i] for key, i in refs]
        self.warmup_var.set(_fmt(self.stimulus.peltier_warmup))
        self._refresh()
        self._set_status(f"{done}.   {len(self._undo)} to undo, {len(self._redo)} to redo.")

    def _refresh(self):
        name = os.path.basename(self.path) if self.path else "Untitled"
        self.title(f"{name}{' *' if self.dirty else ''} — Stimulus Editor")
        self._update_slot_buttons()
        for panel in self._panels():
            panel.refresh()
        self.timeline.redraw()

    # --- files ---------------------------------------------------------------

    def _apply_warmup(self):
        """Take the lead-in from the toolbar: Peltier periods starting at 0 begin this long before it."""
        try:
            warmup = _parse_float(self.warmup_var, "Peltier lead-in")
            if warmup < 0:
                raise ValueError("The Peltier lead-in can't be negative.")
        except ValueError as e:
            self.warmup_var.set(_fmt(self.stimulus.peltier_warmup))
            messagebox.showerror("Invalid lead-in", str(e), parent=self)
            return
        if warmup != self.stimulus.peltier_warmup:
            self.stimulus.peltier_warmup = warmup
            self._commit()
            self._refresh()
        self.warmup_var.set(_fmt(warmup))

    def _set_stimulus(self, stimulus, path):
        self.warmup_var.set(_fmt(stimulus.peltier_warmup))
        self.stimulus = stimulus
        self.path = path
        self.selection = []
        self._saved = copy.deepcopy(stimulus)
        self._committed = copy.deepcopy(stimulus)
        self._committed_selection = []
        self._undo.clear()
        self._redo.clear()
        self._refresh()

    def _confirm_discard(self):
        """True if there are no unsaved changes, or the user saved or chose to discard them."""
        if not self.dirty:
            return True
        answer = messagebox.askyesnocancel("Unsaved changes", "Save changes to this stimulus first?",
                                           parent=self)
        if answer is None:
            return False
        return self.save() if answer else True

    def new_stimulus(self):
        if self._confirm_discard():
            self._set_stimulus(Stimulus(), None)

    def open_stimulus(self, path=None):
        if not self._confirm_discard():
            return
        path = path or filedialog.askopenfilename(
            parent=self, filetypes=[("Stimulus", "*.json"), ("All files", "*.*")])
        if not path:
            return
        try:
            stimulus = Stimulus.load(path)
            self.controller.check(stimulus)
        except Exception as e:
            messagebox.showerror("Can't open stimulus", f"{path}\n\n{e}", parent=self)
            return
        self._set_stimulus(stimulus, path)

    def save(self):
        if self.path is None:
            return self.save_as()
        try:
            self.stimulus.save(self.path)
        except OSError as e:
            messagebox.showerror("Can't save stimulus", str(e), parent=self)
            return False
        self._saved = copy.deepcopy(self.stimulus)
        self._refresh()
        return True

    def save_as(self):
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".json",
                                            filetypes=[("Stimulus", "*.json")])
        if not path:
            return False
        self.path = path
        return self.save()

    # --- compare slots -------------------------------------------------------

    def load_slot(self, slot, current=False):
        """Put a stimulus file in a Compare slot: the one open in the editor, or one picked."""
        if current:
            if self.path is None and not self.save_as():
                return      # it needs a file for the slot to point at
            path = self.path
        else:
            path = filedialog.askopenfilename(
                parent=self, filetypes=[("Stimulus", "*.json"), ("All files", "*.*")])
            if not path:
                return
            try:
                self.controller.check(Stimulus.load(path))
            except Exception as e:
                messagebox.showerror("Can't load stimulus", f"{path}\n\n{e}", parent=self)
                return
        self._set_slot(slot, path)

    def _set_slot(self, slot, path):
        self.slots[slot] = path
        self._update_slot_buttons()

    def _update_slot_buttons(self):
        for slot, (button, key) in self.slot_buttons.items():
            path = self.slots[slot]
            if path is None:
                text = f"▶ {slot}: empty"
            else:
                name = os.path.splitext(os.path.basename(path))[0]
                name = name if len(name) <= 16 else name[:15] + "…"
                edited = path == self.path and self.dirty
                text = f"▶ {slot}: {name}{' *' if edited else ''}"
            button.configure(text=f"{text} ({key})")

    def play_slot(self, slot):
        """Play a Compare slot's stimulus, stopping whatever is playing first."""
        path = self.slots[slot]
        if path is None:
            self._set_status(f"Compare slot {slot} is empty: fill it from its ▾ menu.")
            return
        if path == self.path:
            stimulus = self.stimulus    # as edited, saved or not
        else:
            try:
                stimulus = Stimulus.load(path)
            except Exception as e:
                messagebox.showerror("Can't open stimulus", f"{path}\n\n{e}", parent=self)
                return
        if self.running:
            self._run_stop.set()
            self._run_thread.join(timeout=3)
            if self._run_thread.is_alive():
                self._set_status("The playing stimulus didn't stop in time; try again.")
                return
            self._run_thread = None
            self._set_running(False)
        self.run(stimulus, label=f"{slot}: {os.path.basename(path)}")

    # --- running -------------------------------------------------------------

    @property
    def running(self):
        return self._run_thread is not None

    def run(self, stimulus=None, label=None):
        """Play a stimulus (by default the one being edited) in the background."""
        if self.running:
            return
        # A copy, so edits made while it plays don't affect it.
        stimulus = copy.deepcopy(self.stimulus if stimulus is None else stimulus).enabled_only()
        stimulus = self._without_muted(stimulus)
        if not stimulus.periods:
            self._set_status("Nothing to run: add some periods first, or enable or unmute some.")
            return
        try:
            self.controller.check(stimulus)
        except ValueError as e:
            messagebox.showerror("Can't run stimulus", str(e), parent=self)
            return
        self._run_stop = threading.Event()
        self._run_error = None
        self._run_duration = stimulus.duration
        self._stopped_by_user = False
        self._run_label = (f"{label}   " if label else "") + (
            f"({self._mute_summary()})   " if self.muted else "")
        self._run_thread = threading.Thread(target=self._run_worker, args=(stimulus, self._run_stop),
                                            daemon=True)
        self._run_thread.start()
        self._set_running(True)

    def _run_worker(self, stimulus, stop):
        try:
            self.controller.run(stimulus, stop=stop)
        except Exception as e:
            self._run_error = e

    def stop(self):
        if self.running:
            self._stopped_by_user = True
            self._run_stop.set()

    def _run_finished(self):
        self._run_thread = None
        self._set_running(False)
        if self._run_error is not None:
            self._set_status(f"Stimulus failed: {self._run_error}")
            messagebox.showerror("Stimulus failed", str(self._run_error), parent=self)
        elif self._stopped_by_user:
            self._set_status(f"{self._run_label}Stopped. All actuators off.")
        else:
            self._set_status(f"{self._run_label}Finished. All actuators off.")

    def _set_running(self, running):
        self.run_button.state(["disabled"] if running else ["!disabled"])
        self.stop_button.state(["!disabled"] if running else ["disabled"])
        self.manual.set_running(running)

    def _set_status(self, text):
        self.status.configure(text=text)

    def _poll(self):
        if self._run_thread is not None and not self._run_thread.is_alive():
            self._run_finished()
        elapsed = self.controller.elapsed()
        if elapsed is not None:
            if elapsed < 0:
                self._set_status(f"{self._run_label}Peltier lead-in   {-elapsed:.2f} s to go")
            else:
                self._set_status(f"{self._run_label}Running   "
                                 f"{elapsed:.2f} / {self._run_duration:.2f} s")
        self.timeline.update_live()
        self.manual.update_live()
        self.after(self.POLL_MS, self._poll)

    # --- shutdown ------------------------------------------------------------

    def _on_close(self):
        if self._confirm_discard():
            self.shutdown()

    def shutdown(self):
        """Stop any running stimulus, leave the hardware off/idle and close the window."""
        if self._run_thread is not None:
            self._run_stop.set()
            self._run_thread.join(timeout=3)
        try:
            self.controller.close()
        except Exception as e:
            print(f"Error while shutting down the hardware: {e}")
        self.destroy()


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main():
    editor = StimulusEditor(make_controller(), sys.argv[1] if len(sys.argv) > 1 else None)
    signal.signal(signal.SIGINT, lambda *args: editor.shutdown())   # Ctrl+C in the terminal
    editor.mainloop()


if __name__ == "__main__":
    main()
