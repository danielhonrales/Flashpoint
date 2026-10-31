"""
Pi receiver: plays the saved stimuli when the thermal game reports a combat event.

The game (ThermalGameDemo, CombatEventOutput.cs) sends this Pi one UDP JSON datagram per combat
event, plus a full-state snapshot every 250 ms. Every message carries cumulative counters and the
list of active states, which start and stop the stimuli:

  fire_charge, fire   the player makes the fire pose, then the beam fires   -> patterns/hot_laser.json
  ice_charge          the player makes the ice pose (before the throw)      -> patterns/ice_bomb.json
  shield              the player raises the shield                          -> patterns/shield.json
  blocks              a hit lands on the raised shield                      -> patterns/hit/*.json
  hits                a hit gets through to the player                      -> patterns/hit/*.json

A stimulus is either one file, patterns/<name>.json, or a folder of variants, patterns/<name>/,
from which one is picked at random each time it plays (never the same one twice running). The
hit has hot and cold variants of several impact patterns.

A stimulus started by states stops if they end before it has had its full length: the pose is
dropped before the beam fires or the grenade is thrown, the beam is let go early, or the shield is
lowered early. If they're still active when it finishes, it plays again (another variant, if it
has several), for as long as they last, with only REPEAT_LEAD between plays. The shield loops
seamlessly instead: its pattern is played back to back as one stimulus (up to LOOP_S). States
count as active only while the game keeps sending (its snapshots come every 250 ms), so a dropped
connection doesn't leave a stimulus looping.

The Peltiers are run by their thermistors (ADS1115 A0 and A1, on each Peltier's skin face), not by
the patterns' duty: a Peltier period heats or cools at full power (PELTIER_MAX_DUTY), and a PID
loop per Peltier, on its own thermistor, scales heating back as the skin face nears HOT_SETPOINT,
so it holds there rather than climbing. HOT_LIMIT is a hard limit on top: a Peltier reading it
stops heating at once, and doesn't heat again until it has cooled to HOT_RESUME. Cooling likewise
stops at COLD_LIMIT, until back up to COLD_RESUME. A Peltier whose thermistor can't be read (an
open or shorted NTC, or no ADS1115) doesn't run at all.
Once the grenade is thrown (the "iceThrows" counter goes up), ice_bomb plays to the
end. States count as ended once they have stayed ended for END_GRACE seconds, which bridges the
game switching from fire_charge to fire. A hit interrupts the shield stimulus rather than stopping
it: once the hit has played, the shield stimulus carries on from where it would be by then, if the
shield is still raised.

Watching counters and states rather than event names means a lost event datagram is made up by
the next snapshot, at most 250 ms late. Only one stimulus plays at a time, as they share the
actuators: a new one stops the one playing, but a stimulus that is already playing isn't restarted,
except the hit (the game reports a block up to 4 times a second while a beam is on the shield):
each block starts a new hit variant. A hit heats or cools to match the attack the game names in
the message's ATTACK_FIELD (a fire beam is hot, an ice bomb cold); if it doesn't name one, a new
hit heats if the last one heated and cooled if it cooled, so the Peltiers keep going one way.

Usage: python3 pi_receiver.py --name Player1 [--device quest-a] [--bind 192.168.1.249] [--port 7779]

It listens on every address the Pi has, so it keeps working when the Pi's address changes (a new
network, a new DHCP lease); it logs the Pi's addresses at startup. The Quests find it by name:
every BEACON_S it broadcasts "THERMAL-PI-1:<name>" to UDP DISCOVERY_PORT, and the game's Select Pi
menu shows a button for each Pi it hears, so no Pi address is set in the game. Name the two Pis
Player1 and Player2 (--name; it defaults to the Pi's hostname). The router has to pass broadcasts
between its clients. Several Quests can pick the same Pi, so pass --device to react only to the
wearer's own headset.

Needs stimulus_editor.py (which sets up the devices, as configured at its top) and the stimuli in
patterns/. Any device that can't be opened is simulated, as in the editor, and
listed at startup.
"""

import argparse
import errno
import functools
import json
import math
import os
import random
import signal
import socket
import subprocess
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field, replace

from stimulus_editor import (PELTIER_MAX_DUTY, ErmActivation, LraActivation, PeltierActivation,
                             Stimulus, StimulusController, make_controller)

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

BIND = "0.0.0.0"        # listen on every address the Pi has (--bind picks one instead)
PORT = 7779

# Announcing this Pi to the Quests (PiSelectionMenu.cs in the game): BEACON_PREFIX + its name, as
# UTF-8, broadcast to DISCOVERY_PORT every BEACON_S. The game greys a Pi out after 3 s unheard.
DISCOVERY_PORT = 7780
BEACON_PREFIX = "THERMAL-PI-1:"
BEACON_S = 1.0
MAX_NAME = 24           # the game ignores longer names

# Counters in every game message, with the event the game sends when each goes up. The stimulus
# plays when the counter goes up, and always plays to the end.
COUNTER_TRIGGERS = [
    # counter   event            stimulus
    ("blocks",  "shield_block",  "hit"),
    ("hits",    "hit_received",  "hit"),
]

# States in a message's "active" list. The stimulus plays when one of them becomes active, and
# stops if they all end before it has had its full length, unless the counter given has gone up
# by then: that lets it play to the end.
STATE_TRIGGERS = [
    # states                    stimulus      plays to the end once this goes up
    (("shield",),               "shield",     None),
    (("ice_charge",),           "ice_bomb",   "iceThrows"),
    (("fire_charge", "fire"),   "hot_laser",  None),
]
# If several stimuli start in one message (only after lost datagrams), the first listed wins,
# counters before states.

COUNTERS = ([counter for counter, _, _ in COUNTER_TRIGGERS]
            + [counter for _, _, counter in STATE_TRIGGERS if counter])

# Which attack a hit or block came from: this field of the game's message, whose value is matched
# (ignoring case) against these words. A hit then plays a variant that heats for a hot attack and
# cools for a cold one; with no field, or a value matching neither, a random variant plays.
ATTACK_FIELD = "attack"
HOT_ATTACKS = ("fire", "laser", "heat", "hot", "beam")
COLD_ATTACKS = ("ice", "bomb", "cold", "frost", "grenade")
TEMPERATURE_MATCHED = {"hit"}   # stimuli whose variant follows the attack's temperature

# Stimuli that start again (another variant, heating or cooling as before) when triggered while
# already playing, rather than playing on.
RETRIGGER = {"hit"}

# Stimulus -> the stimulus it interrupts. The interrupted one carries on afterwards from where it
# would be by then, unless its state has ended meanwhile.
INTERRUPTS = {"hit": "shield"}

END_GRACE = 0.3         # seconds a trigger's states must stay ended before its stimulus stops
FRESH_S = 1.0           # seconds a session's states count as current without a new message
REPEAT_LEAD = 0.05      # seconds between one play of a repeating stimulus and the next
SEAMLESS = {"shield"}   # stimuli looped back to back, with no gap at all, while their action lasts
LOOP_S = 60.0           # how long one seamless loop runs (it then repeats like any other)
SESSION_TIMEOUT = 60.0  # seconds without a message before a session is forgotten

# Thermistors (HARDWARE.md §4): an ADS1115 reading each Peltier's NTC divider,
# 3.3 V -> NTC_FIXED_OHMS -> junction (the ADS1115 input) -> NTC -> GND.
ADS_ADDRESS = 0x48
THERMISTOR_INPUTS = [0, 1]      # ADS1115 input per Peltier: A0 Peltier 1, A1 Peltier 2
NTC_SUPPLY_V = 3.3
NTC_FIXED_OHMS = 10_000
NTC_R25 = 10_000                # the NTC's resistance at 25 °C...
NTC_BETA = 3950                 # ...and its beta
# Temperature limits on each Peltier's skin face, °C. Heating is held at HOT_SETPOINT by the PID
# and cut at HOT_LIMIT until back down to HOT_RESUME; cooling is cut at COLD_LIMIT until back up
# to COLD_RESUME (HARDWARE.md §6's TEMP_MIN_C, the cold-pain threshold).
HOT_LIMIT = 40.0
HOT_SETPOINT = 39.5
HOT_RESUME = 39.0
COLD_LIMIT = 15.0
COLD_RESUME = 16.0
# Heating PID: duty fraction = KP * error + KI * integral - KD * rate of rise, error being
# HOT_SETPOINT minus the temperature, clamped to 0-1. Full power until about 1/KP °C below the
# setpoint (sooner while it's rising fast), so the skin face's lag doesn't carry it past.
PID_KP = 0.5            # per °C
PID_KI = 0.05           # per °C·s; integrates only while the output isn't pinned at 0 or 1
PID_KD = 0.4            # per °C/s
PID_RATE_SMOOTHING = 0.3    # 0-1: how much of each new rate-of-rise sample is taken in
THERMAL_TICK = 0.05     # seconds between control updates (each reads both thermistors)
THERMAL_LOG_S = 1.0     # how often the temperatures are logged while a Peltier is on

HERE = os.path.dirname(os.path.abspath(__file__))
PATTERNS = os.path.join(HERE, "patterns")


def load_variants(name):
    """[(label, Stimulus)]: the files in patterns/<name>/ if that folder exists, else
    patterns/<name>.json on its own."""
    folder = os.path.join(PATTERNS, name)
    if os.path.isdir(folder):
        paths = sorted(os.path.join(folder, f) for f in os.listdir(folder) if f.endswith(".json"))
        if not paths:
            raise FileNotFoundError(f"{folder} has no .json patterns")
    else:
        paths = [os.path.join(PATTERNS, f"{name}.json")]
    return [(os.path.basename(path)[:-5], Stimulus.load(path)) for path in paths]


def own_addresses():
    """This Pi's IPv4 addresses on its networks (no internet needed), or [] if it has none."""
    try:
        out = subprocess.run(["hostname", "-I"], capture_output=True, text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [address for address in out.split() if "." in address]


def broadcast_addresses():
    """Where to send the beacon: the limited broadcast, and each network's own broadcast address,
    which still works when the Pi has no default route (a router with no internet)."""
    targets = ["255.255.255.255"]
    try:
        out = subprocess.run(["ip", "-o", "-4", "addr", "show", "scope", "global"],
                             capture_output=True, text=True, timeout=2).stdout
    except (OSError, subprocess.SubprocessError):
        return targets
    for line in out.splitlines():
        words = line.split()
        if "brd" in words and words.index("brd") + 1 < len(words):
            targets.append(words[words.index("brd") + 1])
    return list(dict.fromkeys(targets))


def valid_name(name):
    """A name the game will show: 1 to MAX_NAME characters, none of them control characters."""
    return 0 < len(name) <= MAX_NAME and name.isprintable()


class Beacon:
    """Broadcasts this Pi's name every BEACON_S on a background thread, so the Quests can list
    it. It keeps trying while the Pi has no network, and logs when sending starts and stops
    working."""

    def __init__(self, name):
        self.payload = (BEACON_PREFIX + name).encode()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        working = None
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            while True:
                sent = False
                for target in broadcast_addresses():
                    try:
                        sock.sendto(self.payload, (target, DISCOVERY_PORT))
                        sent = True
                    except OSError:
                        pass    # no network yet, or no route for this address
                if sent != working:
                    log(f"announcing \"{self.payload.decode()}\" on UDP {DISCOVERY_PORT}" if sent else
                        "can't announce this Pi: no network; still trying")
                    working = sent
                if self._stop.wait(BEACON_S):
                    return

    def stop(self):
        self._stop.set()
        self._thread.join()


def log(text):
    print(f"{time.strftime('%H:%M:%S')}  {text}", flush=True)


# ----------------------------------------------------------------------------
# Game messages
# ----------------------------------------------------------------------------

def parse(payload: bytes) -> dict | None:
    """The game message in a datagram, or None if it isn't one."""
    try:
        msg = json.loads(payload)
        if (msg.get("schema") == 1 and isinstance(msg.get("session"), str)
                and isinstance(msg.get("seq"), int) and isinstance(msg.get("event"), str)
                and isinstance(msg.get("active"), list)
                and all(isinstance(msg.get(counter), int) for counter in COUNTERS)):
            return msg
    except (ValueError, AttributeError):
        pass
    return None


def levels(msg: dict) -> dict:
    """Each counter's value, and for each trigger's states 1 if any of them is active, else 0."""
    active = set(msg["active"])
    return ({counter: msg[counter] for counter in COUNTERS}
            | {states: int(not active.isdisjoint(states)) for states, _, _ in STATE_TRIGGERS})


@dataclass
class _Session:
    seq: int
    levels: dict
    seen: float
    ending: dict[str, float] = field(default_factory=dict)  # stimulus -> when its states ended


class SessionTracker:
    """Follows the counters and states of each game session (one run of the app on one headset)."""

    def __init__(self):
        self.sessions: dict[str, _Session] = {}

    def changes(self, msg: dict, now: float) -> tuple[str | None, list[str]]:
        """Take in a message; return the stimulus it starts, or None, and the stimuli it lets play
        to the end. Stimuli whose states end are reported later, by ended()."""
        for key in [key for key, s in self.sessions.items() if now - s.seen > SESSION_TIMEOUT]:
            del self.sessions[key]
        current = levels(msg)
        session = self.sessions.get(msg["session"])
        if session is None:
            # A session we haven't heard from: its counts and states so far are old news, apart
            # from what this message itself reports.
            before = dict(current)
            for counter, event, _ in COUNTER_TRIGGERS:
                if msg["event"] == event:
                    before[counter] -= 1
            for states, _, _ in STATE_TRIGGERS:
                if msg["event"] in [f"{state}_start" for state in states]:
                    before[states] = 0
            session = self.sessions[msg["session"]] = _Session(msg["seq"], before, now)
        elif msg["seq"] <= session.seq:
            return None, []     # duplicate or out-of-order datagram
        before = session.levels
        session.seq, session.levels, session.seen = msg["seq"], current, now

        started = next((stimulus for counter, _, stimulus in COUNTER_TRIGGERS
                        if current[counter] > before[counter]), None)
        for states, stimulus, _ in STATE_TRIGGERS:
            if current[states] > before[states]:
                if stimulus in session.ending:
                    del session.ending[stimulus]    # back within END_GRACE: it carries on
                else:
                    started = started or stimulus
            elif current[states] < before[states]:
                session.ending.setdefault(stimulus, now)
        plays_out = [stimulus for _, stimulus, counter in STATE_TRIGGERS
                     if counter and current[counter] > before[counter]]
        return started, plays_out

    def still_active(self, stimulus: str, now: float) -> bool:
        """Whether the states behind a state-triggered stimulus are active now, in a session
        heard from within FRESH_S (and not on their way out)."""
        states = next((s for s, name, _ in STATE_TRIGGERS if name == stimulus), None)
        return states is not None and any(
            now - session.seen <= FRESH_S and session.levels.get(states)
            and stimulus not in session.ending for session in self.sessions.values())

    def ended(self, now: float) -> list[tuple[str, float]]:
        """Stimuli whose states have stayed ended for END_GRACE seconds, with when they ended."""
        due = []
        for session in self.sessions.values():
            for stimulus, at in list(session.ending.items()):
                if now - at >= END_GRACE:
                    del session.ending[stimulus]
                    due.append((stimulus, at))
        return due


# ----------------------------------------------------------------------------
# Playback
# ----------------------------------------------------------------------------

def attack_heats(msg: dict) -> bool | None:
    """True if the message reports a hot attack, False for a cold one, None if it doesn't say."""
    attack = str(msg.get(ATTACK_FIELD, "")).lower()
    hot = any(word in attack for word in HOT_ATTACKS)
    cold = any(word in attack for word in COLD_ATTACKS)
    return hot if hot != cold else None


def heats(stimulus: Stimulus):
    """Whether each Peltier period heats: the same for the hot (or cold) variants of a stimulus."""
    return {p.heat for p in stimulus.peltier_activations}


def looped(stimulus: Stimulus, seconds: float) -> Stimulus:
    """The stimulus played back to back for about `seconds`, as one stimulus. Periods that meet
    one of the same kind and settings are merged, so a steady hum stays one period."""
    copies = max(1, round(seconds / stimulus.duration))

    def repeat(periods):
        out = []
        for n in range(copies):
            for p in sorted(periods, key=lambda p: p.start):
                p = replace(p, start=round(p.start + n * stimulus.duration, 6))
                same = next((q for q in out if abs(q.end - p.start) < 1e-6 and
                             replace(q, start=0, duration=1) ==
                             replace(p, start=0, duration=1)), None)
                if same:
                    same.duration = round(p.end - same.start, 6)
                else:
                    out.append(p)
        return out

    return Stimulus(repeat(stimulus.erm_activations), repeat(stimulus.peltier_activations),
                    stimulus.peltier_warmup, repeat(stimulus.lra_activations))


def remaining(stimulus: Stimulus, offset: float) -> Stimulus:
    """What is left of a stimulus `offset` seconds in, as a stimulus starting at 0 (no Peltier
    lead-in)."""
    offset = max(0.0, offset)

    def clip(period):
        start = max(period.start, offset)
        return start - offset, period.end - start

    left = lambda periods: [p for p in periods if p.end > offset]
    return Stimulus([ErmActivation(p.erms, *clip(p), intensity=p.intensity)
                     for p in left(stimulus.erm_activations)],
                    [PeltierActivation(p.peltier, *clip(p), p.duty, p.heat)
                     for p in left(stimulus.peltier_activations)],
                    0.0,
                    [LraActivation(p.lra, *clip(p), p.frequency, p.amplitude)
                     for p in left(stimulus.lra_activations)])


class Thermistors:
    """The Peltiers' NTCs, read through the ADS1115. read() gives each one's °C, or None if it
    can't be read or reads open or shorted."""

    def __init__(self):
        self.sim_reason = None
        self._bus = None
        try:
            from smbus2 import SMBus
            self._bus = SMBus(1)
            self._bus.read_byte_data(ADS_ADDRESS, 0x01)     # is it there?
        except Exception as e:
            self.sim_reason = f"ADS1115: {type(e).__name__}: {e}"
            if self._bus is not None:
                self._bus.close()
            self._bus = None

    def _volts(self, channel):
        # Single shot, A<channel> against GND, +-4.096 V range, 860 samples/s.
        config = 0x8000 | ((4 + channel) << 12) | (1 << 9) | (1 << 8) | (7 << 5) | 0x3
        self._bus.write_i2c_block_data(ADS_ADDRESS, 0x01, [config >> 8, config & 0xFF])
        deadline = time.perf_counter() + 0.02
        while not self._bus.read_i2c_block_data(ADS_ADDRESS, 0x01, 2)[0] & 0x80:
            if time.perf_counter() > deadline:
                raise TimeoutError("ADS1115 conversion")
            time.sleep(0.0005)
        high, low = self._bus.read_i2c_block_data(ADS_ADDRESS, 0x00, 2)
        raw = (high << 8) | low
        return (raw - 0x10000 if raw & 0x8000 else raw) * 4.096 / 0x8000

    def read(self):
        temps = []
        for channel in THERMISTOR_INPUTS:
            try:
                v = self._volts(channel)
            except Exception:
                temps.append(None)
                continue
            if not 0.05 < v < NTC_SUPPLY_V - 0.05:     # open or shorted
                temps.append(None)
                continue
            ohms = NTC_FIXED_OHMS * v / (NTC_SUPPLY_V - v)
            temps.append(1 / (1 / 298.15 + math.log(ohms / NTC_R25) / NTC_BETA) - 273.15)
        return temps

    def close(self):
        if self._bus is not None:
            self._bus.close()


@dataclass
class _Loop:
    """One Peltier's control state."""
    temp: float | None = None
    integral: float = 0.0
    rate: float = 0.0           # smoothed °C/s
    hot_cut: bool = False       # past HOT_LIMIT, not yet back to HOT_RESUME
    cold_cut: bool = False      # past COLD_LIMIT, not yet back to COLD_RESUME
    fault: bool = False         # its thermistor can't be read
    scale: float = 1.0          # the heating PID's output


class ThermalGovernor:
    """Runs the Peltiers from their thermistors. It takes over each Peltier's on() and off(): a
    stimulus's on() just says which way it wants that Peltier to go, and every THERMAL_TICK the
    Peltier is driven that way at PELTIER_MAX_DUTY, scaled by the heating PID while heating, or
    off while a limit is cut or its thermistor can't be read."""

    def __init__(self, peltiers, thermistors: Thermistors):
        self.peltiers = peltiers
        self.thermistors = thermistors
        self.loops = [_Loop() for _ in peltiers]
        self._wanted = {}   # Peltier index -> True heating / False cooling, as asked for
        self._on = [p.on for p in peltiers]
        self._off = [p.off for p in peltiers]
        for i, p in enumerate(peltiers):
            p.on = functools.partial(self._request_on, i)
            p.off = functools.partial(self._request_off, i)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._update(THERMAL_TICK, log_changes=False)   # temperatures before the first play
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _request_on(self, i, duty, heat):
        self.peltiers[i].check(duty, heat)
        with self._lock:
            if self._wanted.get(i) != heat:
                loop = self.loops[i]
                loop.integral, loop.rate, loop.scale = 0.0, 0.0, 1.0    # a fresh run
            self._wanted[i] = heat
            self._apply(i)

    def _request_off(self, i):
        with self._lock:
            self._wanted.pop(i, None)
            self._off[i]()

    def _duty(self, i):
        """The duty Peltier i is to run at now (the lock is held)."""
        heat, loop = self._wanted.get(i), self.loops[i]
        if heat is None or loop.fault:
            return 0.0
        if heat:
            return 0.0 if loop.hot_cut else PELTIER_MAX_DUTY * loop.scale
        return 0.0 if loop.cold_cut else PELTIER_MAX_DUTY

    def _apply(self, i):
        """Drive Peltier i at _duty() (the lock is held)."""
        peltier, duty = self.peltiers[i], self._duty(i)
        if duty < 0.01:
            if peltier.is_on:
                self._off[i]()
        elif abs(duty - peltier.duty) > 0.005 or peltier.heat != self._wanted[i]:
            self._on[i](duty, self._wanted[i])

    def _update(self, dt, log_changes=True):
        """Read the thermistors and update each loop (the lock is not held)."""
        if self.thermistors.sim_reason:
            temps = [None] * len(self.peltiers)
        else:
            temps = self.thermistors.read()
        events = []
        with self._lock:
            for i, (loop, temp) in enumerate(zip(self.loops, temps)):
                name = self.peltiers[i].name
                if temp is None:
                    if not loop.fault:
                        events.append(f"{name}: thermistor can't be read, Peltier off")
                    loop.fault, loop.temp = True, None
                    self._apply(i)
                    continue
                if loop.fault:
                    events.append(f"{name}: thermistor back ({temp:.1f} °C)")
                    loop.fault = False
                previous, loop.temp = loop.temp, temp
                if previous is not None:
                    loop.rate += PID_RATE_SMOOTHING * ((temp - previous) / dt - loop.rate)
                if temp >= HOT_LIMIT and not loop.hot_cut:
                    loop.hot_cut = True
                    events.append(f"{name}: {temp:.1f} °C, at the {HOT_LIMIT:g} °C limit, "
                                  "heating off")
                elif temp <= HOT_RESUME and loop.hot_cut:
                    loop.hot_cut = False
                    events.append(f"{name}: {temp:.1f} °C, heating allowed again")
                if temp <= COLD_LIMIT and not loop.cold_cut:
                    loop.cold_cut = True
                    events.append(f"{name}: {temp:.1f} °C, at the {COLD_LIMIT:g} °C limit, "
                                  "cooling off")
                elif temp >= COLD_RESUME and loop.cold_cut:
                    loop.cold_cut = False
                    events.append(f"{name}: {temp:.1f} °C, cooling allowed again")
                if self._wanted.get(i) is True and not loop.hot_cut:
                    error = HOT_SETPOINT - temp
                    output = PID_KP * error + loop.integral - PID_KD * max(0.0, loop.rate)
                    if 0.0 < output < 1.0:      # anti-windup: integrate only when not pinned
                        loop.integral += PID_KI * error * dt
                        output = PID_KP * error + loop.integral - PID_KD * max(0.0, loop.rate)
                    loop.scale = min(1.0, max(0.0, output))
                self._apply(i)
        if log_changes:
            for event in events:
                log(event)
        return temps

    def status(self):
        """Each Peltier's temperature and what it's doing, for the log."""
        parts = []
        with self._lock:
            for i, (peltier, loop) in enumerate(zip(self.peltiers, self.loops)):
                temp = "?" if loop.temp is None else f"{loop.temp:.1f} °C"
                heat = self._wanted.get(i)
                doing = ("idle" if heat is None else
                         f"{'heat' if heat else 'cool'} {self._duty(i) / PELTIER_MAX_DUTY:.0%}")
                parts.append(f"{peltier.name} {temp} {doing}")
        return ", ".join(parts)

    def _run(self):
        last = time.perf_counter()
        last_log = 0.0
        while not self._stop.wait(THERMAL_TICK):
            now = time.perf_counter()
            self._update(now - last)
            last = now
            if self._wanted and now - last_log >= THERMAL_LOG_S:
                log(self.status())
                last_log = now

    def stop(self):
        self._stop.set()
        self._thread.join()
        self.thermistors.close()


class Player:
    """Plays one stimulus at a time on a background thread. Only the main thread calls its
    methods; the background thread just runs the controller."""

    def __init__(self, controller: StimulusController,
                 stimuli: dict[str, list[tuple[str, Stimulus]]]):
        self.controller = controller
        self.stimuli = stimuli      # name -> its variants, (label, stimulus)
        self.name = None            # the stimulus playing, or the last one played
        self.variant = None         # the label of the variant playing, or last played
        self._full = None           # that variant in full (a resumed run plays only its rest)
        self._triggered = 0.0       # perf_counter() when it was triggered
        self._offset = 0.0          # seconds into it this run started (0 unless it was interrupted)
        self._plays_out = False     # True once it is to play to the end whatever its states do
        self._thread = None
        self._stop = threading.Event()
        self._interrupted = None    # (name, triggered, time 0, variant, stimulus) to carry on

    def playing(self):
        return self._thread is not None and self._thread.is_alive()

    def play(self, name, lead=0.2, heat=None):
        """Start the named stimulus (a random variant of it), stopping the one playing, or
        pausing it if the new one interrupts it. False if the named stimulus was already playing.
        `lead` is the pause before it starts. heat=True or False picks a variant that heats or
        cools."""
        retrigger = self.playing() and self.name == name
        if retrigger and name not in RETRIGGER:
            return False
        interrupted = self._interrupted if retrigger else None     # still to carry on after it
        if self.playing() and INTERRUPTS.get(name) == self.name:
            t0 = time.perf_counter() - self._offset - (self.controller.elapsed() or 0.0)
            interrupted = (self.name, self._triggered, t0, self.variant, self._full)
        self.stop()
        self._interrupted = interrupted
        variants = self.stimuli[name]
        if heat is not None:
            matching = [v for v in variants if heats(v[1]) == {heat}]
            variants = matching or variants
        elif retrigger:     # keep heating or cooling, so the Peltiers don't flip and cancel out
            same = [v for v in variants if heats(v[1]) == heats(self._full)]
            variants = same or variants
        if len(variants) > 1 and self.name == name:     # not the same variant twice running
            variants = [v for v in variants if v[0] != self.variant]
        label, stimulus = random.choice(variants)
        if name in SEAMLESS:
            stimulus = looped(stimulus, LOOP_S)
        self._start(name, label, stimulus, stimulus, time.perf_counter(), 0.0, lead)
        return True

    def play_out(self, name):
        """Let the named stimulus, if playing, play to the end whatever its states do."""
        if self.playing() and self.name == name:
            self._plays_out = True

    def update(self):
        """Once an interrupting stimulus has played, carry on with the one it interrupted.
        Returns (name, seconds in) if it did."""
        if self._interrupted is None or self.playing():
            return None
        name, triggered, t0, label, full = self._interrupted
        self._interrupted = None
        offset = time.perf_counter() - t0
        rest = remaining(full, offset)
        if not rest.periods:
            return None
        self._start(name, label, full, rest, triggered, offset, REPEAT_LEAD)
        return name, offset

    def action_ended(self, name, ended_at):
        """The states behind the named stimulus ended at perf_counter() `ended_at`. Stop the
        stimulus (or drop it, if it's interrupted) when they lasted less than the stimulus;
        True if so. States as long as the stimulus still end before it, by the lead-in the
        stimulus starts after, and that last bit is left to finish."""
        if self._interrupted is not None and self._interrupted[0] == name:
            self._interrupted = None
            return True
        if (self.playing() and self.name == name and not self._plays_out
                and ended_at - self._triggered < self._full.duration):
            self.stop()
            return True
        return False

    def _start(self, name, label, full, stimulus, triggered, offset, lead=0.2):
        """Play `stimulus` (all of the variant `full`, or what's left of it)."""
        self.name, self.variant, self._full = name, label, full
        self._triggered, self._offset = triggered, offset
        self._plays_out = False
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(name, stimulus, self._stop, lead),
                                        daemon=True)
        self._thread.start()

    def _run(self, name, stimulus, stop, lead):
        try:
            self.controller.run(stimulus, stop=stop, verbose=False, lead=lead)
        except Exception as e:
            log(f"{name} failed: {e}")

    def stop(self):
        """Stop the stimulus playing, if any; the controller leaves every actuator off."""
        if self._thread is not None:
            self._stop.set()
            self._thread.join()
            self._thread = None


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Play stimuli when the thermal game reports combat events.")
    parser.add_argument("--bind", default=BIND,
                        help=f"address to listen on (default {BIND}: any)")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--device", help="only react to this headset's deviceLabel, e.g. quest-a")
    parser.add_argument("--name", default=socket.gethostname()[:MAX_NAME],
                        help="name the Quests' Select Pi menu shows, Player1 or Player2 "
                             "(default: this Pi's hostname)")
    args = parser.parse_args()
    args.name = args.name.strip()
    if not valid_name(args.name):
        parser.error(f"--name must be 1 to {MAX_NAME} printable characters")
    signal.signal(signal.SIGTERM, lambda *args: sys.exit(0))    # systemctl stop: clean up too

    controller = make_controller()
    governor = ThermalGovernor(controller.peltiers, Thermistors())
    player = Player(controller, {})
    beacon = None
    try:
        for name in dict.fromkeys([name for _, _, name in COUNTER_TRIGGERS]
                                  + [name for _, name, _ in STATE_TRIGGERS]):
            variants = load_variants(name)
            for label, stimulus in variants:
                controller.check(stimulus)
                controller.check_safety(stimulus)
            # Disabled periods never play.
            player.stimuli[name] = [(label, s.enabled_only()) for label, s in variants]
            log(f"{name}: {len(variants)} variant{'s' if len(variants) > 1 else ''}"
                + (f" ({', '.join(label for label, _ in variants)})" if len(variants) > 1 else ""))
        controller.reset()      # start from a known state

        simulated = defaultdict(list)       # reason -> device names
        for name, device in ([(f"ERM {i}", erm) for i, erm in enumerate(controller.erms)]
                             + [(peltier.name, peltier) for peltier in controller.peltiers]
                             + [(f"{lra.name} LRA", lra) for lra in controller.lras]):
            if device.sim_reason:
                simulated[device.sim_reason].append(name)
        for reason, names in simulated.items():
            log(f"SIMULATED {', '.join(names)}: {reason}")

        tracker = SessionTracker()
        repeat = None           # the state-triggered stimulus to repeat while its states last
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.bind((args.bind, args.port))
            except OSError as e:
                hint = ("Something else is listening on that port, probably the old dummy receiver:\n"
                        "    sudo systemctl disable --now thermal-game-receiver"
                        if e.errno == errno.EADDRINUSE else
                        f"This Pi's addresses: {', '.join(own_addresses()) or 'none'}. "
                        "Pass one of them to --bind, or leave --bind out to listen on any.")
                sys.exit(f"Can't listen on {args.bind}:{args.port}: {e}\n{hint}")
            sock.settimeout(0.02)   # short, so ends, repeats and resumes are handled promptly
            log(f"Listening on {args.bind}:{args.port}"
                + (f" for {args.device}" if args.device else " for any headset"))
            addresses = own_addresses()
            if addresses:
                log(f"This Pi's address: {', '.join(addresses)}")
            else:
                log("This Pi has no network address yet; connect it to the Quests' network")
            beacon = Beacon(args.name)  # the Quests list this Pi as args.name
            if governor.thermistors.sim_reason:
                log(f"NO THERMISTORS, Peltiers off: {governor.thermistors.sim_reason}")
            log(f"Thermal control: heating held at {HOT_SETPOINT:g} °C, cut at {HOT_LIMIT:g} °C "
                f"(back at {HOT_RESUME:g}), cooling cut at {COLD_LIMIT:g} °C "
                f"(back at {COLD_RESUME:g}); now {governor.status()}")
            while True:
                for name, ended_at in tracker.ended(time.perf_counter()):
                    if player.action_ended(name, ended_at):
                        log(f"stop {name}: its action ended early")
                resumed = player.update()
                if resumed:
                    log(f"{resumed[0]} carries on from {resumed[1]:.2f} s")
                now = time.perf_counter()
                if repeat and not player.playing():
                    if tracker.still_active(repeat, now):
                        player.play(repeat, REPEAT_LEAD)
                        which = (f" [{player.variant}]" if len(player.stimuli[repeat]) > 1 else "")
                        log(f"repeat {repeat}{which}, its action is still active")
                    else:
                        repeat = None
                try:
                    payload, _ = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                msg = parse(payload)
                if msg is None or (args.device and msg.get("device") != args.device):
                    continue
                started, plays_out = tracker.changes(msg, time.perf_counter())
                for name in plays_out:
                    player.play_out(name)
                if started:
                    if any(name == started for _, name, _ in STATE_TRIGGERS):
                        repeat = started
                    chained = started in RETRIGGER and player.playing() and player.name == started
                    heat = attack_heats(msg) if started in TEMPERATURE_MATCHED else None
                    played = player.play(started, heat=heat)
                    which = (f" [{player.variant}]"
                             if played and len(player.stimuli[started]) > 1 else "")
                    attack = (f" ({ATTACK_FIELD}={msg[ATTACK_FIELD]!r})"
                              if started in TEMPERATURE_MATCHED and ATTACK_FIELD in msg else "")
                    log(f"{msg.get('device')} {msg['event']}{attack} -> {started}{which}"
                        + ("" if played else " (already playing)")
                        + (" (again)" if played and chained else ""))
    except KeyboardInterrupt:
        pass
    finally:
        if beacon:
            beacon.stop()
        player.stop()
        governor.stop()
        controller.close()


if __name__ == "__main__":
    main()
