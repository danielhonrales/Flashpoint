"""
Pi receiver: plays the saved stimuli when the thermal game reports a combat event.

The game (ThermalGameDemo, CombatEventOutput.cs) sends this Pi one UDP JSON datagram per combat
event, plus a full-state snapshot every 250 ms. Every message carries cumulative counters and the
list of active states, and a counter going up or a state becoming active plays a stimulus:

  fire          the player fires the heat beam            -> hot_laser.json
  iceThrows     the player throws an ice grenade          -> ice_bomb.json
  shield        the player raises the shield              -> shield.json
  blocks        a hit lands on the raised shield          -> hit.json

A hit that gets through to the player (the "hits" counter) plays nothing for now.

When a state ends before its stimulus has had its full length (the beam or shield is let go
early), the stimulus stops. hit.json interrupts shield.json rather than stopping it: once the hit
has played, the shield stimulus carries on from where it would be by then, if the shield is
still raised.

Watching counters and states rather than event names means a lost event datagram is made up by
the next snapshot, at most 250 ms late. Only one stimulus plays at a time, as they share the ERMs
and PSUs: a new one stops the one playing, but a stimulus that is already playing isn't restarted
(the game reports a block up to 4 times a second while a beam is on the shield).

Usage: python3 pi_receiver.py [--device quest-a] [--bind 192.168.1.5] [--port 7779]

Each Quest's combat-output.json must point at this Pi, e.g.
    {"udpEnabled":true,"host":"192.168.1.5","port":7779,"deviceLabel":"quest-a"}
Both Quests send here by default, so pass --device to react only to the wearer's own headset.

Needs stimulus_editor.py and the stimulus JSON files in this folder. Any ERM or PSU that can't be
opened is simulated, as in the editor, and listed at startup.
"""

import argparse
import json
import os
import signal
import socket
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass

from stimulus_editor import (ERM, ERM_PINS, PSU, PSU1_PORT, PSU2_PORT, ErmActivation, ErmType,
                             PsuActivation, Stimulus, StimulusController)

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

PI_IP = "192.168.1.5"   # the "host" in each Quest's combat-output.json
PORT = 7779

# A counter in every game message, or a state in its "active" list, and the stimulus played when
# the counter goes up or the state becomes active. The event is the one the game sends when that
# happens, for the first message heard from a game session. If several happen in one message
# (only after lost datagrams), the first listed wins.
TRIGGERS = [
    # counter/state   event            stimulus
    ("blocks",        "shield_block",  "hit"),
    ("shield",        "shield_start",  "shield"),
    ("iceThrows",     "ice_shot",      "ice_bomb"),
    ("fire",          "fire_start",    "hot_laser"),
]
# A state's stimulus stops if the state ends before the stimulus has had its full length. The rest
# are counters, whose stimuli always play to the end.
STATES = {"shield", "fire"}

# Stimulus -> the stimulus it interrupts. The interrupted one carries on afterwards from where it
# would be by then, unless its state has ended meanwhile.
INTERRUPTS = {"hit": "shield"}

SESSION_TIMEOUT = 60.0  # seconds without a message before a session is forgotten

HERE = os.path.dirname(os.path.abspath(__file__))


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
                and all(isinstance(msg.get(key), int) for key, _, _ in TRIGGERS if key not in STATES)):
            return msg
    except (ValueError, AttributeError):
        pass
    return None


def levels(msg: dict) -> dict[str, int]:
    """Each trigger's counter value, or 1/0 for a state that is active/inactive."""
    return {key: int(key in msg["active"]) if key in STATES else msg[key] for key, _, _ in TRIGGERS}


@dataclass
class _Session:
    seq: int
    levels: dict[str, int]
    seen: float


class SessionTracker:
    """Follows the counters and states of each game session (one run of the app on one headset)."""

    def __init__(self):
        self.sessions: dict[str, _Session] = {}

    def changes(self, msg: dict, now: float) -> tuple[str | None, list[str]]:
        """Take in a message; return the stimulus it starts, or None, and the stimuli whose state
        it ends."""
        for key in [key for key, s in self.sessions.items() if now - s.seen > SESSION_TIMEOUT]:
            del self.sessions[key]
        current = levels(msg)
        session = self.sessions.get(msg["session"])
        if session is None:
            # A session we haven't heard from: its counts and states so far are old news, apart
            # from the event this message itself reports.
            before = dict(current)
            for key, event, _ in TRIGGERS:
                if msg["event"] == event:
                    before[key] -= 1
        elif msg["seq"] <= session.seq:
            return None, []     # duplicate or out-of-order datagram
        else:
            before = session.levels
        self.sessions[msg["session"]] = _Session(msg["seq"], current, now)
        started = next((stimulus for key, _, stimulus in TRIGGERS if current[key] > before[key]), None)
        ended = [stimulus for key, _, stimulus in TRIGGERS
                 if key in STATES and current[key] < before[key]]
        return started, ended


# ----------------------------------------------------------------------------
# Playback
# ----------------------------------------------------------------------------

def make_controller() -> StimulusController:
    """The same ERMs and PSUs as stimulus_editor.main(), so stimuli play as they were designed."""
    erms = ([ERM(pin, ErmType.BIG) for pin in ERM_PINS[:4]]
            + [ERM(pin, ErmType.SMALL) for pin in ERM_PINS[4:]])
    psus = [
        PSU(PSU1_PORT, name="PSU1", idle_voltage=0.0),
        PSU(PSU2_PORT, name="PSU2", idle_voltage=0.0),
    ]
    return StimulusController(erms, psus)


def remaining(stimulus: Stimulus, offset: float) -> Stimulus:
    """What is left of a stimulus `offset` seconds in, as a stimulus starting at 0 (no PSU warm-up)."""
    offset = max(0.0, offset)

    def clip(period):
        start = max(period.start, offset)
        return start - offset, period.end - start

    return Stimulus([ErmActivation(p.erms, *clip(p)) for p in stimulus.erm_activations if p.end > offset],
                    [PsuActivation(p.psu, *clip(p), p.voltage)
                     for p in stimulus.psu_activations if p.end > offset])


class Player:
    """Plays one stimulus at a time on a background thread. Only the main thread calls its
    methods; the background thread just runs the controller."""

    def __init__(self, controller: StimulusController, stimuli: dict[str, Stimulus]):
        self.controller = controller
        self.stimuli = stimuli
        self.name = None            # the stimulus playing, or the last one played
        self._triggered = 0.0       # perf_counter() when it was triggered
        self._offset = 0.0          # seconds into it this run started (0 unless it was interrupted)
        self._thread = None
        self._stop = threading.Event()
        self._interrupted = None    # (name, triggered, perf_counter() of its time 0) to carry on

    def playing(self):
        return self._thread is not None and self._thread.is_alive()

    def play(self, name):
        """Start the named stimulus, stopping the one playing, or pausing it if the new one
        interrupts it. False if the named stimulus was already playing."""
        if self.playing() and self.name == name:
            return False
        interrupted = None
        if self.playing() and INTERRUPTS.get(name) == self.name:
            t0 = time.perf_counter() - self._offset - (self.controller.elapsed() or 0.0)
            interrupted = (self.name, self._triggered, t0)
        self.stop()
        self._interrupted = interrupted
        self._start(name, self.stimuli[name], time.perf_counter(), 0.0)
        return True

    def update(self):
        """Once an interrupting stimulus has played, carry on with the one it interrupted.
        Returns (name, seconds in) if it did."""
        if self._interrupted is None or self.playing():
            return None
        name, triggered, t0 = self._interrupted
        self._interrupted = None
        offset = time.perf_counter() - t0
        rest = remaining(self.stimuli[name], offset)
        if not (rest.erm_activations or rest.psu_activations):
            return None
        self._start(name, rest, triggered, offset)
        return name, offset

    def action_ended(self, name):
        """The state behind the named stimulus has ended. Stop the stimulus (or drop it, if it's
        interrupted) when the state was shorter than the stimulus; True if so. A state as long
        as the stimulus still ends before it, by the lead-in the stimulus starts after, and that
        last bit is left to finish."""
        if self._interrupted is not None and self._interrupted[0] == name:
            self._interrupted = None
            return True
        if (self.playing() and self.name == name
                and time.perf_counter() - self._triggered < self.stimuli[name].duration):
            self.stop()
            return True
        return False

    def _start(self, name, stimulus, triggered, offset):
        self.name, self._triggered, self._offset = name, triggered, offset
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, args=(name, stimulus, self._stop),
                                        daemon=True)
        self._thread.start()

    def _run(self, name, stimulus, stop):
        try:
            self.controller.run(stimulus, stop=stop, verbose=False)
        except Exception as e:
            log(f"{name} failed: {e}")

    def stop(self):
        """Stop the stimulus playing, if any; the controller leaves ERMs off and PSUs at idle."""
        if self._thread is not None:
            self._stop.set()
            self._thread.join()
            self._thread = None


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Play stimuli when the thermal game reports combat events.")
    parser.add_argument("--bind", default=PI_IP,
                        help=f"address to listen on (default {PI_IP}; 0.0.0.0 for any)")
    parser.add_argument("--port", type=int, default=PORT)
    parser.add_argument("--device", help="only react to this headset's deviceLabel, e.g. quest-a")
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, lambda *args: sys.exit(0))    # systemctl stop: clean up too

    controller = make_controller()
    player = Player(controller, {})
    try:
        for name in dict.fromkeys(name for _, _, name in TRIGGERS):
            stimulus = Stimulus.load(os.path.join(HERE, f"{name}.json"))
            controller.check(stimulus)
            player.stimuli[name] = stimulus
        controller.reset()      # start from a known state

        simulated = defaultdict(list)       # reason -> device names
        for name, device in ([(f"ERM {i}", erm) for i, erm in enumerate(controller.erms)]
                             + [(psu.name, psu) for psu in controller.psus]):
            if device.sim_reason:
                simulated[device.sim_reason].append(name)
        for reason, names in simulated.items():
            log(f"SIMULATED {', '.join(names)}: {reason}")

        tracker = SessionTracker()
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            try:
                sock.bind((args.bind, args.port))
            except OSError as e:
                sys.exit(f"Can't listen on {args.bind}:{args.port}: {e}\n"
                         f"Give this Pi the address the Quests send to, or pass --bind 0.0.0.0.")
            sock.settimeout(0.05)   # short, so an interrupted stimulus carries on promptly
            log(f"Listening on {args.bind}:{args.port}"
                + (f" for {args.device}" if args.device else " for any headset"))
            while True:
                resumed = player.update()
                if resumed:
                    log(f"{resumed[0]} carries on from {resumed[1]:.2f} s")
                try:
                    payload, _ = sock.recvfrom(4096)
                except socket.timeout:
                    continue
                msg = parse(payload)
                if msg is None or (args.device and msg.get("device") != args.device):
                    continue
                started, ended = tracker.changes(msg, time.monotonic())
                source = f"{msg.get('device')} {msg['event']}"
                for name in ended:
                    if player.action_ended(name):
                        log(f"{source} -> stop {name}")
                if started:
                    log(f"{source} -> {started}"
                        + ("" if player.play(started) else " (already playing)"))
    except KeyboardInterrupt:
        pass
    finally:
        player.stop()
        controller.close()


if __name__ == "__main__":
    main()
