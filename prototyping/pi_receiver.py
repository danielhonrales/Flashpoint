"""
Pi receiver: plays the saved stimuli when the thermal game reports a combat event.

The game (ThermalGameDemo, CombatEventOutput.cs) sends this Pi one UDP JSON datagram per combat
event, plus a full-state snapshot every 250 ms. Every message carries cumulative counters and the
list of active states, which start and stop the stimuli:

  fire_charge, fire   the player makes the fire pose, then the beam fires   -> hot_laser.json
  ice_charge          the player makes the ice pose (before the throw)      -> ice_bomb.json
  shield              the player raises the shield                          -> shield.json
  blocks              a hit lands on the raised shield                      -> hit.json

A hit that gets through to the player (the "hits" counter) plays nothing for now.

A stimulus started by states stops if they end before it has had its full length: the pose is
dropped before the beam fires or the grenade is thrown, the beam is let go early, or the shield is
lowered early. Once the grenade is thrown (the "iceThrows" counter goes up), ice_bomb plays to the
end. States count as ended once they have stayed ended for END_GRACE seconds, which bridges the
game switching from fire_charge to fire. hit.json interrupts shield.json rather than stopping it:
once the hit has played, the shield stimulus carries on from where it would be by then, if the
shield is still raised.

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
import errno
import json
import os
import signal
import socket
import sys
import threading
import time
from collections import defaultdict
from dataclasses import dataclass, field

from stimulus_editor import (ERM, ERM_PINS, PSU, PSU1_PORT, PSU2_PORT, ErmActivation, ErmType,
                             PsuActivation, Stimulus, StimulusController)

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

PI_IP = "192.168.1.5"   # the "host" in each Quest's combat-output.json
PORT = 7779

# Counters in every game message, with the event the game sends when each goes up. The stimulus
# plays when the counter goes up, and always plays to the end.
COUNTER_TRIGGERS = [
    # counter   event            stimulus
    ("blocks",  "shield_block",  "hit"),
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

# Stimulus -> the stimulus it interrupts. The interrupted one carries on afterwards from where it
# would be by then, unless its state has ended meanwhile.
INTERRUPTS = {"hit": "shield"}

END_GRACE = 0.3         # seconds a trigger's states must stay ended before its stimulus stops
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
        self._plays_out = False     # True once it is to play to the end whatever its states do
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

    def play_out(self, name):
        """Let the named stimulus, if playing, play to the end whatever its states do."""
        if self.playing() and self.name == name:
            self._plays_out = True

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

    def action_ended(self, name, ended_at):
        """The states behind the named stimulus ended at perf_counter() `ended_at`. Stop the
        stimulus (or drop it, if it's interrupted) when they lasted less than the stimulus;
        True if so. States as long as the stimulus still end before it, by the lead-in the
        stimulus starts after, and that last bit is left to finish."""
        if self._interrupted is not None and self._interrupted[0] == name:
            self._interrupted = None
            return True
        if (self.playing() and self.name == name and not self._plays_out
                and ended_at - self._triggered < self.stimuli[name].duration):
            self.stop()
            return True
        return False

    def _start(self, name, stimulus, triggered, offset):
        self.name, self._triggered, self._offset = name, triggered, offset
        self._plays_out = False
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
        for name in dict.fromkeys([name for _, _, name in COUNTER_TRIGGERS]
                                  + [name for _, name, _ in STATE_TRIGGERS]):
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
                hint = ("Something else is listening on that port, probably the old dummy receiver:\n"
                        "    sudo systemctl disable --now thermal-game-receiver"
                        if e.errno == errno.EADDRINUSE else
                        "Give this Pi the address the Quests send to, or pass --bind 0.0.0.0.")
                sys.exit(f"Can't listen on {args.bind}:{args.port}: {e}\n{hint}")
            sock.settimeout(0.05)   # short, so ends and interrupted stimuli are handled promptly
            log(f"Listening on {args.bind}:{args.port}"
                + (f" for {args.device}" if args.device else " for any headset"))
            while True:
                for name, ended_at in tracker.ended(time.perf_counter()):
                    if player.action_ended(name, ended_at):
                        log(f"stop {name}: its action ended early")
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
                started, plays_out = tracker.changes(msg, time.perf_counter())
                for name in plays_out:
                    player.play_out(name)
                if started:
                    log(f"{msg.get('device')} {msg['event']} -> {started}"
                        + ("" if player.play(started) else " (already playing)"))
    except KeyboardInterrupt:
        pass
    finally:
        player.stop()
        controller.close()


if __name__ == "__main__":
    main()
