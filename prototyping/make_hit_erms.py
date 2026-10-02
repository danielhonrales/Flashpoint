#!/usr/bin/env python3
"""
Builds hit_erms/<pattern>/<design>.json: LRA hit patterns from hit_variants/ with ERMs added, for
the three-column layout (HARDWARE.md §10).

The idea: the wearer is hit on the outer forearm, and it then reverberates round to the inner
side. The LRAs (on the outer column) give the high-quality impact; the ERMs support its intensity
and carry it across the arm. Heat is kept as it was.

Layout, in cm from the wrist:
    outer column      Top LRA 0, ERM 0 at 4, (Peltier 1 at 8), ERM 1 at 12, Bottom LRA 16
    inner-left        ERM 2 at 0, ERM 3 at 8, ERM 4 at 16            (thumb side)
    inner-right       ERM 5 at 0, ERM 6 at 8, ERM 7 at 16

Each pattern is read for its impacts and its loudness over time on each LRA. An ERM that follows
the loudness takes it from the LRAs by distance along the arm: at 4 cm, 3/4 the Top LRA's and
1/4 the Bottom LRA's. The designs, each moving the hit in a different way:

  wrap    around the arm: the outer column strikes with the LRAs, then the inner-left column,
          then the inner-right, COLUMN_GAP apart. Each column strikes and fades as the next takes
          over; the last carries the reverberation, following the LRAs.
  sweep   along the arm: the impact travels wrist to elbow down the outer column, Top LRA ->
          ERM 0 -> ERM 1 -> Bottom LRA, a 4 cm step every SWEEP_STEP (the Bottom LRA's copy of
          the pattern starts when the sweep reaches it); the inner columns follow each level
          INNER_LAG behind.
  helix   around and along: the impact spirals down the arm, ERM 0 -> 3 -> 6 -> 1 -> 4 -> 7
          (circling the arm twice), a step every HELIX_STEP, fading as it goes.

sweep and helix use Tactile Brush timing, where neighbouring pulses overlap so that separate
points feel like one thing moving.

Usage: python3 make_hit_erms.py                     (the default patterns and designs)
       python3 make_hit_erms.py bounce              (other patterns from hit_variants/)
       python3 make_hit_erms.py echo --designs sweep helix
"""

import argparse
import os

import stimulus_editor as se
from stimulus_editor import ErmActivation, LraActivation, Stimulus, TB_SOA_INTERCEPT, TB_SOA_SLOPE

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, "hit_variants")
TARGET = os.path.join(HERE, "hit_erms")

# Layout (HARDWARE.md §10): each ERM's column and distance from the wrist in cm.
POSITION = {0: ("outer", 4), 1: ("outer", 12),
            2: ("in-L", 0), 3: ("in-L", 8), 4: ("in-L", 16),
            5: ("in-R", 0), 6: ("in-R", 8), 7: ("in-R", 16)}
LENGTH = 16.0                   # cm from the Top LRA to the Bottom LRA
BOTTOM_LRA, TOP_LRA = 0, 1

COLUMNS = [[0, 1], [2, 3, 4], [5, 6, 7]]    # wrap's order round the arm: outer, in-L, in-R
COLUMN_GAP = 0.3        # s between one column starting and the next (wrap)
SWEEP_STEP = 0.15       # s per 4 cm down the arm (sweep)
INNER_LAG = 0.1         # s the inner columns trail the outer one (sweep)
HELIX = [0, 3, 6, 1, 4, 7]  # outer 4 -> in-L 8 -> in-R 8 -> outer 12 -> in-L 16 -> in-R 16
HELIX_STEP = 0.2        # s between helix steps
MAX_PULSE = 0.35        # s: longest pulse in a sweep or helix step
LRA_DELAY = 0.02        # s the LRAs start after the first ERMs: covers the ERMs' spin-up

PATTERNS = ["echo", "dive", "ring", "shockwave"]
DESIGNS = ["wrap", "sweep", "helix"]
FLOOR = 0.4             # lowest ERM level used: much below ~30% duty an ERM may not spin
ERM_SCALE = 0.7         # every ERM level is scaled by this (ERMs were too strong next to the LRAs)
MIN_PULSE = 0.05        # s: an ERM needs ~20-30 ms to spin up
STEP = 0.005            # s: resolution the patterns are read at


def level(a):
    """ERM level for a pattern loudness 0-1."""
    return round(ERM_SCALE * (FLOOR + (1 - FLOOR) * max(0.0, min(1.0, a))), 3)


def pulse(erms, start, duration, a):
    return ErmActivation(list(erms), round(start, 6), round(max(MIN_PULSE, duration), 6),
                         intensity=level(a))


def brush_pulse(step):
    """Tactile Brush: the pulse length that makes onsets `step` apart feel like one stroke."""
    return min(MAX_PULSE, max(MIN_PULSE, (step - TB_SOA_INTERCEPT) / TB_SOA_SLOPE))


# ------------------------------------------------------------------ reading a pattern

class Shape:
    """A pattern's loudness over time on each LRA, and its impact onsets."""

    def __init__(self, stimulus):
        self.lra = stimulus.lra_activations
        self.end = max(p.end for p in self.lra)
        n = int(self.end / STEP) + 2
        self.side = {k: [self._amp(k, i * STEP) for i in range(n)] for k in (BOTTOM_LRA, TOP_LRA)}
        self.env = [max(b, t) for b, t in zip(self.side[BOTTOM_LRA], self.side[TOP_LRA])]
        self.events = self._onsets()

    def _amp(self, side, t):
        active = [p for p in self.lra if p.lra == side and p.start <= t < p.end]
        return max(active, key=lambda p: p.start).amplitude if active else 0.0

    def at(self, t, side):
        """Loudness on one LRA at time t of the pattern (0 outside it)."""
        i = int(round(t / STEP))
        return self.side[side][i] if 0 <= i < len(self.env) else 0.0

    def _onsets(self, rise=0.2, floor=0.3, min_gap=0.06):
        """[(time, strength)]: where loudness jumps up, at least min_gap apart."""
        events, back = [], int(0.02 / STEP)
        for i, a in enumerate(self.env):
            before = self.env[i - back] if i >= back else 0.0
            if a >= floor and a - before >= rise and (not events or i * STEP - events[-1][0] >= min_gap):
                window = range(i, min(len(self.env), i + int(0.03 / STEP)))
                events.append((round(i * STEP, 3), max(self.env[j] for j in window)))
        return events


def loudness(shape, y, t0, t1, bottom_delay=0.0):
    """The loudness an ERM y cm from the wrist should follow over [t0, t1) of the stimulus,
    blended from the two LRAs by distance. The LRAs start at LRA_DELAY (the Bottom one
    bottom_delay later still)."""
    near_top = 1 - y / LENGTH
    times = [t0 + (t1 - t0) * k / 4 for k in range(4)]
    return sum(near_top * shape.at(t - LRA_DELAY, TOP_LRA)
               + (1 - near_top) * shape.at(t - LRA_DELAY - bottom_delay, BOTTOM_LRA)
               for t in times) / len(times)


def follow(shape, erms, t0, t1, scale, step=0.03):
    """Each ERM following the loudness at its own place along the arm, from t0 to t1."""
    out, t = [], t0
    while t < t1:
        for erm in erms:
            a = scale * loudness(shape, POSITION[erm][1], t, t + step)
            if a >= 0.12:
                out.append(pulse([erm], t, step, a))
        t += step
    return out


def shifted(periods, delay):
    return [LraActivation(p.lra, round(p.start + delay, 6), p.duration, p.frequency, p.amplitude,
                          p.enabled) for p in periods]


# ------------------------------------------------------------------ designs
# Each returns (ERM periods, LRA periods).

def wrap(shape, lras):
    """Outer column, then inner-left, then inner-right, COLUMN_GAP apart; each strikes and fades
    as the next takes over; the last carries the reverberation."""
    end = shape.end + LRA_DELAY
    out = []
    for k, column in enumerate(COLUMNS):
        start = k * COLUMN_GAP
        last = k == len(COLUMNS) - 1
        out.append(pulse(column, start, 0.09, 0.85 ** k))
        out += follow(shape, column, start + 0.09, end if last else start + COLUMN_GAP,
                      0.85 if last else 0.6)
    return out, shifted(lras, LRA_DELAY)


def sweep(shape, lras):
    """Wrist to elbow down the outer column (Top LRA, ERM 0, ERM 1, Bottom LRA), a 4 cm step
    every SWEEP_STEP; the inner columns follow each level INNER_LAG behind."""
    arrive = lambda y: y / 4 * SWEEP_STEP          # when the sweep reaches y cm
    duration = brush_pulse(SWEEP_STEP)
    out = []
    for erm, (column, y) in POSITION.items():
        fade = 1 - 0.4 * y / LENGTH                 # it loses energy as it travels
        if column == "outer":
            out.append(pulse([erm], arrive(y), duration, 0.95 * fade))
        else:
            out.append(pulse([erm], arrive(y) + INNER_LAG, duration, 0.8 * fade))
    bottom_start = arrive(LENGTH)
    top = [p for p in lras if p.lra == TOP_LRA]
    bottom = [p for p in lras if p.lra == BOTTOM_LRA]
    return out, shifted(top, LRA_DELAY) + shifted(bottom, LRA_DELAY + bottom_start)


def helix(shape, lras):
    """Spiral down the arm through ERM 0, 3, 6, 1, 4, 7, a step every HELIX_STEP, fading."""
    duration = brush_pulse(HELIX_STEP)
    out = [pulse([erm], k * HELIX_STEP, duration, 0.95 * 0.85 ** k) for k, erm in enumerate(HELIX)]
    return out, shifted(lras, LRA_DELAY)


BUILDERS = {"wrap": wrap, "sweep": sweep, "helix": helix}


# ------------------------------------------------------------------ main

def main():
    parser = argparse.ArgumentParser(description="Add ERMs to LRA hit patterns.")
    parser.add_argument("patterns", nargs="*", default=PATTERNS,
                        help=f"patterns from hit_variants/ (default: {' '.join(PATTERNS)})")
    parser.add_argument("--designs", nargs="+", choices=list(BUILDERS), default=DESIGNS)
    args = parser.parse_args()
    se.DRY_RUN = True                   # checking only; nothing is played
    controller = se.make_controller()
    for name in args.patterns:
        path = os.path.join(SOURCE, f"hit_{name}.json")
        if not os.path.exists(path):
            parser.error(f"no {path}")
        base = Stimulus.load(path)
        shape = Shape(base)
        os.makedirs(os.path.join(TARGET, name), exist_ok=True)
        made = []
        for design in args.designs:
            erms, lras = BUILDERS[design](shape, base.lra_activations)
            stimulus = Stimulus(erms, base.peltier_activations, base.peltier_warmup, lras)
            controller.check(stimulus)
            controller.check_safety(stimulus)
            stimulus.save(os.path.join(TARGET, name, f"{design}.json"))
            made.append(f"{design} {stimulus.duration:.2f}s")
        print(f"{name:10} {len(shape.events):2} impacts: " + ", ".join(made))
    controller.close()


if __name__ == "__main__":
    main()
