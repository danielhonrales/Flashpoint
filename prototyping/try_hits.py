#!/usr/bin/env python3
"""
Plays the LRA hit variants in hit_variants/ one after another, so they can be compared; or, with
--erms, their versions with ERMs added (hit_erms/, made by make_hit_erms.py).

Every variant also heats both Peltiers (70%, at least 1 s), like a hit in the game would.

Usage: python3 try_hits.py                   (the newest round, 3, twice each)
       python3 try_hits.py --round 2         (an earlier round: 1 short, 2 longer/repeated)
       python3 try_hits.py ring bounce       (just these, from either round)
       python3 try_hits.py --current         (also the current hit.json's LRAs and heat, first)
       python3 try_hits.py slam --repeat 5 --gap 0.6
       python3 try_hits.py --erms            (each pattern in hit_erms/ as-is, then its versions)
       python3 try_hits.py --erms dive       (just dive)
       python3 try_hits.py --erms --strategy wrap      (every pattern's wrap version)
       python3 try_hits.py --erms --review   (one at a time: keep, reject, replay; rejected
                                             files move to a _rejected/ folder)

Close the stimulus editor first: it holds the headphone jack. Uses the same devices and settings
as the editor (stimulus_editor.py), so a variant feels here as it will in a stimulus.
"""

import argparse
import glob
import os
import shutil
import time

from stimulus_editor import Stimulus, make_controller

HERE = os.path.dirname(os.path.abspath(__file__))
VARIANTS = os.path.join(HERE, "hit_variants")
ERM_VERSIONS = os.path.join(HERE, "hit_erms")

STRATEGIES = {
    "wrap": "around the arm: outer column, then inner-left, then inner-right",
    "sweep": "along the arm: wrist to elbow down the outer column, inner columns following",
    "helix": "around and along: spirals down the arm through six ERMs",
}

ROUNDS = {
    1: ["crack", "slam", "knock", "ricochet", "drop", "crunch"],         # short
    2: ["rumble", "bounce", "heartbeat", "shockwave", "grind", "ring"],  # longer / repeated
    3: ["dive", "scatter", "shatter", "flutter", "bloom", "echo"],       # falling frequency
}

DESCRIPTIONS = {
    "dive": "impact at resonance, then one smooth glide down to 30 Hz as it fades",
    "scatter": "impact, then top and bottom split apart in pitch and timing as they fade",
    "shatter": "impact, then the tone breaks into sparser, lower fragments, top/bottom at random",
    "flutter": "impact, then the pitch sinks to 12 Hz, turning into separate slow pulses",
    "bloom": "starts high and stiff (110 Hz), swells through resonance, sinks to 35 Hz",
    "echo": "three impacts, each a short dive; the echoes lower, weaker and later",
    "rumble": "hard hit, then a rolling low rumble that wobbles and dies away",
    "bounce": "knocks with shrinking gaps and fading strength: bouncing to rest",
    "heartbeat": "lub-dub, lub-dub (second fainter): a heavy, organic double pulse",
    "shockwave": "pulses alternating bottom/top, spreading out as they weaken",
    "grind": "fast bottom/top bursts swelling then fading: the shield grinding",
    "ring": "big hit, then the shield rings: a smooth decaying tone with a slow shimmer",
    "crack": "one hard hit at resonance, cut dead, faint echo: sharp and dry",
    "slam": "deep blow below resonance that stops quickly: heavy, no tail",
    "knock": "double tap: something bouncing off the shield once",
    "ricochet": "hits the bottom, glances to the top, rising in pitch as it deflects",
    "drop": "fast pitch dive, 140 -> 50 Hz while fading: a 'whoomp'",
    "crunch": "short bursts alternating bottom/top, decaying: gritty, like the shield cracking",
    "current": "the current hit.json's LRAs and heat (the 1 s impact thud)",
}


def main():
    parser = argparse.ArgumentParser(description="Play the LRA hit variants to compare them.")
    parser.add_argument("names", nargs="*", help="variants to play (default: all)")
    parser.add_argument("--round", type=int, choices=sorted(ROUNDS), default=max(ROUNDS),
                        help=f"which round to play (default {max(ROUNDS)}, the newest)")
    parser.add_argument("--current", action="store_true",
                        help="also play the current hit.json's LRAs and heat first")
    parser.add_argument("--erms", action="store_true",
                        help="play the patterns' ERM versions from hit_erms/ (each pattern as-is "
                             "first, unless --strategy is given)")
    parser.add_argument("--strategy", nargs="+", choices=list(STRATEGIES),
                        help="with --erms: only these ERM versions (default: all made)")
    parser.add_argument("--review", action="store_true",
                        help="after each, ask: keep, reject (moved to _rejected/), replay, "
                             "play the LRA-only original, or quit")
    parser.add_argument("--repeat", type=int, default=2, help="times to play each (default 2)")
    parser.add_argument("--gap", type=float, default=1.0, help="seconds between plays (default 1)")
    args = parser.parse_args()

    files = {os.path.basename(f)[4:-5]: f
             for f in sorted(glob.glob(os.path.join(VARIANTS, "hit_*.json")))}
    unknown = [name for name in args.names if name not in files]
    if unknown:
        parser.error(f"no variant {', '.join(unknown)}; there are: {', '.join(files)}")
    default = [name for name in ROUNDS[args.round] if name in files]
    if args.erms:
        chosen = []
        made = sorted(d for d in os.listdir(ERM_VERSIONS)
                      if os.path.isdir(os.path.join(ERM_VERSIONS, d)) and not d.startswith("_")
                      ) if os.path.isdir(ERM_VERSIONS) else []
        for name in args.names or made:
            versions = [s for s in STRATEGIES
                        if os.path.exists(os.path.join(ERM_VERSIONS, name, f"{s}.json"))]
            if not args.strategy and not args.review:   # in a review, 'o' plays the original
                chosen.append((name, Stimulus.load(files[name]), files[name]))
            for strategy in args.strategy or versions:
                path = os.path.join(ERM_VERSIONS, name, f"{strategy}.json")
                if not os.path.exists(path):
                    parser.error(f"no {path}: run make_hit_erms.py first")
                chosen.append((f"{name}/{strategy}", Stimulus.load(path), path))
    else:
        chosen = [(name, Stimulus.load(files[name]), files[name]) for name in (args.names or default)]
    if args.current:
        current = Stimulus.load(os.path.join(HERE, "hit.json"))
        chosen.insert(0, ("current", Stimulus(peltier_activations=current.peltier_activations,
                                              lra_activations=current.lra_activations), None))

    controller = make_controller()
    kept, rejected = [], []

    def play(stimulus, times):
        for n in range(times):
            print(f"  play {n + 1}", flush=True)
            controller.run(stimulus, verbose=False)
            time.sleep(args.gap)

    try:
        for name, stimulus, path in chosen:
            controller.check(stimulus)
            buzz = max((p.end for p in stimulus.lra_activations), default=0.0)
            pattern, _, strategy = name.partition("/")
            about = (f"+ ERMs: {STRATEGIES[strategy]}" if strategy
                     else DESCRIPTIONS.get(name, "") + (" (LRAs only)" if args.erms else ""))
            print(f"\n{name:22} {buzz * 1000:4.0f} ms  {about}")
            play(stimulus, args.repeat)
            if not args.review or path is None:
                continue
            while True:
                answer = input("  k keep / x reject / r replay"
                               + (" / o original" if strategy else "") + " / q quit: ").strip().lower()
                if answer == "r":
                    play(stimulus, 1)
                elif answer == "o" and strategy:
                    print(f"  {pattern} as-is (LRAs only)")
                    play(Stimulus.load(files[pattern]), 1)
                elif answer == "k":
                    kept.append(name)
                    break
                elif answer == "x":
                    reject(path)
                    rejected.append(name)
                    break
                elif answer == "q":
                    raise KeyboardInterrupt
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        controller.close()
        if args.review:
            print(f"\nKept: {', '.join(kept) or 'none'}")
            print(f"Rejected (moved to _rejected/): {', '.join(rejected) or 'none'}")


def reject(path):
    """Move a pattern file into a _rejected/ folder beside its collection, keeping its place."""
    root = ERM_VERSIONS if os.path.commonpath([path, ERM_VERSIONS]) == ERM_VERSIONS else VARIANTS
    target = os.path.join(root, "_rejected", os.path.relpath(path, root))
    os.makedirs(os.path.dirname(target), exist_ok=True)
    shutil.move(path, target)
    print(f"  moved to {os.path.relpath(target, HERE)}")


if __name__ == "__main__":
    main()
