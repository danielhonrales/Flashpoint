"""
Stimulus editor: a GUI for timing ERM vibration motors and two programmable power supplies
from a Raspberry Pi.

A stimulus is a set of activation periods, all timed in seconds from the start of the stimulus:

  ERM periods - which ERMs turn on, when, and for how long. One period can drive several ERMs,
                and periods may overlap: an ERM stays on while any period containing it is active.
  PSU periods - which voltage a supply goes to, when, and for how long. Each supply has its own
                periods, running concurrently with the ERMs and the other supply. If periods on the
                same supply overlap, the one that started most recently wins; when it ends the
                supply falls back to whichever of its periods is still active, else to its idle voltage.

Stimuli are saved as JSON.   Usage: python3 stimulus_editor.py [stimulus.json]

Needs Tkinter, gpiozero and pyserial on the Pi:
    sudo apt install python3-tk python3-gpiozero python3-serial
Any ERM or PSU that can't be opened (library missing, port not found, DRY_RUN set) is simulated
instead, and the editor shows a red SIMULATED banner naming it.
"""

import copy
import functools
import json
import math
import os
import signal
import sys
import threading
import time
import tkinter as tk
from collections import defaultdict
from dataclasses import asdict, dataclass, field
from enum import Enum
from tkinter import filedialog, messagebox, ttk

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

# BCM GPIO number for each ERM.
ERM_PINS = [17, 27, 22, 23]

# Serial ports of the two supplies. /dev/ttyUSB* numbers can swap between boots; the
# /dev/serial/by-path/... links are tied to the physical USB socket and stay put.
PSU1_PORT = "/dev/ttyUSB0"
PSU2_PORT = "/dev/ttyUSB1"
PSU_BAUDRATE = 9600
PSU_LINE_ENDING = ""        # appended to each VSET command; some supplies want "\n"
PSU_MAX_VOLTAGE = 30.0      # default per-supply limit; voltages above it are rejected

DRY_RUN = False             # True: simulate every device, even on the Pi


# ----------------------------------------------------------------------------
# Hardware
# ----------------------------------------------------------------------------

class ErmType(Enum):
    SMALL = "small"
    BIG = "big"


class _NullDevice:
    """Stands in for a GPIO pin or serial port that is simulated."""

    def on(self): pass
    def off(self): pass
    def write(self, data): pass
    def flush(self): pass
    def close(self): pass


class ERM:
    """An ERM switched by one GPIO pin: high = on, low = off.

    The type is kept on the object so small and big ERMs can be driven differently;
    for now both are plain on/off.
    """

    def __init__(self, pin: int, erm_type: ErmType):
        self.pin = pin
        self.erm_type = erm_type
        self.is_on = False
        self.sim_reason = "DRY_RUN is set" if DRY_RUN else None   # why it's simulated, if it is
        self._pin = _NullDevice()
        if not DRY_RUN:
            try:
                from gpiozero import DigitalOutputDevice
                self._pin = DigitalOutputDevice(pin, initial_value=False)
            except Exception as e:
                self.sim_reason = f"{type(e).__name__}: {e}"

    def on(self):
        self._pin.on()
        self.is_on = True

    def off(self):
        self._pin.off()
        self.is_on = False

    def close(self):
        self.off()
        self._pin.close()

    def __str__(self):
        return f"{self.erm_type.value}, pin {self.pin}"


class PSU:
    """A programmable supply set with "VSET<channel>:<volts>" over serial."""

    def __init__(self, port: str, name: str | None = None, idle_voltage: float = 0.0,
                 max_voltage: float = PSU_MAX_VOLTAGE, baudrate: int = PSU_BAUDRATE,
                 channel: int = 1):
        self.name = name or port
        self.idle_voltage = idle_voltage    # held before, between and after activation periods
        self.max_voltage = max_voltage
        self.channel = channel
        self.voltage = None                 # last voltage sent; None until the first command
        self.check_voltage(idle_voltage)
        self.sim_reason = "DRY_RUN is set" if DRY_RUN else None
        self._serial = _NullDevice()
        if not DRY_RUN:
            try:
                import serial
                self._serial = serial.Serial(port, baudrate, timeout=1)
            except Exception as e:
                self.sim_reason = f"{type(e).__name__}: {e}"

    def check_voltage(self, voltage: float):
        if not 0 <= voltage <= self.max_voltage:
            raise ValueError(f"{self.name}: {voltage} V is outside 0-{self.max_voltage} V")

    def set_voltage(self, voltage: float):
        self.check_voltage(voltage)
        self._serial.write(f"VSET{self.channel}:{voltage:.2f}{PSU_LINE_ENDING}".encode("ascii"))
        self._serial.flush()
        self.voltage = voltage

    def close(self):
        self._serial.close()


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

    def __post_init__(self):
        _check_period(self.start, self.duration)
        if not self.erms:
            raise ValueError("an ERM period needs at least one ERM")
        self.start = round(self.start, 6)

    @property
    def end(self):
        return round(self.start + self.duration, 6)


@dataclass
class PsuActivation:
    psu: int            # index into the controller's PSU list
    start: float        # seconds from stimulus start
    duration: float     # seconds
    voltage: float

    def __post_init__(self):
        _check_period(self.start, self.duration)
        self.start = round(self.start, 6)

    @property
    def end(self):
        return round(self.start + self.duration, 6)


@dataclass
class Stimulus:
    erm_activations: list[ErmActivation] = field(default_factory=list)
    psu_activations: list[PsuActivation] = field(default_factory=list)

    @property
    def duration(self):
        return max((p.end for p in [*self.erm_activations, *self.psu_activations]), default=0.0)

    def save(self, path):
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            data = json.load(f)
        return cls(erm_activations=[ErmActivation(**p) for p in data.get("erm_activations", [])],
                   psu_activations=[PsuActivation(**p) for p in data.get("psu_activations", [])])


def _erm_changes(periods: list[ErmActivation], n_erms: int):
    """Resolve overlapping ERM periods into [(time, erm_index, on), ...].
    An ERM is on while at least one period containing it is active."""
    deltas = defaultdict(lambda: [0] * n_erms)
    for period in periods:
        for i in set(period.erms):
            deltas[period.start][i] += 1
            deltas[period.end][i] -= 1
    active = [0] * n_erms
    changes = []
    for t in sorted(deltas):
        for i, delta in enumerate(deltas[t]):
            if delta:
                was_on = active[i] > 0
                active[i] += delta
                if (active[i] > 0) != was_on:
                    changes.append((t, i, not was_on))
    return changes


def _psu_changes(periods: list[PsuActivation], idle_voltage: float):
    """Resolve one supply's (possibly overlapping) periods into [(time, voltage), ...].
    The most recently started active period wins; with none active the supply idles."""
    periods = sorted(periods, key=lambda p: p.start)   # stable: equal starts keep definition order
    changes = []
    voltage = idle_voltage
    for t in sorted({p.start for p in periods} | {p.end for p in periods}):
        active = [p for p in periods if p.start <= t < p.end]
        target = active[-1].voltage if active else idle_voltage
        if target != voltage:
            changes.append((t, target))
            voltage = target
    return changes


# ----------------------------------------------------------------------------
# Playback
# ----------------------------------------------------------------------------

class StimulusController:
    """Owns the ERMs and PSUs and plays Stimulus objects on them."""

    def __init__(self, erms: list[ERM], psus: list[PSU]):
        self.erms = erms
        self.psus = psus
        self.t0 = None      # perf_counter() at time 0 of the running stimulus

    def reset(self):
        """All ERMs off, all PSUs to their idle voltage."""
        for erm in self.erms:
            erm.off()
        for psu in self.psus:
            psu.set_voltage(psu.idle_voltage)

    def elapsed(self):
        """Seconds into the running stimulus (negative during the lead-in), or None when idle."""
        t0 = self.t0
        return None if t0 is None else time.perf_counter() - t0

    def check(self, stimulus: Stimulus):
        """Raise ValueError if the stimulus names a missing ERM/PSU or an out-of-range voltage."""
        for period in stimulus.erm_activations:
            for i in period.erms:
                if not 0 <= i < len(self.erms):
                    raise ValueError(f"{period}: there is no ERM {i}")
        for period in stimulus.psu_activations:
            if not 0 <= period.psu < len(self.psus):
                raise ValueError(f"{period}: there is no PSU {period.psu}")
            self.psus[period.psu].check_voltage(period.voltage)

    def run(self, stimulus: Stimulus, stop: threading.Event | None = None, verbose: bool = True):
        """Play the stimulus and block until it ends or `stop` is set. ERMs and PSUs are reset
        before it starts and again when it ends, fails or is stopped."""
        tracks = self._compile(stimulus)
        stop = stop or threading.Event()
        errors = []
        print_lock = threading.Lock()

        self.reset()
        t0 = self.t0 = time.perf_counter() + 0.2   # give the supplies a moment after reset()

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

        # One thread per track (ERMs, each PSU) so a slow serial write never delays the others.
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
        if errors:
            raise errors[0]

    def _compile(self, stimulus: Stimulus):
        """Validate the stimulus and turn it into independent tracks (ERMs, then one per PSU),
        each a time-sorted list of (time, description, action)."""
        self.check(stimulus)
        erm_track = []
        for t, i, on in _erm_changes(stimulus.erm_activations, len(self.erms)):
            erm = self.erms[i]
            erm_track.append((t, f"ERM {i} ({erm}) {'on' if on else 'off'}",
                              erm.on if on else erm.off))
        tracks = [erm_track]
        for i, psu in enumerate(self.psus):
            periods = [p for p in stimulus.psu_activations if p.psu == i]
            tracks.append([(t, f"{psu.name} -> {v:.2f} V", functools.partial(psu.set_voltage, v))
                           for t, v in _psu_changes(periods, psu.idle_voltage)])
        return [track for track in tracks if track]

    def close(self):
        try:
            self.reset()
        finally:
            for device in [*self.erms, *self.psus]:
                device.close()


# ----------------------------------------------------------------------------
# GUI
# ----------------------------------------------------------------------------

ERM_COLORS = {ErmType.SMALL: "#5b9bd5", ErmType.BIG: "#ed7d31"}
PSU_COLOR = "#b6dca0"
PSU_LINE_COLOR = "#2f6f1e"
SELECTED_COLOR = "#ffd54f"
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
    """One lane per ERM and PSU showing the stimulus, live device state and a playhead.
    Click a bar to select its period."""

    LABEL_W = 210
    AXIS_H = 26
    PAD = 10
    RIGHT_PAD = 24

    def __init__(self, parent, editor):
        super().__init__(parent, background="white", highlightthickness=0, height=280)
        self.editor = editor
        self._periods = {}      # canvas item -> the period it draws
        self._lamps = []        # one per ERM, lit while it is on
        self._readouts = []     # one per PSU, its present voltage
        self._playhead = None
        self._playhead_span = (0, 0)
        self._to_x = None
        self.bind("<Configure>", lambda event: self.redraw())
        self.bind("<Button-1>", self._on_click)

    def redraw(self):
        self.delete("all")
        self._periods.clear()
        self._lamps.clear()
        self._readouts.clear()
        self._playhead = None
        width, height = self.winfo_width(), self.winfo_height()
        if width < self.LABEL_W + 100:
            return      # not laid out yet
        controller, stimulus = self.editor.controller, self.editor.stimulus
        span = max(stimulus.duration, 1.0) * 1.05
        left, right = self.LABEL_W, width - self.RIGHT_PAD
        x = self._to_x = lambda t: left + (right - left) * t / span
        n_lanes = len(controller.erms) + 1.8 * len(controller.psus)
        unit = min(max((height - 2 * self.PAD - self.AXIS_H) / n_lanes, 18), 60)
        lanes = [(self._draw_erm_lane, i, unit) for i in range(len(controller.erms))]
        lanes += [(self._draw_psu_lane, k, 1.8 * unit) for k in range(len(controller.psus))]
        top = self.PAD
        bottom = top + sum(h for _, _, h in lanes)

        step = _tick_step(span, max(2, (right - left) // 80))
        for n in range(int(span / step) + 1):
            tick_x = x(n * step)
            self.create_line(tick_x, top, tick_x, bottom + 4, fill="#e4e4e4")
            self.create_text(tick_x, bottom + 6, anchor="n", text=f"{_fmt(n * step)} s", fill="#555")
        self.create_line(left, bottom, right, bottom, fill="#888")

        y = top
        for draw, index, h in lanes:
            draw(index, y, h, x, span)
            y += h
            self.create_line(0, y, right, y, fill="#d4d4d4")
        self.tag_raise("selected")
        self.tag_raise("overlay")
        self._playhead = self.create_line(0, top, 0, bottom, fill="#d62728", width=2, state="hidden")
        self._playhead_span = (top, bottom)
        self.update_live()

    def _draw_erm_lane(self, i, y, h, x, span):
        erm = self.editor.controller.erms[i]
        mid = y + h / 2
        self._lamps.append(self.create_oval(8, mid - 6, 20, mid + 6, outline="#777", fill=LAMP_OFF))
        self._lane_label(mid, f"ERM {i}   {erm.erm_type.value} · pin {erm.pin}", erm)
        for period in self.editor.stimulus.erm_activations:
            if i in period.erms:
                self._bar(period, x(period.start), y + 4, x(period.end), y + h - 4,
                          ERM_COLORS[erm.erm_type])

    def _draw_psu_lane(self, k, y, h, x, span):
        psu = self.editor.controller.psus[k]
        self._lane_label(y + h / 2, psu.name, psu)
        self._readouts.append(self.create_text(self.LABEL_W - 10, y + h / 2, anchor="e",
                                               fill=PSU_LINE_COLOR))
        periods = sorted((p for p in self.editor.stimulus.psu_activations if p.psu == k),
                         key=lambda p: p.start)
        v_max = max([p.voltage for p in periods] + [psu.idle_voltage]) or 1.0
        base, ceiling = y + h - 4, y + 18    # room above the bars for their voltage labels
        vy = lambda v: base - (base - ceiling) * v / v_max
        for period in periods:
            x0, x1 = x(period.start), x(period.end)
            self._bar(period, x0, vy(period.voltage), x1, base, PSU_COLOR)
            if x1 - x0 > 34:
                self.create_text(x0 + 3, vy(period.voltage) - 1, anchor="sw", fill="#333",
                                 text=f"{_fmt(period.voltage)} V", tags=("overlay",))
        # What the supply will actually output, after resolving overlaps.
        points = [x(0), vy(psu.idle_voltage)]
        voltage = psu.idle_voltage
        for t, next_voltage in _psu_changes(periods, psu.idle_voltage):
            points += [x(t), vy(voltage), x(t), vy(next_voltage)]
            voltage = next_voltage
        points += [x(span), vy(voltage)]
        self.create_line(*points, fill=PSU_LINE_COLOR, width=2, tags=("overlay",))

    def _lane_label(self, mid, text, device):
        simulated = device.sim_reason is not None
        self.create_text(28, mid, anchor="w", text=text + ("   sim" if simulated else ""),
                         fill=SIM_COLOR if simulated else "#222")

    def _bar(self, period, x0, y0, x1, y1, color):
        selected = period is self.editor.selected
        item = self.create_rectangle(x0, y0, max(x1, x0 + 2), y1,
                                     fill=SELECTED_COLOR if selected else color,
                                     outline="#000" if selected else "#555",
                                     width=2 if selected else 1,
                                     tags=("selected",) if selected else ())
        self._periods[item] = period

    def _on_click(self, event):
        hits = self.find_overlapping(event.x - 1, event.y - 1, event.x + 1, event.y + 1)
        period = next((self._periods[item] for item in reversed(hits) if item in self._periods), None)
        self.editor.select(period, show_tab=True)

    def update_live(self):
        """Refresh the ERM lamps, PSU readouts and playhead from the devices."""
        controller = self.editor.controller
        for lamp, erm in zip(self._lamps, controller.erms):
            self.itemconfigure(lamp, fill=LAMP_ON if erm.is_on else LAMP_OFF)
        for readout, psu in zip(self._readouts, controller.psus):
            self.itemconfigure(readout, text="" if psu.voltage is None else f"{psu.voltage:.2f} V")
        if self._playhead is None:
            return
        elapsed = controller.elapsed()
        if elapsed is None:
            self.itemconfigure(self._playhead, state="hidden")
        else:
            playhead_x = self._to_x(max(0.0, elapsed))
            top, bottom = self._playhead_span
            self.coords(self._playhead, playhead_x, top, playhead_x, bottom)
            self.itemconfigure(self._playhead, state="normal")


class _PeriodPanel(ttk.Frame):
    """A table of periods plus a form to add, update and delete them."""

    columns = ()    # (heading, width) pairs; set by subclasses

    def __init__(self, parent, editor):
        super().__init__(parent, padding=8)
        self.editor = editor
        self._rows = {}     # tree item id -> period

        self.tree = ttk.Treeview(self, columns=[name for name, _ in self.columns], show="headings",
                                 height=7, selectmode="browse")
        for name, width in self.columns:
            self.tree.heading(name, text=name)
            self.tree.column(name, width=width, anchor="center")
        scrollbar = ttk.Scrollbar(self, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.tree.bind("<<TreeviewSelect>>", self._on_tree_select)
        self.tree.bind("<Delete>", lambda event: self._delete())

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

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)
        self.refresh()

    # Subclasses provide these.
    def periods(self): raise NotImplementedError
    def _row(self, period): raise NotImplementedError
    def _build_fields(self, form, first_row): raise NotImplementedError
    def _fill_fields(self, period): raise NotImplementedError
    def _make_period(self, start, duration): raise NotImplementedError

    def refresh(self):
        self.tree.delete(*self.tree.get_children())
        self._rows.clear()
        for period in self.periods():
            self._rows[self.tree.insert("", tk.END, values=self._row(period))] = period
        self.show_selection(self.editor.selected)

    def show_selection(self, period):
        iid = next((iid for iid, p in self._rows.items() if p is period), None)
        if iid is not None:
            self.tree.selection_set(iid)
            self.tree.see(iid)
            self.start_var.set(_fmt(period.start))
            self.duration_var.set(_fmt(period.duration))
            self._fill_fields(period)
        elif self.tree.selection():
            self.tree.selection_remove(*self.tree.selection())
        self._update_buttons()

    def selected_period(self):
        selection = self.tree.selection()
        return self._rows.get(selection[0]) if selection else None

    def _on_tree_select(self, event):
        period = self.selected_period()
        if period is not None and period is not self.editor.selected:
            self.editor.select(period)
        self._update_buttons()

    def _update_buttons(self):
        state = ["!disabled"] if self.selected_period() else ["disabled"]
        self.update_button.state(state)
        self.delete_button.state(state)

    def _read_form(self):
        try:
            start = _parse_float(self.start_var, "Start")
            duration = _parse_float(self.duration_var, "Duration")
            return self._make_period(start, duration)
        except ValueError as e:
            messagebox.showerror("Invalid period", str(e), parent=self)
            return None

    def _add(self):
        period = self._read_form()
        if period is not None:
            self.editor.add_period(period)

    def _update(self):
        old = self.selected_period()
        new = self._read_form() if old else None
        if new is not None:
            self.editor.replace_period(old, new)

    def _delete(self):
        period = self.selected_period()
        if period is not None:
            self.editor.remove_period(period)


class ErmPanel(_PeriodPanel):
    columns = (("ERMs", 140), ("Start (s)", 90), ("Duration (s)", 90), ("End (s)", 90))

    def periods(self):
        return sorted(self.editor.stimulus.erm_activations, key=lambda p: p.start)

    def _row(self, period):
        return (", ".join(str(i) for i in sorted(set(period.erms))),
                _fmt(period.start), _fmt(period.duration), _fmt(period.end))

    def _build_fields(self, form, first_row):
        box = ttk.LabelFrame(form, text="ERMs", padding=(6, 2))
        box.grid(row=first_row, column=0, columnspan=2, sticky="we", pady=(6, 0))
        self.erm_vars = []
        for i, erm in enumerate(self.editor.controller.erms):
            var = tk.BooleanVar()
            ttk.Checkbutton(box, text=f"{i}: {erm}", variable=var).grid(
                row=i % 4, column=i // 4, sticky="w", padx=(0, 10))
            self.erm_vars.append(var)

    def _fill_fields(self, period):
        for i, var in enumerate(self.erm_vars):
            var.set(i in period.erms)

    def _make_period(self, start, duration):
        erms = [i for i, var in enumerate(self.erm_vars) if var.get()]
        if not erms:
            raise ValueError("Tick at least one ERM.")
        return ErmActivation(erms, start, duration)


class PsuPanel(_PeriodPanel):
    columns = (("Voltage (V)", 140), ("Start (s)", 90), ("Duration (s)", 90), ("End (s)", 90))

    def __init__(self, parent, editor, index):
        self.index = index
        self.psu = editor.controller.psus[index]
        super().__init__(parent, editor)

    def periods(self):
        return sorted((p for p in self.editor.stimulus.psu_activations if p.psu == self.index),
                      key=lambda p: p.start)

    def _row(self, period):
        return (_fmt(period.voltage), _fmt(period.start), _fmt(period.duration), _fmt(period.end))

    def _build_fields(self, form, first_row):
        self.voltage_var = tk.StringVar()
        ttk.Label(form, text="Voltage (V)").grid(row=first_row, column=0, sticky="w", pady=2)
        ttk.Entry(form, textvariable=self.voltage_var, width=10).grid(
            row=first_row, column=1, sticky="w", pady=2)
        ttk.Label(form, foreground="#666",
                  text=f"0 – {_fmt(self.psu.max_voltage)} V, idles at {_fmt(self.psu.idle_voltage)} V"
                  ).grid(row=first_row + 1, column=0, columnspan=2, sticky="w")

    def _fill_fields(self, period):
        self.voltage_var.set(_fmt(period.voltage))

    def _make_period(self, start, duration):
        voltage = _parse_float(self.voltage_var, "Voltage")
        self.psu.check_voltage(voltage)
        return PsuActivation(self.index, start, duration, voltage)


class ManualPanel(ttk.Frame):
    """Direct control of each ERM and PSU, e.g. for checking the wiring. Locked while a stimulus runs."""

    def __init__(self, parent, editor):
        super().__init__(parent, padding=8)
        self.editor = editor
        controller = editor.controller
        self._widgets = []      # everything to lock while a stimulus runs

        erm_box = ttk.LabelFrame(self, text="ERMs", padding=6)
        erm_box.grid(row=0, column=0, sticky="nw")
        self.erm_vars = []
        for i, erm in enumerate(controller.erms):
            var = tk.BooleanVar()
            button = ttk.Checkbutton(erm_box, text=f"ERM {i} ({erm})", variable=var,
                                     command=functools.partial(self._toggle, i))
            button.grid(row=i % 4, column=i // 4, sticky="w", padx=(0, 10), pady=1)
            self.erm_vars.append(var)
            self._widgets.append(button)

        psu_box = ttk.LabelFrame(self, text="PSUs", padding=6)
        psu_box.grid(row=0, column=1, sticky="nw", padx=(14, 0))
        self.voltage_vars = []
        self.readouts = []
        for k, psu in enumerate(controller.psus):
            var = tk.StringVar(value=_fmt(psu.idle_voltage))
            ttk.Label(psu_box, text=psu.name).grid(row=k, column=0, sticky="w", padx=(0, 6))
            entry = ttk.Entry(psu_box, textvariable=var, width=8)
            entry.grid(row=k, column=1, pady=2)
            entry.bind("<Return>", lambda event, k=k: self._set_voltage(k))
            ttk.Label(psu_box, text="V").grid(row=k, column=2, padx=(2, 6))
            button = ttk.Button(psu_box, text="Set", command=functools.partial(self._set_voltage, k))
            button.grid(row=k, column=3)
            readout = ttk.Label(psu_box, width=12, foreground=PSU_LINE_COLOR)
            readout.grid(row=k, column=4, padx=(8, 0))
            self.voltage_vars.append(var)
            self.readouts.append(readout)
            self._widgets += [entry, button]

        reset = ttk.Button(self, text="All ERMs off, PSUs to idle", command=self.reset_devices)
        reset.grid(row=1, column=0, columnspan=2, sticky="w", pady=(12, 0))
        self._widgets.append(reset)

    def _toggle(self, i):
        erm = self.editor.controller.erms[i]
        if self.erm_vars[i].get():
            erm.on()
        else:
            erm.off()

    def _set_voltage(self, k):
        if self.editor.running:
            return
        psu = self.editor.controller.psus[k]
        try:
            psu.set_voltage(_parse_float(self.voltage_vars[k], "Voltage"))
        except Exception as e:
            messagebox.showerror(psu.name, str(e), parent=self)

    def reset_devices(self):
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
        for label, psu in zip(self.readouts, controller.psus):
            label.configure(text="" if psu.voltage is None else f"now {psu.voltage:.2f} V")


class StimulusEditor(tk.Tk):
    POLL_MS = 50

    def __init__(self, controller: StimulusController, path: str | None = None):
        super().__init__()
        self.controller = controller
        self.stimulus = Stimulus()
        self.path = None
        self.dirty = False
        self.selected = None        # the period highlighted in the timeline and its table
        self._run_thread = None
        self._run_stop = None
        self._run_error = None
        self._run_duration = 0.0
        self._stopped_by_user = False

        ttk.Style(self).theme_use("clam")
        self.geometry(f"{min(1100, self.winfo_screenwidth() - 40)}x"
                      f"{min(700, self.winfo_screenheight() - 80)}")
        self.minsize(760, 440)

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
        self.psu_panels = []
        for k, psu in enumerate(controller.psus):
            panel = PsuPanel(self.tabs, self, k)
            self.tabs.add(panel, text=f"{psu.name} periods")
            self.psu_panels.append(panel)
        self.manual = ManualPanel(self.tabs, self)
        self.tabs.add(self.manual, text="Manual control")

        self.bind("<Control-n>", lambda event: self.new_stimulus())
        self.bind("<Control-o>", lambda event: self.open_stimulus())
        self.bind("<Control-s>", lambda event: self.save())
        self.bind("<F5>", lambda event: self.run())
        self.bind("<Escape>", lambda event: self.stop())
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self._set_status("Ready.  F5 runs the stimulus, Esc stops it.")
        self._refresh()
        if path:
            self.open_stimulus(path)
        self.after_idle(self.manual.reset_devices)   # start from a known state
        self._poll()

    def _build_toolbar(self):
        bar = ttk.Frame(self, padding=6)
        bar.pack(side=tk.TOP, fill=tk.X)
        for text, command in [("New", self.new_stimulus), ("Open…", self.open_stimulus),
                              ("Save", self.save), ("Save as…", self.save_as)]:
            ttk.Button(bar, text=text, command=command).pack(side=tk.LEFT, padx=(0, 4))
        ttk.Separator(bar, orient=tk.VERTICAL).pack(side=tk.LEFT, fill=tk.Y, padx=8)
        self.run_button = ttk.Button(bar, text="▶ Run (F5)", command=self.run)
        self.run_button.pack(side=tk.LEFT, padx=(0, 4))
        self.stop_button = ttk.Button(bar, text="■ Stop (Esc)", command=self.stop, state="disabled")
        self.stop_button.pack(side=tk.LEFT)

        devices = ([(f"ERM {i}", erm) for i, erm in enumerate(self.controller.erms)]
                   + [(psu.name, psu) for psu in self.controller.psus])
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
        return [self.erm_panel, *self.psu_panels]

    def _panel_for(self, period):
        return self.erm_panel if isinstance(period, ErmActivation) else self.psu_panels[period.psu]

    def _list_for(self, period):
        if isinstance(period, ErmActivation):
            return self.stimulus.erm_activations
        return self.stimulus.psu_activations

    def select(self, period, show_tab=False):
        self.selected = period
        for panel in self._panels():
            panel.show_selection(period)
        if show_tab and period is not None:
            self.tabs.select(self._panel_for(period))
        self.timeline.redraw()

    def add_period(self, period):
        self._list_for(period).append(period)
        self._changed(select=period)

    def replace_period(self, old, new):
        periods = self._list_for(old)
        periods[_index_of(periods, old)] = new
        self._changed(select=new)

    def remove_period(self, period):
        periods = self._list_for(period)
        del periods[_index_of(periods, period)]
        self._changed(select=None)

    def _changed(self, select):
        self.dirty = True
        self.selected = select
        self._refresh()

    def _refresh(self):
        name = os.path.basename(self.path) if self.path else "Untitled"
        self.title(f"{name}{' *' if self.dirty else ''} — Stimulus Editor")
        for panel in self._panels():
            panel.refresh()
        self.timeline.redraw()

    # --- files ---------------------------------------------------------------

    def _set_stimulus(self, stimulus, path):
        self.stimulus = stimulus
        self.path = path
        self.dirty = False
        self.selected = None
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
        self.dirty = False
        self._refresh()
        return True

    def save_as(self):
        path = filedialog.asksaveasfilename(parent=self, defaultextension=".json",
                                            filetypes=[("Stimulus", "*.json")])
        if not path:
            return False
        self.path = path
        return self.save()

    # --- running -------------------------------------------------------------

    @property
    def running(self):
        return self._run_thread is not None

    def run(self):
        if self.running:
            return
        stimulus = copy.deepcopy(self.stimulus)     # edits made while it plays don't affect it
        if not (stimulus.erm_activations or stimulus.psu_activations):
            self._set_status("Nothing to run: add some periods first.")
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
            self._set_status("Stopped. ERMs off, PSUs at idle.")
        else:
            self._set_status("Finished. ERMs off, PSUs at idle.")

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
            self._set_status(f"Running   {max(0.0, elapsed):.2f} / {self._run_duration:.2f} s")
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
    erms = [
        ERM(ERM_PINS[0], ErmType.SMALL),
        ERM(ERM_PINS[1], ErmType.SMALL),
        ERM(ERM_PINS[2], ErmType.BIG),
        ERM(ERM_PINS[3], ErmType.BIG),
    ]
    psus = [
        PSU(PSU1_PORT, name="PSU1", idle_voltage=0.0),
        PSU(PSU2_PORT, name="PSU2", idle_voltage=0.0),
    ]
    editor = StimulusEditor(StimulusController(erms, psus), sys.argv[1] if len(sys.argv) > 1 else None)
    signal.signal(signal.SIGINT, lambda *args: editor.shutdown())   # Ctrl+C in the terminal
    editor.mainloop()


if __name__ == "__main__":
    main()
