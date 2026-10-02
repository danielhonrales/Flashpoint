# HARDWARE.md — Wearable Haptic Interface

Reference for anyone (human or agent) writing control code for this device.
Target platform: Raspberry Pi 4 Model B, Raspberry Pi OS (Bookworm), Python 3.

---

## 1. What the device is

An arm-worn multisensory interface driven by game events from a Meta Quest 3
over LAN. Four output modalities:

| Modality | Count | Hardware |
|---|---|---|
| Vibrotactile (broadband) | 8 | bHaptics ERM coin motors |
| Vibrotactile (resonant) | 2 | Vybronics VG2230001H LRA, 70 Hz |
| Thermal | 2 | TEC1-12704 Peltier, 30×30 mm |
| Visual | 10 px | WS2812B LED strip |

Thermal targets are roughly ±5 °C around skin baseline (~32 °C), i.e. ~27–37 °C.
Typical actuation pattern: ~7 s bursts every ~20 s.

---

## 2. Power architecture

Two independent 5 V / 3 A USB-C power banks. **They share ground and nothing else.**

### Bank A — logic
- Raspberry Pi 4 via USB-C.
- LED strip and 74AHCT125 via Pi pin 2 (5 V).
- Budget: Pi 3–6 W + LEDs 1–1.5 W.

### Bank B — actuator bus
- USB-C sink breakout (5.1 kΩ CC resistors) → switch → 2.5 A PTC fuse →
  1000 µF bulk cap → terminal block.
- Feeds: ULN2803 COM + ERM positives, both DRV8876 VIN, PAM8406 VCC,
  100 Ω dummy load.
- Budget at peak: Peltiers ~8 W (duty-capped) + ERMs ~2 W + amp ~0.5 W
  ≈ 12 W of 15 W available.

### Rules
- **Never** connect bus +5 V to a Pi 5 V pin. Ground is the only shared net.
- Star ground at the bulk cap negative; Pi GND ties in with one wire.
- The 100 Ω dummy load exists so Bank B does not auto-shut-off during idle gaps.
  Do not remove it.

---

## 3. Pi header pin map

| Pin | Signal | Destination |
|---|---|---|
| 1 | 3.3 V | PCA9685 VCC, ADS1115 VDD, NTC divider tops |
| 2 | 5 V | LED strip +5V, 74AHCT125 VCC |
| 3 | GPIO2 / SDA | PCA9685, ADS1115 |
| 5 | GPIO3 / SCL | PCA9685, ADS1115 |
| 6, 9, 14 | GND | Common ground rail |
| 11 | GPIO17 | DRV8876 #1 PH (direction) |
| 13 | GPIO27 | DRV8876 #1 nSLEEP (low = driver asleep, outputs off) |
| 15 | GPIO22 | DRV8876 #2 PH (direction) |
| 18 | GPIO24 | PAM8406 SD (amp enable; low = shutdown) |
| 19 | GPIO10 | DRV8876 #2 nSLEEP (low = driver asleep, outputs off) |
| 22 | GPIO25 | PCA9685 OE (HIGH = all PWM outputs off) |
| 3.5 mm jack | Audio L / R | PAM8406 L in / R in |

Notes:
- LED data has no pin yet: GPIO10 is now DRV8876 #2 nSLEEP (see §5, LEDs).
- SPI must be enabled for the LEDs (`raspi-config`); set `core_freq_min=500` in
  `/boot/config.txt` for stable WS2812B timing.
- **Do not use GPIO18 for the LEDs.** It shares hardware PWM with the audio
  jack, which drives the LRAs.

---

## 4. I²C bus

| Address | Device | Role |
|---|---|---|
| 0x40 | PCA9685 | All PWM generation, 16 channels |
| 0x48 | ADS1115 | 4-channel 16-bit ADC, thermistor reads |

PCA9685 frequency is **global** across all channels. Set it to **150 Hz** (≈149 Hz
actual). At 1–1.5 kHz the ERMs' PWM is an audible whine; at 150 Hz it sits near their
own vibration frequency and blends into their buzz.
Trade-off: the Peltiers share this frequency. They are usually run at ≥1 kHz; at 150 Hz
their thermal mass still smooths each pulse, but supply ripple is higher. If that
matters, move the Peltier ENs to a second PCA9685 (e.g. 0x41) at 1–1.5 kHz.

### PCA9685 channel map

| Channel | Load | Notes |
|---|---|---|
| 0–7 | ERM 1–8 | via ULN2803 inputs 1–8 |
| 8–13 | spare | |
| 14 | DRV8876 #1 EN | Peltier 1 power |
| 15 | DRV8876 #2 EN | Peltier 2 power |

PCA9685 `V+` terminal is **unconnected**. Only each channel's signal pin is used.

### ADS1115 channel map

| Channel | Signal |
|---|---|
| A0 | Peltier 1 thermistor divider junction |
| A1 | Peltier 2 thermistor divider junction |
| A2 / A3 | Optional DRV8876 IPROPI current sense |

Divider: 3.3 V → 10 kΩ → junction → 10 kΩ NTC (β 3950) → GND.
NTC beads are epoxied to the **skin-side** face of each Peltier.

---

## 5. Output stages

### ERMs — ULN2803 (low-side switching)
- PCA9685 ch *n* → ULN input pin *n+1*; output pin = 19 − input pin.
- ERM + → +5 V bus; ERM − → ULN output. The chip **sinks only**.
- Pin 10 (COM) → +5 V (internal flyback diodes). Pin 9 → GND.
- **Verify the bHaptics ERMs' rated voltage.** The 0.75 cap below assumes a 3 V motor
  on the 5 V rail (it was set for the earlier JIEYI JYC1027); recompute as rated V / 5 V.

| ERM | PCA ch | ULN in | ULN out |
|---|---|---|---|
| 1–8 | 0–7 | 1–8 | 18–11 |

### Peltiers — DRV8876 (Pololu carrier, one per module)
- Mode: **PH/EN**. PMODE → GND. nSLEEP on a GPIO: software keeps the driver asleep whenever
  the Peltier is off, which also stops it if I²C fails.
- EN = PWM duty (power). PH = direction: on both current units **PH LOW heats the skin face**
  (`heat_ph` in stimulus_editor.py; verify per unit).
- OUT1/OUT2 → Peltier leads. Outputs are **bridged**; never ground either one.
- 470 µF across VIN/GND at each driver.
- TEC1-12704 ≈ 2.9–3.2 Ω → **~1.6 A at 5 V**. DRV8876 TSSOP is 1.3 A continuous,
  so duty above ~0.66 exceeds the rating (RMS = 1.6 A × √duty). The software allows up to
  1.0; long periods that high risk the driver's thermal shutdown, and both Peltiers high at
  once exceed the bus's 2.5 A fuse.
- Heat sink (≥30×30 mm) on the outward face is **mandatory** whenever powered.

### LRAs — PAM8406 class-D amp
- 5 V supply. Stereo input (L / G / R) from the Pi audio jack.
- L+/L− → Bottom LRA (near the elbow); R+/R− → Top LRA (near the wrist). Bridged outputs;
  never ground a terminal. (Previously called LRA 1 / LRA 2, or Left / Right.)
- SD pin (GPIO24) mutes the amp between stimuli.
- Drive at ~70 Hz (resonance). Target ~2.0 Vrms at the terminals.
- Amplitude-modulate envelopes; abrupt starts produce audible clicks.
- Optional third LRA: 74HC4052 analog mux on the amp *inputs* plus a second amp
  board, selecting 2 of 3 sites. The Pi only has 2 audio channels.

### LEDs — WS2812B
- Data through a 74AHCT125 (3.3 V → 5 V), 330 Ω in series. **Needs a new data pin:** GPIO10
  (SPI0 MOSI) is now DRV8876 #2 nSLEEP. SPI1 MOSI on GPIO20 (`dtoverlay=spi1-1cs`) is the
  nearest equivalent.
- 470 µF across strip power at the strip.
- **Cap brightness at 0.5** — 10 px at full white is ~3 W.

---

## 6. Software limits (enforce these in code)

```python
MAX_DUTY_ERM      = 0.75   # rated V / 5 V; 0.75 assumes 3 V (verify for bHaptics)
MAX_DUTY_PELTIER  = 1.00   # raised from 0.55: see §5 (RMS > 1.3 A above ~0.66)
MAX_LED_BRIGHT    = 0.50   # power budget
PWM_FREQ_HZ       = 150    # global, PCA9685 (ERM hum; see §4)
TEMP_MAX_C        = 42.0   # skin burn threshold
TEMP_MIN_C        = 15.0   # cold-pain threshold
CONTROL_RATE_HZ   = 50     # PID loop
```

### Required safety behaviour
- Hard cutoff if either thermistor reads > `TEMP_MAX_C` or < `TEMP_MIN_C`.
- Sensor-fault detection: an open or shorted NTC reads as an extreme value —
  treat as a fault and disable that Peltier.
- Fault action: set GPIO25 **HIGH** (PCA9685 OE) to kill all PWM outputs at once.
- Watchdog: if the control loop stalls, drive OE high.
- Thermal control is closed-loop PID on the thermistor. Duty sets *power*,
  not temperature. Clamp integral term (anti-windup) — bursts saturate it.
- Heating is Joule-assisted; cooling fights it. Use separate gains or a higher
  cooling duty ceiling.

---

## 7. Networking

- Quest 3 → Pi over LAN, **UDP** (late packets are worse than lost ones).
- Static IPs on both; use a dedicated travel router, not venue Wi-Fi.
- Disable Wi-Fi power save: `iw wlan0 set power_save off`.
  This is the most common cause of sporadic 100 ms+ latency.
- Latency budget: ~5–20 ms network vs ~20–30 ms ERM spin-up and seconds of
  thermal response. The network is not the bottleneck.

---

## 8. Python dependencies

```bash
sudo pip3 install adafruit-circuitpython-pca9685 adafruit-circuitpython-ads1x15 \
  adafruit-circuitpython-neopixel-spi gpiozero --break-system-packages
```

Device handles:

```python
import board, busio
from adafruit_pca9685 import PCA9685
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from gpiozero import DigitalOutputDevice

i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c);  pca.frequency = 150         # 0x40
ads = ADS.ADS1115(i2c)                            # 0x48

ph1 = DigitalOutputDevice(17)   # Peltier 1 direction
sleep1 = DigitalOutputDevice(27, initial_value=False)   # Peltier 1 nSLEEP, low = asleep
ph2 = DigitalOutputDevice(22)   # Peltier 2 direction
sleep2 = DigitalOutputDevice(10, initial_value=False)   # Peltier 2 nSLEEP, low = asleep
amp = DigitalOutputDevice(24, initial_value=False)   # PAM8406 SD
oe  = DigitalOutputDevice(25, initial_value=True)    # PCA OE, HIGH = off
```

---

## 9. Known gotchas

- **PMODE floating** on a DRV8876 puts it in independent half-bridge mode and
  EN/PH stop behaving. It must be grounded.
- **ULN2803 pin 10 unconnected** removes flyback protection and will kill the chip.
- **Peltier polarity varies** between units. Verify heat vs cool per module before
  trusting `PH`.
- **Never power a Peltier without its heat sink**, even briefly.
- **Thermistor noise**: run NTC pairs as twisted 28 AWG, away from Peltier output
  wires. The divider reference and ADC reference are the same 3.3 V rail, so
  keep switching loads off that rail.
- **Bank auto-shutoff**: if the bus dies during idle gaps, check the dummy load.
- **Bus sag below 4.8 V** under load means wire gauge or duty caps, not software.

---

## 10. Placement on the forearm

Directions: **top** = towards the wrist, **bottom** = towards the elbow, **inner** = ventral
(palm side), **outer** = dorsal (back of the forearm), **left** = thumb side. Positions are cm
from the wrist end of the layout.

Three columns run along the forearm, and a Peltier sits in the middle of each side:

| cm from wrist | Outer column (4 cm spacing) | Inner-left column (8 cm) | Inner-right column (8 cm) | Inner centre |
|---|---|---|---|---|
| 0 | **Top LRA** (audio R) | ERM 2 | ERM 5 | |
| 4 | ERM 0 | | | |
| 8 | **Peltier 1** | ERM 3 | ERM 6 | **Peltier 2** |
| 12 | ERM 1 | | | |
| 16 | **Bottom LRA** (audio L) | ERM 4 | ERM 7 | |

| ERM (editor index) | PCA ch | Place |
|---|---|---|
| 0 | 0 | outer column, 4 cm |
| 1 | 1 | outer column, 12 cm |
| 2 / 3 / 4 | 2 / 3 / 4 | inner-left column, 0 / 8 / 16 cm |
| 5 / 6 / 7 | 5 / 6 / 7 | inner-right column, 0 / 8 / 16 cm |

- Assumed, to confirm: the inner columns span the same 0–16 cm as the outer column (ERM 2 and 5
  at the wrist end), and Peltier 1 is the outer one.
- Hits land on the **outer** forearm first (the LRAs, ERM 0/1, Peltier 1) and reverberate onto
  the inner side.
- Motion paths this allows: **along** the arm down the outer column (Top LRA → ERM 0 → ERM 1 →
  Bottom LRA, 4 cm steps) or an inner column (8 cm steps); **around** it across the three
  columns; or **both**, e.g. a helix ERM 0 → 3 → 6 → 1 → 4 → 7.
- **Calibration**: the Top LRA is 50% stronger than the Bottom one, so `LRA_GAINS` in
  stimulus_editor.py plays it at 2/3 amplitude to even them out.
