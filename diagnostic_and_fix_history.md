# Project History: Holmarc Stage Calibration & Circle Distortion Resolution

This document serves as a comprehensive record of the engineering diagnostics, experimental findings, fix attempts, and the final successful resolution for the Holmarc electro-lithography stage circle distortion issue.

---

## 1. Project Background
The system uses a Holmarc stepper motor stage with a coordinate resolution of **$0.2\ \mu\text{m/step}$** to plot circular patterns. During circular interpolation, the axes ($X$ and $Y$) must move coordinates simultaneously, requiring precise, independent velocity control on each COM port.

---

## 2. Phase 1: Problem Identification (Circle Distortion)
### The Symptom
When drawing circles at slow path velocities (e.g., $10\ \mu\text{m/s}$ or $50\text{ steps/s}$), the stage produced flat lines at the cardinal points (axes reversal extremes) instead of a smooth circle.

```
       Original Distorted Circle               Corrected Circle (Slight Backlash)
               
                 Top Flat                                     Top
              .-----------.                                . - - - .
            /               \                            /           \
           /                 \                          /             \
Left Flat |                   | Right Flat        Left |               | Right
           \                 /                          \             /
            \               /                            \           /
              '-----------'                                ' - - - '
               Bottom Flat                                  Bottom
```

### Initial Diagnoses
1. **Mechanical Backlash**: Play in the lead screw nuts/gears during axis direction reversal.
   * *Status*: **Disproved** as the primary cause. While minor backlash exists, it could not explain the large, flat, extended lines at the cardinal points.
2. **Serial Communication Jitter**: Start timing latency between X and Y axes threads.
   * *Status*: **Secondary factor**. Contributed ~140 ms constant latency per segment, but was not the source of the cardinal flats.

---

## 3. Phase 2: Discovery of the Firmware Speed Bands Bug
Through systematic timing sweeps (`speed_sweep_test.py`), we ran calibration runs over different speeds and measured the expected vs. actual duration of travel.

### The Bug
The controller's firmware systematically overrode specific commanded speed ranges:
1. **Band A ($20\text{ to }30\text{ steps/s}$)**: Overridden exponentially up to $1200\text{ steps/s}$.
2. **Band B ($130\text{ to }240\text{ steps/s}$)**: Overridden to run up to $12\times$ faster than commanded.

### Root Cause: Serial State Parser Collision
The Holmarc stepper controller is a single-axis board using a flawed command parameter parser. When setting speeds, the python script splits the speed into 3 bytes (`MSB`, `CSB`, `LSB`).
* The controller's serial Interrupt Service Routine (ISR) intercepts incoming bytes and checks them against global command codes (such as `ACCELERATION = 20`, `DECELERATION = 30`, `Dir_plus = 125`), even when it is supposed to be in the "parameter receiving" state.
* When a speed byte value matched a command code, the state machine aborted speed configuration and executed the code, corrupting the timer register and causing the stage to move at uncontrolled high speeds.

---

## 4. Phase 3: Fix Attempts & Results

### Attempt 1: Independent Speed Shifting
* **Approach**: Shifted the speed of each axis independently out of the bad bands (e.g., if X speed fell in $[20, 30]$, we shifted it to $31$ steps/s while keeping Y speed at its original value).
* **Result**: **REJECTED**.
* **Reasoning**: This resolved the massive speed overrides but broke **coordinate synchronization**. Changing the speed ratio ($\frac{v_x}{v_y}$) altered the vector slope of the segments, causing the circle geometry to warp.

### Attempt 2: Proportional Speed Scaling (Broad Bands)
* **Approach**: Introduced a proportional scaling filter inside `compute_synchronized_speeds` in [main.py](file:///D:/Python%20codes/hmccontroller/main.py). If either axis speed fell into a bad band ($[20, 30]$ or $[130, 240]$ steps/s), both speeds were scaled by the same factor $k \approx 1.0$ (stepping outward: $1.01, 0.99, \dots$).
* **Result**: **PARTIALLY ACCEPTED**.
* **Reasoning**: This successfully resolved the flat-line overrides at the cardinal points while keeping coordinate synchronization. However, the circle remained slightly distorted (pinched at the diagonals).
* **New Challenge**: The broad band ranges forced scaling factors $k$ to deviate by up to **$\pm 21\%$** ($k = 0.79$ or $k = 1.21$). Over 20% of all circle segments were running at the wrong speeds, leading to noticeable geometric inaccuracies.

---

## 5. Phase 4: Fine-Grained Calibration & Final Resolution

### High-Resolution Sweep
To minimize the speed deviations, we ran a high-resolution, 1-step increment sweep from $10$ to $250\text{ steps/s}$ using `speed_fine_sweep.py` over a safe $20\ \mu\text{m}$ offset (to avoid home switch limits).

The sweep revealed the exact boundaries of the bad firmware bands:
* **Band A**: $[16, 30]$ steps/s (15 values)
* **Band B**: $[123, 244]$ steps/s (122 values)

### The Final Fix
We updated [main.py](file:///D:/Python%20codes/hmccontroller/main.py) to use these precise, empirical blacklist ranges:
```python
def is_bad_speed(spd):
    spd_int = round(spd)
    if 16 <= spd_int <= 30:
        return True
    if 123 <= spd_int <= 244:
        return True
    return False
```

### Result: ACCEPTED & FULLY VERIFIED
* Because the bad bands were narrowed, the scaling factor $k$ now stays extremely close to $1.0$ (typically within $\pm 1\%$ to $2\%$) to dodge the blacklist boundaries.
* **Verification**: A circle of **$500\ \mu\text{m}$ radius** was plotted and confirmed to run perfectly without any geometry distortion.

---

## 6. Key Takeaways for Future Systems
1. **Parameter Safety in Parsers**: Firmware serial parsers must use strict state tracking (e.g., counting expected parameter bytes) rather than globally intercepting all incoming bytes as potential commands.
2. **Coordinated Motion Workarounds**: In decoupled axis systems, speed changes must be proportional across all axes to maintain trajectory angles.
3. **High-Resolution Calibration**: Coarse calibration sweeps (10-step increments) can mischaracterize single byte-collisions as broad failure bands. A high-resolution sweep is necessary to identify exact boundaries.
