# Electro-Lithography System — LabVIEW Architecture & Implementation Plan

This document contains the complete system architecture, algorithms, serial communication protocols, state machine design, and LabVIEW implementation plan for converting the Python-based electro-lithography control system to a fully native LabVIEW application.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [System Architecture](#2-system-architecture)
3. [LabVIEW Project Structure](#3-labview-project-structure)
4. [Serial Communication Protocol (Motor Controller)](#4-serial-communication-protocol-motor-controller)
5. [SMU Communication Protocol (SCPI over VISA)](#5-smu-communication-protocol-scpi-over-visa)
6. [State Machine Design](#6-state-machine-design)
7. [Algorithms](#7-algorithms)
8. [Parallel Loop Architecture](#8-parallel-loop-architecture)
9. [Complete Mode 6 Sequence](#9-complete-mode-6-sequence)
10. [Error Handling Strategy](#10-error-handling-strategy)
11. [Configuration Constants](#11-configuration-constants)
12. [Coordinate File Format](#12-coordinate-file-format)
13. [Coordinate Pattern Generator Utilities](#13-coordinate-pattern-generator-utilities)
14. [Bug Fixes from Python Codebase](#14-bug-fixes-from-python-codebase)
15. [Implementation Priority](#15-implementation-priority)

---

## 1. System Overview

### 1.1 What Is Electro-Lithography?

Electro-lithography is a scanning probe, maskless lithography technique that uses electrical stimuli to directly write patterns on surfaces at micro-to-nano length scales. A conductive probe is brought into contact with a substrate, and a controlled electric field or current is applied through the tip, causing localized material transport. This enables direct writing of patterns (lines, dots, shapes) without photomasks.

**How it Works:**
- A conductive probe tip (like those in Scanning Probe Microscopes) is positioned above or in contact with a thin metallic film or substrate
- A controlled electric field or current is applied through the tip, causing localized material transport, either by electromigration or electrochemical reactions at the probe-sample interface
- This enables direct writing of patterns (e.g., lines, dots, shapes) without requiring photomasks, often under ambient conditions and with resolutions below 10 nm

### 1.2 Hardware Components

| Component | Model/Type | Interface |
|-----------|-----------|-----------|
| Motor Controller | Holmarc (XYZ) | Serial (19200 baud, 8N1) |
| Micropositioners | XYZ Stage | Driven by Holmarc controller |
| SMU | Keithley (2400/2450/2600) | USB VISA (SCPI commands) |
| Probe | Conductive SPM-style tip | Mounted on Z positioner |
| Substrate | Thin metallic film | Mounted on XY stage |

### 1.3 System Specifications

| Parameter | Value |
|-----------|-------|
| Motion Resolution | 0.2 um/step |
| Max Travel (per axis) | 50,000 um (50 mm) |
| Max Steps (per axis) | 16,777,216 |
| Speed Range | 10 - 50,000 steps/sec |
| SMU VISA Address | `USB0::0x2184::0x007D::gey853397::INSTR` |
| Serial Baud Rate | 19200 |

---

## 2. System Architecture

### 2.1 High-Level Architecture

```
+----------------------------------------------------------------------+
|                        LabVIEW Main Application                      |
|                                                                      |
|  +-------------+    +--------------+    +------------------------+  |
|  |  Front Panel |    | State Machine|    |  Error Handler         |  |
|  |  (UI)        |<-->| (Sequencer)  |<-->|  (Logging/Recovery)    |  |
|  +------+------ +    +------+-------+    +------------------------+  |
|         |                  |                                         |
|    +----+------------------+---------------------------+            |
|    |              Parallel Loops (Producer/Consumer)     |            |
|    |                                                     |            |
|    |  +--------------+     +------------------------+  |            |
|    |  | Motor Control |     |  SMU Control            |  |            |
|    |  | Loop          |     |  Loop                   |  |            |
|    |  |               |     |                          |  |            |
|    |  | - Set Speed   |     |  - Init/Reset SMU       |  |            |
|    |  | - Move XYZ    |     |  - Source Current/Volt   |  |            |
|    |  | - Home Axes   |     |  - Read Voltage          |  |            |
|    |  | - Read Pos    |     |  - Read Current          |  |            |
|    |  | - Limit Detect|     |  - Output On/Off         |  |            |
|    |  +------+------- +     +-----------+--------------+  |            |
|    |         |                          |                  |            |
|    |    +----+----+              +-----+------+          |            |
|    |    | VISA    |              | VISA        |          |            |
|    |    | Serial  |              | USB/GPIB    |          |            |
|    |    +----+----+              +-----+------+          |            |
|    +---------+--------------------------+-----------------+            |
|              |                          |                             |
+--------------+--------------------------+---------------------------+
               |                          |
    +----------+----------+    +----------+----------+
    |  Holmarc Motor      |    |  Keithley SMU        |
    |  Controller         |    |  (2400/2450/2600)    |
    |  (XYZ Stepper)      |    |                      |
    |  19200 baud serial  |    |  USB VISA            |
    +----------+----------+    +----------+----------+
               |                          |
    +----------+----------+    +----------+----------+
    |  XYZ Micropositioner|    |  Probe-Substrate     |
    |  (0.2um resolution) |    |  Interface            |
    +---------------------+    +---------------------+
```

### 2.2 Core Workflow (Mode 6 - Full Lithography)

1. Initialize SMU (current source mode for contact detection)
2. Move XY to starting position from coordinate file
3. Lower Z in steps while measuring current to find contact point (current spike = probe-substrate contact)
4. Switch SMU to voltage source mode for patterning
5. Execute pattern: for each coordinate point:
   - Read voltage via SMU
   - If voltage out of range, adjust Z by delta_z
   - Move XY to next point
   - Handle liftoff flag (raise probe, move, re-find contact)
6. After pattern complete: SMU output off, home all axes

---

## 3. LabVIEW Project Structure

```
ElectroLithography.lvproj/
|
+-- SubVIs/
|   +-- Motor Controller/
|   |   +-- Motor_Init.vi                 <- Open serial, verify device ID
|   |   +-- Motor_Close.vi                <- Close serial port
|   |   +-- Motor_SetSpeed.vi             <- Set XYZ speed (steps/sec)
|   |   +-- Motor_Move.vi                 <- Send move command, wait completion
|   |   +-- Motor_MoveHome.vi             <- Home all axes sequentially
|   |   +-- Motor_Stop.vi                 <- Emergency stop
|   |   +-- Motor_ReadPosition.vi         <- Read actual steps from controller
|   |   +-- Motor_ReadStatus.vi           <- Poll movement status byte
|   |   +-- Motor_CheckLimits.vi          <- Check limit switch flags
|   |   +-- Motor_Utils/
|   |       +-- Byte3_Byte2_Byte1_Byte0.vi   <- Split 32-bit to 4 bytes
|   |       +-- MSB_CSB_LSB.vi               <- Split 24-bit to 3 bytes
|   |       +-- Build_Command.vi              <- Assemble command packet
|   |
|   +-- SMU Controller/
|   |   +-- SMU_Init.vi                   <- Open VISA, *RST, *CLS
|   |   +-- SMU_Close.vi                  <- Output off, close VISA
|   |   +-- SMU_Reset.vi                  <- *CLS + *RST
|   |   +-- SMU_SourceCurrent.vi          <- Use Case 1: current source + V compliance
|   |   +-- SMU_SourceVoltage.vi          <- Use Case 2: voltage source + I compliance
|   |   +-- SMU_ReadVoltage.vi            <- READ? -> parse voltage -> classify
|   |   +-- SMU_ReadCurrent.vi            <- READ? -> parse current -> classify
|   |   +-- SMU_OutputOnOff.vi            <- OUTP ON/OFF
|   |
|   +-- Pattern Loader/
|   |   +-- LoadCoordinateFile.vi         <- Read .txt file -> parse X,Y,flag arrays
|   |   +-- ValidateCoordinates.vi        <- Bounds check, format validation
|   |
|   +-- Utilities/
|       +-- ListCOMPorts.vi               <- Enumerate serial ports
|       +-- AngleCalculation.vi           <- atan2, sin, cos for velocity decomposition
|       +-- Logging.vi                    <- Write to file/log
|
+-- State Machines/
|   +-- MainSelector.vi                   <- Top-level menu/mode selector
|   +-- ManualMove.vi                     <- Mode 1: XYZ manual move
|   +-- HomeAll.vi                        <- Mode 2: Home all axes
|   +-- OpenLoopPattern.vi                <- Mode 3: Execute pattern from file (no feedback)
|   +-- ZProbe.vi                         <- Mode 4: Z probe with speed control
|   +-- ZProbePauseResume.vi              <- Mode 5: Z probe with pause/resume
|   +-- LithographyRun.vi                 <- Mode 6: Full lithography (Z feedback + pattern)
|
+-- Parallel Loop Architectures/
|   +-- Z_FeedbackLoop.vi                 <- Real-time Z adjustment loop
|   +-- UserInputListener.vi              <- Speed/step-size input listener
|
+-- Front Panels/
|   +-- MainPanel.vi                      <- Main UI with mode selection
|   +-- ManualMovePanel.vi                <- XYZ jog controls
|   +-- LithographyPanel.vi               <- Parameter inputs + status display
|   +-- PatternViewer.vi                  <- Visualize loaded pattern
|
+-- Libraries/
|   +-- MotorProtocol.lvlib               <- Encapsulate serial protocol
|   +-- SMUProtocol.lvlib                 <- Encapsulate VISA SCPI commands
|
+-- Config/
    +-- SystemConfig.vi                   <- Centralized constants
```

---

## 4. Serial Communication Protocol (Motor Controller)

### 4.1 Protocol Overview

All communication is byte-level over serial at 19200 baud, 8N1. The controller uses a request-response protocol where every transmitted byte must be acknowledged.

### 4.2 Command Reference

#### SET SPEED (Command: 0x22 = 34)

Sets the speed for all three axes in steps/sec.

```
TX: [0x22]
RX: [0x0A]  (ACK)

TX: [MSB_X] [CSB_X] [LSB_X]    <- X speed (24-bit, 3 bytes)
RX: [0x0A] after each byte

TX: [MSB_Y] [CSB_Y] [LSB_Y]    <- Y speed (24-bit, 3 bytes)
RX: [0x0A] after each byte

TX: [MSB_Z] [CSB_Z] [LSB_Z]    <- Z speed (24-bit, 3 bytes)
RX: [0x0A] after each byte
```

Conversion: `steps_per_sec = microns_per_sec / 0.2`

#### MOVE (Command: 0x13 = 19)

Sends relative movement distances for all three axes.

```
TX: [0x13]
RX: [0x0A]

For each axis (A, B, C):
  TX: [0x0A]                    <- Request to send axis data
  TX: [byte3] [byte2] [byte1] [byte0]  <- Steps (32-bit, 4 bytes)
  TX: [0x7D] or [0xAF]         <- Direction: 0x7D=positive, 0xAF=negative
  RX: [0x0A] after each byte
```

Conversion: `steps = distance_microns / 0.2`

Step splitting (32-bit to 4 bytes):
```
byte0 = value % 256
byte1 = (value % 65536) // 256
byte2 = (value % 16777216) // 65536
byte3 = value // 16777216
```

#### READ STATUS (Command: 0xA3 = 163)

Polls the movement status after a MOVE command.

```
TX: [0xA3]
RX: [0xA4]  (READ_ACK)

TX: [0x0A]
RX: [status_byte]
```

Status byte values:

| Value (Dec) | Value (Hex) | Meaning |
|-------------|-------------|---------|
| 170 | 0xAA | Move completed |
| 41 | 0x29 | X home limit reached |
| 40 | 0x28 | X far limit reached |
| 43 | 0x2B | Y home limit reached |
| 42 | 0x2A | Y far limit reached |
| 45 | 0x2D | Z home limit reached |
| 44 | 0x2C | Z far limit reached |

#### READ ACTUAL STEPS (Command: 0xA3 = 163)

Reads the actual number of steps the controller moved on each axis.

```
TX: [0xA3]
RX: [0xA4]

For each axis (X, Y, Z):
  TX: [0x0A] -> RX: [byte0]
  TX: [0x0A] -> RX: [byte1]
  TX: [0x0A] -> RX: [byte2]
  TX: [0x0A] -> RX: [byte3]
```

Reconstruction: `steps = (byte3 * 16777216) + (byte2 * 65536) + (byte1 * 256) + byte0`

Actual distance: `microns = steps * 0.2`

#### STOP (Command: 0x68 = 104)

Immediately halts movement.

```
TX: [0x68]
RX: [0x69]  (STOP_ACK)
```

#### DEVICE ID (Command: 0x64 = 100)

Verifies communication with the controller.

```
TX: [0x64]
RX: [0xC8]  (DEVICE_ACK = 200)
```

---

## 5. SMU Communication Protocol (SCPI over VISA)

### 5.1 Initialization

```
VISA Open: USB0::0x2184::0x007D::gey853397::INSTR
Timeout: 5000ms
Write Termination: \n
Read Termination: \n

Write: *CLS          <- Clear status
Write: *RST          <- Reset to defaults
Wait: 100ms
```

### 5.2 Use Case 1 - Current Source (Contact Detection)

Sources a known current and measures voltage. A voltage drop indicates probe-substrate contact.

```
Write: SOUR:FUNC CURR
Write: SOUR:CURR {current_A}          <- e.g., 0.2e-6 for 0.2 uA
Write: SENS:VOLT:PROT {compliance_V}  <- e.g., 1.0 V
```

### 5.3 Use Case 2 - Voltage Source (Patterning)

Sources a known voltage and measures current. Used during pattern tracing with Z feedback.

```
Write: SOUR:FUNC VOLT
Write: SOUR:VOLT {voltage_V}          <- e.g., 0.5 V
Write: SENS:CURR:PROT {compliance_A}  <- e.g., 1000e-6 for 1 mA
```

### 5.4 Reading Measurements

```
Write: OUTP ON
Query: READ?
Response: "{voltage},{current}"       <- CSV string, e.g., "0.4523,1.234e-06"

Parse:
  voltage = float(response.split(",")[0])
  current = float(response.split(",")[1])
```

### 5.5 Output Control

```
Write: OUTP OFF    <- Turn off output
```

---

## 6. State Machine Design

### 6.1 Main Application State Machine

```
+-------------------------------------------------------------------+
|                     MAIN STATE MACHINE                             |
|                                                                    |
|  +----------+                                                     |
|  | S_IDLE   | <- Wait for user to select mode                     |
|  +----+-----+                                                     |
|       |                                                           |
|  +----v------------------------------------------------------+   |
|  | S_DISPATCH                                                 |   |
|  |   mode == 1 -> ManualMove.vi                               |   |
|  |   mode == 2 -> HomeAll.vi                                  |   |
|  |   mode == 3 -> OpenLoopPattern.vi                          |   |
|  |   mode == 4 -> ZProbe.vi                                   |   |
|  |   mode == 5 -> ZProbePauseResume.vi                        |   |
|  |   mode == 6 -> LithographyRun.vi                           |   |
|  +-----------------------------------------------------------+   |
|       |                                                           |
|  +----v-----+                                                    |
|  | S_DONE   | <- Return to idle, show completion message          |
|  +----------+                                                     |
+-------------------------------------------------------------------+
```

### 6.2 Lithography Run State Machine (Mode 6)

This is the most complex state machine. It orchestrates the entire lithography process.

```
+----------------------------------------------------------------------+
|                    LITHOGRAPHY RUN STATE MACHINE                      |
|                                                                      |
|  +----------+                                                        |
|  | S_INIT   | <- Read all parameters from UI:                        |
|  |          |   curr_ua, volt_compliance, volt_source, curr_comp,    |
|  |          |   volt_low, volt_high, delta_z, z_contact_step,        |
|  |          |   xy_speed, liftoff_height, filename                   |
|  +----+-----+                                                        |
|       |                                                               |
|  +----v-----+                                                        |
|  | S_SMU_   | <- SMU_Init -> SMU_Reset -> SMU_SourceCurrent          |
|  | CONFIG   |   (Use Case 1: current source for contact detection)   |
|  +----+-----+                                                        |
|       |                                                               |
|  +----v-----+                                                        |
|  | S_LOAD_  | <- LoadCoordinateFile.vi                               |
|  | PATTERN  |   Parse all lines -> arrays X[], Y[], Flag[]           |
|  |          |   Store first_x = X[0], first_y = Y[0]                |
|  +----+-----+                                                        |
|       |                                                               |
|  +----v-----+                                                        |
|  | S_MOVE_  | <- dx = first_x - current_x                            |
|  | TO_START |   dy = first_y - current_y                             |
|  |          |   Motor_SetSpeed(5000, 5000, 5000)                     |
|  |          |   Motor_Move(dx, dy, 0)                                |
|  |          |   Wait for completion                                  |
|  +----+-----+                                                        |
|       |                                                               |
|  +----v-----+                                                        |
|  | S_Z_     | <- Call ZProbeRoutine:                                 |
|  | PROBE    |   Repeat: Motor_Move(0,0,step) -> ReadCurrent -> check |
|  |          |   Until current > threshold                            |
|  |          |   Store z_contact_point                                |
|  +----+-----+                                                        |
|       |                                                               |
|  +----v-----+                                                        |
|  | S_SWITCH | <- SMU_SourceVoltage(volt_source, curr_comp)           |
|  | TO_      |   Now in voltage-source mode for Z feedback            |
|  | PATTERN  |                                                        |
|  +----+-----+                                                        |
|       |                                                               |
|  +----v-----+                                                        |
|  | S_TRACE  | <- Execute pattern loop (see Section 9)                |
|  | PATTERN  |   For each coordinate point:                           |
|  |          |     Read voltage -> Adjust Z -> Move XY -> Handle flag |
|  +----+-----+                                                        |
|       | (all coordinates done)                                       |
|  +----v-----+                                                        |
|  | S_SMU_   | <- SMU_OutputOff                                       |
|  | OFF      |                                                        |
|  +----+-----+                                                        |
|       |                                                               |
|  +----v-----+                                                        |
|  | S_HOME   | <- Motor_MoveHome (all axes sequentially)              |
|  +----+-----+                                                        |
|       |                                                               |
|  +----v-----+                                                        |
|  | S_DONE   | <- Display final position, cleanup, return to idle     |
|  +----------+                                                        |
+----------------------------------------------------------------------+
```

---

## 7. Algorithms

### 7.1 Z-Probe Contact Detection

Finds the exact Z position where the probe makes electrical contact with the substrate by monitoring current.

```
ALGORITHM: FindContactPoint
===========================

Input:
  smu_handle          - SMU VISA session
  lower_current_ua    - Current threshold in uA (e.g., 0.2)
  initial_step_size   - Initial Z step in um (e.g., 1000)

Output:
  z_contact_point     - Z position (um from home) where contact occurs

PROCEDURE:
  1. total_distance <- 0
  2. step_size <- initial_step_size

  3. REPEAT:
     a. Motor_Move(0, 0, step_size)           // Move Z down
     b. Wait for motor completion
     c. total_distance <- total_distance + step_size
     d. SMU_Write("OUTP ON")
     e. response <- SMU_Query("READ?")
     f. Parse current_A from response
     g. Log: "Step {i}: Z={total_distance} um, I={current_A} A"
     h. IF current_A > lower_current_ua * 1e-6:
        i.  Log: "Contact found at {total_distance} um"
        ii. RETURN total_distance
     i. Sleep(100ms)

  4. UNTIL contact found or max distance exceeded
```

### 7.2 Real-Time Z-Feedback Adjustment

Maintains the water bridge between probe and substrate by adjusting Z based on voltage readings.

```
ALGORITHM: ZFeedbackAdjust
==========================

Input:
  smu_handle          - SMU VISA session
  voltage_low_thresh  - Lower voltage threshold (V)
  voltage_high_thresh - Upper voltage threshold (V)
  delta_z             - Z adjustment step (um)
  prev_z              - Current Z position (um)

Output:
  new_z               - Adjusted Z position (um)

PROCEDURE:
  1. SMU_Write("OUTP ON")
  2. response <- SMU_Query("READ?")
  3. Parse voltage_V from response
  4. Log: "Voltage: {voltage_V} V"

  5. IF voltage_low_thresh < voltage_V < voltage_high_thresh:
     a. Log: "Voltage in range - Z aligned"
     b. RETURN prev_z                          // No change

  6. ELSE IF voltage_V < voltage_low_thresh:
     a. Log: "Voltage too low - probe too far, moving closer"
     b. RETURN prev_z - delta_z                // Move probe DOWN (closer)

  7. ELSE IF voltage_V > voltage_high_thresh:
     a. Log: "Voltage too high - probe too close, moving away"
     b. RETURN prev_z + delta_z                // Move probe UP (away)
```

**Physical interpretation:**
- Low voltage = probe is too far from substrate = move closer (decrease Z)
- High voltage = probe is too close = move away (increase Z)
- In-range = water bridge is stable = no adjustment needed

### 7.3 Liftoff and Re-engage

Handles probe liftoff between non-contiguous pattern segments.

```
ALGORITHM: LiftoffReengage
==========================

Input:
  X[], Y[], Flag[]    - Coordinate arrays from pattern file
  liftoff_height      - Height to raise probe (um)
  z_contact_point     - Last known contact Z position

State:
  liftoff_active <- FALSE
  prev_flag <- 0
  prev_z <- z_contact_point

FOR i = 0 TO N-1:
  flag <- Flag[i]
  x <- X[i], y <- Y[i]

  CASE (prev_flag, flag):

    (0 -> 1):  TRANSITION: Contact -> Liftoff
      z <- prev_z - liftoff_height              // Raise probe
      liftoff_active <- TRUE
      Execute XY move at raised Z

    (1 -> 1):  CONTINUED: Still lifted
      z <- prev_z                               // Maintain height
      Execute XY move at raised Z

    (1 -> 0):  TRANSITION: Liftoff -> Re-engage
      z <- FindContactPoint()                   // Re-probe for contact
      liftoff_active <- FALSE

    (0 -> 0):  CONTINUOUS: Normal patterning
      z <- ZFeedbackAdjust()                    // Maintain water bridge
      Execute XY move with adjusted Z

  prev_x <- x
  prev_y <- y
  prev_z <- z
  prev_flag <- flag
```

### 7.4 Velocity Decomposition

Computes XY velocity components so the probe maintains constant speed regardless of direction.

```
ALGORITHM: ComputeVelocity
==========================

Input:
  dx, dy              - Relative movement in microns
  speed               - Desired speed in um/s

Output:
  vx, vy              - Velocity in steps/sec for each axis

PROCEDURE:
  1. theta <- atan2(dx, dy)               // Angle from Y-axis
  2. sin_theta <- sin(theta)
  3. cos_theta <- cos(theta)
  4. vx_microns <- speed * sin_theta      // X velocity in um/s
  5. vy_microns <- speed * cos_theta      // Y velocity in um/s
  6. vx_steps <- vx_microns / 0.2         // Convert to steps/sec
  7. vy_steps <- vy_microns / 0.2         // Convert to steps/sec
  8. RETURN vx_steps, vy_steps

NOTE: atan2(dx, dy) uses Y-axis reference (non-standard but consistent
      with original Python code). In LabVIEW use "Inverse Tangent (2arg)".
```

### 7.5 Position Tracking

After each move, read back actual steps from the controller to maintain accurate position.

```
ALGORITHM: UpdatePosition
=========================

After each Motor_Move(dx, dy, dz):

  // 1. Read actual steps from controller
  actual_x, actual_y, actual_z <- Motor_ReadPosition()

  // 2. Convert to microns
  actual_um_x <- actual_x * 0.2
  actual_um_y <- actual_y * 0.2
  actual_um_z <- actual_z * 0.2

  // 3. Update position based on direction and limits
  IF x_home_limit_reached:
    x_position <- 0
  ELSE IF dx > 0:
    x_position <- x_position + actual_um_x
  ELSE:
    x_position <- x_position - actual_um_x

  // Repeat for Y and Z axes
```

---

## 8. Parallel Loop Architecture

LabVIEW's native parallelism replaces Python's threading model. The architecture uses Producer/Consumer loops for decoupled operation.

```
+--------------------------------------------------------------+
|                    MAIN APPLICATION                           |
|                                                               |
|  +--------------------------------------------------------+ |
|  |              UI LOOP (Producer)                         | |
|  |                                                         | |
|  |  - Read parameter inputs from front panel               | |
|  |  - Start/Stop button handling                           | |
|  |  - Speed override slider                                | |
|  |  - Step size override                                   | |
|  |                                                         | |
|  |  Output: Command Queue ----------------------------+    | |
|  +--------------------------------------------------|----+ |
|                                                     |       |
|  +--------------------------------------------------|----+ |
|  |              MOTOR CONTROL LOOP (Consumer)       |<---+ |
|  |                                                   |       |
|  |  Dequeue command:                                 |       |
|  |    SET_SPEED  -> Motor_SetSpeed.vi                |       |
|  |    MOVE_XYZ   -> Motor_Move.vi                    |       |
|  |    HOME       -> Motor_MoveHome.vi                |       |
|  |    STOP       -> Motor_Stop.vi                    |       |
|  |    READ_POS   -> Motor_ReadPosition.vi            |       |
|  |                                                   |       |
|  |  After move: Update position display on UI        |       |
|  |  On limit:   Trigger error handler                |       |
|  +---------------------------------------------------+       |
|                                                               |
|  +--------------------------------------------------------+ |
|  |              SMU MEASUREMENT LOOP                       | |
|  |                                                         | |
|  |  Triggered by pattern trace state:                      | |
|  |    READ_VOLTAGE -> SMU_ReadVoltage.vi                   | |
|  |    READ_CURRENT -> SMU_ReadCurrent.vi                   | |
|  |                                                         | |
|  |  Output: Measurement Queue ------------------------+    | |
|  +--------------------------------------------------|----+ |
|                                                     |       |
|  +--------------------------------------------------|----+ |
|  |         Z FEEDBACK LOOP (Consumer)              |<---+ |
|  |                                                   |       |
|  |  Dequeue measurement:                             |       |
|  |    voltage_V <- measurement.voltage               |       |
|  |    IF V < V_low:  dz = -delta_z  (closer)        |       |
|  |    IF V > V_high: dz = +delta_z  (away)          |       |
|  |    IF in range:   dz = 0                          |       |
|  |                                                   |       |
|  |  Output: Z adjustment to Motor Control Loop       |       |
|  +---------------------------------------------------+       |
+--------------------------------------------------------------+
```

**Key advantage over Python:** All three loops run in parallel with deterministic timing. The Z feedback loop does not block motor commands or UI updates.

---

## 9. Complete Mode 6 Sequence

```
================================================================
MODE 6: FULL LITHOGRAPHY RUN - COMPLETE SEQUENCE
================================================================

BEGIN
  |
  +-- READ UI PARAMETERS
  |   curr_ua, volt_compliance, volt_source, curr_comp,
  |   volt_low, volt_high, delta_z, z_contact_step,
  |   xy_speed, liftoff_height, filename
  |
  +-- INITIALIZE HARDWARE
  |   SMU_Init()
  |   SMU_Reset()
  |   SMU_SourceCurrent(curr_ua, volt_compliance)   <- Use Case 1
  |
  +-- LOAD PATTERN FILE
  |   Read "{filename}.txt"
  |   Parse into arrays: X[], Y[], Flag[]
  |   first_x = X[0], first_y = Y[0]
  |
  +-- MOVE TO START POSITION
  |   dx = first_x - current_x
  |   dy = first_y - current_y
  |   Motor_SetSpeed(5000, 5000, 5000)
  |   Motor_Move(dx, dy, 0)
  |   Wait for completion
  |
  +-- FIND INITIAL CONTACT POINT
  |   z_contact = ZProbeRoutine(z_contact_step, curr_ua)
  |   current_z = z_contact
  |   SMU_Reset()
  |
  +-- SWITCH TO PATTERNING MODE
  |   SMU_SourceVoltage(volt_source, curr_comp)     <- Use Case 2
  |
  +-- TRACE PATTERN
  |   prev_x = first_x, prev_y = first_y
  |   prev_z = z_contact
  |   prev_flag = 0
  |   liftoff_active = FALSE
  |
  |   FOR i = 1 TO N-1:
  |     x = X[i], y = Y[i], flag = Flag[i]
  |     dx = x - prev_x
  |     dy = y - prev_y
  |
  |     IF flag == 1 AND prev_flag == 0:
  |       // LIFTOFF
  |       z = prev_z - liftoff_height
  |       liftoff_active = TRUE
  |
  |     ELIF flag == 1 AND prev_flag == 1:
  |       // STAY LIFTED
  |       z = prev_z
  |
  |     ELIF flag == 0 AND liftoff_active:
  |       // RE-ENGAGE
  |       z = ZProbeRoutine(z_contact_step, 0.2)
  |       liftoff_active = FALSE
  |
  |     ELIF flag == 0 AND NOT liftoff_active:
  |       // FEEDBACK Z ADJUST
  |       direction = SMU_ReadVoltage(volt_low, volt_high)
  |       IF direction == 2: z = prev_z - delta_z
  |       ELIF direction == 3: z = prev_z + delta_z
  |       ELSE: z = prev_z
  |
  |     dz = z - prev_z
  |
  |     // VELOCITY DECOMPOSITION
  |     theta = atan2(dx, dy)
  |     vx = xy_speed * sin(theta) / 0.2
  |     vy = xy_speed * cos(theta) / 0.2
  |     Motor_SetSpeed(vx, vy, 1000)
  |
  |     // EXECUTE MOVE
  |     Motor_Move(dx, dy, dz)
  |     Wait for completion
  |
  |     // UPDATE STATE
  |     prev_x = x, prev_y = y, prev_z = z
  |     prev_flag = flag
  |
  +-- CLEANUP
  |   SMU_OutputOff()
  |   Motor_MoveHome()
  |
  +-- DISPLAY "DONE"
END
```

---

## 10. Error Handling Strategy

```
ERROR CLASSIFICATION AND RESPONSE:
===================================

+---------------------+----------------------------------------------+
| Error Source         | Recovery Action                              |
+---------------------+----------------------------------------------+
| Serial timeout       | Retry up to 3 times, then abort             |
| Invalid ACK byte     | Flush input buffer, retry command           |
| Limit switch hit     | Stop motor, alert user, home axis           |
| SMU read failure     | Retry read, skip point if persistent        |
| SMU compliance trip  | Turn off output, abort pattern              |
| Motor not responding | Reset serial port, retry                    |
| File not found       | Show dialog, prompt for new file            |
| User abort (Stop)    | Stop motor -> SMU off -> Home -> Done       |
| Communication lost   | Re-init hardware, resume or abort           |
+---------------------+----------------------------------------------+

ALL ERRORS:
  1. Log to file with timestamp, error type, and context
  2. Display on front panel (red status indicator + message)
  3. Attempt automatic recovery if possible
  4. If unrecoverable: SMU off -> Motor stop -> Home all axes -> Done
```

---

## 11. Configuration Constants

All system parameters centralized in `SystemConfig.vi`:

```
MOTOR CONTROLLER CONSTANTS:
===========================

Communication:
  BAUD_RATE            = 19200
  PARITY               = None (0)
  STOP_BITS            = 1
  DATA_BITS            = 8
  READ_TIMEOUT_MS      = 100
  WRITE_TIMEOUT_MS     = 100

Motion:
  RESOLUTION           = 0.2       (um per step)
  MAX_STEPS            = 16777216  (per axis)
  MAX_TRAVEL_UM        = 50000     (50 mm per axis)
  SPEED_MIN            = 10        (steps/sec)
  SPEED_MAX            = 50000     (steps/sec)

Protocol Bytes:
  DEVICE_ID            = 100  (0x64)
  DEVICE_ACK           = 200  (0xC8)
  OK_ACK               = 10   (0x0A)
  MOVE_COMPLETED       = 170  (0xAA)
  STOP_CMD             = 104  (0x68)
  STOP_ACK             = 105  (0x69)
  READ_CMD             = 163  (0xA3)
  READ_ACK             = 164  (0xA4)
  SPEED_CMD            = 34   (0x22)
  MOVE_CMD             = 19   (0x13)
  DIR_PLUS             = 125  (0x7D)
  DIR_MINUS            = 175  (0xAF)

Limit Switch Codes:
  X_HOME_LIMIT         = 41   (0x29)
  X_FAR_LIMIT          = 40   (0x28)
  Y_HOME_LIMIT         = 43   (0x2B)
  Y_FAR_LIMIT          = 42   (0x2A)
  Z_HOME_LIMIT         = 45   (0x2D)
  Z_FAR_LIMIT          = 44   (0x2C)

SMU CONSTANTS:
==============

  VISA_ADDRESS         = "USB0::0x2184::0x007D::gey853397::INSTR"
  TIMEOUT_MS           = 5000
  WRITE_TERMINATION    = "\n"
  READ_TERMINATION     = "\n"

DEFAULT OPERATION VALUES:
=========================

  XY_SPEED             = 5000    (um/s)
  Z_SPEED              = 1000    (um/s)
  HOMING_SPEED         = 5000    (um/s)
  INITIAL_Z_STEP       = 1000    (um)
  DELTA_Z              = 1.0     (um)
  LIFTOFF_HEIGHT       = 10000   (um)
```

---

## 12. Coordinate File Format

Pattern files are plain text with one coordinate per line:

```
Format: X, Y, Flag
Where:
  X     = X position in microns (absolute)
  Y     = Y position in microns (absolute)
  Flag  = 0 (probe touching/engaged) or 1 (probe lifted/retracted)

Example (two parallel lines with liftoff between them):
  10000, 0, 0
  10100, 0, 0
  10200, 0, 0
  ...
  14000, 0, 0      <- end of first line
  14000, 0, 1      <- liftoff
  14000, 4000, 1   <- still lifted, moving to start of second line
  14000, 4000, 0   <- re-engage (contact will be re-detected)
  13900, 4000, 0
  ...
```

---

## 13. Coordinate Pattern Generator Utilities

The following pattern generation functions (from the original Python `test.py`) should be recreated as LabVIEW SubVIs:

| Function | Purpose | Output Format |
|----------|---------|---------------|
| `GenerateParallelLines` | Create N parallel lines with specified spacing | X, Y, Flag per line |
| `GenerateGridLines` | Horizontal + vertical grid pattern | X, Y per point |
| `GenerateCircleCoordinates` | Circle with specified radius and center | Absolute or relative X, Y |
| `ShiftCoordinates` | Add buffer offset to existing coordinates | Modified X, Y, Flag |
| `GenerateSnakePattern` | E-shaped serpentine pattern | X, Y per point |

---

## 14. Bug Fixes from Python Codebase

These bugs were identified in the original Python code and must NOT be carried over to the LabVIEW implementation:

| Bug | Location | Fix for LabVIEW |
|-----|----------|-----------------|
| Dual position tracking | `hmccontroller.py:115-136` vs `602-611` | Use single position variable (hardware-reported) |
| Z-axis double negation | `hmccontroller.py:48` + `hmccontroller.py:607` | Consistent Z sign convention throughout |
| Module-level side effects | `hmccontroller.py:786-790` | No global objects; init in startup state only |
| `contact_point_move` missing read | `hmccontroller.py:711-778` | Always read back actual steps after move |
| Dead `else` block | `main.py:459-477` | Remove unreachable code |
| Inconsistent `read_move_bytes` | `hmccontroller.py:506-518` | Uniform treatment for all axes |
| Feedback direction comments | `main.py:447-453` | Ensure comments match actual behavior |
| Missing delay serial close/open | `main.py:240-241` | Always include 100ms delay between close/open |

---

## 15. Implementation Priority

Recommended order of implementation:

| Phase | Component | Effort | Dependencies |
|-------|-----------|--------|--------------|
| 1 | SystemConfig constants | Low | None |
| 2 | Motor Controller SubVIs | Medium | Phase 1 |
| 3 | SMU Controller SubVIs | Medium | Phase 1 |
| 4 | ManualMove (Mode 1) | Low | Phases 2, 3 |
| 5 | HomeAll (Mode 2) | Low | Phase 2 |
| 6 | LoadCoordinateFile | Low | None |
| 7 | OpenLoopPattern (Mode 3) | Medium | Phases 2, 6 |
| 8 | ZProbe (Mode 4) | Medium | Phases 2, 3 |
| 9 | LithographyRun (Mode 6) | High | Phases 2, 3, 6, 8 |
| 10 | Pattern generator SubVIs | Low | None |
| 11 | Error handling + logging | Medium | All above |
| 12 | UI polish + PatternViewer | Low | All above |

---

## 16. Technologies Used

- **LabVIEW** (full native rewrite, no Python Nodes)
- **NI-VISA** for serial and USB instrument communication
- **Holmarc Motor Controllers** (XYZ stepper, 0.2 um resolution)
- **Keithley SMU** (Source Measurement Unit, SCPI over USB VISA)
- **Micropositioners** (XYZ stage)
- (Planned) **Nanopositioner** for sub-micron accuracy
- (Planned) **Microscope Camera Integration** for live monitoring

---

## Learn More

To understand the principles behind this system and the underlying electro-lithography technique, refer to the research by [Dr. Santanu Talukder](https://sites.google.com/site/santanutalukderiiscnanoscience/research?authuser=0#h.gpgsm7u1n3ba).
