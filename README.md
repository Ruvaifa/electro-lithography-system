# Electro-Lithography System

This repository contains code for a **custom-built Electro-Lithography system** that enables micron-level precision probe movement with real time Z axis feedback. The system is designed to facilitate patterning on surfaces at micro and eventually nano scales, by detecting probe and sample contact through electrical current feedback.

> 🧪 Still under active development, features such as nanometer scale positioning, live imaging, and improved path optimization are planned.

---
## 🧬 What Is Electro‑Lithography?
Electro lithography is a scanning probe, mask less lithography technique that uses electrical stimuli to directly write patterns on surfaces at micro to nano length scales.

**How it Works?**
- A conductive probe tip (like those in Scanning Probe Microscopes) is positioned above or in contact with a thin metallic film or substrate
- A controlled electric field or current is applied through the tip, causing localized material transport, either by electromigration or electrochemical reactions at the probe sample interface .
-  This enables direct writing of patterns (e.g., lines, dots, shapes) without requiring photomasks, often under ambient conditions and with resolutions below 10 nm

## 🔬 Project Overview.

Electro lithography is a cutting edge nanofabrication technique where a conductive probe is brought into contact with a surface to define patterns. The system uses precise motor controls and current-based contact detection to create lithographic patterns.

This project aims to provide:

- Automated probe movement in X, Y, and Z directions
- Real-time Z-axis feedback using current sensing from an SMU (Source Measurement Unit)
- Execution of custom lithographic patterns based on coordinate files
- Scalability toward nanometer precision and visual monitoring

---

## 📁 Repository Structure

- `hmccontroller.py` – Handles communication and control of the X, Y, Z micro-positioning motors.
- `main.py` – Used for executing lithography runs based on coordinate input and voltage/current feedback.
- `coordinates.txt` – Input file with X Y Z movement coordinates and 0/1 flags for liftoff state(for making open loop patterns).
- `README.md` – You’re reading it!

---

## ⚙️ How It Works

1. **Coordinate Input**  
   A `.txt` file provides a sequence of (X, Y) positions and optional liftoff commands (0/1) for the probe, where 0 indicates **touch** and 1 indicates **no touch**, this tells the code when to lift the probe up and down. It is used when we have to make open loop patterns (when we have to make pattern at two places which are not join, Example- 5 parallel lines)

2. **Motor Movement**  
   Motors are controlled through a serial interface using the `hmccontroller.py` module to move in precise 0.2 μm steps (configurable).

3. **Z axis Feedback Loop**  
   The system continuously reads current values from an SMU. A spike in current indicates probe contact with the substrate, which triggers fine Z adjustments. The water bridge formed between the probe and sample is very delicate, to maintain the water bridge we have to keep adjusting Z coordinate on every step. The formation of water bridge is detected by reading voltage values through the SMU. To read more about the process read the research paper by Dr. Santanu Talukder linked at the end.

4. **Pattern Execution**  
   A pattern is traced as the probe moves based on the input coordinates while managing liftoff and re engagement at each step.

---

## 🛠️ Technologies Used

- **Python**
- **Serial Communication (pySerial)**
- **Holmarc Motor Controllers**
- **Source Measurement Unit (SMU)**
- **Micropositioners (XYZ stage)**
- (Planned) **Nanopositioner**
- (Planned) **Microscope Camera Integration**

---
## Report
   [![View Report](https://img.shields.io/badge/Open-PDF-red)](Report.pdf)

## 📈 Future Work

- [ ] Integrate high resolution nanopositioner for sub micron accuracy
- [ ] Camera module with lens for live monitoring
- [ ] Improve real time feedback system and pattern resolution
- [ ] GUI interface for easy pattern design and control
- [ ] System safety and emergency stop mechanisms

---

## 📚 Learn More

To understand the principles behind this system and the underlying electro-lithography technique, refer to the research by [Dr. Santanu Talukder](https://sites.google.com/site/santanutalukderiiscnanoscience/research?authuser=0#h.gpgsm7u1n3ba).

---

## LabVIEW Implementation Architecture

### 1. System Architecture

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

---

### 2. LabVIEW Project Structure

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

### 3. Detailed Code Flow

#### 3.1 Startup Flow

```
START
  |
  +-> List COM Ports -> Display in dropdown
  |
  +-> User selects port
  |
  +-> Motor_Init.vi
  |     +-> VISA Open (19200, 8N1, timeout 100ms)
  |     +-> VISA Write: [0x64] (device ID request = 100)
  |     +-> VISA Read: wait for byte
  |     +-> Check byte == 0xC8 (device ACK = 200)?
  |          +-> YES -> proceed
  |          +-> NO  -> error dialog, retry/abort
  |     +-> Set com_open = TRUE
  |
  +-> Motor_MoveHome.vi (on_startup)
  |     +-> SetSpeed(5000, 5000, 5000)
  |     +-> Z Home: Move(0, 0, 16777216) -> wait for z_home_limit (45)
  |     +-> Close/Open serial (reset)
  |     +-> X Home: Move(-16777216, 0, 0) -> wait for x_home_limit (41)
  |     +-> Close/Open serial (reset)
  |     +-> Y Home: Move(0, -16777216, 0) -> wait for y_home_limit (43)
  |     +-> Close/Open serial (reset)
  |     +-> Initialize all positions to 0
  |
  +-> Show Main Panel -> User selects mode
```

#### 3.2 Serial Protocol (Motor Controller)

Every motor command follows this byte-level protocol:

```
MOTOR COMMAND PACKET STRUCTURE:
===============================

1. SET SPEED (Command byte: 0x22 = 34)
   TX: [0x22] [MSB_X] [CSB_X] [LSB_X] [MSB_Y] [CSB_Y] [LSB_Y] [MSB_Z] [CSB_Z] [LSB_Z]
   RX: ACK(0x0A) after each byte
   Speed = input_microns_per_sec / 0.2 (convert to steps/sec)

2. MOVE (Command byte: 0x13 = 19)
   TX: [0x13]
   RX: ACK(0x0A)
   TX: [Move_axis_A_cmd] -> [B3] [B2] [B1] [B0] [Direction]
   RX: ACK after each byte
   TX: [Move_axis_B_cmd] -> [B3] [B2] [B1] [B0] [Direction]
   RX: ACK after each byte
   TX: [Move_axis_C_cmd] -> [B3] [B2] [B1] [B0] [Direction]
   RX: ACK after each byte

   Steps = distance_microns / 0.2
   Direction: 0x7D (+) or 0xAF (-)

3. READ STATUS (Command byte: 0xA3 = 163)
   TX: [0xA3]
   RX: [0xA4] (READ_ACK)
   TX: [0x0A] -> RX: [status_byte]

   Status bytes:
     0xAA (170) = move completed
     0x29 (41)  = X home limit
     0x28 (40)  = X far limit
     0x2B (43)  = Y home limit
     0x2A (42)  = Y far limit
     0x2D (45)  = Z home limit
     0x2C (44)  = Z far limit

4. READ ACTUAL STEPS (Command byte: 0xA3 = 163)
   TX: [0xA3] -> RX: [0xA4]
   For each axis (X, Y, Z):
     TX: [0x0A] -> RX: [byte0]
     TX: [0x0A] -> RX: [byte1]
     TX: [0x0A] -> RX: [byte2]
     TX: [0x0A] -> RX: [byte3]
   Steps = (byte3 * 16777216) + (byte2 * 65536) + (byte1 * 256) + byte0

5. STOP (Command byte: 0x68 = 104)
   TX: [0x68]
   RX: [0x69] (stop ACK)
```

#### 3.3 SMU Communication (SCPI over VISA)

```
SMU COMMAND STRUCTURE:
======================

INIT:
  VISA Open (USB0::0x2184::0x007D::gey853397::INSTR)
  Write: *CLS
  Write: *RST
  Wait: 100ms

USE CASE 1 (Current Source, Voltage Compliance - for contact detection):
  Write: SOUR:FUNC CURR
  Write: SOUR:CURR {current_A}          <- e.g., 0.2e-6 for 0.2uA
  Write: SENS:VOLT:PROT {compliance_V}  <- e.g., 1.0V

USE CASE 2 (Voltage Source, Current Compliance - for patterning):
  Write: SOUR:FUNC VOLT
  Write: SOUR:VOLT {voltage_V}          <- e.g., 0.5V
  Write: SENS:CURR:PROT {compliance_A}  <- e.g., 1000e-6 for 1mA

READ:
  Write: OUTP ON
  Query: READ?
  Response: "{voltage},{current}" (CSV string)
  Parse: voltage = float(parts[0]), current = float(parts[1])

OUTPUT OFF:
  Write: OUTP OFF
```

---

### 4. State Machine Architecture (Mode 6 - Full Lithography)

```
+----------------------------------------------------------------------+
|                    LITHOGRAPHY RUN STATE MACHINE                      |
|                                                                      |
|  +----------+                                                        |
|  | S_INIT   | <- Read parameters from UI                              |
|  +----+-----+   (voltage, current, thresholds, delta_z,              |
|       |          liftoff_height, filename, speed)                     |
|       v                                                               |
|  +----------+                                                        |
|  | S_SMU_   | <- SMU_Init -> SMU_Reset -> SMU_SourceCurrent           |
|  | CONFIG   |   (Use Case 1: current source for contact detection)    |
|  +----+-----+                                                        |
|       v                                                               |
|  +----------+                                                        |
|  | S_LOAD_  | <- LoadCoordinateFile.vi                               |
|  | PATTERN  |   Parse first line -> get first_x, first_y              |
|  +----+-----+   Store all (X, Y, flag) arrays in shift register      |
|       v                                                               |
|  +----------+                                                        |
|  | S_MOVE_  | <- dx = first_x - current_x                            |
|  | TO_START |   dy = first_y - current_y                             |
|  |          |   SetSpeed(5000, 5000, 5000)                           |
|  |          |   Motor_Move(dx, dy, 0)                                |
|  |          |   Wait for completion                                  |
|  +----+-----+                                                        |
|       v                                                               |
|  +----------+                                                        |
|  | S_Z_     | <- Z_Probe subroutine:                                 |
|  | PROBE    |   1. Move Z down by z_contact_step                     |
|  |          |   2. Read current via SMU                               |
|  |          |   3. If current > threshold -> CONTACT FOUND            |
|  |          |   4. Else -> repeat from step 1                        |
|  |          |   5. Store z_contact_point                             |
|  +----+-----+                                                        |
|       v                                                               |
|  +----------+                                                        |
|  | S_SWITCH | <- SMU_SourceVoltage (Use Case 2)                       |
|  | TO_      |   Now reading voltage for Z feedback                   |
|  | PATTERN  |                                                         |
|  +----+-----+                                                        |
|       v                                                               |
|  +----------+   +-----------------------------------------------+    |
|  | S_TRACE  |--->| For each coordinate (X[i], Y[i], Flag[i]):   |    |
|  | PATTERN  |   |                                                |    |
|  |          |<---|  IF i == 0:                                   |    |
|  |          |   |    Skip (already at start position)           |    |
|  |          |   |                                                |    |
|  |          |   |  IF Flag[i] == 1 AND prev_flag == 0:         |    |
|  |          |   |    -> LIFTOFF:                                 |    |
|  |          |   |      z = prev_z - liftoff_height              |    |
|  |          |   |      dz = z - prev_z                          |    |
|  |          |   |      liftoff_active = TRUE                    |    |
|  |          |   |                                                |    |
|  |          |   |  IF Flag[i] == 1 AND prev_flag == 1:         |    |
|  |          |   |    -> STAY LIFTED:                             |    |
|  |          |   |      z = prev_z (no Z change)                 |    |
|  |          |   |                                                |    |
|  |          |   |  IF Flag[i] == 0 AND liftoff_active:         |    |
|  |          |   |    -> RE-ENGAGE:                               |    |
|  |          |   |      Call Z_Probe subroutine                  |    |
|  |          |   |      z = new z_contact_point                  |    |
|  |          |   |      liftoff_active = FALSE                   |    |
|  |          |   |                                                |    |
|  |          |   |  IF Flag[i] == 0 AND NOT liftoff:            |    |
|  |          |   |    -> FEEDBACK Z ADJUST:                       |    |
|  |          |   |      Read voltage via SMU                     |    |
|  |          |   |      IF V < V_thresh_low:                     |    |
|  |          |   |        z = prev_z - delta_z  (move closer)    |    |
|  |          |   |      ELSE IF V > V_thresh_high:               |    |
|  |          |   |        z = prev_z + delta_z  (move away)      |    |
|  |          |   |      ELSE:                                     |    |
|  |          |   |        z = prev_z (no change)                 |    |
|  |          |   |                                                |    |
|  |          |   |  COMPUTE:                                      |    |
|  |          |   |    dx = X[i] - prev_x                         |    |
|  |          |   |    dy = Y[i] - prev_y                         |    |
|  |          |   |    dz = z - prev_z                            |    |
|  |          |   |    theta = atan2(dx, dy)                       |    |
|  |          |   |    vx = speed * sin(theta)                    |    |
|  |          |   |    vy = speed * cos(theta)                    |    |
|  |          |   |                                                |    |
|  |          |   |  EXECUTE:                                      |    |
|  |          |   |    SetSpeed(vx, vy, 1000)                     |    |
|  |          |   |    Motor_Move(dx, dy, dz)                     |    |
|  |          |   |    Wait for completion                        |    |
|  |          |   |                                                |    |
|  |          |   |  UPDATE:                                       |    |
|  |          |   |    prev_x = X[i], prev_y = Y[i], prev_z = z  |    |
|  |          |   |    prev_flag = Flag[i]                        |    |
|  |          |   |    i = i + 1                                  |    |
|  +----+-----+   +-----------------------------------------------+    |
|       v (all coordinates done)                                        |
|  +----------+                                                        |
|  | S_SMU_   | <- SMU_OutputOff                                      |
|  | OFF      |                                                        |
|  +----+-----+                                                        |
|       v                                                               |
|  +----------+                                                        |
|  | S_HOME   | <- Motor_MoveHome (all axes)                           |
|  +----+-----+                                                        |
|       v                                                               |
|  +----------+                                                        |
|  | S_DONE   | <- Display final position, cleanup                     |
|  +----------+                                                        |
+----------------------------------------------------------------------+
```

---

### 5. Algorithms

#### 5.1 Z-Probe Contact Detection

```
ALGORITHM: Find Contact Point
=============================

Input:  smu_handle, lower_current_threshold (uA), initial_step_size (uM)
Output: z_contact_point (uM from home)

PROCEDURE:
  1. total_distance <- 0
  2. step_size <- initial_step_size
  3. REPEAT:
     a. Send Motor_Move(0, 0, step_size)       // Move Z down
     b. Wait for motor completion
     c. total_distance <- total_distance + step_size
     d. current_A <- SMU_Query("READ?")         // Read current
     e. Parse current from response
     f. IF current_A > lower_current_threshold * 1e-6:
        i.  PRINT "Contact found at {total_distance} uM"
        ii. RETURN total_distance
     g. Sleep(100ms)
  4. UNTIL contact found (or timeout/max distance)
```

#### 5.2 Real-Time Z-Feedback

```
ALGORITHM: Z Feedback Adjustment
================================

Input:  smu_handle, voltage_low_thresh, voltage_high_thresh, delta_z
Output: new_z_position

PROCEDURE:
  1. voltage_V <- SMU_Query("READ?")             // Read voltage
  2. Parse voltage from response
  3. IF voltage_low_thresh < voltage_V < voltage_high_thresh:
     a. PRINT "Voltage in range - Z aligned"
     b. RETURN prev_z                            // No change needed
  4. ELSE IF voltage_V < voltage_low_thresh:
     a. PRINT "Voltage too low - probe too far from surface"
     b. RETURN prev_z - delta_z                  // Move probe CLOSER (down)
  5. ELSE IF voltage_V > voltage_high_thresh:
     a. PRINT "Voltage too high - probe too close"
     b. RETURN prev_z + delta_z                  // Move probe AWAY (up)

NOTE: "Down" means negative Z (toward substrate).
      The water bridge voltage increases as probe gets closer.
      Low voltage = probe too far. High voltage = probe too close.
```

#### 5.3 Liftoff / Re-engage

```
ALGORITHM: Liftoff and Re-engage
================================

Input:  coordinate_array[], flag_array[]
Output: pattern executed with probe management

STATE: liftoff_active <- FALSE
       prev_flag <- 0

FOR i = 0 TO N-1:
  flag <- flag_array[i]
  CASE (prev_flag, flag):
    (0, 1):   // Transition: contact -> liftoff
      z <- prev_z - liftoff_height               // Raise probe
      liftoff_active <- TRUE
      Execute XY move at raised Z
    (1, 1):   // Still lifted
      z <- prev_z                                // Maintain height
      Execute XY move at raised Z
    (1, 0):   // Transition: liftoff -> contact
      z <- FindContactPoint()                    // Re-probe
      liftoff_active <- FALSE
    (0, 0):   // Continuous contact
      z <- ZFeedbackAdjust()                     // Maintain water bridge
      Execute XY move with adjusted Z
  prev_flag <- flag
  prev_z <- z
```

#### 5.4 Velocity Decomposition

```
ALGORITHM: Compute XY Velocity Components
=========================================

Input:  dx, dy (relative movement in microns), speed (uM/s)
Output: vx, vy (velocity components in steps/sec)

PROCEDURE:
  1. theta <- atan2(dx, dy)                    // Angle from Y-axis
  2. sin_theta <- sin(theta)
  3. cos_theta <- cos(theta)
  4. vx_microns <- speed * sin_theta           // X velocity in uM/s
  5. vy_microns <- speed * cos_theta           // Y velocity in uM/s
  6. vx_steps <- vx_microns / 0.2              // Convert to steps/sec
  7. vy_steps <- vy_microns / 0.2              // Convert to steps/sec
  8. RETURN vx_steps, vy_steps

NOTE: atan2(dx, dy) uses Y-axis reference (non-standard).
      This is consistent with the original Python code.
      In LabVIEW, use "Inverse Tangent (2arg)" with (dx, dy).
```

#### 5.5 Position Tracking

```
ALGORITHM: Dual Position Update
===============================

After each Motor_Move(dx, dy, dz):

  // 1. Read actual steps from controller
  actual_steps_x, actual_steps_y, actual_steps_z <- Motor_ReadPosition()

  // 2. Convert to microns
  actual_um_x <- actual_steps_x * 0.2
  actual_um_y <- actual_steps_y * 0.2
  actual_um_z <- actual_steps_z * 0.2

  // 3. Update hardware-reported position
  IF limit_x_home_reached:
    x_hw_position <- 0
  ELSE:
    IF dx > 0: x_hw_position <- x_hw_position + actual_um_x
    ELSE:      x_hw_position <- x_hw_position - actual_um_x

  // (Same for Y and Z)

  // 4. Update commanded position (for user display)
  x_cmd_position <- x_cmd_position + dx
  y_cmd_position <- y_cmd_position + dy
  z_cmd_position <- z_cmd_position + (-dz)  // Z sign inversion

  // 5. USE ONLY x_hw_position/y_hw_position/z_hw_position
  //    for all subsequent calculations (fix Python bug)
```

---

### 6. Parallel Loop Architecture

```
+--------------------------------------------------------------+
|                    MAIN APPLICATION                           |
|                                                               |
|  +--------------------------------------------------------+ |
|  |              UI LOOP (Producer)                         | |
|  |                                                         | |
|  |  - Read parameter inputs                                | |
|  |  - Start/Stop button                                    | |
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

---

### 7. Complete Mode 6 Sequence (Pseudocode)

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

### 8. Error Handling Strategy

```
ERROR HANDLER VI:
=================

+---------------------+----------------------------------------------+
| Error Source         | Recovery Action                              |
+---------------------+----------------------------------------------+
| Serial timeout       | Retry 3x, then abort                        |
| Invalid ACK          | Flush buffer, retry                         |
| Limit switch hit     | Stop, alert, home                           |
| SMU read failure     | Retry, skip point                           |
| SMU compliance trip  | Stop output, abort                          |
| Motor not responding | Reset serial, retry                         |
| File not found       | Dialog, re-enter file                       |
| User abort (Stop)    | Stop motor, SMU off, home all axes          |
+---------------------+----------------------------------------------+

ALL ERRORS:
  1. Log to file (timestamp, error type, context)
  2. Display on UI (red status indicator)
  3. Attempt recovery if possible
  4. If unrecoverable: SMU off -> Motor stop -> Home -> Done
```

---

### 9. Configuration Constants

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


