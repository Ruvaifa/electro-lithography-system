# Graph Report - .  (2026-06-30)

## Corpus Check
- 56 files · ~166,309 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 285 nodes · 422 edges · 37 communities (16 shown, 21 thin omitted)
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 29 edges (avg confidence: 0.82)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_HMC Stage Stepper Driver|HMC Stage Stepper Driver]]
- [[_COMMUNITY_Basler Microscope Camera Driver|Basler Microscope Camera Driver]]
- [[_COMMUNITY_Lithography Feedback & Z-Axis Mechanics|Lithography Feedback & Z-Axis Mechanics]]
- [[_COMMUNITY_LabVIEW Lithography Integration|LabVIEW Lithography Integration]]
- [[_COMMUNITY_Legacy Stage Diagnostic Scripts|Legacy Stage Diagnostic Scripts]]
- [[_COMMUNITY_CLI Mode Dispatcher & Speed Sync|CLI Mode Dispatcher & Speed Sync]]
- [[_COMMUNITY_Frontend API Integration Class|Frontend API Integration Class]]
- [[_COMMUNITY_Circle Distortion Calibration History|Circle Distortion Calibration History]]
- [[_COMMUNITY_Pattern Geometry Coordinates Generator|Pattern Geometry Coordinates Generator]]
- [[_COMMUNITY_SMU Keithley Driver (Version 2)|SMU Keithley Driver (Version 2)]]
- [[_COMMUNITY_Concentric Circles Pattern Analysis|Concentric Circles Pattern Analysis]]
- [[_COMMUNITY_Micrograph Resolution Stepping Smoothness|Micrograph Resolution Stepping Smoothness]]
- [[_COMMUNITY_Dual-Graph Context Policy Rules|Dual-Graph Context Policy Rules]]
- [[_COMMUNITY_Circle & Coordinate Trajectory Inputs|Circle & Coordinate Trajectory Inputs]]
- [[_COMMUNITY_Parallel Coordinates Spacing Patterns (Set 1)|Parallel Coordinates Spacing Patterns (Set 1)]]
- [[_COMMUNITY_Parallel Coordinates Spacing Patterns (Set 2)|Parallel Coordinates Spacing Patterns (Set 2)]]
- [[_COMMUNITY_Diagnostic Trajectory Outputs|Diagnostic Trajectory Outputs]]
- [[_COMMUNITY_LabVIEW Stage Position Tracking|LabVIEW Stage Position Tracking]]
- [[_COMMUNITY_Large Trajectory Patterns|Large Trajectory Patterns]]
- [[_COMMUNITY_Square Trajectory Pattern|Square Trajectory Pattern]]
- [[_COMMUNITY_Test Coordinates Data|Test Coordinates Data]]
- [[_COMMUNITY_LabVIEW Manual Stage Movement|LabVIEW Manual Stage Movement]]
- [[_COMMUNITY_LabVIEW Motor Limit Checks|LabVIEW Motor Limit Checks]]
- [[_COMMUNITY_LabVIEW Motor Shut Down|LabVIEW Motor Shut Down]]
- [[_COMMUNITY_LabVIEW Motor Initialization|LabVIEW Motor Initialization]]
- [[_COMMUNITY_LabVIEW Motor Homing sequence|LabVIEW Motor Homing sequence]]
- [[_COMMUNITY_LabVIEW Motor Status Read|LabVIEW Motor Status Read]]
- [[_COMMUNITY_LabVIEW Motor Immediate Stop|LabVIEW Motor Immediate Stop]]
- [[_COMMUNITY_LabVIEW SMU Connection Close|LabVIEW SMU Connection Close]]
- [[_COMMUNITY_LabVIEW SMU Output Control|LabVIEW SMU Output Control]]
- [[_COMMUNITY_LabVIEW SMU Protocol Library|LabVIEW SMU Protocol Library]]
- [[_COMMUNITY_LabVIEW Stage Coordinate Validation|LabVIEW Stage Coordinate Validation]]
- [[_COMMUNITY_Parallel Line Trajectory Design|Parallel Line Trajectory Design]]

## God Nodes (most connected - your core abstractions)
1. `HmcControlCs` - 51 edges
2. `App` - 22 edges
3. `BaslerCamera` - 19 edges
4. `LithographyRun.vi` - 15 edges
5. `LithographySystem` - 12 edges
6. `compute_synchronized_speeds()` - 9 edges
7. `Fine-Grained Calibration` - 7 edges
8. `TestByteEncoding` - 6 edges
9. `TestSpeedCalculation` - 6 edges
10. `main()` - 5 edges

## Surprising Connections (you probably didn't know these)
- `Electro-Lithography Concept` --semantically_similar_to--> `Liquid Electromigration Principle`  [INFERRED] [semantically similar]
  README.md → Report.pdf
- `Python Main Script (main.py)` --semantically_similar_to--> `LithographyRun.vi`  [INFERRED] [semantically similar]
  README.md → Labview integration/README.md
- `Proportional Speed Scaling` --references--> `compute_synchronized_speeds()`  [EXTRACTED]
  diagnostic_and_fix_history.md → main.py
- `Dual-Graph Context Policy` --semantically_similar_to--> `Dual-Graph Context Policy Rules`  [INFERRED] [semantically similar]
  CLAUDE.md → .agent/rules/graperoot.md
- `Python Holmarc Motor Controller (hmccontroller.py)` --semantically_similar_to--> `MotorProtocol.lvlib`  [INFERRED] [semantically similar]
  README.md → Labview integration/README.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **LabVIEW Motor Control VISA VIs** — labview_integration_readme_motor_setspeed, labview_integration_readme_motor_move, labview_integration_readme_motor_readposition, labview_integration_readme_motor_readstatus, labview_integration_readme_motor_checklimits [EXTRACTED 1.00]
- **SMU SCPI Command Interface VIs** — labview_integration_readme_smu_init, labview_integration_readme_smu_reset, labview_integration_readme_smu_sourcecurrent, labview_integration_readme_smu_sourcevoltage, labview_integration_readme_smu_readvoltage, labview_integration_readme_smu_readcurrent [EXTRACTED 1.00]
- **Electro-Lithography Control Algorithms** — labview_integration_readme_z_probe_contact_detection, labview_integration_readme_real_time_z_feedback, labview_integration_readme_liftoff_reengage, labview_integration_readme_velocity_decomposition, labview_integration_readme_position_tracking [EXTRACTED 1.00]
- **Concentric Circles Patterning Flow** — hmccontroller_circles_plot_concentric_circles_pattern, hmccontroller_circles_plot_drawing_path, hmccontroller_circles_plot_travel_liftoff_path, hmccontroller_circles_plot_vertices [INFERRED 0.95]
- **Circle Lithography Quality Verification** — images_1_micrograph, images_1_successful_circular_trajectory, images_1_exposure_line_thickness_variation [INFERRED 0.85]
- **Calibration and Resolution Flow** — diagnostic_and_fix_history_firmware_speed_bands_bug, diagnostic_and_fix_history_fine_grained_calibration, diagnostic_and_fix_history_empirical_speed_blacklist, diagnostic_and_fix_history_proportional_speed_scaling [EXTRACTED 1.00]
- **Calibration Shape Test Patterns** — circles_data, hexagons_data, coords_square_data [INFERRED 0.85]

## Communities (37 total, 21 thin omitted)

### Community 0 - "HMC Stage Stepper Driver"
Cohesion: 0.12
Nodes (3): Exception, HmcControlCs, TestByteEncoding

### Community 1 - "Basler Microscope Camera Driver"
Cohesion: 0.10
Nodes (15): BaslerCamera, BaslerCameraSettings, list_basler_cameras(), main(), Basler camera live-view support using the pypylon SDK.  This module is intenti, Grab and return one frame as an OpenCV BGR image.          This is useful for, Display a blocking live video stream.          Press the exit key, default 'q', Start live view in a background thread.          This is the integration-frien (+7 more)

### Community 2 - "Lithography Feedback & Z-Axis Mechanics"
Cohesion: 0.09
Nodes (18): abort_patterning(), classify_voltage(), ensure_z_below_limit(), feedback_direction_label(), find_contact_point(), find_contact_point_custom(), PatternAbort, Raised when patterning needs to be aborted due to safety or user request. (+10 more)

### Community 3 - "LabVIEW Lithography Integration"
Cohesion: 0.08
Nodes (31): AngleCalculation.vi, Liftoff and Re-engage Algorithm, LithographyRun.vi, LoadCoordinateFile.vi, Logging.vi, Motor_Move.vi, Motor_SetSpeed.vi, MotorProtocol.lvlib (+23 more)

### Community 4 - "Legacy Stage Diagnostic Scripts"
Cohesion: 0.14
Nodes (11): list_ports(), main(), list_ports(), main(), test_speed_band(), list_ports(), main(), main() (+3 more)

### Community 5 - "CLI Mode Dispatcher & Speed Sync"
Cohesion: 0.11
Nodes (7): compute_synchronized_speeds(), format_elapsed_time(), list_ports(), main(), print_timing_summary(), Compute per-axis speeds so both X and Y finish each segment in exactly the same, TestSpeedCalculation

### Community 6 - "Frontend API Integration Class"
Cohesion: 0.13
Nodes (9): LithographySystem, Core api class for the electro-lithography system., Return available COM ports as list of dicts., Connect to all three axes. Returns success status., Home all axes in startup sequence., Move logical Z distance (relative move)., Get absolute current position of all axes., Stop all axis movements immediately. (+1 more)

### Community 7 - "Circle Distortion Calibration History"
Cohesion: 0.17
Nodes (16): Circles Coordinate Data, Project History: Holmarc Stage Calibration & Circle Distortion Resolution, Circle Distortion, Coordinated Motion Workarounds, Empirical Speed Blacklist, Fine-Grained Calibration, Firmware Speed Bands Bug, Parameter Safety in Parsers (+8 more)

### Community 8 - "Pattern Geometry Coordinates Generator"
Cohesion: 0.14
Nodes (6): generate_concentric_circles(), plot_concentric_circles(), plot_concentric_hexagons(), Optional visualization of the generated concentric hexagons path using matplotli, Generates coordinates for concentric circles of defined radii.     Points are c, Optional visualization of the generated concentric circles path using matplotlib

### Community 9 - "SMU Keithley Driver (Version 2)"
Cohesion: 0.25
Nodes (10): check_current(), check_smu_errors(), check_voltage(), init_smu(), read_current_sample(), _read_sample(), read_voltage_sample(), reset_smu() (+2 more)

### Community 10 - "Concentric Circles Pattern Analysis"
Cohesion: 0.60
Nodes (5): Concentric Circles Patterning Path & Liftoff Signals Chart, Concentric Circles Pattern, Drawing Path (flag=0), Travel/Liftoff Path (flag=1), Patterning Vertices

### Community 12 - "Micrograph Resolution Stepping Smoothness"
Cohesion: 0.67
Nodes (3): Concentric Ghost Circle Trace, Stepping Smoothness at 0.2 µm Resolution, Zoomed-in Circle Segment Microscope Image

## Knowledge Gaps
- **45 isolated node(s):** `Dual-Graph Context Policy Rules`, `Dual-Graph Context Policy`, `MotorProtocol.lvlib`, `SMUProtocol.lvlib`, `Motor_Init.vi` (+40 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **21 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `HmcControlCs` connect `HMC Stage Stepper Driver` to `Legacy Stage Diagnostic Scripts`, `CLI Mode Dispatcher & Speed Sync`, `Frontend API Integration Class`?**
  _High betweenness centrality (0.163) - this node is a cross-community bridge._
- **Why does `App` connect `Legacy Stage Diagnostic Scripts` to `Lithography Feedback & Z-Axis Mechanics`, `CLI Mode Dispatcher & Speed Sync`, `Frontend API Integration Class`?**
  _High betweenness centrality (0.056) - this node is a cross-community bridge._
- **Why does `LithographySystem` connect `Frontend API Integration Class` to `HMC Stage Stepper Driver`, `Legacy Stage Diagnostic Scripts`?**
  _High betweenness centrality (0.055) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `HmcControlCs` (e.g. with `LithographySystem` and `TestByteEncoding`) actually correct?**
  _`HmcControlCs` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `LithographySystem` (e.g. with `App` and `HmcControlCs`) actually correct?**
  _`LithographySystem` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Core api class for the electro-lithography system.`, `Return available COM ports as list of dicts.`, `Connect to all three axes. Returns success status.` to the rest of the system?**
  _74 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `HMC Stage Stepper Driver` be split into smaller, more focused modules?**
  _Cohesion score 0.11522048364153627 - nodes in this community are weakly interconnected._