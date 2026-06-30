# Graph Report - .  (2026-06-30)

## Corpus Check
- 42 files · ~165,815 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 222 nodes · 335 edges · 33 communities (10 shown, 23 thin omitted)
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 24 edges (avg confidence: 0.79)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Stage Operation Group 0|Stage Operation Group 0]]
- [[_COMMUNITY_Axis Controller Framework|Axis Controller Framework]]
- [[_COMMUNITY_Diagnostics & Calibration History|Diagnostics & Calibration History]]
- [[_COMMUNITY_Axis Controller Framework|Axis Controller Framework]]
- [[_COMMUNITY_Axis Controller Framework|Axis Controller Framework]]
- [[_COMMUNITY_Axis Controller Framework|Axis Controller Framework]]
- [[_COMMUNITY_Stage Operation Group 6|Stage Operation Group 6]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Axis Controller Framework|Axis Controller Framework]]
- [[_COMMUNITY_Stage Operation Group 9|Stage Operation Group 9]]
- [[_COMMUNITY_Stage Operation Group 10|Stage Operation Group 10]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Stage Operation Group 19|Stage Operation Group 19]]
- [[_COMMUNITY_Stage Operation Group 20|Stage Operation Group 20]]
- [[_COMMUNITY_Stage Operation Group 21|Stage Operation Group 21]]
- [[_COMMUNITY_Stage Operation Group 22|Stage Operation Group 22]]
- [[_COMMUNITY_Stage Operation Group 23|Stage Operation Group 23]]
- [[_COMMUNITY_Stage Operation Group 24|Stage Operation Group 24]]
- [[_COMMUNITY_Stage Operation Group 25|Stage Operation Group 25]]
- [[_COMMUNITY_Stage Operation Group 26|Stage Operation Group 26]]
- [[_COMMUNITY_Stage Operation Group 27|Stage Operation Group 27]]
- [[_COMMUNITY_Stage Operation Group 28|Stage Operation Group 28]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]
- [[_COMMUNITY_Pattern Coordinates & Geometry|Pattern Coordinates & Geometry]]

## God Nodes (most connected - your core abstractions)
1. `HmcControlCs` - 43 edges
2. `App` - 22 edges
3. `BaslerCamera` - 19 edges
4. `LithographyRun.vi` - 15 edges
5. `main()` - 13 edges
6. `SmuVoltageSampler` - 7 edges
7. `ZFeedbackWorker` - 7 edges
8. `VoltageFeedbackState` - 6 edges
9. `Circle Distortion at Cardinal Points` - 6 edges
10. `main()` - 5 edges

## Surprising Connections (you probably didn't know these)
- `Electro-Lithography Concept` --semantically_similar_to--> `Liquid Electromigration Principle`  [INFERRED] [semantically similar]
  README.md → Report.pdf
- `Python Main Script (main.py)` --semantically_similar_to--> `LithographyRun.vi`  [INFERRED] [semantically similar]
  README.md → Labview integration/README.md
- `Successful Circular Trajectory` --conceptually_related_to--> `Circle Distortion at Cardinal Points`  [INFERRED]
  images/1.jpg → diagnostic_and_fix_history.md
- `Dual-Graph Context Policy` --semantically_similar_to--> `Dual-Graph Context Policy Rules`  [INFERRED] [semantically similar]
  CLAUDE.md → .agent/rules/graperoot.md
- `Python Holmarc Motor Controller (hmccontroller.py)` --semantically_similar_to--> `MotorProtocol.lvlib`  [INFERRED] [semantically similar]
  README.md → Labview integration/README.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Calibration and Circle Distortion Resolution Flow** — diagnostic_and_fix_history_circle_distortion, diagnostic_and_fix_history_firmware_speed_bands_bug, diagnostic_and_fix_history_fine_grained_calibration [INFERRED 0.95]
- **Plotter Coordinate Patterns** — circles_coordinates, hexagons_coordinates, coords_square_coordinates, coords_parallel_coordinates [INFERRED 0.85]
- **Concentric Circles Patterning Flow** — hmccontroller_circles_plot_concentric_circles_pattern, hmccontroller_circles_plot_drawing_path, hmccontroller_circles_plot_travel_liftoff_path, hmccontroller_circles_plot_vertices [INFERRED 0.85]
- **LabVIEW Motor Control VISA VIs** — labview_integration_readme_motor_setspeed, labview_integration_readme_motor_move, labview_integration_readme_motor_readposition, labview_integration_readme_motor_readstatus, labview_integration_readme_motor_checklimits [EXTRACTED 1.00]
- **SMU SCPI Command Interface VIs** — labview_integration_readme_smu_init, labview_integration_readme_smu_reset, labview_integration_readme_smu_sourcecurrent, labview_integration_readme_smu_sourcevoltage, labview_integration_readme_smu_readvoltage, labview_integration_readme_smu_readcurrent [EXTRACTED 1.00]
- **Electro-Lithography Control Algorithms** — labview_integration_readme_z_probe_contact_detection, labview_integration_readme_real_time_z_feedback, labview_integration_readme_liftoff_reengage, labview_integration_readme_velocity_decomposition, labview_integration_readme_position_tracking [EXTRACTED 1.00]

## Communities (33 total, 23 thin omitted)

### Community 0 - "Stage Operation Group 0"
Cohesion: 0.10
Nodes (15): BaslerCamera, BaslerCameraSettings, list_basler_cameras(), main(), Basler camera live-view support using the pypylon SDK.  This module is intenti, Grab and return one frame as an OpenCV BGR image.          This is useful for, Display a blocking live video stream.          Press the exit key, default 'q', Start live view in a background thread.          This is the integration-frien (+7 more)

### Community 1 - "Axis Controller Framework"
Cohesion: 0.08
Nodes (31): AngleCalculation.vi, Liftoff and Re-engage Algorithm, LithographyRun.vi, LoadCoordinateFile.vi, Logging.vi, Motor_Move.vi, Motor_SetSpeed.vi, MotorProtocol.lvlib (+23 more)

### Community 2 - "Diagnostics & Calibration History"
Cohesion: 0.11
Nodes (15): Attempt 2: Proportional Speed Scaling, classify_voltage(), compute_synchronized_speeds(), feedback_direction_label(), format_elapsed_time(), list_ports(), main(), print_timing_summary() (+7 more)

### Community 3 - "Axis Controller Framework"
Cohesion: 0.12
Nodes (15): Square Plotting Coordinates, Calibration and Circle Distortion Resolution History, Holmarc Stepper Motor Stage, App, list_ports(), main(), list_ports(), main() (+7 more)

### Community 5 - "Axis Controller Framework"
Cohesion: 0.14
Nodes (14): Circle Plotting Coordinates, Circle Distortion at Cardinal Points, High-Resolution Sweep Calibration, Firmware Speed Bands Bug, Attempt 1: Independent Speed Shifting, Serial State Parser Collision, Successful Circular Trajectory, Concentric Ghost Circle Trace (+6 more)

### Community 6 - "Stage Operation Group 6"
Cohesion: 0.25
Nodes (10): check_current(), check_smu_errors(), check_voltage(), init_smu(), read_current_sample(), _read_sample(), read_voltage_sample(), reset_smu() (+2 more)

### Community 7 - "Pattern Coordinates & Geometry"
Cohesion: 0.14
Nodes (6): generate_concentric_circles(), plot_concentric_circles(), plot_concentric_hexagons(), Optional visualization of the generated concentric hexagons path using matplotli, Generates coordinates for concentric circles of defined radii.     Points are c, Optional visualization of the generated concentric circles path using matplotlib

### Community 8 - "Axis Controller Framework"
Cohesion: 0.53
Nodes (6): Concentric Circles Patterning, Drawing Path (flag=0), Exposure Segmentation Mechanism, Concentric Circles Patterning Path & Liftoff Signals Plot, Travel/Liftoff Path (flag=1), Pattern Vertices

## Knowledge Gaps
- **43 isolated node(s):** `Holmarc Stepper Motor Stage`, `Commanded vs. Actual Speed Duration Data`, `Calibration Log Output`, `Circle Plotting Coordinates`, `Hexagon Plotting Coordinates` (+38 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **23 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `HmcControlCs` connect `Axis Controller Framework` to `Diagnostics & Calibration History`, `Axis Controller Framework`?**
  _High betweenness centrality (0.123) - this node is a cross-community bridge._
- **Why does `Circle Distortion at Cardinal Points` connect `Axis Controller Framework` to `Diagnostics & Calibration History`, `Axis Controller Framework`?**
  _High betweenness centrality (0.056) - this node is a cross-community bridge._
- **Why does `App` connect `Axis Controller Framework` to `Diagnostics & Calibration History`?**
  _High betweenness centrality (0.043) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `HmcControlCs` (e.g. with `SmuVoltageSampler` and `VoltageFeedbackState`) actually correct?**
  _`HmcControlCs` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `App` (e.g. with `SmuVoltageSampler` and `VoltageFeedbackState`) actually correct?**
  _`App` has 3 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Basler camera live-view support using the pypylon SDK.  This module is intenti`, `Optional camera settings used during initialization.`, `Small wrapper around pypylon for live display and future integration.      Exa` to the rest of the system?**
  _63 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Stage Operation Group 0` be split into smaller, more focused modules?**
  _Cohesion score 0.10080645161290322 - nodes in this community are weakly interconnected._