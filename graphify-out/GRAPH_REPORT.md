# Graph Report - .  (2026-06-22)

## Corpus Check
- 32 files · ~131,095 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 167 nodes · 264 edges · 22 communities (8 shown, 14 thin omitted)
- Extraction: 94% EXTRACTED · 6% INFERRED · 0% AMBIGUOUS · INFERRED: 16 edges (avg confidence: 0.73)
- Token cost: 175,000 input · 8,000 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Main Coordinator and Voltage Feedback|Main Coordinator and Voltage Feedback]]
- [[_COMMUNITY_Basler Camera Integration|Basler Camera Integration]]
- [[_COMMUNITY_LabVIEW Architecture and Electro-Lithography Reports|LabVIEW Architecture and Electro-Lithography Reports]]
- [[_COMMUNITY_Holmarc Motor Control Core|Holmarc Motor Control Core]]
- [[_COMMUNITY_SMU Driver and Measurement|SMU Driver and Measurement]]
- [[_COMMUNITY_Dual-Graph Context Policy Rules|Dual-Graph Context Policy Rules]]
- [[_COMMUNITY_Motor Position Tracking|Motor Position Tracking]]
- [[_COMMUNITY_Manual Motor Move|Manual Motor Move]]
- [[_COMMUNITY_Motor Limit Safety Check|Motor Limit Safety Check]]
- [[_COMMUNITY_Motor VISA Port Close|Motor VISA Port Close]]
- [[_COMMUNITY_Motor VISA Port Initialization|Motor VISA Port Initialization]]
- [[_COMMUNITY_Motor Homing Functionality|Motor Homing Functionality]]
- [[_COMMUNITY_Motor Axis Status Reader|Motor Axis Status Reader]]
- [[_COMMUNITY_Motor Stop VI|Motor Stop VI]]
- [[_COMMUNITY_SMU Serial Close|SMU Serial Close]]
- [[_COMMUNITY_SMU Output Toggle|SMU Output Toggle]]
- [[_COMMUNITY_SMU VISA Library Protocol|SMU VISA Library Protocol]]
- [[_COMMUNITY_LabVIEW Coordinates Validator|LabVIEW Coordinates Validator]]

## God Nodes (most connected - your core abstractions)
1. `HmcControlCs` - 35 edges
2. `BaslerCamera` - 19 edges
3. `LithographyRun.vi` - 15 edges
4. `App` - 14 edges
5. `main()` - 12 edges
6. `SmuVoltageSampler` - 7 edges
7. `ZFeedbackWorker` - 7 edges
8. `VoltageFeedbackState` - 6 edges
9. `main()` - 5 edges
10. `check_smu_errors()` - 5 edges

## Surprising Connections (you probably didn't know these)
- `Electro-Lithography Concept` --semantically_similar_to--> `Liquid Electromigration Principle`  [INFERRED] [semantically similar]
  README.md → Report.pdf
- `Python Main Script (main.py)` --semantically_similar_to--> `LithographyRun.vi`  [INFERRED] [semantically similar]
  README.md → Labview integration/README.md
- `Electro-Lithography System Overview` --cites--> `Dr. Santanu Talukder`  [EXTRACTED]
  README.md → Report.pdf
- `Python Holmarc Motor Controller (hmccontroller.py)` --semantically_similar_to--> `MotorProtocol.lvlib`  [INFERRED] [semantically similar]
  README.md → Labview integration/README.md
- `Nanoscale Water Bridge Formation` --conceptually_related_to--> `Real-Time Z-Feedback Adjustment`  [INFERRED]
  Report.pdf → Labview integration/README.md

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **LabVIEW Motor Control VISA VIs** — labview_integration_readme_motor_setspeed, labview_integration_readme_motor_move, labview_integration_readme_motor_readposition, labview_integration_readme_motor_readstatus, labview_integration_readme_motor_checklimits [EXTRACTED 1.00]
- **SMU SCPI Command Interface VIs** — labview_integration_readme_smu_init, labview_integration_readme_smu_reset, labview_integration_readme_smu_sourcecurrent, labview_integration_readme_smu_sourcevoltage, labview_integration_readme_smu_readvoltage, labview_integration_readme_smu_readcurrent [EXTRACTED 1.00]
- **Electro-Lithography Control Algorithms** — labview_integration_readme_z_probe_contact_detection, labview_integration_readme_real_time_z_feedback, labview_integration_readme_liftoff_reengage, labview_integration_readme_velocity_decomposition, labview_integration_readme_position_tracking [EXTRACTED 1.00]

## Communities (22 total, 14 thin omitted)

### Community 0 - "Main Coordinator and Voltage Feedback"
Cohesion: 0.10
Nodes (15): App, classify_voltage(), feedback_direction_label(), format_elapsed_time(), list_ports(), main(), print_timing_summary(), read_voltage_sample() (+7 more)

### Community 1 - "Basler Camera Integration"
Cohesion: 0.10
Nodes (15): BaslerCamera, BaslerCameraSettings, list_basler_cameras(), main(), Basler camera live-view support using the pypylon SDK.  This module is intenti, Grab and return one frame as an OpenCV BGR image.          This is useful for, Display a blocking live video stream.          Press the exit key, default 'q', Start live view in a background thread.          This is the integration-frien (+7 more)

### Community 2 - "LabVIEW Architecture and Electro-Lithography Reports"
Cohesion: 0.08
Nodes (31): AngleCalculation.vi, Liftoff and Re-engage Algorithm, LithographyRun.vi, LoadCoordinateFile.vi, Logging.vi, Motor_Move.vi, Motor_SetSpeed.vi, MotorProtocol.lvlib (+23 more)

### Community 4 - "SMU Driver and Measurement"
Cohesion: 0.25
Nodes (10): check_current(), check_smu_errors(), check_voltage(), init_smu(), read_current_sample(), _read_sample(), read_voltage_sample(), reset_smu() (+2 more)

## Knowledge Gaps
- **26 isolated node(s):** `Coordinates File (coordinates.txt)`, `MotorProtocol.lvlib`, `SMUProtocol.lvlib`, `Motor_Init.vi`, `Motor_Close.vi` (+21 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **14 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `HmcControlCs` connect `Holmarc Motor Control Core` to `Main Coordinator and Voltage Feedback`, `Controller Initialization`?**
  _High betweenness centrality (0.120) - this node is a cross-community bridge._
- **Why does `App` connect `Main Coordinator and Voltage Feedback` to `Controller Initialization`?**
  _High betweenness centrality (0.028) - this node is a cross-community bridge._
- **Are the 3 inferred relationships involving `HmcControlCs` (e.g. with `SmuVoltageSampler` and `VoltageFeedbackState`) actually correct?**
  _`HmcControlCs` has 3 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `App` (e.g. with `SmuVoltageSampler` and `VoltageFeedbackState`) actually correct?**
  _`App` has 3 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Basler camera live-view support using the pypylon SDK.  This module is intenti`, `Optional camera settings used during initialization.`, `Small wrapper around pypylon for live display and future integration.      Exa` to the rest of the system?**
  _40 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Main Coordinator and Voltage Feedback` be split into smaller, more focused modules?**
  _Cohesion score 0.1006006006006006 - nodes in this community are weakly interconnected._
- **Should `Basler Camera Integration` be split into smaller, more focused modules?**
  _Cohesion score 0.10080645161290322 - nodes in this community are weakly interconnected._