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

## 🔬 Project Overview

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


