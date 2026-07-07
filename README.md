# EDRO-16: Design of a Low-Cost Humanoid Robot for STEM Education in Resource-Constrained Classrooms

<p align="center">
  <img src="https://via.placeholder.com/1200x500.png?text=EDRO-16+Humanoid+Robot+Platform" alt="EDRO-16 Project Banner" width="100%">
</p>

<p align="center">
  <a href="https://github.com/OsamaIM/EDRO-16/stargazers"><img src="https://img.shields.io/github/stars/OsamaIM/EDRO-16?style=for-the-badge&color=blue" alt="Stars"></a>
  <a href="https://github.com/OsamaIM/EDRO-16/network/members"><img src="https://img.shields.io/github/forks/OsamaIM/EDRO-16?style=for-the-badge&color=darkcyan" alt="Forks"></a>
  <a href="https://github.com/OsamaIM/EDRO-16/blob/main/LICENSE"><img src="https://img.shields.io/github/license/OsamaIM/EDRO-16?style=for-the-badge&color=green" alt="License"></a>
  <a href="https://youtube.com/playlist?list=PLQrHOmw8Q7Ug&si=Uitsfv6Wf2Tod0b9"><img src="https://img.shields.io/badge/YouTube-Playlist-red?style=for-the-badge&logo=youtube" alt="YouTube Playlist"></a>
</p>

---

## 📝 Project Overview

Access to robotics education remains globally restricted due to high cost barriers, with conventional commercial platforms frequently exceeding USD \$800. **EDRO-16** (*Education Driven Robot with 16 Degrees of Freedom*) targets this challenge directly by leveraging frugal engineering concepts to build an open, customizable, and completely programmable humanoid robot platform.

Constructed via consumer-grade Fused Deposition Modeling (FDM) 3D printing and widely accessible commodity electronic boards, EDRO-16 achieves a complete bill-of-materials (BOM) cost of just **USD \$58.09**. The firmware is engineered in native bare-metal C++ utilizing Visual Studio Code and PlatformIO, eschewing simplified visual block coding environments to teach developers direct hardware register interactions and low-overhead timing optimizations.

### 🌟 Key Performance Metrics
* **Total Build Cost:** \$58.09 USD
* **Physical Profile:** Height: 24.23 cm | Weight: ~475 g (Compact and backpack-portable)
* **Locomotion Stability:** Steady open-loop bipedal translation at $0.87 \pm 0.24 \text{ cm/s}$ with zero kinematic tipping failures during evaluation.
* **Power Endurance:** Battery subsystem manages up to 303 minutes during standby idle routines and 75 minutes of continuous full-body dance/gait execution.

---

## 📺 Prototype Demonstration & Playlists

To observe the physical kinematics configurations, bipedal translation runs, and adaptive gesture interaction loops, visit the official video showcase log link:

🔗 **[EDRO-16 Humanoid Prototype - YouTube Video Playlist](https://youtube.com/playlist?list=PLQrHOmw8Q7Ug&si=Uitsfv6Wf2Tod0b9)**

---

## 🛠️ System Architecture

### 1. Kinematics & Degrees of Freedom (DoF) Allocation
EDRO-16 distributes its 16 servo actuators across 5 major biological joints to optimize structural equilibrium and spatial maneuvering flexibility:

| Body Region | DoF per Unit | Quantity | Total DoF | Implemented Articulations |
| :--- | :---: | :---: | :---: | :--- |
| **Head** | 1 | 1 | **1** | Neck Pan Axis |
| **Arms** | 3 | 2 | **6** | Shoulder Pitch, Shoulder Roll, Elbow Flexion |
| **Legs** | 4 | 2 | **8** | Hip Roll, Hip Pitch, Knee Pitch, Ankle Pitch |
| **Waist** | 1 | 1 | **1** | Core Torso Rotation |
| **Total** | — | — | **16** | **Full-Body Synchronized Motion** |

### 2. Electronics Stack Setup
* **Microcontroller Unit:** ESP32-WROOM SoC (Dual-Core Xtensa LX6 processor working at 240 MHz, featuring native 2.4GHz Wi-Fi/BLE network handling).
* **Joint Actuation:** 16 × high-durability micro MG90S metal gear servos.
* **PWM Co-processor Board:** PCA9685 16-Channel 12-bit PWM generator driven over a dedicated $I^2C$ serial interface to eliminate microsecond timing jitter on the ESP32 computing cores.
* **Telemetry Core:** Chassis-mounted Infrared (IR) Proximity Sensor arrays for obstacle profiling and distance tracking.
* **Power Distribution Network:** Driven via a BEAT LiPo 2S 1500mAh 35C high-discharge cell (7.4V nominal output) filtered through an inline high-amperage 5V step-down buck converter to isolate motor switching surges from the logic circuits.

---

## 📐 Locomotion & Analytical Mathematical Theory

To enable low-latency calculation on the localized microcontroller, the gait engine computes positions using a robust analytical inverted pendulum physics model:

### A. Footprint Stability Zone
Multi-joint movements must tightly constrain the center of mass tracking within the physical boundaries of the active structural support polygon ($60\text{mm} \times 80\text{mm}$):
$$\dot{x}_{com} \in [x_{min}, x_{max}], \quad y_{com} \in [y_{min}, y_{max}]$$
The static vertical Center of Mass height parameter $h$ is maintained at a locked offset of $133\text{mm}$ from the baseline ground vector.

### B. Linear Inverted Pendulum Model (LIPM)
Forward stepping dynamics are modeled by projecting horizontal structural forces through a linearized pendulum configuration:
$$\ddot{x} = \frac{g}{h}x$$
Assuming standard gravity constraints $g = 9.81 \text{ m/s}^2$ and a fixed Center of Mass length $h = 0.133 \text{ m}$, the embedded system operates over a constant dynamic transformation constant of $\frac{g}{h} = 73.76 \text{ rad}^2/\text{s}^2$, saving processing cycles on runtime path calculations.

### C. Actuator Torque Loading Metrics
Joint loading forces are checked against stall profiles to guarantee mechanical overhead:
$$\tau = mgl \cdot \sin(\theta)$$
* **Hip Actuation Modules:** Running at an absolute step limit of $\theta = 12^\circ$, a sub-mass loading index $m = 0.181 \text{ kg}$, and structural swing radius $l = 0.061 \text{ m}$, local torque translates to $\tau_{hip} = 0.022 \text{ N}\cdot\text{m}$ ($0.224 \text{ kg}\cdot\text{cm}$), establishing a **8.9× safety overhead** against MG90S maximum stall torque.
* **Knee Actuation Modules:** Mapping out a max lift deflection of $\theta = 14^\circ$, lower leg component mass $m = 0.090 \text{ kg}$, and effective center distance $l = 0.030 \text{ m}$, local load measures $\tau_{knee} = 0.0064 \text{ N}\cdot\text{m}$ ($0.065 \text{ kg}\cdot\text{cm}$), maintaining a **30.6× operating safety margin**.

---

## 💻 Firmware Installation & Environment Setup

The embedded codebase is constructed natively inside **Visual Studio Code** using the **PlatformIO IDE** unified management tool. Follow these setup steps to compile and deploy the system logic:

### System Pre-requisites
1. Install [Visual Studio Code](https://code.visualstudio.com/).
2. Launch the Extensions interface (`Ctrl+Shift+X`), search for **PlatformIO IDE**, and trigger initialization.

### Compilation & Flash Deployment

1. **Clone Local Repository:**
   ```bash
   git clone [https://github.com/OsamaIM/EDRO-16.git](https://github.com/OsamaIM/EDRO-16.git)
   cd EDRO-16
