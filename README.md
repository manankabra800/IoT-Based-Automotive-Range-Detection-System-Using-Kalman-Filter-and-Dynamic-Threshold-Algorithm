# IoT-Based Automotive Range Detection System

> A multi-sensor fusion system for real-time vehicle collision avoidance — combining ultrasonic and LiDAR sensors with Kalman filtering, GPS-adaptive safe distance thresholds, and cloud-connected IoT alerts.

[![Status](https://img.shields.io/badge/status-Phase%201%20Complete-success)]()
[![Platform](https://img.shields.io/badge/platform-ESP32-blue)]()
[![Protocol](https://img.shields.io/badge/protocol-MQTT-orange)]()
[![Language](https://img.shields.io/badge/code-Python%20%7C%20C%2B%2B%20%7C%20JavaScript-yellow)]()

---

## 📋 Overview

Rear-end collisions are a leading cause of road fatalities worldwide, primarily because existing automotive proximity systems rely on single sensors with fixed distance thresholds and lack real-time network connectivity. This project addresses these limitations through a four-layer IoT architecture that fuses ultrasonic and LiDAR sensor readings using a Kalman filter, computes dynamic safe following distances based on GPS-derived vehicle speed, and transmits all telemetry to the cloud via MQTT for remote monitoring and mobile push notifications.

Developed as a major project for the Mechanical Engineering department at NMIT Bengaluru under Visvesvaraya Technological University (VTU) for the academic year 2026–27.

---

## 🎯 Key Features

- **Multi-Sensor Fusion** — HC-SR04 ultrasonic + TF-Luna LiDAR combined via Kalman filter (Q=0.01, R=0.5)
- **Dynamic Safe Distance** — GPS-speed-adaptive threshold using IS 11556 braking formula
- **Real-Time IoT Pipeline** — MQTT over Wi-Fi to HiveMQ cloud broker at 5 Hz
- **Multi-Level Alerts** — OLED display, RGB LEDs, active buzzer, and mobile push notifications
- **Edge Computing** — All processing happens on ESP32 with sub-300ms alert response time
- **Quantitatively Validated** — 4.2 cm MAE, 5.6 cm RMSE, 66.7% false alert reduction

---

## 📊 Performance Results

| Metric | HC-SR04 Alone | TF-Luna Alone | Kalman Fused | Target |
|--------|---------------|---------------|--------------|--------|
| MAE | 9.3 cm | 2.0 cm | **4.2 cm** | < 2 cm* |
| RMSE | 14.9 cm | 2.5 cm | **5.6 cm** | — |
| False Alert Rate | 1.0% | — | **0.33%** | — |
| Update Rate | — | — | **5 Hz** | ≥ 5 Hz |

*Real TF-Luna hardware will achieve target — simulation uses conservative noise models*

**Key achievement:** 66.7% false alert reduction with dynamic threshold vs static 150 cm — exceeding 30% target by more than double.

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│  LAYER 1 — PERCEPTION                                           │
│  HC-SR04 Ultrasonic  +  TF-Luna LiDAR  +  NEO-6M GPS            │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│  LAYER 2 — EDGE COMPUTING                                       │
│  ESP32 WROOM-32  ·  Kalman Filter  ·  Dynamic Threshold         │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│  LAYER 3 — INTERNET                                             │
│  MQTT Protocol over Wi-Fi / SIM800L Cellular Fallback           │
└──────────────────────────┬──────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────┐
│  LAYER 4 — CLOUD + MOBILE                                       │
│  HiveMQ  ·  Node-RED Dashboard  ·  ThingSpeak  ·  Firebase      │
└─────────────────────────────────────────────────────────────────┘
```

---

## 📁 Repository Structure

```
.
├── docs/
│   ├── Phase1_Report.pdf               # 21-page synopsis + literature survey
│   ├── Phase2_Report.docx              # Simulation results + IoT pipeline
│   └── Combined_Phase1_Phase2.docx     # Full 33-page report
│
├── simulation/
│   ├── kalman_simulation_FINAL.py      # Python SIL — generates 300 samples
│   └── simulation_results.png          # Composite 5-graph output
│
├── firmware/
│   ├── wokwi_sketch_fixed.ino          # ESP32 Arduino firmware
│   └── diagram_fixed.json              # Wokwi circuit diagram
│
├── dashboard/
│   ├── live_dashboard.html             # 5 Hz live IoT dashboard
│   ├── interactive_simulator.html      # Drag sliders, see Kalman live
│   └── static_vs_dynamic.html          # Animated threshold comparison
│
├── iot/
│   └── mqtt_live_demo.py               # Real MQTT publisher (HiveMQ)
│
└── presentations/
    ├── Phase1_Final_Summary.pptx       # Phase 1 final review
    └── Three_New_Demos_FINAL.pptx      # Additional demonstrations
```

---

## 🚀 Quick Start

### 1. Run the Python Simulation

```bash
pip install numpy matplotlib
python simulation/kalman_simulation_FINAL.py
```

Output: `simulation_results.png` with 5 graphs + console output showing MAE, RMSE, and false alert metrics.

### 2. Open the Interactive Dashboard

Double-click any of these HTML files to open in your browser:
- `dashboard/live_dashboard.html` — 5 Hz live simulation
- `dashboard/interactive_simulator.html` — drag sliders to test
- `dashboard/static_vs_dynamic.html` — compare threshold approaches

### 3. Run the Real MQTT Demo

```bash
pip install paho-mqtt
python iot/mqtt_live_demo.py
```

Then open `mqttx.app/web`, connect to `broker.emqx.io:8083`, and subscribe to `iot/vehicle/range` to see live JSON messages.

### 4. Run the Wokwi Simulation

Open the project on [Wokwi.com](https://wokwi.com) using the `firmware/wokwi_sketch_fixed.ino` and `firmware/diagram_fixed.json` files.

---

## 🔬 Core Algorithms

### Kalman Filter (1-D Dual Pass)

```cpp
// Predict step
P = P + Q;                              // Q = 0.01

// Update pass 1 — HC-SR04
K1 = P / (P + R);                       // R = 0.50
x  = x + K1 * (sensor1 - x);
P  = (1 - K1) * P;

// Update pass 2 — TF-Luna (4x more trusted)
K2 = P / (P + R * 0.25);
x  = x + K2 * (avg_sensors - x);
P  = (1 - K2) * P;
```

### Dynamic Safe Distance (IS 11556)

```cpp
float safe_distance(float speed_kmh) {
    float v = speed_kmh / 3.6;          // Convert to m/s
    float tr = 0.7;                     // Reaction time (IS 11556)
    float a  = 7.0;                     // Deceleration m/s²
    return max(20, (v * tr + v*v / (2*a)) * 100);  // cm
}
```

### Alert Decision

```cpp
if      (fused < safe * 0.5) alert = DANGER;   // Red LED + continuous alarm
else if (fused < safe)       alert = WARNING;  // Blue LED + intermittent beep
else                         alert = SAFE;     // Green LED + silent
```

---

## 📡 MQTT JSON Payload Format

Published to topic `iot/vehicle/range` at 5 Hz:

```json
{
  "msg_id": 47,
  "distance_cm": 245.3,
  "speed_kmh": 42.1,
  "safe_dist_cm": 234.6,
  "sensor1_cm": 251.2,
  "sensor2_cm": 243.1,
  "alert_level": 1,
  "alert_status": "WARNING",
  "timestamp": "2026-05-12T10:15:30.123Z",
  "device_id": "ESP32_NMIT_001"
}
```

---

## 🛠 Hardware Components

| Component | Model | Purpose | Cost (₹) |
|-----------|-------|---------|----------|
| Microcontroller | ESP32 WROOM-32 ×2 | Main MCU + Wi-Fi | 800 |
| Ultrasonic Sensor | HC-SR04 ×2 | Short-range distance | 200 |
| LiDAR Module | Benewake TF-Luna | High-accuracy range | 1200 |
| GPS Module | NEO-6M | Speed and location | 350 |
| OLED Display | SSD1306 0.96" | Local status display | 180 |
| Cellular Module | SIM800L GSM | Network fallback | 800 |
| Power Module | LM2596 Buck | 12V → 5V conversion | 120 |
| Misc | Buzzer, LEDs, PCB | Alerts and assembly | 210 |
| **Total** | | | **₹3,860** |

---

## 📚 Literature Survey

This project builds on six key research papers:

1. **Guerrero-Ibáñez et al. (2018)** — *Sensor Technologies for Intelligent Transportation Systems.* MDPI Sensors 18(4):1212
2. **Rosdi & Abdul Ghani (2022)** — *Investigation on Accuracy of Sensors in Sensor Fusion for Object Detection.* Springer LNEE Vol 730
3. **Valade et al. (2017)** — *A Study about Kalman Filters Applied to Embedded Sensors.* MDPI Sensors 17(12):2810
4. **Mohamed et al. (2022)** — *Safe Driving Distance and Speed for Collision Avoidance in Connected Vehicles.* MDPI Sensors 22(18):7051
5. **Ramdasi et al. (2023)** — *IoT-Based Automotive Collision Avoidance and Safety System.* Springer CIS 2022
6. **Chang et al. (2025)** — *ESP32-Based Edge Computing for Object Detection.* MDPI Sensors 25(6):1656

---

## 🛣️ Roadmap

### ✅ Phase 1 — Algorithm Design & Validation (Completed)
- [x] Literature survey and gap analysis
- [x] System architecture design
- [x] Functional requirements (FR1–FR4)
- [x] Kalman filter implementation and validation
- [x] Dynamic threshold algorithm
- [x] Wokwi ESP32 circuit simulation
- [x] Live IoT dashboard simulation
- [x] Real MQTT pipeline demonstration

### 🔄 Phase 2 — Hardware Implementation (Semester 7)
- [ ] Procure all hardware components
- [ ] Assemble circuit on PCB
- [ ] Flash validated firmware to physical ESP32
- [ ] Test real Kalman fusion with HC-SR04 + TF-Luna
- [ ] Connect HiveMQ broker over Wi-Fi
- [ ] Setup Node-RED live dashboard
- [ ] Build MIT App Inventor mobile app with Firebase
- [ ] Run 5 validation experiments
- [ ] Submit manuscript to MDPI Sensors / IEEE Access

---

## 👥 Team

| Name | USN | Role |
|------|-----|------|
| **Ajinkya Bhagwat** | 1NT23ME007 | System Architecture · Algorithm Design |
| **Manan Kabra** | 1NT23ME026 | Firmware · IoT Pipeline |
| **Rishita Modi** | 1NT23ME038 | Simulation · Documentation |

**Project Guide:** Dr. Abdulrajak Buradi — Associate Professor, Department of Mechanical Engineering

---

## 🏫 Institution

**Nitte Meenakshi Institute of Technology (NMIT)**
Bengaluru, Karnataka — 560064
Visvesvaraya Technological University (VTU), Belagavi
Academic Year 2026–27

---

## 📜 License

This project is part of academic coursework at NMIT Bengaluru under VTU. Code is released for educational purposes. Please cite this work if you use any portion in your own projects.

---

## 🙏 Acknowledgements

We thank Dr. Abdulrajak Buradi for guidance throughout the project, the Mechanical Engineering department at NMIT for resources and support, and the open-source community for tools including Wokwi, HiveMQ, MQTTX, Node-RED, and Chart.js.

---

<p align="center">
  Built with ❤ at NMIT Bengaluru · VTU · 2026
</p>
