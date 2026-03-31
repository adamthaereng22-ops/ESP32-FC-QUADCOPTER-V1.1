# 🚁 ESP32 Quadcopter Flight Controller (GPS Hold & Cascaded PID)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue)](https://www.espressif.com/)

## 📌 Overview

This repository contains a complete flight control firmware for an **X‑configuration quadcopter** using an **ESP32**. The code implements:

- **MPU6050** IMU with complementary filter for attitude estimation (Roll, Pitch, Yaw).
- **BME280** environmental sensor: temperature, pressure, and relative altitude.
- **GPS** module with **GPS Hold** functionality.
- **iBUS** receiver protocol for RC input.
- **Cascaded PID controllers**: angle + rate for Roll/Pitch, rate PID for Yaw.
- **Safety**: arming only when level and throttle low, slew‑rate limiting on motor outputs.

> **Note**: This version does **not** include real‑time aerodynamic calculations (Reynolds number, air density, viscosity). It focuses on attitude stabilization and GPS position holding.

---

## 🛠️ Key Features

- **Complementary filter** for smooth attitude estimation.
- **GPS Hold** using PID correction on latitude/longitude.
- **Automatic gyro calibration** at startup.
- **BME280** altitude stabilization reference (relative to start).
- **Cascaded PID** (outer angle loop, inner rate loop) for stable flight.
- **iBUS** telemetry‑ready input.

---

## 🚀 Getting Started

### Hardware Required
- ESP32 Dev Module
- MPU6050 (I²C)
- BME280 (I²C)
- GPS module (NEO‑6M or similar, UART)
- iBUS receiver (UART)
- 4x ESCs with motors (X configuration)
- LiPo battery (3S/4S)

### Wiring

| Component | ESP32 Pins |
|-----------|------------|
| MPU6050   | SDA 21, SCL 22 |
| BME280    | SDA 21, SCL 22 |
| GPS       | RX 17, TX 5 |
| iBUS RX   | RX 16 |
| ESC M1    | 13 |
| ESC M2    | 14 |
| ESC M3    | 25 |
| ESC M4    | 26 |



### Software Setup
1. Install [Arduino IDE](https://www.arduino.cc/en/software).
2. Add ESP32 board support:  
   `Tools → Board → Boards Manager → Search "ESP32" → Install`.
3. Install libraries:
   - `Adafruit MPU6050`
   - `Adafruit BME280`
   - `Adafruit Sensor`
   - `ESP32Servo`
   - `TinyGPSPlus`
4. Clone or download this repository.
5. Open the `ESP32-FC.ino` file in Arduino IDE.
6. Select board: **ESP32 Dev Module**.
7. Upload the sketch.

### Calibration & Arming
- Gyroscope offsets are automatically calibrated during the first few seconds.
- Place the quad on a level surface.
- Move the **ARM switch** (channel 5) to high (1700+ µs) while throttle is at minimum.
- Motors will spin to idle speed (1050 µs) when armed.

---

## 🧪 PID Tuning

Default gains are provided and can be adjusted in the code:

| Parameter | Variable | Typical Range |
|-----------|----------|---------------|
| Angle P (Roll/Pitch) | `KpAngR`, `KpAngP` | 1.0 – 2.0 |
| Rate P (Roll/Pitch)  | `KpRateR`, `KpRateP` | 0.2 – 0.5 |
| Rate I (Roll/Pitch)  | `KiRateR`, `KiRateP` | 0.02 – 0.08 |
| Yaw Rate P           | `KpYaw` | 0.5 – 1.0 |
| GPS Hold P           | `KpGps` | 0.5 – 1.2 |

---

## 📈 Future Improvements

- [ ] Add SD card logging for flight data.
- [ ] Implement real‑time aerodynamic calculations (Reynolds, density, viscosity) using BME280 data.
- [ ] Write a scientific paper on the aerodynamic effects on quadcopter stability.

---

## 📄 License

This project is licensed under the MIT License – see the [LICENSE](LICENSE) file.

## ✍️ Citation

If you use this work in your research, please cite:

```bibtex
@software{Adam_ESP32_QuadFlightController_2026,
  author = {Adam thear abdel nabi},
  title = {ESP32 Quadcopter Flight Controller with GPS Hold},
  year = {2026/3/31},
  url = {https://github.com/adamtheareng22/ESP32_QuadFlightController}
}
