# 🚁 ESP32 Quadcopter Flight Controller (GPS Hold + Cascaded PID)

![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)
![Platform: ESP32](https://img.shields.io/badge/Platform-ESP32-blue)

## 📌 What is this?

This is the code I wrote for my ESP32-based quadcopter. It's not a commercial product – just my own flight controller that actually flies.

**What it does:**
- Reads **MPU6050** (gyro + accelerometer) to figure out which way the drone is facing (Roll, Pitch, Yaw).
- Uses a **complementary filter** to smooth out the sensor noise.
- Reads **BME280** for temperature, air pressure, and relative altitude (so I know how high it is).
- Has **GPS Hold** – when I flip a switch on my transmitter, the drone tries to stay in one spot using GPS.
- Uses **cascaded PID** (angle loop + rate loop) for Roll and Pitch, and a separate PID for Yaw.
- Safety stuff: won't arm unless it's level and throttle is down. Also limits how fast motor signals change.

> **Note:** This code does **not** calculate things like Reynolds number or air density. It's purely for flight stabilization and GPS position holding.

---

## 🛠️ Main Features

- **Complementary filter** – better than raw sensor data.
- **GPS Hold** – uses PID to correct position errors.
- **Gyro calibration** – happens automatically when you power on.
- **BME280 altitude** – resets to zero at startup, so you get relative height.
- **Cascaded PID** – outer loop for angle, inner loop for rate. Feels more responsive.
- **iBUS input** – works with my FlySky receiver.

---

## 🚀 Getting Started

### What you need
- ESP32 Dev Board
- MPU6050 (I²C)
- BME280 (I²C)
- GPS module (NEO‑6M or similar, UART)
- iBUS receiver (UART)
- 4x ESCs + motors (X configuration)
- LiPo battery (3S or 4S)

### Wiring (quick reference)

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

*(I'll add a wiring diagram picture later)*

### Software Setup

1. Install [Arduino IDE](https://www.arduino.cc/en/software).
2. Add ESP32 support:  
   `Tools → Board → Boards Manager → Search "ESP32" → Install`
3. Install these libraries (all from Library Manager):
   - `Adafruit MPU6050`
   - `Adafruit BME280`
   - `Adafruit Sensor`
   - `ESP32Servo`
   - `TinyGPSPlus`
4. Download this repo (clone or zip).
5. Open `ESP32-FC.ino` in Arduino IDE.
6. Select board: **ESP32 Dev Module**.
7. Upload.

### Arming & Calibration

- Gyro calibrates itself in the first few seconds – keep the drone still on a level surface.
- To arm: throttle stick at minimum, then flip the **ARM switch** (channel 5) to high (1700+ µs). Motors will spin at idle speed (1050 µs).
- Disarm by flipping the switch back.

---

## 🧪 PID Tuning (my current gains)

I tuned these for a typical 250‑400mm frame. Feel free to adjust:

| Parameter | Variable in code | My range |
|-----------|------------------|-----------|
| Angle P (Roll/Pitch) | `KpAngR`, `KpAngP` | 1.0 – 2.0 |
| Rate P (Roll/Pitch)  | `KpRateR`, `KpRateP` | 0.2 – 0.5 |
| Rate I (Roll/Pitch)  | `KiRateR`, `KiRateP` | 0.02 – 0.08 |
| Yaw Rate P           | `KpYaw` | 0.5 – 1.0 |
| GPS Hold P           | `KpGps` | 0.5 – 1.2 |

---

## 📈 What I want to add next

- [ ] Log flight data to an SD card.
- [ ] Maybe add Bluetooth telemetry to see sensor data on my phone.
- [ ] Write a proper report/paper about this project.

---

## 📄 License

This project is licensed under **GNU General Public License v3.0** – see the [LICENSE](LICENSE) file.  
In short: you can use, modify, and share this code, but any modified version must also be open source under GPLv3.

## ✍️ Citation (if you use this in your work)

```bibtex
@software{Adam_ESP32_QuadFlightController_2026,
  author = {Adam Thear Abdel Nabi},
  title = {ESP32 Quadcopter Flight Controller with GPS Hold},
  year = {2026},
  url = {https://github.com/adamtheareng22/ESP32_QuadFlightController}
}
