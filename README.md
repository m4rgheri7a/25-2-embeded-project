# 🏠 AI-based Smart Home Energy Saving System

> **2025-2 Pusan National University | Embedded System Design & Experiment (CB1501025)** > **Team 3 Term Project**

This project implements an intelligent smart home system using **STM32F107** and **TensorFlow Lite for Microcontrollers**. The system optimizes energy consumption by analyzing environmental data (Temperature, Humidity, Light, CO2) and detecting human presence.

## 👥 Team Members
* **Member**: [박지환](https://github.com/jihwan32)
* **Member**: [정유성](https://github.com/yoosung5480)
* **Member**: [신의철](https://github.com/mini-apple)
* **Member**: [황선웅](https://github.com/m4rgheri7a)

---

## 📖 Project Overview
[cite_start]The goal of this project is to move beyond simple conditional controls and implement **data-driven AI predictive control** for energy efficiency[cite: 8, 9].

* **AI Control**: Uses a TinyML model to detect human presence and automatically switch power-saving modes.
* **Environmental Monitoring**: Real-time collection of Temperature, Humidity, Illuminance, and CO2 levels.
* **Remote Control**: Bluetooth-based smartphone interface for monitoring and manual override.
* **Visualization**: Real-time status display via TFT-LCD.

---

## ⚙️ Key Features

### 1. AI-Driven Automation (Auto Mode)
[cite_start]The system utilizes **TensorFlow Lite** to analyze sensor data and predict human presence[cite: 20]. [cite_start]Based on the analysis, it switches between four operation modes[cite: 23]:

| Mode | State | Description |
| :--- | :--- | :--- |
| **Mode 0** | **Sleep** | Activated when no human is detected. [cite_start]All devices turn off to save energy[cite: 27, 93]. |
| **Mode 1** | **Lighting** | [cite_start]LED automatically turns ON when low light is detected[cite: 95]. |
| **Mode 2** | **Ventilation** | [cite_start]Servo motor opens the window/vent if Temp/Humidity exceeds the threshold[cite: 26, 97]. |
| **Mode 3** | **Cooling** | [cite_start]DC Fan activates if a rapid temperature rise is predicted[cite: 27, 99]. |

### 2. Manual Override (Manual Mode)
* [cite_start]Users can control the system manually via a smartphone app connected through **Bluetooth (FB755AC)**[cite: 10, 16].
* [cite_start]Commands like `LED_OFF` or `MODE=1` instantly override the AI logic[cite: 79].

### 3. Monitoring
* [cite_start]**TFT-LCD**: Displays current sensor readings, current mode, and AI prediction results[cite: 12, 24].
* [cite_start]**App**: Receives real-time status updates via Bluetooth[cite: 78].

---

## 🛠 Hardware Specifications

| Component | Model | Description |
| :--- | :--- | :--- |
| **MCU** | STM32F107 | [cite_start]Main Controller (Cortex-M3) [cite: 9] |
| **Temp/Hum Sensor** | AM2302 (DHT22) | [cite_start]Range: -40~80℃ / 0~99.9% RH [cite: 29] |
| **CO2 Sensor** | HX-2000U | [cite_start]Electrochemical CO2 Gas Sensor (UART) [cite: 29] |
| **Light Sensor** | CDS | [cite_start]Detects ambient light levels [cite: 29] |
| **Bluetooth** | FB755AC | [cite_start]Embedded Bluetooth Module for App communication [cite: 56] |
| **Display** | [cite_start]3.2" TFT LCD | Visual interface for system status [cite: 58] |
| **Actuator (Vent)** | Servo Motor | [cite_start]Controls window/ventilation mechanism [cite: 29] |
| **Actuator (Cool)** | DC Fan (MGA4005LR) | [cite_start]5V, 4600RPM Fan for cooling [cite: 48] |
| **Lighting** | High-brightness LED | [cite_start]5mm White LED for illumination [cite: 30] |

---

## 🚀 System Logic Flow

1.  [cite_start]**Initialization**: Initialize sensors, LCD, and Bluetooth on power-up[cite: 68].
2.  [cite_start]**Data Acquisition**: Periodically read data from DHT22, CDS, and CO2 sensors[cite: 71].
3.  **Inference & Decision**:
    * **Check Mode**: Is it Auto or Manual?
    * **Auto**: Run TFLite model to check for human presence.
        * *No Human* → **Mode 0 (Sleep)**
        * [cite_start]*Human Present* → Determine optimal mode (1, 2, or 3) based on environmental logic[cite: 72, 73].
4.  [cite_start]**Actuation**: Drive LEDs, Servo Motors, or DC Fans based on the determined mode[cite: 27].
5.  [cite_start]**Feedback**: Update LCD display and send data to the smartphone[cite: 77, 78].

---

## 💻 Tech Stack
* **Language**: C / C++
* **Framework**: TensorFlow Lite for Microcontrollers
* **IDE**: STM32CubeIDE / Keil uVision
* **Hardware Interface**: UART, GPIO, I2C/SPI (for LCD)

---

## 📸 Demo
TBD
---
### 📝 License
This project was developed for the **Embedded System Design and Experiment** course at **Pusan National University**.