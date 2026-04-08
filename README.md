# advanced-gait-analysis-system
Advanced Gait Analysis and Foot Pressure Mapping System
🧠 Advanced Gait Analysis & Foot Pressure Mapping System

📌 Overview
This project presents a low-cost, wearable gait analysis system that captures and visualizes human walking patterns in real time. It integrates Force Sensitive Resistors (FSRs) for plantar pressure mapping and an MPU6050 IMU for motion tracking, all processed using an ESP32 microcontroller.

The system provides real-time biomechanical insights such as pressure distribution, step detection, cadence, and foot orientation through a custom-built visualization interface.

🚀 Key Features
🔴 Real-time Foot Pressure Heatmap
Visualizes plantar pressure using a color-coded (green → red) gradient
👣 Step Detection & Cadence Estimation
Adaptive threshold-based step detection using FSR data
📐 IMU-Based Motion Analysis
Computes pitch and roll to analyze foot orientation
📊 Center of Pressure (COP) Tracking
Estimates pressure distribution across gait phases
🎛️ Interactive GUI (Processing)
Drag & calibrate sensor positions
Adjustable foot model
Real-time metrics panel
📁 CSV Data Streaming
Structured data output for logging and further analysis

🏗️ System Architecture
FSR Sensors (Heel, Arch, Forefoot, Toe)
                │
                ▼
          ESP32 (ADC + Processing)
                │
     MPU6050 (I2C - Motion Data)
                │
                ▼
     Serial Communication (USB)
                │
                ▼
   Processing GUI (Heatmap + Metrics)

🔌 Hardware Components
ESP32 (ESP32-C3)
4 × Force Sensitive Resistors (FSR)
MPU6050 (Accelerometer + Gyroscope)
10kΩ Resistors (Voltage Divider)
Breadboard & Wiring
Custom insole setup

💻 Software Stack
Component	Technology Used
Firmware	Arduino (C/C++)
Visualization	Processing (Java-based)
Communication	Serial (USB)


📈 Results
Reliable real-time data acquisition using ESP32
Clear visualization of gait phases (heel-strike → toe-off)
Accurate step detection using adaptive thresholds
Stable IMU-based orientation tracking

⚠️ Limitations
Yaw not computed (no sensor fusion)
Limited sensor density (4 FSR points)
Requires calibration for consistent readings
Single-foot prototype

Healthcare
Sports analytics
Rehabilitation
