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
📊 Data Format (ESP32 Output)

The ESP32 streams data at 10 Hz in CSV format:

ts_ms,heel,arch,fore,toe,pitch_deg,roll_deg,yaw_deg

Example:

1023,120,300,500,200,10.5,-3.2,0.0
⚙️ Setup Instructions
🔹 1. ESP32 Firmware
Open Arduino IDE
Select board: ESP32C3 Dev Module
Upload firmware from:
/firmware/esp32_gait_analysis/
🔹 2. GUI (Processing)
Install Processing IDE
Open:
/gui/heatmap/heatmap.pde
Set correct serial port:
String PORT_NAME = "COM3";
Run ▶️
🎮 Controls (GUI)
Key	Function
R	Toggle reference image
G	Toggle draggable handles
1–4	Select sensors (Toe → Heel)
S	Save foot shape
L	Load saved shape
[ / ]	Adjust overlay opacity
📷 Output Visualization
Heatmap showing pressure distribution
Roll bar (pronation/supination)
Pitch-based gait phase hints
Step count & cadence
Center of Pressure (COP) marker
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
🔮 Future Scope
🤖 Machine Learning for gait classification
📱 Mobile app using BLE/WiFi
☁️ Cloud data storage & analytics
👟 Dual-foot system for symmetry analysis
🔄 Sensor fusion (Madgwick/Mahony filter)
👨‍💻 Contributors
Suraj Kalyanaraman
Team Members
⭐ Why this project stands out
Combines embedded systems + biomechanics + visualization
Fully real-time system (hardware → software pipeline)
Practical applications in:
Healthcare
Sports analytics
Rehabilitation
