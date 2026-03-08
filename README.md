

# 🚗 ESC EKF Fusion + Safety Supervisor

### Electronic Stability Control Simulation in C

This project implements a **realistic Electronic Stability Control (ESC) simulator** using **multi-sensor fusion and an Extended Kalman Filter (EKF)** to estimate vehicle dynamics, detect skid conditions, and activate **ESC Safe Mode** when stability risks occur.

The simulator demonstrates how modern automotive safety systems combine **sensor data, filtering algorithms, and safety supervision logic** to maintain vehicle stability.

---

# 📌 Project Overview

Electronic Stability Control (ESC) is a critical automotive safety system that prevents loss of control during sharp turns or slippery road conditions.

This project simulates an ESC unit capable of:

* Fusing multiple vehicle sensors
* Estimating vehicle motion using EKF
* Detecting potential skid conditions
* Monitoring sensor health
* Activating Safe Mode during instability
* Logging diagnostic reports

The implementation models **core logic used in automotive Electronic Control Units (ECUs)**.

---

# ⚙️ Key Features

✔ Multi-sensor fusion for vehicle state estimation
✔ **Extended Kalman Filter (EKF)** implementation
✔ Sensor fault detection and health monitoring
✔ Skid detection using traction analysis
✔ ESC **Safe Mode activation** for critical conditions
✔ Human-readable diagnostic logging
✔ Interactive and batch simulation modes
✔ Optional **MQTT telemetry publishing**

---

# 🧠 Sensors Used in Simulation

The system processes data from simulated automotive sensors:

| Sensor                | Purpose                                     |
| --------------------- | ------------------------------------------- |
| Gyroscope             | Measures yaw rate                           |
| Wheel Speed           | Determines vehicle velocity                 |
| Lateral Accelerometer | Measures sideways acceleration              |
| Vision Curvature      | Road curvature estimate from lane detection |
| GPS Curvature         | Road curvature estimate from GPS            |

These sensors are fused to estimate:

* vehicle heading
* velocity
* turning curvature

---

# 🧠 System Pipeline

```
Sensor Inputs
      ↓
Sensor Health Monitoring
      ↓
Extended Kalman Filter
      ↓
Vehicle State Estimation
      ↓
Skid Detection
      ↓
Safety Supervisor
      ↓
ESC Safe Mode
```

---

# 📁 Project Structure

```
esc-stability-control-simulator
│
├── ESC_Traction_Simulation.c
├── mqtt_publisher.c
├── demo_sensors.csv
├── MakeFile
├── README.md
├── .gitignore
└── paho.mqtt.c
```

| File                      | Description                       |
| ------------------------- | --------------------------------- |
| ESC_Traction_Simulation.c | Main ESC simulation program       |
| mqtt_publisher.c          | Optional MQTT telemetry publisher |
| demo_sensors.csv          | Example dataset for batch testing |
| MakeFile                  | Build configuration               |
| paho.mqtt.c               | Eclipse Paho MQTT client library  |

---

# 🛠 Requirements

* GCC Compiler
* Linux / Ubuntu / WSL recommended
* No external dependencies required

Check GCC installation:

```bash
gcc --version
```

---

# ▶️ Compilation

Compile the simulator:

```bash
gcc -o esc_sim ESC_Traction_Simulation.c -lm
```

This generates the executable:

```
esc_sim
```

---

# 🚀 Running the Simulation

Run the program:

```bash
./esc_sim
```

Menu options will appear:

```
1) Interactive sensor simulation
2) Batch CSV simulation
3) View report tail
4) Exit
```

---

# 1️⃣ Interactive Mode

Manually simulate sensor inputs.

Example input:

```
gyro accel wheel vision_kappa vision_conf gps_kappa gps_conf abs_flag
```

Example values:

```
0.1 0.5 12 0.03 0.8 0.02 0.7 0
```

Missing sensors can be simulated using:

```
-1
```

---

# 2️⃣ Batch CSV Mode

Run simulation using recorded data.

Example:

```
CSV path: demo_sensors.csv
Road condition (1 Dry,2 Wet,3 Snow,4 High-grip): 2
```

Example output:

```
t=0.25s | v=1.46 m/s | R=66.86 m | a_lat=0.03 | usage=0.01 | safe_mode=NO
```

---

# 📄 Diagnostic Output

Simulation logs are written to:

```
esc_report.txt
```

Each log entry includes:

* timestamp
* sensor availability
* EKF state estimates
* traction usage
* stability status
* Safe Mode activation reason
* innovation diagnostics

---

# 📡 MQTT Telemetry (Optional)

The project includes an MQTT publisher for streaming ESC telemetry.

This allows simulation data to be sent to:

* remote dashboards
* vehicle telemetry systems
* IoT monitoring tools

Uses **Eclipse Paho MQTT C client**.

---

# 🎯 Learning Outcomes

This project demonstrates practical understanding of:

* Automotive safety systems
* Vehicle dynamics modeling
* Multi-sensor fusion
* Extended Kalman Filters
* Embedded control system design
* Real-time safety supervision

---

# 🔮 Future Improvements

Possible enhancements include:

* Integration with **CAN bus vehicle data**
* Real-time graphical vehicle simulation
* Advanced tire friction models
* Hardware deployment on embedded controllers
* Integration with **ADAS modules**

---

# 👨‍💻 Contributors

* Manas Kotian
* Sujit Nirmal
* Soham Gulhane
* Aadesh Khamkar

Guide: **Prof. Usha Jadhav**

---

# ⭐ Support

If you found this project useful, consider **starring the repository ⭐**

---

# Clone with Submodules

This project includes the **Eclipse Paho MQTT library**.

Clone using:

```bash
git clone --recurse-submodules https://github.com/sohamgulhane13/esc-stability-control-simulator.git
```

---


