# ESC EKF Fusion + Safety Supervisor

### Electronic Stability Control Simulation in C

This project implements a realistic simulation of an **Electronic Stability Control (ESC)** unit using:

- Multi-sensor fusion
- Extended Kalman Filter (EKF)
- Sensor health monitoring
- Traction and skid detection
- Safety Supervisor with Safe Mode
- Human-readable reporting

The goal is to estimate the car’s dynamic state, detect skids, and trigger ESC Safe Mode when necessary.

---

# ⚙️ Features

- Reads 5 automotive sensors:
  - Gyroscope (yaw rate)
  - Wheel speed
  - Lateral accelerometer
  - Vision curvature (lane-based)
  - GPS curvature
- Automatic sensor fault detection (timeout, stuck, invalid)
- EKF-based state estimation (heading, speed, curvature)
- Lateral force computation & skid prediction
- Safe Mode activation for:
  - Sensor failures
  - Large inconsistencies
  - Watchdog timeouts
  - Near-skid conditions
- Human-readable logs saved to `esc_report.txt`
- Live console summaries during simulation

---

# 📁 Project Structure

| File                        | Description                    |
| --------------------------- | ------------------------------ |
| `ESC_Traction_Simulation.c` | Main C source file             |
| `demo_sensors.csv`          | Example dataset for batch mode |
| `esc_report.txt`            | Log file (auto-generated)      |

---

# 🛠 Requirements

- **GCC compiler**
- **Linux / Ubuntu / WSL** (recommended)
- No external libraries needed

Check if GCC is installed:

```bash
gcc --version
```

---

# ▶️ Compilation

Open a terminal in the project folder and run:

```bash
gcc -o esc_sim ESC_Traction_Simulation.c -lm
```

This generates the executable:

```
esc_sim
```

---

# 🚀 Running the Program

Run:

```bash
./esc_sim
```

You will see:

```
Menu:
  1) Interactive (simulate sensors)
  2) Batch CSV
  3) View report tail
  4) Exit
```

---

# 1️⃣ Interactive Mode

Choose:

```
1
```

Enter sensor values manually:

```
gyro accel wheel vision_kappa vision_conf gps_kappa gps_conf abs_flag
```

Example:

```
0.1 0.5 12 0.03 0.8 0.02 0.7 0
```

You may enter `-1` for missing data:

```
-1 0.3 11 -1 -1 0.01 0.6 0
```

Stop with:

```
Continue? (y/n): n
```

---

# 2️⃣ Batch CSV Mode

Choose:

```
2
```

Provide the dataset (included):

```
CSV path: demo_sensors.csv
```

Choose road condition:

```
Road condition (1 Dry,2 Wet,3 Snow,4 High-grip): 2
```

Output example:

```
t=0.25s | v=1.46 m/s | R=66.86 m | a_lat=0.03 | usage=0.01 | safe_mode=NO
```

At the end, the program auto-displays the last part of the log.

---

# 3️⃣ View Report Tail

Choose:

```
3
```

This prints the latest section of:

```
esc_report.txt
```

To view the whole file manually:

```bash
cat esc_report.txt
```

---

# 📄 Output: esc_report.txt

The report includes:

- Timestamp
- Sensor availability
- Sensor confidence
- EKF state (psi, v, kappa)
- Turn radius
- Lateral acceleration
- Traction usage (%)
- Whether ESC Safe Mode is active
- Reason for Safe Mode
- Sensor fault status
- Innovation diagnostics

This file is automatically appended with every timestep.

---

# 📘 How the System Works (Short Explanation)

1. **Sensor Health Check**  
   Detects missing, stuck, or faulty sensors and disables them.

2. **EKF Prediction**  
   Predicts heading, speed, and curvature.

3. **EKF Update**  
   Fuses all healthy sensors to refine state estimation.

4. **Skid Detection**  
   Calculates lateral acceleration and compares it to tyre grip.

5. **Safe Mode Activation**  
   Triggered when sensors fail or skid likelihood is high.

6. **Logging**  
   A human-readable diagnostic block is appended to `esc_report.txt`.

7. **Live Summary**  
   One-line status updates are printed for each timestep.

---

# ❗ Troubleshooting

### Recompile:

```bash
gcc -o esc_sim ESC_Traction_Simulation.c -lm
```

### Ensure CSV file is in same folder:

```bash
ls
```

### Move file if needed:

```bash
mv demo_sensors.csv .
```

---

# 🎉 You're Ready to Use the ESC Simulator

For questions or debugging, check:

- `esc_report.txt`
- Terminal summary lines
- Code comments inside the `.c` file

---

If you'd like a **GitHub-optimized README**, **PDF version**, or **auto-generated Doxygen-style documentation**, just ask!
