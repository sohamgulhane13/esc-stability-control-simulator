# Electronic Stability Control (ESC) Simulator

This project simulates the behavior of Electronic Stability Control (ESC) systems used in modern vehicles to prevent skidding and maintain vehicle stability.

## Overview

Electronic Stability Control (ESC) is an important automotive safety feature that detects loss of traction and automatically applies braking to individual wheels to maintain vehicle control.

This project demonstrates how ESC systems detect instability and correct vehicle motion using vehicle dynamics models.

## Features

- Vehicle dynamics simulation using Bicycle Model
- Slip ratio and lateral acceleration detection
- Understeer and oversteer identification
- Wheel-specific braking correction
- Real-time visualization through web dashboard

## Methodology

The project is based on three major components:

### 1. Vehicle Dynamics Model
- Bicycle model for lateral vehicle motion
- Linear tire model for cornering forces
- Yaw rate and slip angle calculation

### 2. ESC Detection Logic

Skid conditions detected using:

- Lateral acceleration > 0.8g
- Slip ratio > 0.25
- Body slip angle > 3°
- Yaw rate mismatch > 15%

### 3. Brake Intervention

ESC determines instability type:

- **Oversteer → brake outside front wheel**
- **Understeer → brake inside rear wheel**

This stabilizes the vehicle and prevents loss of control.

## Technologies Used

- Python
- Vehicle Dynamics Modeling
- HTML/CSS/JavaScript
- Data Visualization

## Applications

- Driver safety systems
- Automotive algorithm testing
- Driver training simulators
- Autonomous vehicle research

## Future Improvements

- Implement Pacejka Magic Formula tire model
- Add ABS and Traction Control integration
- 3D simulation environment
- Hardware-in-loop vehicle testing

## Team Members

- Manas Kotian
- Sujit Nirmal
- Soham Gulhane
- Aadesh Khamkar

Guide: **Prof. Usha Jadhav**
