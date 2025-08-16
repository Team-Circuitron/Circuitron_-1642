# HARDWARE OVERVIEW
Overview of the hardware used in the robot

## 📑 Table of Contents
- [Overview](#overview)
- [Hardware Overview](#hardware-overview)
  - [Processing Unit](#processing-unit)
  - [Locomotion](#locomotion)
  - [Motor Driver](#motor-driver)
  - [Power Management](#power-management)
  - [Sensors](#sensors)
  - [Camera System](#camera-system)
  - [User Interface](#user-interface)
- [Dimensions & Weight](#dimensions--weight)
- [Advantages](#advantages)


---

## Overview
Our robot is built to balance modularity, durability, and efficiency for real-world robotics challenges. The system integrates advanced hardware with a clean software stack to ensure smooth operation, quick iteration, and easy debugging during competitions.  

---

## Hardware Overview

### Processing Unit
- **Raspberry Pi Zero**  
Acts as the brain of the robot, managing camera input, motor control, and sensor integration.  
Compact yet powerful enough for real-time decision-making.  

### Locomotion
- **Custom DC Motor (8000 rpm at 3.7v)**  
- **Coupled with TB6612FNG motor driver**  
Provides precise torque and control for dynamic movement in competition.  

### Motor Driver
- **TB6612FNG Dual H-Bridge Driver**  
Handles motor currents efficiently while minimising heat.  
Supports smooth acceleration and braking.  

### Power Management
- **Buck Converter (5V, 3A)**  
Ensures stable voltage delivery to sensitive components such as Raspberry Pi, IMU, and sensors.  

### Sensors
- **Limit Switch**  
Used for collision detection
- **9-axis IMU (Gyroscope + Accelerometer + Magnetometer)**  
Provides orientation and stability feedback.  

### Camera System
- **Raspberry Pi Camera Module**  
Essential for wall detection, object detection, and parking.  

### User Interface
- **Push Buttons**  
Allow reset or emergency stop functionality.  

---

## Dimensions & Weight
- **Weight:** ~650 g  
- **Dimensions:** ~16 cm (L) × 12 cm (W) × 10 cm (H)

Compact enough for agility, yet stable for high-speed operation.  

---

## Advantages
- Lightweight and portable for quick field setup.  
- Modular hardware design for easy replacements.  
- Reliable motor driver ensures consistent performance.  
- Raspberry Pi Zero offers a balance of performance and size.  
- Stable 5V 3A power system prevents brownouts.  
- IMU provides accurate feedback for autonomous navigation.  
- Camera enables vision-based strategies.  
- User-friendly with push buttons for control.  
- Collision safety with built-in limit switch.  

