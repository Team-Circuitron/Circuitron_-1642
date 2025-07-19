# Circuitron - WRO Future Engineers 2025
**Contemplate · Create · Conquer**

This repository contains details about the design, evolution, and continuous development of our self-driving robot vehicle, built and programmed by Team Circuitron for the Future Engineers 2025 category of the World Robot Olympiad (WRO).

---

## Member Details
1. Pranay Agarwal (Software)  
2. Aditya Parikh (Hardware)  
3. Paritra Gada (Hardware)

---

## Table of Contents

- `t-photos` : Contains 2 photos of the team, including an official and a funny photo  
- `v-photos` : Contains 6 photos of the vehicle from all sides, including top and bottom views  
- `video` : Contains a `video.md` file linking to a video showing the robot in action  
- `Hardware` : Includes hardware overview, design process, final design, and component choices  
- `Code` : Includes all source code controlling the vehicle's behavior  
- `Strategy` : Explanation of wall-following logic and decision-making approach  
- `models` *(optional)* : Files for 3D printing, laser cutting, or CNC (if applicable)  
- `other` *(optional)* : Supporting documents like connection instructions, datasets, specs  
- `special-thanks` : Acknowledgments and credits

---

## Hardware Overview

# Hardware Overview


---

## Table of Contents
1. [Hardware Overview](#hardware-overview)
2. [Processing Unit](#processing-unit)
3. [Vision System](#vision-system)
4. [Locomotion](#locomotion)
5. [Steering System](#steering-system)
6. [Power Management](#power-management)
7. [Chassis](#chassis)
8. [License](#license)

---

## Hardware Overview

Our robotic platform is engineered with a modular hardware architecture  
to deliver high performance, flexibility, and support iterative development cycles.

---

## Processing Unit

**Raspberry Pi 4 Model B**

- CPU: Quad-core Cortex-A72 (ARM v8)
- RAM: 2GB / 4GB / 8GB (depending on configuration)
- Interfaces: GPIO, USB, HDMI, CSI (camera)

This board supports seamless integration of sensor arrays and enables  
Real-time control algorithms essential for autonomous navigation and decision-making.

---

## Vision System

**Camera:** Raspberry Pi Camera Module

**Key Features:**
- Obstacle detection
- Line tracking
- Environmental mapping
- Native compatibility with the Raspberry Pi ecosystem

---

## Locomotion

**Wheels:** LEGO wheels for modularity and superior material quality  
**Motor:** [Insert Model] DC motor  

**Optimised for:**
- Size
- Weight
- Torque output
- Efficient mobility across diverse terrains

---

## Steering System

**Servo Motor:** [Insert Model] high-precision servo  

**Features:**
- Responsive and accurate control
- Reliable navigation in dynamic environments

---

## Power Management

**Buck Converter:** [Insert Model]  
**Input Voltage:** [Insert Voltage]V  

**Features:**
- Steps down the voltage to the required levels for all subsystems
- Ensures stable operation
- Protects sensitive electronics

---

## Chassis

**Framework:** LEGO-based chassis  

**Benefits:**
- Modularity for rapid iterations
- Ease of customisation
- Supports quick repositioning of components and continuous mechanical optimisation

---
## Design

## Final Design

## Advantages and Disadvantages
- Advantages and disadvantages of the overall design  
- Only the advantages of the final design

## Choice of Components

## Math

## Strategy
Rather than using the ultrasonic sensor, we have decided to use the Raspberry Pi camera, utilising the contours of the walls to execute a wall-following program. When the robot detects an obstacle, it uses that as a reference to wall-follow between the wall and the obstacle. For turning, we use the blue or orange lines on the game field to take a turn, and if the next obstacle is too close, we will take a dummy reverse and resume our wall-following code.

## Code

## Special Thanks
