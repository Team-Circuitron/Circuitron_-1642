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
# Autonomous Robot Navigation Strategy

A navigation strategy for an autonomous robotics platform utilizing camera vision, PID control, and multi-sensor integration.

---


##  Core Strategy

### 1. Initialization
- Start the system and initialize `Round_Count = 0`.
- Check if `Round_Count >= 24`. 
  - **If YES:** Terminate the process.
  - **If NO:** Proceed to data collection.

### 2. Data Collection
- Capture real-time camera data to analyze the surroundings.
- Detect environmental features (walls, objects, magnets, lines).

### 3. Object and Obstacle Handling
- **Red/Green Object Detection:**
  - If detected, determine object positions relative to the robot.
  - For a single object: Register position (left/right) and adjust path.
  - For two objects:
    - Compute the distance between them.
    - **If gap feasible:** Navigate between objects using reverse PID.
    - **If gap not feasible:** Trigger a pre-set parking algorithm.
- **Magnet Detection:**
  - If magnets are detected, apply PID logic to center the robot.

### 4. Wall Following
- **Black Wall Detection:**
  - **If walls detected:** Apply PID logic to center between two walls.
  - **If no walls detected:** Apply PID to follow a single wall, keeping error = 0.

### 5. Navigation and Control
- Continuously use PID to:
  - Maintain drive system stability.
  - Adjust direction dynamically.
  - Ensure smooth motor operation during transitions.

### 6. Line Detection and Round Management
- Use color sensor to detect orange/blue lines (checkpoints).
- Upon detection:
  - **If value detected:** Keep `Round_Count` unchanged; keep sensor active.
  - **If no value detected:** Increment `Round_Count` by 1 and disable sensor for a fixed time.

---

##  Key Algorithms

1. **PID Wall Following:**
   Maintains robot alignment with reference walls or center path.
2. **Reverse PID Obstacle Navigation:**
   Used for moving between detected objects or obstacle pairs.
3. **Round Counter Management:**
   Tracks progress and manages sensor activation.

---

##  Termination
The system stops once `Round_Count >= 24`, signaling the completion of all operational rounds.

---

##  Notes
This strategy was developed to ensure robust navigation and adaptability to dynamic environments during autonomous operation.

## Code

## Special Thanks
