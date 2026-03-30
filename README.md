# NXP Line Follower - Hackathon 2026

This repository contains the control logic and hardware configuration for an autonomous line-following robot, developed during the NXP Hardware Hackathon held at the University of Bucharest.

🏆 **Result:** 9th Place out of 16 teams  
⏱️ **Best Lap Time:** 20.6 seconds

## 📸 Project Showcase

<p align="center">
  <a href="https://www.youtube.com/watch?v=XaZLE-19FP8">
    <img src="https://img.youtube.com/vi/XaZLE-19FP8/0.jpg" alt="NXP Line Follower Demo" width="600">
  </a>
</p>

## ⚙️ Hardware Overview
Based on the official NXP Cup Line Follower kit, the robot was assembled and programmed using the following components:
* **Microcontroller:** NXP FRDM-MCXN947 Development Board (Arm Cortex-M33 @ 150 MHz)
* **Custom Shield:** NXPCUP-Shield-FRDM-MCXN947
* **Sensors:** 8x Infrared reflective sensors with LM339 comparators (Digital 0/1 output)
* **Actuators:** 2x DC Geared Motors (25GA-370) driven by 2x DRV8833 Dual H-Bridge motor drivers
* **Power Management:** 7.4V 2200mAh 2S LiPo battery regulated to 5V via MP1584EN step-down converter

## 🧠 Control System & Logic
The robot's firmware is implemented in bare-metal C. It utilizes a combination of digital signal processing and control theory to maintain high speeds while ensuring stability.

### 1. Position Estimation (Weighted Average)
Instead of using simple binary states, the algorithm processes the 8-bit sensor array into a continuous error value:
* **Weighted Sum:** Each sensor is assigned a weight (0 to 7000). The `calculate_position` function computes a weighted average of all active sensors to determine the line's center.
* **Granularity:** This allows for precise error calculation even when the line is between two sensors, enabling smoother steering corrections.

### 2. Steering Control (PD Loop)
A Proportional-Derivative (PD) controller is used to manage the robot's heading:
* **Proportional ($K_p$):** Corrects the current error relative to the center.
* **Derivative ($K_d$):** Predicts future error by calculating the rate of change (`delta_error`), effectively dampening oscillations and preventing oversteering at high speeds.

### 3. Slew Rate Limiter & Traction Control
To maintain physical stability and prevent wheel slip:
* **Acceleration Ramp:** The `actual_base_speed` does not jump instantly to the target. It increments by a fixed `ACCEL_STEP` every cycle (2ms), mimicking a **Slew Rate Limiter**.
* **Instant Deceleration:** While acceleration is gradual to maintain traction, the system allows for near-instant power cuts when a sharp curve or emergency braking is required.

### 4. Predictive Active Braking
The system "anticipates" sharp corners by monitoring the derivative of the error:
* **Brake Penalty:** A `Kd_Brake` multiplier is applied to the target speed based on the magnitude of change in error (`abs_delta`). 
* **Outcome:** The robot automatically slows down before entering a sharp turn and accelerates back to `MAX_SPEED` on straights.

### 5. Recovery Logic (Blind Pivoting)
If the line is completely lost (e.g., during an aggressive maneuver):
* **Memory-Based Recovery:** The robot remembers the `last_position` before the loss.
* **Pivot Search:** It performs a stationary pivot at `PIVOT_SPEED` in the direction where the line was last seen until the sensor array re-acquires the track.

## 🚀 Repository Structure
* `source/` - Contains the main control logic (`main.c`) and configuration headers.
* `nxpcup-linefollower.mex` - The NXP Config Tools file detailing pin routing and clock setups.

## 📚 Resources & Documentation
* [NXP Cup Line Follower Official Docs](https://gitfront.io/r/cristiannxp/xHvo63TFm7h4/NXP-Linefollower/) - The technical reference, hardware schematics, and SDK guidelines provided by NXP used to assemble the kit and develop the control algorithms.
