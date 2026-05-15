# Real-Time IMU Sensor Fusion on a Microcontroller
### A Comparative Study Using Madgwick Orientation Filtering

This repository contains the full implementation, experimental data, analysis scripts, and report for a real-time IMU sensor fusion project developed as part of an academic project for the **Embodied Intelligence Club** at the **University of Patras**, Greece.

---

## Overview

The project implements the **Madgwick gradient-descent orientation filter** in three configurations and evaluates their performance through a comparative study:

- **6-axis IMU on LSM6DS3TR** — accelerometer + gyroscope only, running on the Arduino's onboard sensor
- **6-axis IMU on Xsens MTi-3** — same filter configuration but on the industrial-grade module, with magnetometer disabled
- **9-axis MARG on Xsens MTi-3** — full configuration incorporating the magnetometer for complete orientation estimation including heading

All filter implementations run in **real time on an Arduino Uno WiFi Rev2**, with orientation estimates streamed over serial to MATLAB for live visualisation and logging.

---

## Hardware
- **Arduino Uno WiFi Rev2** — central processing unit, runs the Madgwick filter
- **LSM6DS3TR** (onboard Arduino) — 6-axis IMU (accelerometer + gyroscope)
- **Xsens MTi-3-5A-T** — 9-axis MARG (accelerometer + gyroscope + magnetometer)

---

## What Was Done

- Implemented the Madgwick filter in Arduino C++ for both IMU and MARG configurations
- Interfaced the Xsens MTi-3 with the Arduino via UART
- Characterised raw sensor noise and bias for both sensor platforms
- Compared the three filter configurations outputs
- Performed systematic β tuning (convergence speed vs steady-state noise tradeoff)
- Performed ζ tuning (gyroscope bias estimation convergence rate)
- Investigated filter performance at 10, 25, 50, and 100 Hz update rates
- Implemented a real-time 3D orientation viewer in MATLAB using MathWorks helper classes

---

## Video Demo

A live demonstration of the real-time orientation viewer running on the Xsens MTi-3 MARG implementation can be viewed [here](https://www.youtube.com/watch?v=QnyklZCOkVg).
