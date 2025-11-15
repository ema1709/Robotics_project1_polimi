# Robotics Project 1 – Politecnico di Milano  
ROS-based Odometry, GPS Fusion, and Sector Timing

This repository contains the implementation of the **first robotics project** for the Robotics course at Politecnico di Milano.  
The task is to develop three ROS nodes able to extract odometry from vehicle data, compute odometry from GPS, and determine lap sectors from telemetry.

The project was implemented in **C++** using **ROS (Noetic)** and tested on the provided **ROS bag file** (`project.bag`).

---

## 🚗 Project Overview

The goal of the project is to reconstruct the motion of a differential-steering vehicle using two independent data sources:

1. **Vehicle telemetry (/speedsteer)**  
2. **Front GPS receiver (/swiftnav/front/gps_pose)**  

and then compute:

- **Odometer-based odometry**
- **GPS-based odometry in ENU frame**
- **Sector times & mean speed**

All nodes are launched from a single `launch.launch` file and visualized in **RViz** with TF frames and odometry arrows.

---

## 🧱 Repository Structure

Robotics_project1_polimi/
│
├── first_project/ # ROS package
│ ├── src/
│ │ ├── odometer.cpp # Node 1: odometry from vehicle data
│ │ ├── gps_odometer.cpp # Node 2: ENU odometry from GPS
│ │ ├── sector_times.cpp # Node 3: sector timing
│ │
│ ├── msg/
│ │ └── sector_times.msg # Custom message definition
│ │
│ ├── launch/
│ │ └── launch.launch # Full project launcher
│ │
│ ├── CMakeLists.txt
│ ├── package.xml
│
└── README.md
