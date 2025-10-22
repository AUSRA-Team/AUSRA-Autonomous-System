# AUSRA-Autonomous-System

[cite_start]This repository contains the complete ROS 2 software stack for the **AUSRA (Autonomous Unified Swarm Robotics Architecture)** project[cite: 29].

The system implements a high-level/low-level architecture. [cite_start]A **Jetson Orin Nano** [cite: 30] [cite_start]runs ROS 2 Humble for high-level tasks like Perception, SLAM, and Navigation [cite: 4, 31, 98][cite_start], while an **ESP32-S3** [cite: 61, 119] [cite_start]runs micro-ROS for real-time motor control and hardware interfacing[cite: 64, 65].

## System Architecture

![AUSRA System Architecture](docs/system_architecture.png)