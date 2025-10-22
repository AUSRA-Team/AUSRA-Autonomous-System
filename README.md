# AUSRA-Autonomous-System

This repository contains the complete ROS 2 software stack for the **AUSRA (Autonomous Unified Swarm Robotics Architecture)** project.

The system implements a high-level/low-level architecture. A **Jetson Orin Nano** runs ROS 2 Humble for high-level tasks like Perception, SLAM, and Navigation, while an **ESP32-S3** runs micro-ROS for real-time motor control and hardware interfacing.

## System Architecture

![AUSRA System Architecture](Docs/system_architecture.png)