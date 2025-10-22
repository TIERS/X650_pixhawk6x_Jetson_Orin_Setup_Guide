# X650 Pixhawk 6X Jetson Orin Setup Guide with Fast Lio



## Overview

A guide for setting up a high-performance autonomous drone using the X650 frame, Pixhawk 6X flight controller, and Jetson Orin NX computing platform, with livox mid360 and realsense d435. This setup represents one of the most popular and powerful configurations for custom-built multicopter drones, combining robust hardware with advanced autonomous capabilities. 

More importantly, we also offer detailed instructions to set up LIO algothrims.

## Hardware and Software Components

### Hardware Components:
- **Holybro Pixhawk 6X** - Advanced flight controller
- **Jetson Orin NX 16GB** - Edge AI computing unit
- **Holybro Jetson Baseboard** - Integration board for Jetson
- **Holybro X650** - Carbon fiber quadcopter frame
- **Livox MID360** - 3D LiDAR sensor for SLAM or data collection
- **Holybro RTK F9P Bundle** - High-precision GPS with RTK capabilities (optional)
- **realsense D435** - Stereo camera for stereo vision or data collection

## Software Components and Core Systems:

- **JetPack 5.1.2**
- **ROS1 Noetic**
- **PX4:1.15.4**
- **QGC:4.4.4**
- **MAVROS**
- **LIO (LiDAR-Inertial Odometry):Fast-LIO**
- **MAVSDK**



## Purpose of This Repository

When setting up this drone configuration, I discovered that the official documentation lacks sufficient detail and contains numerous pitfalls that can cause significant delays. This repository aims to:

- Provide detailed summary notes from practice and official documents, so easy to understand and follow the important steps.
- Highlight common pitfalls and how to avoid them, so save time for others undertaking similar projects

## How to Use This Guide

1. **Read Official Documentation First** - Make sure to read the official docs carefully, notice this repo cannot replace the official documentation. You still need to study the official documentation to get the full understanding of different concepts and components.
2. **Follow This Guide** - Use this repository to understand potential issues and best practices
3. **Combine Both Resources** - Cross-reference official docs with this guide for a complete understanding

## Repository Structure

- **`README.md`** - readme file
- **`note.md`** - Main documentation file
- **`lio`** - LIO related settings and files
- **`own_notebook/`** - Personal notes (contains mixed Chinese/English content, no need to read this one)
- **`images/`** - Supporting images and diagrams

