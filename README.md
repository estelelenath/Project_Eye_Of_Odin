# Project Eye of Odin

**Project Eye of Odin** is an advanced robotics platform powered by NVIDIA Jetson devices, seamlessly merging real-time sensor data (camera, LiDAR) with AI/SLAM algorithms and a flexible web-based dashboard. By leveraging ZeroMQ and WebSockets, it handles high-throughput imaging and point clouds for efficient operator oversight, 3D environment mapping, and minimal-latency teleoperation—even across multiple robots. Our ultimate vision is a **“linked integrated information process system”** that combines AI-driven modules (e.g., object detection, 3D segmentation, multi-SLAM, obstacle detection), 5G connectivity for wide-range collaboration and teleoperation and Cloud integration to store and analyze large-scale sensor logs over time. **Project Eye of Odin**, it aims to build a unified ecosystem that empowers users to **monitor, analyze, and command** robots across all in a single "window".

---

## Table of Contents

1. [Introduction](#introduction)  
2. [System Architecture](#system-architecture)  
   1. [ROS2 Nodes](#ros2-nodes)  
   2. [Odin Bridge (ROS2 → ZMQ)](#odin-bridge-ros2--zmq)  
   3. [ZMQ Bridge (ZMQ → WebSocket)](#zmq-bridge-zmq--websocket)  
   4. [Observer (FastAPI + Dashboard)](#observer-fastapi--dashboard)  
3. [Features](#features)  
4. [Prerequisites & Setup](#prerequisites--setup)  
5. [How to Run](#how-to-run)  
6. [Detailed Implementation](#detailed-implementation)  
   1. [CSI/USB Camera Nodes](#csiusb-camera-nodes)  
   2. [LiDAR Node](#lidar-node)  
   3. [Odin Bridge Implementation](#odin-bridge-implementation)  
   4. [ZMQ Bridge & WebSocket Communication](#zmq-bridge--websocket-communication)  
   5. [FastAPI Server & Dashboard](#fastapi-server--dashboard)  
7. [Performance Considerations](#performance-considerations)  
8. [Future Work & Extensions](#future-work--extensions)  
   1. [Multiple Cameras and AI Processing](#multiple-cameras-and-ai-processing)  
   2. [Multiple Jetsons](#multiple-jetsons)  
   3. [Jetson Device State Updates](#jetson-device-state-updates)  
   4. [Maps & SLAM Integration](#maps--slam-integration)  
   5. [5G Internet Connection](#5g-internet-connection)  
   6. [Cloud Integration](#cloud-integration)  
9. [Folder Structure](#folder-structure)  
10. [License](#license)  
11. [References & Further Reading](#references--further-reading)

---

## Introduction

**Project Eye of Odin** focuses on **real-time data acquisition** from multiple Jetson-based sensors (cameras and LiDARs). It provides a robust pipeline to stream these sensor feeds to a remote or local dashboard via **ZeroMQ** and **FastAPI WebSocket**. We aim to incorporate AI-based object detection, 3D segmentation, multi-SLAM, obstacle detection, and 5G-based wide-range operations in the near future, culminating in an integrated command-and-control system for advanced robotics.

---

## System Architecture

The project consists of four layers:

1. **ROS2 Nodes**: Handle direct sensor interactions (camera, LiDAR), publishing data as ROS topics.  
2. **Odin Bridge**: Subscribes to those ROS2 sensor topics and republishes them via **ZeroMQ** (PUB).  
3. **ZMQ Bridge**: Subscribes to ZeroMQ (SUB) messages, caches them, and forwards them to **WebSocket** connections.  
4. **Observer (FastAPI)**: Hosts the **dashboard** (HTML/CSS/JS) and provides REST/WebSocket endpoints.

A simplified overview:
```bash
[Sensors: CSI Camera, USB Camera, LiDAR] --(ROS2)--> [ROS2 Nodes] --(ROS2 Topics)--> [Odin Bridge] --(ZeroMQ PUB)--> [ZMQ Bridge (ZeroMQ SUB)] --(WebSocket)--> [FastAPI] --> [Browser Dashboard]
```


### ROS2 Nodes
- **Camera Nodes**: `csi_camera_node.py`, `usb_camera_node.py`
- **LiDAR Node**: `sllidar_node.cpp`

### Odin Bridge (ROS2 → ZMQ)
- **bridge_node.py**:  
  - Subscribes to camera & LiDAR ROS topics.  
  - Publishes them via ZeroMQ at `tcp://*:5555`.  
  - `*` means it listens on all interfaces for inbound connections.

### ZMQ Bridge (ZMQ → WebSocket)
- **zmq_bridge.py**:  
  - Connects to `tcp://127.0.0.1:5555` as SUB (assuming same machine).  
  - Maintains data caches and rate limits.  
  - Pushes data to WebSocket clients.

### Observer (FastAPI + Dashboard)
- **main.py**:  
  - Runs on `0.0.0.0:8000` so external devices can connect too.  
  - `/ws/camera`, `/ws/lidar` for streaming.  
  - `/api/status` for Jetson info, connected clients, etc.
- **dashboard.html + main.js**:  
  - Real-time camera feed, LiDAR plot, system status, placeholders for map or advanced analysis.

---

## Features

- **Multi-Camera Support** with auto-detection (CSI/USB).  
- **LiDAR Integration** with 2D or future 3D visualization.  
- **ZeroMQ** pipeline for compressed images and JSON-based LiDAR data.  
- **FastAPI** for the web UI, with a minimal resource footprint and high concurrency.  
- **Scalable** to multiple Jetsons, multiple sensors, optional AI inference pipelines.

---

## Prerequisites & Setup

- **OS**: Ubuntu 22.04 (ROS2 Humble recommended)  
- **Dependencies**:  
  - `ros2` (Humble), `colcon build`, `opencv-python`, `fastapi`, `uvicorn[standard]`, `jinja2`, `aiofiles`, `pyzmq`, `plotly` (client-side)  
- For more specific instructions, please see **Project Deploy** documentation.

---

## How to Run

0. **Build & Source**  
   ```bash
   cd ~/ws/Project_Eye_Of_Odin
   colcon build --symlink-install
   source install/setup.bash
    ```

1. **Launch Sensors**  
   ```bash
   ros2 launch project_eye_of_odin project_eye_of_odin.launch.py
    ```

2. **Run Odin Bridge**  
   ```bash
   ros2 launch odin_bridge odin_bridge.launch.py jetson_ids:='["001"]'
    ```
- To add more Jetsons: jetson_ids:='["001","00A"]'

3. **Start FastAPI Observer**  
   ```bash
   cd src/observer
   uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
    ```

4. **Open Browser**  
   ```bash
   ocal: http://127.0.0.1:8000
   LAN/Network: http://<device_ip>:8000
    ```

---

## Detailed Implementation

The project consists of four layers:

### CSI/USB Camera Nodes
- Publish images to `/jetson_XXX/cameras/csi_0/image_raw` or `/jetson_XXX/cameras/usb_0/image_raw`.
- GStreamer-based pipelines for hardware acceleration on Jetson.

### LiDAR Node
- `sllidar_node.cpp` sends `sensor_msgs/LaserScan` to `/jetson_XXX/lidar/scan`.
- Example: SLLIDAR S2 or C1.

### Odin Bridge Implementation
- `bridge_node.py`:  
  - Subscribes to camera & LiDAR ROS topics.
  - Compresses images (JPEG), serializes LiDAR data (JSON).
  - Publishes via ZeroMQ `(tcp://*:5555)`.

### ZMQ Bridge & WebSocket Communication
- `zmq_bridge.py`:  
  - SUB to `tcp://127.0.0.1:5555`.
  - Rate-limits camera frames (10 FPS) and LiDAR (5 FPS).
  - WebSocket broadcast to `/ws/camera` or `/ws/lidar` clients.

### FastAPI Server & Dashboard
- `main.py`:  
  - On startup, `bridge.start()` (async).
  - `/ws/camera`, `/ws/lidar` for real-time streams.
  - `/api/status` for system data (Jetson IDs, etc.).

- `dashboard.html + main.js`:
  - TailwindCSS layout, Plotly for LiDAR polar plots, `<img>` for camera feeds.
  - Future expansions: AI overlays, multi-SLAM maps, obstacle detection warnings.

---

## Performance Considerations
- ZeroMQ is used instead of standard rosbridge_suite for efficiency in streaming large images.
- JPEG compression significantly reduces bandwidth for real-time camera feeds.
- LiDAR scans (JSON arrays) remain relatively small but can be further compressed if needed.

---

## Future Work & Extensions

### Multiple Cameras and AI Processing
- Dynamically manage camera feeds in the UI if more than 4 exist.
- Incorporate AI (e.g., YOLO, 3D segmentations) on Jetson or central server.
- Potential to overlay bounding boxes or semantic masks in real-time.

### Multiple Jetsons
- Handle multiple devices via jetson_ids parameter.
- Each device can have multiple sensors; data merges on the centralized dashboard.
- Potential for multi-robot orchestration or fleet management.

### Jetson Device State Updates
- CPU load, GPU usage, temperature, battery, etc., published from ROS2 → ZeroMQ → WebSocket.
- Present these stats in the “System Status” panel.

Maps & SLAM Integration
- Integrate Google Maps or Leaflet.js for GPS location tracking.
- Multi-SLAM: Combine scans from multiple LiDARs to produce unified 2D/3D maps.
- Visualize real-time occupancy or 3D point cloud in the browser.

5G Internet Connection
- Overcome local network constraints using 5G or LTE.
- ZeroMQ works well as long as the port is open or behind a VPN.
- Remote teleoperation with minimal latency, especially if combined with AI edge-processing.

Cloud Integration
- Upload sensor logs and AI outputs to a cloud backend.
- Historical playback, big-data analytics, or training.
- Possibly use AWS S3, or a custom server for storage and retrieval.

---

## Folder Structure
```bash
Project_Eye_Of_Odin/
├─ src/
│  ├─ project_eye_of_odin/   # Meta ROS2 pkg
│  ├─ csi_camera/            # Camera nodes
│  ├─ odin_bridge/           # Odin Bridge
│  ├─ sllidar_ros2/          # LiDAR node
│  └─ observer/              # FastAPI + ZMQ bridge + Dashboard
│      ├─ app/
│      │   ├─ main.py
│      │   ├─ templates/
│      │   │   └─ dashboard.html
│      │   ├─ static/
│      │   │   └─ js/
│      │   │       └─ main.js
│      └─ bridge/
│          └─ zmq_bridge.py
├─ install/
├─ build/
├─ README.md
└─ ...
```

## References & Further Reading
- ROS2 Documentation
https://docs.ros.org/en/
- FastAPI Official
https://fastapi.tiangolo.com/
- ZeroMQ
https://zeromq.org/
- Plotly for Python/JS
https://plotly.com/
- NVIDIA Jetson resources
https://developer.nvidia.com/embedded-computing