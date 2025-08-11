# ROS1 Multi-Device Project (Lidar LD19 + Intel RealSense D435)

## Description
The project uses a Raspberry Pi as a ROS Master to connect and publish data from:
- DTOF LIDAR LD19
- Intel RealSense D435

A PC running Ubuntu 20.04 connects to the ROS Master on the Raspberry Pi and visualizes the data in RViz.

---

## Requirements
- **Raspberry Pi:**
- Ubuntu 20.04 / Raspberry Pi OS with ROS1 Noetic support
- ROS1 Noetic installed
- Packages: `ldlidar_stl_ros`, `realsense2_camera`
- Working directory `catkin_ws` or `workspace`

- **PC:**
- Ubuntu 20.04 with ROS1 Noetic
- RViz installed (`sudo apt install ros-noetic-rviz`)

---

## Network setup
Both devices must be on the same local network.

Example IP addresses:
- Raspberry Pi: `192.168.1.151`
- PC: `192.168.1.137`

---

## Launch
move to folder rosrpi/workspace/src/core/launch
### On Raspberry Pi
```
. ros_env_setup_rpi .sh
```

### On PC
```
. ros_env_setup_pc.sh
```
