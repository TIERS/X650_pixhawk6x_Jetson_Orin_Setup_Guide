# Livox MID360 LiDAR and FAST-LIO MAVROS Integration Setup Guide

This guide covers the complete setup process for the Livox MID360 LiDAR sensor with FAST-LIO and PX4 integration via MAVROS.

## Table of Contents

- [MID360 LiDAR Connection and Driver Setup](#mid360-lidar-connection-and-driver-setup)
- [FAST-LIO Setup and Configuration](#fast-lio-setup-and-configuration)
- [Passing Odometry Data to PX4](#passing-odometry-data-to-px4)
  - [Coordinate System Explanation](#coordinate-system-explanation)
  - [Method 1: Vision Pose Topic](#method-1-vision-pose-topic)
  - [Method 2: Odometry Topic](#method-2-odometry-topic)
  - [Method 3: TF Listener](#method-3-tf-listener)

---

## MID360 LiDAR Connection and Driver Setup

### Download and Install Driver

1. Download the driver from the official repository:
   - Repository: [https://github.com/Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
   - Read the `README.md` file for detailed installation instructions

### Driver Configuration

Configure the driver by editing the following files:
- **Config file**: `/config/MID360_config.json` (inside the driver folder)
- **Launch file**: `/launch_ROS1/msg_MID360.launch`

Example launch files are provided in the driver folder:

#### Available Launch Files

| Launch File | Description |
|------------|-------------|
| `rviz_MID360.launch` | Connect to MID360 LiDAR device<br>Publish PointCloud2 format data<br>Autoload RViz |
| `msg_MID360.launch` | Connect to MID360 LiDAR device<br>Publish Livox customized pointcloud data |

---

## FAST-LIO Setup and Configuration

### Installation

1. Download and install FAST-LIO from the official repository:
   - Repository: [https://github.com/hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)
   - Follow the installation instructions in the repository

### Configuration Updates

2. **Update driver references**: Change all instances of `livox_ros_driver` to `livox_ros_driver2` throughout the FAST-LIO folder

3. **Verify configuration files**:
   - Check `config/mid360.yaml` for correct parameters
   - Check `launch/mapping_mid360.launch` for correct launch settings

---

## Passing Odometry Data to PX4

### Coordinate System Explanation

We use MAVROS to pass odometry data to PX4. **MAVROS automatically handles NED (North-East-Down) conversions**.

#### PX4 Coordinate Systems

PX4 uses **FRD** (X Forward, Y Right, Z Down) for the local body frame and reference frame. When using magnetometer heading, the PX4 reference frame X-axis aligns with north, creating the **NED** (X North, Y East, Z Down) frame.

#### Frame Comparison Table

| Frame | PX4 | ROS |
|-------|-----|-----|
| **Body** | FRD (X Forward, Y Right, Z Down) | FLU (X Forward, Y Left, Z Up)<br>Usually named `base_link` |
| **World** | FRD or NED (X North, Y East, Z Down) | FLU or ENU (X East, Y North, Z Up)<br>Named `odom` or `map` |

#### Origin Setting

PX4 sets the NED origin at the first valid global position fix (GPS lat/lon/alt). If GPS is unavailable, it uses the arming position as the origin. All local navigation and position control are expressed relative to this fixed reference.

### Required Documentation

Before proceeding, please read these essential PX4 documentation pages:
- [Computer Vision Integration](https://docs.px4.io/main/en/advanced/computer_vision)
- [Visual Inertial Odometry](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)

### TF Frame Configuration

#### Original FAST-LIO Frames

The odometry output from FAST-LIO provides only two TF frames:
- **`camera_init`**: Initial position of the LiDAR (world frame origin)
- **`body`**: Current position of the LiDAR

#### Required Additional Frames

According to PX4 documentation, we need:
- **`odom`**: Initial position of the robot body (world frame)
- **`base_link`**: Current position of the robot body

#### Setting Up TF Transforms

To integrate with PX4, we need to establish TF transforms that link:
1. `body` (LiDAR) to `base_link` (robot body)
2. `odom` to `camera_init` (world frame alignment)

**Note**: The following values are specific to my MID360 mounting configuration (mounted at an angle). You will need to adjust these values based on your own sensor mounting.

Add these static transform publishers to your FAST-LIO launch file:

```xml
<!-- Transform from LiDAR body frame to robot base_link -->
<node pkg="tf2_ros" type="static_transform_publisher" name="tf_baseLink_externalPoseChildFrame"
      args="-0.05763 0 -0.14743 0 -2.3562 0 body base_link"/>
      <!-- Note: 'body' here refers to the LiDAR's body frame -->

<!-- Transform from odom to camera_init (world frame alignment) -->
<node pkg="tf2_ros" type="static_transform_publisher" name="tf_odom_externalPoseParentFrame"
      args="0.0635 0 -0.1450 0 2.3562 0 odom camera_init"/>
```

#### Frame Initialization

When the LIO algorithm starts, it sets the first camera frame position as the coordinate origin. At initialization, `camera_init` and `odom` are at the same position.

---

## Integration Methods

There are three methods to send odometry data from FAST-LIO to PX4:

### Method 1: Vision Pose Topic

Send pose data directly to PX4 via the topic: **`mavros/vision_pose/pose`**

**Implementation**: See code at `lidar_to_mavros/src/lidar_to_mavros.cpp`

**Pros**: Simple and direct pose transmission  
**Cons**: Doesn't include velocity information

---

### Method 2: Odometry Topic

Send complete odometry data (pose + velocity) to PX4 via the topic: **`mavros/odometry/out`**

**Implementation**: See code at `lidar_to_mavros/src/lidar_to_mavros/other_methods`

**Pros**: Includes both pose and velocity information  
**Cons**: Slightly more complex message structure

---

### Method 3: TF Listener

Configure MAVROS to directly listen to TF transforms instead of using topics.

#### Configuration

Modify the `px4_config.yaml` file used when launching MAVROS. Set `listen: true` in the vision_pose section:

```yaml
# vision_pose_estimate
vision_pose:
  tf:
    listen: true           # Enable TF listener (disables topic subscribers)
    frame_id: "odom"
    child_frame_id: "base_link"
    rate_limit: 10.0
```

**Important**: When `listen` is set to `true`, **Methods 1 and 2 (topic-based) will be disabled**. MAVROS will only use the TF transform between `frame_id` and `child_frame_id` as the localization data.

**Pros**: 
- Cleaner integration using ROS TF system
- No need for separate relay nodes
- Automatically uses existing TF tree

**Cons**: 
- Requires proper TF tree setup
- Less explicit data flow

---

## Launch Sequence

To run the complete system:

See `lidar_to_mavros_tmux.sh`

1. Start the Livox driver: `roslaunch livox_ros_driver2 msg_MID360.launch`
2. Start FAST-LIO: `roslaunch fast_lio mapping_mid360.launch`
3. Start MAVROS with appropriate configuration
4. (Optional for Methods 1-2) Start the odometry relay node: 

---

## Troubleshooting

- **No TF transforms**: Ensure all static transform publishers are running
- **PX4 not receiving data**: Check MAVROS connection and topic/TF configuration
- **Coordinate misalignment**: Verify your sensor mounting transform values
- **Performance issues**: Adjust `rate_limit` in configuration files

---

## References

- [Livox ROS Driver 2](https://github.com/Livox-SDK/livox_ros_driver2)
- [FAST-LIO](https://github.com/hku-mars/FAST_LIO)
- [PX4 Computer Vision](https://docs.px4.io/main/en/advanced/computer_vision)
- [PX4 Visual Inertial Odometry](https://docs.px4.io/main/en/computer_vision/visual_inertial_odometry)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
