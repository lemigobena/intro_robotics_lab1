# Lab 1 Exercise: Program diffdrive_bot

This repository contains the solution for Lab 1 of *Introduction to Robotics and Intelligent Systems* (BSc in Software Engineering, Addis Ababa University).

## Packages Included

1. **`diffdrive_bot`**: Robot URDF model extended with a backwards-facing depth camera and a central IMU, Gazebo Harmonic simulation worlds, and bridge configurations.
2. **`lab_interfaces`**: Standard messages, services, and actions for diffdrive_bot (`ObstacleInfo`, `SetSpeed`, `DriveDistance`).
3. **`my_robot_interfaces`**: Custom interface package defining:
   - `msg/RobotStatus.msg` (`string state`, `float32 obstacle_distance`)
   - `srv/SetTurnSpeed.srv` (`float32 turn_speed` -> `bool success`, `string message`)
   - `action/Rotate.action` (`float32 angle` -> `float32 turned` result & feedback)
4. **`my_robot_nodes`**: Python nodes and launch files:
   - `turn_away`: Reactive obstacle avoidance node with dynamic parameters and control services.
   - `rotate_server`: Action server for precise in-place rotation based on `/odom` yaw integration.
   - `square_driver`: Action client executing 4 legs of drive + 90-degree left turn.
   - `square.launch.py`: Integrated launch file.

---

## Build Instructions

Clone this repository into the `src` folder of a ROS 2 workspace (or copy the `lab1_exercise` folder into `~/ros2_ws/src`):

```bash
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash # or /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Step-by-Step Execution & Verification Guide

### Step 1: Depth Camera and IMU Sensors
1. **Model & TF Verification**:
   ```bash
   ros2 launch diffdrive_bot display.launch.py
   ros2 run tf2_ros tf2_echo base_link depth_camera_link
   ```
   *Expected output*: Camera link positioned at $x = -0.215\,\text{m}$, rotated $\pi$ ($180^\circ$) about $z$.
2. **Gazebo Depth Sensor & IMU Verification**:
   ```bash
   ros2 launch diffdrive_bot gazebo.launch.py
   ```
   - In RViz, add an **Image** display for `/depth_camera/image` and a **PointCloud2** display for `/depth_camera/points`. The camera sees the back wall.
   - Test IMU rate:
     ```bash
     ros2 topic hz /imu
     ```
     *Expected output*: ~50 Hz.
   - Test IMU values:
     ```bash
     ros2 topic echo /imu
     ```
     *Expected output*: `linear_acceleration.z` $\approx 9.8\,\text{m/s}^2$.

---

### Step 2, 3, 4, 5: Obstacle Reaction (`turn_away` Node)

1. Launch simulation and control:
   ```bash
   ros2 launch diffdrive_bot control.launch.py
   ```
2. In another terminal, run `turn_away`:
   ```bash
   ros2 run my_robot_nodes turn_away
   ```
3. Enable obstacle reaction:
   ```bash
   ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: true}"
   ```
   *Behavior*: Robot drives forward towards obstacles and turns away when `too_close` is detected.
4. Dynamically adjust parameters:
   ```bash
   ros2 param list /turn_away
   ros2 param set /turn_away forward_speed 0.4
   ```
5. Inspect custom `/robot_status` topic:
   ```bash
   ros2 topic echo /robot_status
   ```
   *Expected output*: Displays `state` ("forward", "turning", or "stopped") and `obstacle_distance`.
6. Test turn speed service:
   ```bash
   ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 1.0}"
   ros2 service call /set_turn_speed my_robot_interfaces/srv/SetTurnSpeed "{turn_speed: 3.0}" # Rejected (> max_turn_speed)
   ros2 service call /turn_away_enable std_srvs/srv/SetBool "{data: false}" # Robot stops
   ```

---

### Step 6: Action Server (`rotate_server`)

1. With Gazebo simulation running (`ros2 launch diffdrive_bot control.launch.py`), start the rotate server:
   ```bash
   ros2 run my_robot_nodes rotate_server
   ```
2. Send a 90-degree left turn goal:
   ```bash
   ros2 action send_goal --feedback /rotate my_robot_interfaces/action/Rotate "{angle: 1.57}"
   ```
   *Expected output*: Robot turns a quarter turn to the left, feedback climbs to $\approx 1.57$, and action succeeds.

---

### Step 7: Drive in a Square (`square.launch.py`)

Run the complete autonomous square navigation with a single launch command:
```bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.8
```
*Expected behavior*:
- Launches Gazebo world, diffdrive_bot robot state publisher, controllers, `rotate_server`, and `square_driver`.
- The robot navigates 4 sides of length 0.8 m, each followed by a 90° turn.
- The robot returns close to its initial starting pose.

---

## Required Submission Artifacts

1. **RViz Depth Camera & Point Cloud**:
   - `docs/screenshots/rviz_depth_camera_pointcloud.png`
2. **rqt_graph during Square Driving**:
   - Run `rqt_graph` while `square.launch.py` is executing.
   - `docs/screenshots/rqt_graph_square.png`
