# me332

**SUSTech ME332 Final Project (Fall 2025)**

This repository contains the code and configuration for the ME332 course final project, developed using ROS 2 Humble and simulated on Ignition Gazebo. The project implements an autonomous mobile robot capable of navigation, mapping, obstacle avoidance, and teleoperation through various input modalities.

**Team Member:** Xiaohe Yu

---

## ⚙️ Features

- **Autonomous Navigation** using Nav2 with SLAM, costmaps, and global planning
- **Obstacle Avoidance** and dynamic map management
- **Multiple Teleoperation Modes** (keyboard, voice commands, hand gestures)
- **Frame Correction** for laser/odometry transforms
- **ROS2 Launch Scripts** to start simulation environments and controllers
- **Ignition Gazebo Simulation** with custom worlds and robot model
- **RViz Configurations** for mapping, navigation, and URDF visualization

## 🚀 Setup and Installation
**Tips:**
    Conda environment sometimes cause problem in ROS2, deactivate before using this. UV not tested yet.

**Useful tools:**
https://fishros.com

1. **Prerequisites**
   - Ubuntu 22.04 LTS
   - ROS 2 Humble installed
   - Ignition Gazebo (as required by ROS2 Humble)
   - Python 3.10+ with `colcon` build tools

2. **Clone the Repository**

   ```bash
   git clone https://github.com/yuxiaohe422/me332.git
   cd me332
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Install Python Dependencies**

   ```bash
   pip install -r src/me332/requirements.txt
   ```

4. **Environment Setup**
   Source your workspace each time:
   ```bash
   source install/setup.bash # or . install/setup.bash
   ```

## 📦 Running the Project

- **Start Simulation**
  ```bash
  ros2 launch me332 basic.launch.py
  # or advanced.launch.py for navigation with Nav2
  ```

- **Run Teleoperation Nodes**
  ```bash
  ros2 run me332 keyboard_teleop.py
  ros2 run me332 voice_teleop.py
  ros2 run me332 hand_gesture.py
  ```

- **Launch Navigation2**
  ```bash
  ros2 launch me332_navigation2 nav2.launch.py
  ```

- **Visualize in RViz**
  Load one of the provided RViz config files under `src/me332/rviz/`

## 🧪 Testing

Unit tests ensure code style and licensing compliance. Run them with:

```bash
colcon test --packages-select me332
```

Tests include flake8, pep257 and copyright checks.

## 📝 Configuration

Parameter files are located in `src/me332/config/`. Modify these YAML files to tune
EKF, Nav2, and controller settings. Launch scripts reference them directly.

## 🌍 Maps and Worlds

- Maps (`.pgm`/`.yaml`) under `src/me332/map/` for SLAM/navigation.
- Ignition world files under `src/me332/world/` for simulation scenarios.

## 📦 Package Details

- `me332` — Main Python ROS2 package with nodes for teleop, navigation helpers,
  and utilities.
- `me332_navigation2` — CMake-based package to configure and launch Nav2.

## 📂 Additional Resources

- Robot URDF/XACRO in `src/me332/resource/robot_model.xacro`
- Speech models : 'vosk-model-cn-0.22' and 'vosk-model-small-cn-0.22' recommended, not attached. 
  - Try: run src/me332/me332/voice_teleop.py to get detailed messages.
- Configuration for RViz under `src/me332/rviz/`

## 🛠️ Contribution

This project is intended as a final assignment; modifications are permitted for
learning and demonstration purposes.

## 📄 License

Specify license information here (e.g., MIT, Apache 2.0).

---

> *For more details, explore the source code and configuration directories.*
