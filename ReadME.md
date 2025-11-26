# Morphing Airfoil Control System - EECS 106A Project

This repository contains the ROS 2 control package for the EECS 106A Morphing Airfoil project. It implements a PID controller to adjust wing camber based on lift and airspeed sensor feedback.

## 🚀 Quick Start

**Prerequisite:** You must have `conda` (Miniforge recommended) installed.

### Setup Environment
```bash
conda config --add channels conda-forge
conda config --add channels robostack-staging
conda config --remove channels defaults
conda create -n ros ros-humble-desktop
conda activate ros
conda install compilers cmake pkg-config make ninja colcon-common-extensions
```

### Build & Run
```bash
# From the root of your workspace (e.g., ~/ros2_ws)
colcon build --symlink-install
source install/setup.zsh   # use setup.bash if on Linux/Intel Mac
ros2 launch morphing_airfoil airfoil.launch.py
```

## 🛠 Detailed Installation Guide

Installing ROS 2 on Apple Silicon can be tricky. We use **RoboStack**, which runs ROS 2 inside a Conda environment. This is much faster than a Virtual Machine.

### 1. Install Miniforge or Conda
If you don't have Conda, install Miniforge (specifically `Miniforge3-MacOSX-arm64`).

### 2. Configure Channels
Tell Conda where to find robotics packages. Run these commands in your terminal:
```bash
conda config --add channels conda-forge
conda config --add channels robostack-staging
conda config --remove channels defaults
```

### 3. Create the ROS Environment
Create an isolated environment named `ros` and install ROS 2 Humble.
```bash
conda create -n ros ros-humble-desktop
```

### 4. Install Build Tools
You need specific compilers to build our custom Python package.
```bash
conda activate ros
conda install compilers cmake pkg-config make ninja colcon-common-extensions
```

> **Troubleshooting:** If you see an error about `Symbol not found: _EVP_DigestSqueeze` after installation, you can safely ignore it. It is a known bug with the signature verifier on Macs and does not affect ROS.

## 📦 How to Build the Code

1. **Navigate to your workspace:**
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ```

2. **Clone this repo:**
    (Assuming you are inside `src`)
    ```bash
    git clone <YOUR_REPO_URL>
    ```

3. **Build:**
    Go back to the root workspace folder and build.
    ```bash
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

## 🏃‍♂️ How to Run

### 1. Activate the Environment
Every time you open a new terminal, you must run:
```bash
conda activate ros
source ~/ros2_ws/install/setup.zsh
```

### 2. Launch the System
This starts the Controller, Sensor Simulation, and Actuator Simulation nodes all at once.
```bash
ros2 launch morphing_airfoil airfoil.launch.py
```

### 3. Verify Connections
To see the node graph (topics and connections):
```bash
rqt_graph
```

## 🧪 Running Unit Tests

We have unit tests for the PID logic to ensure the math is correct before connecting real hardware.

```bash
cd ~/ros2_ws
colcon test --packages-select morphing_airfoil
```

To see the results:
```bash
colcon test-result --all
```
