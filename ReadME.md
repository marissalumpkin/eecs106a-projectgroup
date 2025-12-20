# Morphing Airfoil Control System - EECS 106A Project

This repository contains the ROS 2 control package for the EECS 106A Morphing Airfoil project. It implements a PID controller to adjust wing camber based on lift and airspeed sensor feedback.

## 🔌 Hardware Requirements

*   **Microcontroller:** Arduino Nano ESP32
*   **Load Cell:** ShangHJ Digital Load Cell (5kg)
*   **ADC Module:** HX711 (for Load Cell)
*   **Pitot Tube:** Analog Airspeed Sensor
*   **Computer:** Mac (Apple Silicon) or Linux machine running ROS 2

## 🚀 Quick Start

**Prerequisite:** You must have `conda` (Miniforge recommended) installed.

### 1. Setup Environment
```bash
conda config --add channels conda-forge
conda config --add channels robostack-staging
conda config --remove channels defaults
conda create -n ros ros-humble-desktop
conda activate ros
conda install compilers cmake pkg-config make ninja colcon-common-extensions pyserial
```

### 2. Build the Workspace
```bash
# From the root of your workspace (e.g., ~/ros2_ws)
colcon build
source install/setup.zsh
```

## 🤖 Arduino Setup

Before running ROS, you must set up the Arduino Nano ESP32 to read sensors and send data over USB.

1.  **Install Arduino IDE:** Download and install the latest Arduino IDE.
2.  **Install Libraries:**
    *   Open Arduino IDE.
    *   Go to **Sketch -> Include Library -> Manage Libraries**.
    *   Search for and install **"HX711 Arduino Library"** (by Bogdan Necula).
3.  **Upload Code:**
    *   Open `src/arduino_sensors/arduino_sensors.ino`.
    *   Connect your Arduino Nano ESP32 via USB.
    *   Select your board and port in **Tools -> Port**.
    *   Click **Upload**.

### Wiring Guide

#### Load Cell (HX711)
*   **DT** -> Pin 7
*   **SCK** -> Pin 4
*   **VCC/GND** -> 3.3V/GND

#### Pitot Tube
*   **Signal** -> Pin A0
*   **VCC/GND** -> 3.3V/GND

#### Servo (HD0521MG)
*   **Signal (Orange/Yellow)** -> Pin 5
*   **Power (Red)** -> VBUS (5V)
*   **GND (Brown)** -> GND

## 🏃‍♂️ How to Run

### Option A: Simulation Mode (Default)
Run this to test the logic without hardware connected.
```bash
ros2 launch morphing_airfoil airfoil.launch.py
```
*You should see: `[INFO] ... Sensor Node Started in SIMULATION MODE`*

### Option B: Hardware Mode
Run this when the Arduino is connected.

1.  **Find your Serial Port:**
    ```bash
    ls /dev/tty.*
    # Look for something like /dev/tty.usbmodem... or /dev/ttyUSB0
    ```

2.  **Launch:**
    ```bash
    ros2 launch morphing_airfoil airfoil.launch.py use_sim:=False serial_port:=/dev/tty.usbmodem1234
    ```
    *(Replace `/dev/tty.usbmodem1234` with your actual port)*

    *Also add either `controller_type:=pid` for pid or `controller_type:=reactive` for reactive controller*

## 🧪 Running Unit Tests

We have unit tests for the PID logic to ensure the math is correct.

```bash
colcon test --packages-select morphing_airfoil
colcon test-result --all
```
