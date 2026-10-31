# ESP32 micro-ROS 5-Motor + Gripper Robotic Arm Controller

Welcome! This repository contains the complete firmware, ROS 2 integration scripts, and debugging tools to control a 5-DOF robotic arm (Base, Shoulder, Elbow, Differential Bevel Wrist) plus an RC Gripper Servo using an **ESP32** connected via USB serial to **ROS 2 (Jazzy / Humble)**.

---

## Table of Contents
1. [System Architecture](#system-architecture)
2. [Hardware Pinout & Wiring](#hardware-pinout--wiring)
3. [Differential Wrist Bevel Kinematics](#differential-wrist-bevel-kinematics)
4. [Prerequisites & One-Time Laptop Setup](#prerequisites--one-time-laptop-setup)
   - [1. Install Arduino CLI](#1-install-arduino-cli)
   - [2. Install ESP32 Core v2.0.17](#2-install-esp32-core-v2017)
   - [3. Install micro_ros_arduino Library](#3-install-micro_ros_arduino-library)
   - [4. Build micro-ROS Agent](#4-build-micro-ros-agent)
   - [5. Configure USB Permissions](#5-configure-usb-permissions)
5. [Quickstart Guide (Step-by-Step)](#quickstart-guide-step-by-step)
   - [Step 1: Flash the ESP32](#step-1-flash-the-esp32)
   - [Step 2: Start micro-ROS Agent](#step-2-start-micro-ros-agent)
   - [Step 3: Monitor Live Telemetry](#step-3-monitor-live-telemetry)
   - [Step 4: Send Test Commands](#step-4-send-test-commands)
6. [ROS 2 Topic Reference](#ros-2-topic-reference)
7. [Troubleshooting & Common Pitfalls](#troubleshooting--common-pitfalls)

---

## System Architecture

```text
  ┌────────────────────────────────────────────────────────────┐
  │                        LAPTOP (ROS 2)                      │
  │                                                            │
  │   [PS5 Teleop / Talker]         [arm_feedback Echo]        │
  │            │ /arm_cmd                     ▲                │
  │            ▼ (50 Hz)                      │ /arm_feedback  │
  │   ┌────────────────────────────────────────────────────┐   │
  │   │              micro_ros_agent (ROS 2)               │   │
  │   └─────────────────────────┬──────────────────────────┘   │
  └─────────────────────────────┼──────────────────────────────┘
                                │ USB Serial (115200 baud)
  ┌─────────────────────────────┼──────────────────────────────┐
  │                             ▼                              │
  │                  ESP32 micro-ROS Node                      │
  │                                                            │
  │  • Sign-Magnitude PWM + DIR Outputs (20 kHz, 8-bit)        │
  │  • Hardware Register Readback (ledcRead + digitalRead)     │
  │  • Failsafe: Instant stop on agent disconnect              │
  │                                                            │
  │    GPIO 18/19    GPIO 21/22    GPIO 23/25    GPIO 26/27    GPIO 32/33    GPIO 13   │
  │        │             │             │             │             │            │      │
  └────────┼─────────────┼─────────────┼─────────────┼─────────────┼────────────┼──────┘
           ▼             ▼             ▼             ▼             ▼            ▼
     ┌───────────┐ ┌───────────┐ ┌───────────┐ ┌───────────────────────┐ ┌─────────────┐
     │ Base MDD  │ │Shldr MDD  │ │ Elbow MDD │ │ Wrist Dual MDD10A     │ │ Gripper     │
     │ Driver M1 │ │ Driver M2 │ │ Driver M1 │ │ M1 (Left)   M2 (Right)│ │ RC Servo    │
     └───────────┘ └───────────┘ └───────────┘ └───────────────────────┘ └─────────────┘
```

---

## Hardware Pinout & Wiring

Each DC motor connects to a **Cytron MDD10A** (or similar Sign-Magnitude driver) using **1 PWM Pin + 1 Direction (DIR) Pin**:

| Joint / Actuator | Subscribed Topic Index | ESP32 PWM / TX Pin | ESP32 DIR / RX Pin | Channel / Peripheral | Breakout Header / Wiring |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Motor 1: Base** | `data[0]` | **GPIO 5** | **GPIO 15** | LEDC Ch 0 | Quad 3 (Free jumpers, 1/2 speed, ±125 limit) |
| **Motor 2: Shoulder** | `data[1]` | **GPIO 0** | **GPIO 4** | LEDC Ch 1 | Quad 1 (Arm Quad cable) |
| **Motor 3: Elbow** | `data[2]` | **GPIO 32** | **GPIO 33** | LEDC Ch 2 | Quad 1 (Arm Quad cable) |
| **Motor 4: Wrist 1** | Combined Mixer | **GPIO 26** | **GPIO 27** | LEDC Ch 3 | Differential Bevel M1 |
| **Motor 5: Wrist 2** | Combined Mixer | **GPIO 18** | **GPIO 19** | LEDC Ch 4 | Differential Bevel M2 (Quad 4) |
| **Actuator 6: Gripper** | `data[5]` | **GPIO 17 (TX1)** | **GPIO 16 (RX1)** | UART1 (1 Mbps) | ST3215 Serial Bus Servo (ID 1) |
| **Status LED** | Onboard | **GPIO 2** | — | — | Heartbeat / Connection LED |

### Critical Wiring Rules:
1. **Common Ground (MANDATORY)**: Connect an ESP32 `GND` pin to the `GND` logic pins on **all** Cytron MDD10A motor driver boards. Without a common ground, PWM/DIR signals float and motors will not move.
2. **3.3V Logic Level**: Cytron MDD10A inputs accept 1.8V to 12V logic, so they interface with ESP32 3.3V pins directly without level shifters.
3. **External LEDs**: Do **not** wire LEDs in parallel across the PWM/DIR wires and ground; LEDs can clamp the pin voltage to ~2.0V, preventing the motor driver from registering logic HIGH. (The Cytron MDD10A already has onboard LEDs for each channel).

---

## Differential Wrist Bevel Kinematics

The wrist mechanism uses two bevel gears meshing with a central differential gear:
- **Wrist Pitch** (`data[3]`, $V_{\text{pitch}} \in [-255, +255]$): Rotating both motors in the **SAME direction** tilts the wrist up or down.
- **Wrist Roll** (`data[4]`, $V_{\text{roll}} \in [-255, +255]$): Rotating the motors in **OPPOSITE directions** spins the wrist axially.

### Mathematical Mixer:
$$M_{\text{wrist1}} = V_{\text{pitch}} + V_{\text{roll}}$$
$$M_{\text{wrist2}} = V_{\text{pitch}} - V_{\text{roll}}$$

- **Vector Normalization**: If simultaneous pitch and roll exceed the 8-bit bound ($|M_1| > 255$ or $|M_2| > 255$), both channels are scaled proportionally so the motion angle is preserved without clipping.
- **Direction Inversion**: If either wrist motor spins the wrong way due to gearbox or wire orientation, set `INVERT_WRIST1_DIR` or `INVERT_WRIST2_DIR` to `true` at the top of [`esp32_microros_motor_controller.ino`](file:///home/kratos/opendayjugaad/esp32_microros_motor_controller/esp32_microros_motor_controller.ino).

---

## ST3215 Serial Bus Servo Gripper

The end-effector uses a Feetech **ST3215** high-torque serial bus servo interfaced via a Waveshare Bus Servo Adapter (A) connected to ESP32 **UART1** (`RX = GPIO 16`, `TX = GPIO 17`) at **1,000,000 baud** (1 Mbps).

- **Servo ID**: 1
- **Calibration Range**:
  - Raw `1830` $\rightarrow$ Fully Open ($0^\circ$)
  - Raw `800` $\rightarrow$ Fully Closed ($90^\circ$)
- **Control Rate & Smoothing**:
  - Operates on a 40 ms incremental stepping loop (`GRIPPER_STEP_INTERVAL_MS = 40`) spanning $1^\circ$ per step.
  - Initial position feedback (`gripper.ReadPos(-1)`) starts commands from the physical position on boot.
- **Input Command (`arm_cmd.data[5]`)**:
  - `+1` (or $>0$): Continuously open gripper toward 1830.
  - `0`: Hold current position.
  - `-1` (or $<0$): Continuously close gripper toward 800.

---

## Prerequisites & One-Time Laptop Setup

Follow these steps once on any fresh Ubuntu laptop.

### 1. Install Arduino CLI
```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
export PATH="$HOME/.local/bin:$PATH"
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
```

### 2. Install ESP32 Core v2.0.17
> [!IMPORTANT]
> You **must** install `esp32:esp32@2.0.17` (not version 3.x). The precompiled micro-ROS static library is compiled against the 2.0.x ESP-IDF / FreeRTOS ABI. Core 3.x is incompatible and will produce linker errors.

```bash
arduino-cli config init
arduino-cli config set board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32@2.0.17
```

### 3. Install micro_ros_arduino Library
Clone or download the `micro_ros_arduino` library into your Arduino libraries folder:
```bash
mkdir -p ~/Arduino/libraries
cd ~/Arduino/libraries

# If you have the zip file (e.g. micro_ros_arduino-kilted.zip):
unzip -q ~/Downloads/micro_ros_arduino-kilted.zip -d ~/Arduino/libraries/

# Or clone directly (jazzy branch for ROS 2 Jazzy, or humble for ROS 2 Humble):
git clone -b jazzy https://github.com/micro-ROS/micro_ros_arduino.git
```

### 4. Build micro-ROS Agent
If you don't already have `micro_ros_agent` built in your ROS 2 workspace:
```bash
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git

cd ~/microros_ws
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build

source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent_ws.sh
```

### 5. Configure USB Permissions
Add your user to the `dialout` group to access `/dev/ttyUSB0` without sudo:
```bash
cd /path/to/opendayjugaad
./setup_permissions.sh

# Apply group changes immediately (or logout & log back in)
newgrp dialout
```

---

## Quickstart Guide (Step-by-Step)

### Step 1: Flash the ESP32
Connect the ESP32 via USB and run:
```bash
cd /path/to/opendayjugaad
./flash_motors.sh /dev/ttyUSB0
```
> [!TIP]
> If the uploader gets stuck displaying `Connecting........_____.....`, press and hold the physical **BOOT** button on the ESP32 board until the writing percentage starts.

---

### Step 2: Start micro-ROS Agent
In **Terminal 1**, run:
```bash
cd /path/to/opendayjugaad
./run_agent.sh /dev/ttyUSB0
```
- **Connection Indicator**: The onboard blue LED (GPIO 2) blinks while searching for the agent, and turns **solid ON** the instant the agent connects.

---

### Step 3: Monitor Live Telemetry
In **Terminal 2**, view the live 10-element hardware readback array:
```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /arm_feedback
```
Every element corresponds to physical timer registers and logic pins:
```yaml
data: [
  0, 0,    # Base:      [PWM (0-255), DIR (0/1)]
  0, 0,    # Shoulder:  [PWM (0-255), DIR (0/1)]
  0, 0,    # Elbow:     [PWM (0-255), DIR (0/1)]
  0, 0,    # Wrist 1:   [PWM (0-255), DIR (0/1)]
  0, 0     # Wrist 2:   [PWM (0-255), DIR (0/1)]
]
```

---

### Step 4: Send Test Commands

#### A. Automated 5-DOF Sequential Sine Wave Test
In **Terminal 3**, run the talker to smoothly test each joint one by one:
```bash
cd /path/to/opendayjugaad
./run_motor_talker.sh --mode sine
```
*Cycle sequence*: `Base` $\rightarrow$ `Shoulder` $\rightarrow$ `Elbow` $\rightarrow$ `Wrist Pitch` (both rotate together) $\rightarrow$ `Wrist Roll` (rotate opposite).

#### B. Discrete Step Test
```bash
./run_motor_talker.sh --mode step
```

#### C. Manual Static Values
```bash
# Example: Base at 150, Elbow at -180, Wrist Pitch at 100
./run_motor_talker.sh --mode manual --base 150 --elbow -180 --wrist-pitch 100
```

#### D. Direct ROS 2 Topic Publish
```bash
source /opt/ros/jazzy/setup.bash

# Format: [Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper]
ros2 topic pub --once /arm_cmd std_msgs/msg/Int16MultiArray "{data: [200, -150, 120, 100, 0, 0]}"

# Emergency Stop (all 0)
ros2 topic pub --once /arm_cmd std_msgs/msg/Int16MultiArray "{data: [0, 0, 0, 0, 0, 0]}"
```

#### E. ST3215 Gripper Hardware Diagnostic Tool
If the serial bus servo ever does not respond, run the automated hardware diagnostic scanner:
```bash
cd /path/to/opendayjugaad
./run_gripper_diagnostic.sh /dev/ttyUSB0
```
This tool sweeps all baud rates (1M, 500k, 115.2k, etc.), scans both RX/TX pin orientations, scans IDs 0–15, reads real-time voltage/temperature, and runs an automated physical motion test.

---

## ROS 2 Topic Reference

| Topic | Message Type | Rate | Description |
| :--- | :--- | :---: | :--- |
| **`/arm_cmd`** | `std_msgs/msg/Int16MultiArray` | Up to 50 Hz | **Input Commands** to ESP32: `[Base, Shoulder, Elbow, WristPitch, WristRoll, Gripper]` (values range `-255` to `+255`). |
| **`/arm_feedback`**| `std_msgs/msg/Int16MultiArray` | 1:1 on cmd tick | **Real-Time Readback** from ESP32 timer registers: `[B_pwm, B_dir, S_pwm, S_dir, E_pwm, E_dir, W1_pwm, W1_dir, W2_pwm, W2_dir]`. Emits 1 Hz heartbeat when idle. |

---

## Troubleshooting & Common Pitfalls

### 1. `Permission denied: '/dev/ttyUSB0'` (`errno 13`)
- **Fix**: Run `./setup_permissions.sh` or execute:
  ```bash
  sudo usermod -aG dialout $USER
  sudo chmod 666 /dev/ttyUSB0
  ```

### 2. `Could not open /dev/ttyUSB0, port doesn't exist / multiple access`
- **Cause**: An active `micro_ros_agent` process is already holding `/dev/ttyUSB0` open in the background.
- **Fix**: Kill the background process holding the port:
  ```bash
  fuser -k /dev/ttyUSB0
  ```
  *(Note: `./flash_motors.sh` does this automatically).*

### 3. ESP32 Stuck on `Connecting........_____.....` during upload
- **Fix**: Press and hold the physical **BOOT** button on the ESP32 module as soon as `./flash_motors.sh` starts until the upload progress counter appears.

### 4. ESP32 Bootloops / Agent Won't Connect
- **Cause**: Dynamic memory allocation crash in micro-ROS or unsupported board core.
- **Verification**: This firmware uses **static buffers** (`msg_data_buffer`, `feedback_data_buffer`) to prevent heap fragmentation. Ensure you are using core `esp32:esp32@2.0.17`.

### 5. Motor Spins in the Wrong Direction
- **Fix**: Open [`esp32_microros_motor_controller.ino`](file:///home/kratos/opendayjugaad/esp32_microros_motor_controller/esp32_microros_motor_controller.ino) and toggle the relevant inversion flag at the top:
  ```c
  #define INVERT_BASE_DIR          false
  #define INVERT_SHOULDER_DIR      false
  #define INVERT_ELBOW_DIR         true
  #define INVERT_WRIST1_DIR        false
  #define INVERT_WRIST2_DIR        false
  ```
  Then re-flash with `./flash_motors.sh`.

### 6. Motor Doesn't Spin at Low Speeds
- **Cause**: High mechanical gearbox friction (deadband).
- **Fix**: Most reduction gearboxes require a minimum PWM of `40` to `60` to overcome static friction. Test with `200` to confirm wiring. Check that the onboard LED for that channel on the Cytron MDD10A illuminates.
