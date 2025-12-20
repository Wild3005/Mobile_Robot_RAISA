# RAISA Humanoid – Physically-Aware Social Navigation

## Concept Overview

Traditional social navigation focuses primarily on collision avoidance.  
In this system, **RAISA predicts a human’s working space based on leg posture**.

Using a chairless-chair concept and dual-leg sensing:
- **Bent legs** or **sitting posture** indicate instability or increased leg-space demand
- The robot dynamically **increases its inflation radius (safety distance)** in the navigation costmap
- This enables **physically-aware, socially compliant navigation behavior**

---

## Terminology & System Definition

To avoid ambiguity, the following terminology is used throughout this documentation:

### RAISA (Humanoid Robot System)

**RAISA** refers to the **entire humanoid robot system**, including:
- Upper body structure
- Onboard PC (RAISA PC)
- Perception sensors (UWB, camera, dual-leg system)
- Decision-making software (FSM, Master Node)
- User Interface (Web UI)

RAISA is responsible for:
- Human-aware perception
- Social navigation decision logic
- High-level control commands

---

### REEMAN (Mobile Robot Base)

**REEMAN** refers to the **mobile robot base** (lower body / chassis) that provides:
- Locomotion and low-level motion execution
- Wheel control and base odometry
- Hardware-level motor control

REEMAN **does not make decisions autonomously**.  
All navigation commands sent to the REEMAN base originate from **RAISA’s onboard PC**.

---

### RAISA PC (Main Controller)

The **RAISA PC** is the onboard computer mounted on the robot that:
- Runs ROS 2
- Hosts the Master Node
- Processes perception data
- Sends velocity and control commands to the REEMAN base via GET and POST requests

Communication between RAISA PC and the REEMAN base is handled via a dedicated ROS 2 communication node (`io_reeman_node`).

---


## Project Scope & Objectives

### Purpose

RAISA is designed to support the following human-aware navigation behaviors:
- Sitting Approach
- Standing Approach
- Cross Behind
- Escort Mode
- Active Yielding
- Standard Passing
- Stop and Wait
- Proceed with Caution

### Design Philosophy
- Finite State Machine (FSM)–based behavior control
- Modular and sensor-driven architecture
- Clear separation between perception, decision, and actuation layers

---

## System Architecture

### High-Level Architecture

```mermaid
graph TD
    A[Sensors / Perception Nodes / UI] --> B(ROS Topics)
    B --> C[Master Node]
    C --> D[Communication Node]
    D --> E[Navigation / Motion]
````

---

### Master Node Responsibilities

* Sensor fusion
* FSM state handling
* High-level decision making
* UI and external command integration

---

## Operating Modes

### Mode A: Navigation Mode

**Primary Sensors**

* Pozyx UWB Anchors
* Dual-leg angle values (ESP32 + magnetic encoders)

### Mode B: Interaction Mode

**Primary Sensors**

* Pozyx UWB Anchors

---

## Deployment & Access

### System Access

```bash
ssh raisa@<robot_ip>
cd raisa_social
```

---

## Configuration Guide

### Mode Selection (FSM Configuration)

```cpp
int MODE = MODE_INTERACTION;
// or
int MODE = MODE_NAVIGATION;
```

---

### Launch Configuration Strategy

* Enable only required nodes
* Disable unused perception pipelines
* Maintain only **one active tracking mode**

From `ros2_utils/launch/all.launch.py`:

```python
return LaunchDescription(
    [
        rosapi_node,
        web_video_server,
        ui_server,
        rosbridge_server,

        hand_track,        # Hand gesture detection (STOP)
        uwb_localization,  # UWB positioning
        dual_leg,          # ESP32 leg sensors

        master,            # Master controller

        # keyboard_input,
        # wifi_control,

        io_reeman_node,    # Reeman robot base I/O
        # ds4_driver,
    ]
)
```

---

## Getting Started

### 1. Clone the Repository

```bash
git clone <repo-url>
cd Mobile_Robot_RAISA
```

---

### 2. Install Dependencies

Ensure ROS 2 is installed (**Humble recommended**).

Install all required Python and C++ dependencies for each package.
Refer to `package.xml` and `CMakeLists.txt` for details.

#### Install necessary dependencies
```
sudo apt install ros-humble-rosbridge-server
sudo apt install ros-humble-rosbridge-suite
sudo apt install libyaml-cpp-dev 
```

#### Install cpr 
```
cd cpr && mkdir build && cd build
cmake .. -DCPR_USE_SYSTEM_CURL=ON
cmake --build . --parallel
sudo cmake --install .
```

#### Reinitialize Workspace (Destructive Operation)

This script **recreates the entire ROS 2 workspace structure** by deleting and regenerating all packages.

&#9888; **WARNING**  
Running this script will:
- **Delete the entire `src/` directory**
- Permanently remove all existing source code inside `src/`
- Recreate ROS 2 packages from scratch using the provided maintainer name and email

Only run this script if:
- You are setting up the project for the **first time**, or
- You intentionally want to **reset the workspace to a clean state**

#### Usage
```bash
./init_all.sh "<maintainer_name>" <maintainer_email>
```

example:

```bash
./init_all.sh "John Doe" "john.doe@example.com"
```

>Do NOT run this script on a workspace containing valuable code unless it has been backed up.

---

### 3. Configure Reeman Robot IP

* Ensure the Reeman robot and the PC are connected to the **same network**
* Example robot base IP:

  ```
  192.168.1.100
  ```
* Update the IP in communication-related nodes or UI configuration files

---

### 4. Build the Workspace

```bash
./make.sh
```

or using colcon:

```bash
colcon build --symlink-install
```

---

### 5. Source the Workspace

```bash
source install/setup.bash
```

---

### 6. Edit Configuration (Optional)

* Modify YAML files inside the `config` directory
* Verify IP addresses and port numbers
* Ensure sensor parameters match the deployed hardware

---

### 7. Run the System

Using the provided script:

```bash
. ./run.sh
```

or directly using ROS 2 launch:

```bash
ros2 launch ros2_utils all.launch.py
```

---

### 8. Access the Web UI

The Web UI allows switching between **Interaction Mode** and **Navigation Mode**.

* Ensure `ui_server.py` is running
* Open a browser and navigate to:

```text
http://<robot_ip>:8000
```

Example:

```text
http://10.7.101.50:8000
```

---

## Troubleshooting

### Robot Base Not Responding

Ensure the Reeman robot base IP is correctly configured.

Example screenshot:

![Reeman Base IP](/images/reeman_api.png)

Update the IP in the launch file:

```python
io_reeman_node = Node(
    package='communication',
    executable='io_reeman_node',
    name='io_reeman_node',
    parameters=[
        {
            "reeman_ros_ip": "10.7.101.173",  # Update this IP
            "min_request_period_speed_ms": 500,
            "polling_period_ms": 1000,
        },
    ],
    output='screen',
    respawn=True,
)
```

---

### Web UI Connection Issues

Check the PC network configuration:

```bash
ifconfig
```

If the robot base IP is:

```text
10.7.101.173
```

Your PC should have an IP in the same subnet:

```text
10.7.101.XXX
```

if correctly the ip, you show see web ui in this below:

![Raisa Web UI](/images/web_ui.png)

---

### ESP32 Dual-Leg Sensor Issues

* ESP32 devices must be connected to the **same network**
* Ensure communication uses **port 80**
* Recommended network scanning tool: **Net Scan**

---

### Camera Detection Issues

Check available camera devices on the RAISA mini PC:

```bash
ls /dev/v4l/by-id/
```

Camera devices usually appear as:

```text
video0
```

---

## Notes & Best Practices

* Do not run Navigation Mode and Interaction Mode simultaneously
* Always recalibrate UWB after system reboot
* Verify frame alignment before navigation testing
* Use `ros2 bag` to record data for debugging and analysis

---

## Future Improvements

* Multi-human tracking support
* Behavior Tree (BT) replacement for FSM
* Dynamic costmap adjustment based on human posture
* Socially-aware Nav2 integration

---
