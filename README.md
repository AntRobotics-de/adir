![License MIT](https://img.shields.io/github/license/AntRobotics-de/adir?color=blue)
# Adir

Welcome to the official repository for Adir, the differential drive robot designed for versatile indoor/ outdoor navigation. This README provides instructions on how to get Adir up and running on your system.

- [Adir](#adir)
  - [Prerequisites](#prerequisites)
  - [Environment Setup](#environment-setup)
  - [Installation](#installation)
      - [Adding apt repository](#adding-apt-repository)
      - [Adding rosdep definition](#adding-rosdep-definition)
      - [Cloning the Repository](#cloning-the-repository)
  - [Launching Adir](#launching-adir)
  - [Additional information](#additional-information)
    - [Debug information](#debug-information)



## Prerequisites

- Ubuntu 22.04
- ROS2 Humble

## Environment Setup
- If you are using CAN by connecting to the given port on ADIR and CAN bus is up & running, then we can skip this step. 
- If you are using USB cable, please download the adir.sh bash script first and run it using the following commands: 
    ```bash
    chmod +x adir.sh
    sudo ./adir.sh
    ```
  Once succesfully run, then we can launch the run files.

**Please note that the can bit rate is currently set to `500000`.**


## Installation

To use Adir, you must first install the necessary Debian packages. These packages contain all the dependencies and drivers required for Adir to operate seamlessly. The packages can latter be downloaded using rosdep. In order for rosdep to find the packages, we need to add the Antrobotics repository and the rosdep definitions for the packages.

#### Adding apt repository
```bash
curl -s --compressed "https://antrobotics-de.github.io/ppa/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/antrobotics_ppa.gpg >/dev/null
sudo curl -s --compressed -o /etc/apt/sources.list.d/antrobotics.list "https://antrobotics-de.github.io/ppa/antrobotics.list"
sudo apt update
```

#### Adding rosdep definition

```bash
echo "yaml https://antrobotics-de.github.io/ppa/rosdep/antrobotics.yaml" | sudo tee /etc/ros/rosdep/sources.list.d/antrobotics.list >/dev/null
rosdep update
```
#### Cloning the Repository

```bash
mkdir -p adir_ws/src
cd adir_ws/src
git clone -b ros2-humble https://github.com/AntRobotics-de/adir.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```


## Launching Adir

To start using Adir, you can use the following command:

```bash
ros2 launch odometry odometry.launch.py
```

To start moving it, we can publish forward and angular velocities from the terminal or scripts on ros2 topic `/cmd_vel`


## Additional information

### Debug information

Topics Available:

/motor_controller/status                 -  can verify if stall is detected or if ADIR is running on RC/ autonomy commands\
/motor_controller/fault_state            -  can verify if a situation like overheat, overvoltage, undervoltage, short circuit, emergency stop or mosfet failures happen using boolean\
/motor_controller/motor*/runtime_status  -  can verify if a situation like motor_stalled, forward_limit_triggered, reverse_limit_triggered, amps_trigger_activated happen using boolean\
/motor_controller/voltage                -  can verify the voltage of ADIR. Please note if below 21V, please charge the ADIR.

