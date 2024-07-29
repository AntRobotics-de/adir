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

### Topics Used in This Project

Below is a list of the topics used in this project along with their respective message types and descriptions.

| Topic Name (Publishers)       | Message Type          | Description                                             |
| ---------------- | --------------------- | ------------------------------------------------------- |
| `/adir/odom`     | `nav_msgs/Odometry`     | Publishes the adir odometry information(currently only using wheel encoders)                              |
| `/motor_controller/voltage  `       | `std_msgs/Float` | Publishes robot voltage. `Please charge when it falls below 23V`           |
| `/motor_controller/status`       | `ros_can_interfaces/Frame` | Publishes information on serial,pulse or analog mode and stall detection           |
| `/motor_controller/fault_state`     | `ros_can_interfaces/Frame`     | Publishes information on overheating, undervoltage, overvoltage, shortcircuit, emergency stop, motor sensor setup fault & mosfet failure                             |
| `/motor_controller/motor*/runtimestatus`     | `ros_can_interfaces/Frame`     | Publishes information on amps limit active, motor stall. loop error detection, safety stop active, forward limit triggered, reverse limit triggered & amps trigger activated                             |

| Topic Name (Subscribers)      | Message Type          | Description                                             |
| ---------------- | --------------------- | ------------------------------------------------------- |
| `/cmd_vel`       | `geometry_msgs/Twist` | Subscribes to velocity commands for the robot           |



### Services 


| Service Name       | Service Type          | Description                                             |
| ---------------- | --------------------- | ------------------------------------------------------- |
| `/ResetOdometry`     | `nav_msgs/Odometry`     | Resets the adir wheel odometry                              |                        |








