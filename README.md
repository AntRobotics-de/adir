# Adir

Welcome to the official repository for Adir, the differential drive robot designed for versatile indoor/ outdoor navigation. This README provides instructions on how to get Adir up and running on your system.

### Prerequisites

- A computer running a Debian-based Linux distribution (currently Ubuntu 22).
- ROS2 Humble installed

## Installation

To use Adir, you must first install the necessary Debian packages. These packages contain all the dependencies and drivers required for Adir to operate seamlessly.

```bash
sudo dpkg -i ros-humble-adir-ros-can-interfaces_0.0.0-0jammy_amd64.deb
sudo dpkg -i ros-humble-adir-ros-can_0.0.1-0jammy_amd64.deb
sudo dpkg -i ros-humble-adir-can-commands-interfaces_0.0.0-0jammy_amd64.deb
sudo dpkg -i ros-humble-adir-can-commands_0.0.0-0jammy_amd64.deb
```


## Run

To start using Adir, you can use the following command:

```bash
ros2 launch adir_can_commands adir_interface_launch.py
```

To start moving it, we can publish forward and angular velocities from the terminal or scripts on ros2 topic ' /cmd_cel  '

We can also run the odometry node as starting basis to get odom topic in ros2. 
```bash
mkdir -p adir_ws/src
cd adir_ws/src
git clone -b ros2-humble https://github.com/AntRobotics-de/adir.git
cd ..
colcon build --symlink-install
```

After succesfully installing we can launcth the odom launch file
```bash
ros2 launch odometry test.launch.py
```


## Additional information

- If using USB cable, please download the adir.sh bash script first and run it using the following commands: 

```bash
chmod +x adir.sh
sudo ./adir.sh
```
  Once succesfully run, then we can launch the run files.

- If using CAN by connecting to the given port on ADIR and CAN bus is up & running, then we can straight away launch the scripts. Please note that the bit rate is currently set at 500kbps.

