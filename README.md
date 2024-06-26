# ROS2 KUKA Drivers

This repository contains ROS2 drivers for all KUKA operating systems.
This repository was forked from kroshu/kuka_drivers and includes modifications to enable 7th axis (linear rail) control over RSI.

Github CI | SonarCloud
------------| ---------------
[![Build Status](https://github.com/kroshu//kuka_drivers/workflows/CI/badge.svg?branch=master)](https://github.com/kroshu/ros2_kuka_sunrise_fri_driver/actions) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_kuka_drivers&metric=alert_status)](https://sonarcloud.io/dashboard?id=kroshu_kuka_drivers)

# Requirements
The drivers require a system with ROS installed. It is recommended to use Ubuntu 22.04 with ROS humble. Iron Irwini has breaking changes in the moveit API, thus it is not yet supported.
It is also recommended to use a client machine with a real-time kernel, as all three drivers require cyclic, real-time communication. Due to the real-time requirement, Windows systems are not recommended and covered in the documentation.


# Installation
The driver is not available as a binary package, building from source is necessary.

Create ROS2 workspace (if not already created).
```bash
mkdir -p ~/ros2_ws/src
```

Clone KUKA ROS2 repositories.
```bash
cd ~/ros2_ws/src
git clone https://github.com/kroshu/kuka_drivers.git
vcs import < kuka_drivers/upstream.repos
```

Install and initialize rosdep (if not already done)
```bash
sudo apt install python3-rosdep
sudo rosdep init
```

Install dependencies using `rosdep`.
```bash
cd ~/ros2_ws
rosdep update
sudo apt upgrade
rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
```

Clone and build kuka-external-control-sdk in a different workspace.
 - This library is necessary for the iiQKA ExternalAPI.Control driver
 - The library is not a ROS2 package, therefore a different workspace is necessary, otherwise colcon will fail to build it
 - To install dependencies for your platform, check out the [Requirements](https://github.com/kroshu/kuka-external-control-sdk/blob/master/kuka_external_control_sdk/doc/SDK_howto.md) section of the kuka-external-control-sdk documentation.
```bash
mkdir -p ~/sdk_ws/src
cd ~/sdk_ws/src
git clone https://github.com/kroshu/kuka-external-control-sdk.git
mkdir -p ~/sdk_ws/src/kuka-external-control-sdk/kuka-external-control-sdk/build
cd ~/sdk_ws/src/kuka-external-control-sdk/kuka-external-control-sdk/build
cmake ..
make install
```

Build KUKA packages.
```bash
cd ~/ros2_ws
colcon build
```

Source built KUKA packages.
```bash
# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source ~/ros2_ws/install/setup.bash
```

# Getting Started
Documentation of this project can be found on the repository's [Wiki](https://github.com/kroshu/kuka_drivers/wiki) page.

If you find something confusing, not working, or would like to contribute, please read our [contributing guide](CONTRIBUTING.md) before opening an issue or creating a pull request.
