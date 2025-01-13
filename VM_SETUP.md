# Virtual Machine Setup Guide

This guide walks you through setting up the development environment using Ubuntu 20.04 in a virtual machine. This setup is recommended for Windows and macOS users as it avoids the complexities of setting up GUI applications like Gazebo through Docker on these operating systems.

## Prerequisites

- [VirtualBox](https://www.virtualbox.org/wiki/Downloads) installed on your system
- [Ubuntu 20.04 LTS Desktop ISO](https://releases.ubuntu.com/20.04/) downloaded
- At least 50GB of free disk space
- Minimum 8GB RAM (16GB recommended)
- CPU with virtualization support enabled in BIOS

## Virtual Machine Setup

1. Create a new Virtual Machine in VirtualBox:
   - Click "New"
   - Name: "RLS_Development"
   - Type: "Linux"
   - Version: "Ubuntu (64-bit)"
   - Memory size: 8192 MB (or half your system RAM)
   - Create a virtual hard disk: 50GB (VDI, dynamically allocated)

2. Configure VM Settings:
   - System > Processor: Assign at least 2 CPU cores
   - Display > Video Memory: 128 MB
   - Enable 3D Acceleration
   - Network > Adapter 1: NAT

3. Install Ubuntu 20.04:
   - Start the VM and select your Ubuntu ISO
   - Follow the installation wizard
   - Choose "Minimal installation"
   - Select "Install third-party software"
   - Create your user account and password

## Environment Setup

1. Set environment variables:

   ```bash
   echo "export DEBIAN_FRONTEND=noninteractive" >> ~/.bashrc
   echo "export TZ=Asia/Singapore" >> ~/.bashrc
   echo "export QT_QUICK_BACKEND=software" >> ~/.bashrc
   source ~/.bashrc
   ```

2. Install required packages:

   ```bash
   sudo apt-get update && \
   sudo apt-get install -y --no-install-recommends \
       build-essential \
       cmake \
       curl \
       ffmpeg \
       git \
       gnupg2 \
       libatlas-base-dev \
       libboost-filesystem-dev \
       libboost-program-options-dev \
       libboost-system-dev \
       libboost-test-dev \
       libhdf5-dev \
       libeigen3-dev \
       libflann-dev \
       libfreeimage-dev \
       libgflags-dev \
       libglew-dev \
       libgoogle-glog-dev \
       libmetis-dev \
       libprotobuf-dev \
       libqt5opengl5-dev \
       libsqlite3-dev \
       libavcodec-dev \
       libavformat-dev \
       libavutil-dev \
       libswscale-dev \
       libgtk-3-dev \
       nano \
       protobuf-compiler \
       python-is-python3 \
       python3-pip \
       qtbase5-dev \
       software-properties-common \
       sudo \
       unzip \
       vim-tiny \
       wget
   ```

3. Install Git LFS:

   ```bash
   curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
   sudo apt-get update
   sudo apt-get install -y git-lfs
   ```

4. Add Mesa repository and update:

   ```bash
   sudo add-apt-repository ppa:kisak/kisak-mesa
   sudo apt-get update
   ```

## ROS Installation

1. Setup ROS repository:

   ```bash
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo apt-get update
   sudo apt-get install -y ros-noetic-desktop-full
   ```

2. Initialize rosdep:

   ```bash
   sudo apt-get install -y python3-rosdep
   sudo rosdep init
   rosdep update
   ```

3. Setup ROS environment:

   ```bash
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

4. Install additional ROS packages:

   ```bash
   sudo apt-get install -y --no-install-recommends \
       ros-noetic-ros-numpy \
       ros-noetic-moveit \
       ros-noetic-ros-control \
       ros-noetic-ros-controllers \
       ros-noetic-rviz-imu-plugin
   ```

## Fetch Robot Setup

1. Create and setup ROS workspace:

   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone https://github.com/RobotIL-rls/fetch_ros.git
   git clone https://github.com/RobotIL-rls/fetch_gazebo.git
   cd fetch_gazebo
   git checkout gazebo11
   cd ..
   git clone https://github.com/RobotIL-rls/robot_controllers.git
   ```

2. Install dependencies and build:

   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   catkin_make
   ```

3. Source the workspace:

   ```bash
   echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Digital Twin Setup

1. Clone the repository:

   ```bash
   cd ~
   git clone https://github.com/H-tr/digital_twin
   cd digital_twin
   ```

2. Install Python dependencies:

   ```bash
   python3 -m pip install --upgrade pip setuptools
   pip install -e .
   ```

3. Setup Git LFS and pull large files:

   ```bash
   git lfs install
   git lfs pull
   ```

## Running the Environment

Follow the instructions in the main README and examples folder to start using the digital twin environment.
