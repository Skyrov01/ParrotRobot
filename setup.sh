#!/bin/bash
set -e

echo "🔧 Updating system..."
sudo apt update && sudo apt upgrade -y

echo "🔧 Installing base tools..."
sudo apt install -y \
    software-properties-common \
    curl \
    locales \
    git \
    build-essential \
    python3-pip \
    python3-venv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    libgpiod-dev

echo "🔧 Generating locale..."
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "🔧 Adding ROS 2 apt repository..."
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

echo "🔧 Installing ROS 2 Jazzy Jalisco Desktop..."
sudo apt update
sudo apt install -y ros-jazzy-desktop

echo "🔧 Sourcing ROS 2 in .bashrc..."
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source /opt/ros/jazzy/setup.bash

echo "🔧 Initializing rosdep..."
sudo rosdep init || true
rosdep update

echo "🔧 (Optional) Installing ros2_control..."
sudo apt install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers

echo "🔧 Installing Python dependencies..."
pip3 install --upgrade pip
pip3 install flask flask-cors nicegui requests gpiozero

echo "✅ Setup complete!"
echo "👉 Run 'source ~/.bashrc' or reboot before using ROS 2 Jazzy with your parrot robot."

