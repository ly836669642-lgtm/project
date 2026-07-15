#!/usr/bin/env bash
# Install ROS 2 Jazzy + all dependencies for the i2ros autonomous driving project.
# Run:  bash ~/ros2_ws/install_ros2_jazzy.sh   (will ask for your sudo password)
set -euo pipefail

echo "==> [1/5] Enabling universe repo and base tools..."
sudo apt update
sudo apt install -y software-properties-common curl git unzip
sudo add-apt-repository -y universe

echo "==> [2/5] Adding the official ROS 2 apt source..."
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo "$VERSION_CODENAME")_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

echo "==> [3/5] Installing ROS 2 Jazzy desktop + build tools (several GB, takes a while)..."
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-dev-tools \
  python3-colcon-common-extensions \
  git-lfs

echo "==> [4/5] Installing project-specific ROS packages (perception stack)..."
sudo apt install -y \
  ros-jazzy-depth-image-proc \
  ros-jazzy-octomap-server \
  ros-jazzy-octomap-rviz-plugins \
  ros-jazzy-pcl-ros \
  ros-jazzy-tf2-tools

echo "==> [5/5] git lfs init + shell setup..."
git lfs install
if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
  echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
  echo "    added 'source /opt/ros/jazzy/setup.bash' to ~/.bashrc"
fi

echo ""
echo "All done! Open a NEW terminal (or 'source ~/.bashrc'), then ROS 2 is ready."
