#!/bin/bash
set -o pipefail

# This script builds all the installed ROS packages and sets up the bashrc.

# Set up a ROS workspace
mkdir -p $COLCON_WS/src
cd $COLCON_WS/src
echo '[ -f "/opt/ros/jazzy/setup.bash" ] && source /opt/ros/jazzy/setup.bash' >> ${HOME}/.bashrc
echo '[ -f "${COLCON_WS}/install/setup.bash" ] && source ${COLCON_WS}/install/setup.bash'  >> ${HOME}/.bashrc

mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts
# git config --global url.https://github.com/.insteadOf git@github.com:
# git config --global advice.detachedHead false

cd $COLCON_WS/src

# Install livox sdk for livox ros driver 2
# Official version does not support 24.04, so we use this one
git clone https://github.com/qza36/Livox-SDK2
cd Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install


# Clone livox ros driver 2
cd $COLCON_WS/src
git clone https://github.com/Shahere/livox_ros_driver2.git
cd livox_ros_driver2
if [ -f package.xml ]; then
    rm package.xml
fi
cp -f package_ROS2.xml package.xml
cp -rf launch_ROS2/ launch/

source /opt/ros/jazzy/setup.bash

cd $COLCON_WS
colcon build --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=jazzy --packages-select livox_ros_driver2

cd $COLCON_WS/src

# Clone BIEVR-LIO
git clone git@github.com:ethz-asl/BIEVR-LIO.git

cd $COLCON_WS
# Build it!!
colcon build --packages-up-to bievr_lio_ros2
