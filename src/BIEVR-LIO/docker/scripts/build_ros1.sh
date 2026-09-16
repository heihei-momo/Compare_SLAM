#!/bin/bash
set -o pipefail

# This script builds all the installed ROS packages and sets up the bashrc.
cd $CATKIN_WS

# Set up a ROS workspace
mkdir -p $CATKIN_WS/src
cd $CATKIN_WS
catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

cd $CATKIN_WS/src

# git config --global url.https://github.com/.insteadOf git@github.com:
# git config --global advice.detachedHead false
# Trust github's host key so the non-interactive SSH clone doesn't hang/fail
mkdir -p ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

# Install livox sdk for livox ros driver
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make
sudo make install

cd $CATKIN_WS/src

# Clone livox ros driver
git clone https://github.com/livox-SDK/livox_ros_driver

# Install livox sdk for livox ros driver 2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install

# Clone livox ros driver 2
git clone https://github.com/livox-SDK/livox_ros_driver2
cd livox_ros_driver2
if [ -f package.xml ]; then
    rm package.xml
fi
cp -f package_ROS1.xml package.xml
export ROS_EDITION=ROS1
cd $CATKIN_WS/src

# Clone BIEVR-LIO
git clone git@github.com:ethz-asl/BIEVR-LIO.git
cd BIEVR-LIO

# Build it!!
catkin build --continue bievr_lio_ros -DROS_EDITION=ROS1

# Add sourcing of the repo to the ~/.bashrc
echo 'source $CATKIN_WS/devel/setup.bash' >> ~/.bashrc
