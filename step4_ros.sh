#############################################
# INSTALL ros_korstex and ros_kortex_vision #
#############################################

###############################################################################
###############################################################################
# 1. install catkin

sudo apt install python-catkin-pkg

###############################################################################
# 2. install ROS kenetic for ubuntu 16.04 LTS
# Credit to the ros tutorial at http://wiki.ros.org/kinetic/Installation/Ubuntu


#Setup your sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

#Set up your keys
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

#Installation
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full

#Initialize rosdep
sudo rosdep init
rosdep update

#Dependencies for building packages
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

#Environment setup
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
source /opt/ros/kinetic/setup.bash


