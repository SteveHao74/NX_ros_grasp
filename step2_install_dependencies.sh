#############################################
# INSTALL ros_korstex and ros_kortex_vision #
#############################################

###############################################################################

# Init

cd ~

sudo apt-get update
sudo apt-get upgrade

sudo apt-get install zip

sudo apt-get install build-essential cmake unzip pkg-config
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk-3-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python3-dev

# install catkin
sudo apt install python-catkin-pkg

# dependencies for building packages
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

# install git
sudo apt install git

# dependencies for protobuf
sudo apt-get install autoconf automake libtool curl make g++ unzip

# depedencies for kortex vision
sudo apt install gstreamer1.0-tools gstreamer1.0-libav libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev

sudo apt-get install ros-kinetic-rgbd-launch

# it's needed for demo with the vision
sudo apt-get install python-pip
sudo apt-get install python3-pip


###############################################################################


# Update and upgrade all the new apt installed
sudo apt-get update
sudo apt-get upgrade

# Restart the computer
sudo reboot

