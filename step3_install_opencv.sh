#############################################
# INSTALL Opencv 4.0.1 #
#############################################


#Step 5: Download OpenCV 4

cd ~

sudo pip install imutils
sudo pip install --upgrade imutils

wget -O opencv.zip https://github.com/opencv/opencv/archive/4.0.1.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.0.1.zip

unzip opencv.zip
unzip opencv_contrib.zip

cd opencv-4.0.1
mkdir build
cd build
cmake ..
sudo make install -j$(nproc)

sudo pip install opencv-python
sudo pip3 install opencv-python
