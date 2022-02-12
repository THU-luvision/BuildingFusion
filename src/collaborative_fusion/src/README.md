# FlashFusion
Project for FlashFusion: An efficient dense 3D reconstruction that only relies on CPU computing.

Related papers:

1.  Multi-Index Hashing for Loop closure Detection. International Conference on Multimedia Expo, 2017. Best Student Paper Awards. 

2.  Beyond SIFT Using Binary features in Loop Closure Detection. IROS 2017. 

3.  FlashFusion: Real-time Globally Consistent Dense 3D Reconstruction using CPU Computing. RSS 2018.

# Prerequisites  ################################################################

Start from Zero to install FlashFusion:	

1. basic softwares:
chrome/sublime/terminator:
sudo add-apt-repository ppa:webupd8team/sublime-text-3

sudo apt-get update 

sudo apt-get install terminator sublime-text-installer p7zip-full dtrx


1. required library:
sudo apt-get install build-essential pkg-config cmake \
libwxgtk3.0-dev libftdi-dev freeglut3-dev \
zlib1g-dev libusb-1.0-0-dev libudev-dev libfreenect-dev \
libdc1394-22-dev libavformat-dev libswscale-dev 

sudo apt-get install libassimp-dev libjpeg-dev libopencv-dev \
libeigen3-dev libsuitesparse-dev libpcap-dev libsuitesparse-dev \
 build-essential cmake git ffmpeg libopencv-dev libgtk-3-dev \
python-numpy python3-numpy libdc1394-22 libdc1394-22-dev 

sudo apt-get install libjpeg-dev libpng-dev libtiff5-dev libjasper-dev libavcodec-dev \
 libavformat-dev libswscale-dev libxine2-dev libgstreamer1.0-dev \
libgstreamer-plugins-base1.0-dev libv4l-dev libtbb-dev qtbase5-dev \
 libfaac-dev libmp3lame-dev libopencore-amrnb-dev
 
sudo apt-get install libopencore-amrwb-dev libtheora-dev libvorbis-dev libxvidcore-dev \
 x264 v4l-utils unzip libglew-dev libpython2.7-dev ffmpeg libavcodec-dev\
 libavutil-dev libavformat-dev libswscale-dev libglfw3-dev  

sudo apt-get install libxmu-dev libxi-dev libboost-all-dev  libxmu-dev libxi-dev


2. Eigen 3.2.8


3. opencv3.3.0
compile:

cmake -D CMAKE_BUILD_TYPE=RELEASE  -DBUILD_opencv_stitching=OFF -D CMAKE_INSTALL_PREFIX=/usr/local -D WITH_TBB=ON -D WITH_V4L=ON -D WITH_QT=ON -D WITH_OPENGL=ON -D WITH_CUDA=OFF -DENABLE_PRECOMPILED_HEADERS=OFF -D CMAKE_BUILD_TYPE=RELEASE  -D BUILD_DOCS=OFF -D BUILD_PERF_TESTS=OFF  -D BUILD_TESTS=OFF ..

make -j
sudo make install
sudo /bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
sudo ldconfig


4. openni
mkdir build
cd build
cmake ..
make -j
sudo make install
in case that you can not install the openni driver sucessfully:
https://codeyarns.com/2015/08/04/how-to-install-and-use-openni2/
sudo ln -s /lib/x86_64-linux-gnu/libudev.so.1.6.4 /lib/x86_64-linux-gnu/libudev.so.0

6. sophus
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout b474f05
mkdir build
cd build
cmake ..
make
sudo make install 

7. intel realsense driver



9. qt_with_ros https://github.com/ros-industrial/ros_qtc_plugin/wiki/1.-How-to-Install-(Users)#section1.2


# Installation ################################################################
$ ./build.sh



# Usage ##############################################################
./flashfusion DataFolder ParametersFile #Resolution #inputID

For each input parameter:

DataFolder: base folder containing test data or output data.

The basefolder should contain:
1.  calib.txt for calibration parameters
2.  associate.txt for input rgbd images
3.  groundtruth.txt for TUM RGBD dataset groundtruth data.
An example datafolder is provided in this link:  https://hkustconnect-my.sharepoint.com/:u:/g/personal/lhanaf_connect_ust_hk/Ecfgf2RxoCpJgdnQ7_JgeKkBVsTFAmr_nBYAbnK9aedsRQ?e=VAF6cX

ParametersFile: input parameters, an example is given in ../settings.yaml

#Resolution: Reconstruction voxel resolution, ranges from 0.005 to 0.04

#inputID: 

  0: offline data stored in DataFolder
  1: Online data using ASUS Xtion
  2: Online data using Intel Realsense
  
example:

./flashfusion  /home/luvision/Downloads/RGBD_Dataset/fr3room ../settings.yaml 0.005 0

**This version includes some functions about multiple cameras.**


