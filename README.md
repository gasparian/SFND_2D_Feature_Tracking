# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally  
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

### Building OpenCV on Linux  

If you already got some other opencv version, you first can uninstall it (e.g. via `make`):  
```
cd ~/opencv/build
sudo make uninstall
cd .. && sudo rm -r build
cd && sudo rm -r /usr/include/opencv2 /usr/include/opencv /usr/include/opencv /usr/include/opencv2 \
                 /usr/share/opencv /usr/share/OpenCV /usr/share/opencv /usr/share/OpenCV /usr/bin/opencv* \
                 /usr/lib/libopencv* /usr/local/include/opencv2 /usr/local/include/opencv \
                 /usr/local/include/opencv /usr/local/include/opencv2 /usr/local/share/opencv \
                 /usr/local/share/OpenCV /usr/local/share/opencv /usr/local/share/OpenCV \
                 /usr/local/bin/opencv* /usr/local/lib/libopencv* 
```  

And then build an actual version along with the opencv-contrib:  
```
git clone --depth 10 --branch 4.1.0 https://github.com/opencv/opencv ~/opencv
git clone --depth 10 --branch 4.1.0 https://github.com/opencv/opencv_contrib ~/opencv_contrib
mkdir -p ~/opencv/build && cd ~/opencv/build
sudo cmake -D CMAKE_BUILD_TYPE=RELEASE \
           -D CMAKE_INSTALL_PREFIX=/usr/local/ \
           -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
           -D OPENCV_ENABLE_NONFREE=ON \
           -D WITH_IPP=OFF \
           -D WITH_CUDA=OFF \
           -D WITH_OPENCL=ON \
           -D BUILD_TESTS=ON \
           -D BUILD_PERF_TESTS=OFF \
           -D INSTALL_PYTHON_EXAMPLES=ON \
           -D OPENCV_GENERATE_PKGCONFIG=ON \
           ..
sudo make -j"$(nproc)" install
sudo ldconfig
rm -rf ~/.cache/*  
```  

## Project build Instructions  

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.
