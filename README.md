# MARS-VINS

## CHANGES MADE IN THIS REPOSITORY

* Build with docker to allow for running on 16.04 ubuntu
* Add missing build libraries
* Build using the catkin build system (separate binary for each distortion type)
* Runs on both the EuRoC mav and TUM-VI datasets
* Matlab conversion script for cam+imu extrinsics
* NOTE: there is a visualization bug in the binary
* NOTE: thus the feature overlay is broken, but estimate is not effected


## BUILDING WITH DOCKER

* Install Docker CE [link](https://docs.docker.com/install/linux/docker-ce/ubuntu/)
    * curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
    * sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
    * sudo apt-get update
    * sudo apt-get install docker-ce docker-ce-cli containerd.io


* Install [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) so we can use OpenGL
    *  curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
    *  distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
    *  curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
    * sudo apt-get update
    * sudo apt-get install -y nvidia-docker2

* Fix libGL swrast error
    * We need to allow for the container to get our nvidia driver
    * Temp fix is [here](https://github.com/jessfraz/dockerfiles/issues/253#issuecomment-373043685)
    * Need to edit the dockerfile to have the same nvidia-384 driver (change the version number based on your system)
    * We need to symbolically link our drivers to it
    * This should allow displaying of gui
    
* Build, enable x11, and enter the container
    * docker build --tag=marsvins .
    * xhost +"local:docker@"
    * nvidia-docker run -v /home/patrick/datasets/:/datasets -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=$DISPLAY  -v /usr/lib/nvidia-384:/usr/lib/nvidia-384 -v /usr/lib32/nvidia-384:/usr/lib32/nvidia-384 --privileged --device /dev/dri -it marsvins


* Run the example algorithm with MARS data (*run these inside of the docker container*)
    * /docker_build.sh
    * /docker_run.sh
    * rosrun mars_vins example_app_equidistant /datasets/mars/config.txt /datasets/mars/output.txt /datasets/mars/Regular_1/imu_data.txt /datasets/mars/Regular_1/img_data/left/ /datasets/mars/Regular_1/img_data/right/


* To fix no runtime named nvidia
    * Restart the processes should fix this
    * sudo systemctl daemon-reload
    * sudo systemctl restart docker






---
---
---




## Copyright, Patent & License Notice
The MARS VINS software is copyrighted by the Regents of the University of
Minnesota. Use of the software may be covered by at least US patents and
applications 9,243,916, 9,658,07, 14/733,468 and 15/601,261 assigned to Regents
of the University of Minnesota. The software can be freely used for educational
and research purposes by non-profit institutions and US government agencies
only. Other organizations are allowed to use the MARS VINS software only for
evaluation purposes, and any further uses will require prior approval from
http://z.umn.edu/marsvins. The software may not be sold or redistributed
without prior approval. One may make copies of the software for their use
provided that the copies, are not sold or distributed, are used under the same
terms and conditions.

As unestablished research software, this code is provided on an "as is" basis
without warranty of any kind, either expressed or implied. The downloading, or
executing any part of this software constitutes an implicit agreement to these
terms. These terms and conditions are subject to change at any time without
prior notice.

============================================================================

If you want to use this library in your research, please cite:

1. M. K. Paul, K. Wu, J. Hesch, E. Nerurkar, and S. I. Roumeliotis, 
“A Comparative Analysis of Tightly-coupled Monocular, Binocular, and Stereo 
VINS”, International Conference on Robotics and Automation (ICRA), 
Singapore, May 29-Jun. 3, 2017.

============================================================================

Notes: 
- The desktop version of the code is not optimized, please do not use it
for timing comparisons.
- Both global shutter and rolling shutter cameras are supported.
- The code currently supports 3 types of camera models: 1) equidistant, 2) fov, and 3) radial-tangential.
- A sample config file is provided in the "config" folder.
- The provided generic_driver.cpp and vins_example_app.cpp contatins drivers and example code to run the filter from logged data. Sample datasets are available at: http://mars.cs.umn.edu/research/stereo_vins.php.
If you want to try your own datasets or run on with online data, please modify the driver and example app accordingly.
- For the sample app, the input "imu-path" should contain the path of the IMU file. The IMU file should be formatted as:
   ```timestamp (s), gyro measurement (gx, gy, gz), accel measurement (ax, ay, az)```
- The input "image-path" should contain the path of the image folder. The image folder should contain the images in .pgm format and the image names should be in the format m0000000.pgm (e.g., m0000100.pgm).
The image folder should also contain a "timestamps.txt" file. The timestamps file should be formatted as:```
   image_name, timestamp (s)```
- The left-right images are expected to the timesynced, hence the corresponding timestamps.txt files should be identical.

============================================================================

## Quick Install Instructions
Follow these steps to build an example program on linux (built for Ubuntu 14.04)
This program will run stereo vins on a given dataset (see http://mars.cs.umn.edu/research/stereo_vins.php for some example datasets).
```
cd MARS-VINS
mkdir build
cd build
cmake .. -DPLATFORM=x86 -DCAMERA_MODEL=equidistant
make example_app
make install
./example_app <config file> <output file> <imu data file> <left image directory> <right image directory>
```
If you get this error ```error while loading shared libraries: libmars_core.so: cannot open shared object file: No such file or directory```, please add the "build" directory to your LD_LIBRARY_PATH.

```export LD_LIBRARY_PATH="path-to-build-directory":$LD_LIBRARY_PATH```
## Custom install
The following options for MARS-VINS are currently available:

Platforms:

- Linux : Intel-64bit (x86)

- Linux : ARM-64bit (arm64)

- Android : ARM-64-bit (arm64)

Camera models:

- equidistant

- fov 

- radial_tangential

example_app can be compiled for any of the above combinations in the following
fashion.

For linux builds:
```
cd MARS-VINS
cd build
cmake -DPLATFORM=<x86 or arm64> -DCAMERA_MODEL=<equidistant or fov or  or radial_tangential> ..
make example_app
make install
./example_app <config file> <output file> <imu data file> <left image directory> <right image directory>
```

For android builds (adb required to interface with android, android device should be rooted):
```
cd MARS-VINS
cd build_android
## edit cmake_android.sh to select your camera model
sh cmake_android.sh
make example_app
make install
adb push ./example_app <directory on android device>
adb shell
cd <directory on android device>
./example_app <config file> <output file> <imu data file> <left image directory> <right image directory>
```
