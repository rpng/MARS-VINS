
#FROM ros:indigo-ros-core-trusty
FROM nvidia/opencl:devel-ubuntu14.04

# install ros indigo
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get -y update
RUN apt-get -y upgrade

RUN apt-get -y install ros-indigo-ros-base
RUN apt-get -y install ros-indigo-cv-bridge ros-indigo-image-transport



# install any dependencies we need
# note: we need the qt4 and gstream as mars' opencv was built with it
RUN apt-get -y install gdb
RUN apt-get -y install libeigen3-dev
RUN apt-get -y install libglew-dev
RUN apt-get -y install libqt4-dev libqt4-core libqt4-gui libqt4-xml libqt4-opengl
RUN apt-get -y install libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev
RUN apt-get -y install libglu1-mesa-dev freeglut3-dev mesa-common-dev mesa-utils libgl1-mesa-glx
RUN apt-get -y install nano
RUN apt-get -y install libboost-all-dev

# X11 gui stuff
RUN apt-get install -qqy x11-apps

# copy in source code
RUN mkdir -p /ws/src/mars_vins/
COPY . /ws/src/mars_vins/
COPY docker_run.sh /docker_run.sh
COPY docker_build.sh /docker_build.sh
COPY docker_mc.sh /docker_mc.sh

# load the ROS enviromential variables
RUN bash /opt/ros/indigo/setup.bash

# load our nvidia driver from the host file
# https://github.com/jessfraz/dockerfiles/issues/253#issuecomment-373043685
ENV PATH /usr/lib/nvidia-384/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/lib/nvidia-384:/usr/lib32/nvidia-384:${LD_LIBRARY_PATH}









