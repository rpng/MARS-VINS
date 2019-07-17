cmake_minimum_required(VERSION 2.8)
project(mars_vins)




set(PLATFORM x86) # x86, arm64




##################################################
# Dependencies
##################################################

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS roscpp rosbag std_msgs geometry_msgs nav_msgs sensor_msgs cv_bridge message_filters)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

# display message to user
message(STATUS "EIGEN VERSION: " ${EIGEN3_VERSION})
message(STATUS "OPENCV VERSION: " ${OpenCV_VERSION})
message(STATUS "BOOST VERSION: " ${Boost_VERSION})

# get main build library
if (ANDROID)
	set(core_lib_dir ${CMAKE_CURRENT_SOURCE_DIR}/core/lib_android64/)
	set(third_party_dir ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib_android64/)
elseif(${PLATFORM} STREQUAL "ARM64")
	set(core_lib_dir ${CMAKE_CURRENT_SOURCE_DIR}/core/lib_arm64/)
	set(third_party_dir ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib_arm64/)
elseif (${PLATFORM} STREQUAL "x86")
	set(core_lib_dir ${CMAKE_CURRENT_SOURCE_DIR}/core/lib_x86/)
	set(third_party_dir ${CMAKE_CURRENT_SOURCE_DIR}/third_party/lib_x86/)
endif()

# enable compile optimizations and c++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# describe catkin project
catkin_package(
	CATKIN_DEPENDS roscpp rosbag std_msgs geometry_msgs nav_msgs sensor_msgs cv_bridge message_filters
	INCLUDE_DIRS core/include/
)

# include our header files
include_directories(
	core/include/
	${EIGEN3_INCLUDE_DIR}
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

# Set link libraries used by all binaries
set(MARS_VINS_LIBS
	#-lmars_core
	-lgstreamer-0.10
	-lGLEW
	-lGL
	# liblibpng.a
	# liblibwebp.a
	# libzlib.a
	${third_party_dir}/libopencv_contrib.so
	${third_party_dir}/libopencv_nonfree.so
	${third_party_dir}/libopencv_videostab.so
	${third_party_dir}/libopencv_photo.so
	${third_party_dir}/libopencv_objdetect.so
	${third_party_dir}/libopencv_video.so
	${third_party_dir}/libopencv_ml.so
	${third_party_dir}/libopencv_calib3d.so
	${third_party_dir}/libopencv_features2d.so
	${third_party_dir}/libopencv_highgui.so
	${third_party_dir}/libopencv_imgproc.so
	${third_party_dir}/libopencv_core.so
)


##################################################
# Make binary files!
##################################################
add_executable(example_app_equidistant example_app/generic_driver.cpp example_app/vins_example_app.cpp)
target_link_libraries(example_app_equidistant ${core_lib_dir}/equidistant/libmars_core.so ${MARS_VINS_LIBS})

add_executable(example_app_fov example_app/generic_driver.cpp example_app/vins_example_app.cpp)
target_link_libraries(example_app_fov ${core_lib_dir}/fov/libmars_core.so ${MARS_VINS_LIBS})

add_executable(example_app_radial_tangential example_app/generic_driver.cpp example_app/vins_example_app.cpp)
target_link_libraries(example_app_radial_tangential ${core_lib_dir}/radial_tangential/libmars_core.so ${MARS_VINS_LIBS})


add_executable(ros_serial_equidistant example_ros/ros_driver.cpp example_ros/vins_ros_app.cpp)
target_link_libraries(ros_serial_equidistant ${core_lib_dir}/equidistant/libmars_core.so ${MARS_VINS_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ros_serial_fov example_ros/ros_driver.cpp example_ros/vins_ros_app.cpp)
target_link_libraries(ros_serial_fov ${core_lib_dir}/fov/libmars_core.so ${MARS_VINS_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

add_executable(ros_serial_radial_tangential example_ros/ros_driver.cpp example_ros/vins_ros_app.cpp)
target_link_libraries(ros_serial_radial_tangential ${core_lib_dir}/radial_tangential/libmars_core.so ${MARS_VINS_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES})


