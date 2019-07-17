#ifndef ROS_WRAPPER_H
#define ROS_WRAPPER_H

#include <deque>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


namespace MARS {

    struct Generic_Image {
        unsigned char * data;
        int height;
        int width;
        int stride;
        double timestamp;
    };

    struct Generic_IMU {
        double accelX;
        double accelY;
        double accelZ;

        double gyroX;
        double gyroY;
        double gyroZ;

        double timestamp;
    };

    class GenericDriver {
    public:

        // Constructor and deconstructors
        GenericDriver(const ros::NodeHandle& n);
        GenericDriver(const GenericDriver&) = delete;
        GenericDriver operator=(const GenericDriver&) = delete;
        ~GenericDriver() {
            delete bag;
            delete view_imu;
            delete view_cam0;
            delete view_cam1;
        }

        // Interface required by the mars_core library.
        void startCamera(int cam_id);
        void getImage(struct Generic_Image* img, int cam_id);
        void startIMU();
        void getIMU(Generic_IMU* imu);

        // number of images we have
        size_t num_images() {
            return view_cam0->size();
        }

        // gets what was the last image we processed
        double last_timestamp() {
            return last_left_time;
        }

    private:

        double last_left_time = -1;


        rosbag::Bag* bag;

        rosbag::View* view_imu;
        rosbag::View::iterator view_imu_iter;
        rosbag::View* view_cam0;
        rosbag::View::iterator view_cam0_iter;
        rosbag::View* view_cam1;
        rosbag::View::iterator view_cam1_iter;


    };

//typedef GenericDriver::Ptr GenericDriverPtr;
//typedef GenericDriver::ConstPtr GenericDriverConstPtr;

} // end namespace MARS

#endif