
#include "ros_driver.h"




namespace MARS {

    GenericDriver::GenericDriver(const ros::NodeHandle& nh) {

        //===================================================================================
        //===================================================================================
        //===================================================================================

        // Our camera topics (left and right stereo)
        std::string topic_imu;
        std::string topic_camera0;
        std::string topic_camera1;
        nh.param<std::string>("topic_imu", topic_imu, "/imu0");
        nh.param<std::string>("topic_camera0", topic_camera0, "/cam0/image_raw");
        nh.param<std::string>("topic_camera1", topic_camera1, "/cam1/image_raw");

        // Location of the ROS bag we want to read in
        std::string path_to_bag;
        nh.param<std::string>("path_bag", path_to_bag, "/datasets/eth/V1_01_easy.bag");
        ROS_INFO("ros bag path is: %s", path_to_bag.c_str());

        // Get our start location and how much of the bag we want to play
        // Make the bag duration < 0 to just process to the end of the bag
        double bag_start, bag_durr;
        nh.param<double>("bag_start", bag_start, 0);
        nh.param<double>("bag_durr", bag_durr, -1);
        ROS_INFO("bag start: %.1f",bag_start);
        ROS_INFO("bag duration: %.1f",bag_durr);

        //===================================================================================
        //===================================================================================
        //===================================================================================

        // Load rosbag here, and find messages we can play
        bag = new rosbag::Bag(path_to_bag, rosbag::bagmode::Read);

        // Start a few seconds in from the full view time
        // If we have a negative duration then use the full bag length
        rosbag::View view_full;
        view_full.addQuery(*bag);
        ros::Time time_init = view_full.getBeginTime();
        time_init += ros::Duration(bag_start);
        ros::Time time_finish = (bag_durr < 0)? view_full.getEndTime() : time_init + ros::Duration(bag_durr);
        ROS_INFO("time start = %.6f", time_init.toSec());
        ROS_INFO("time end   = %.6f", time_finish.toSec());

        // our views
        view_imu = new rosbag::View(*bag, rosbag::TopicQuery(topic_imu), time_init, time_finish);
        view_imu_iter = view_imu->begin();
        view_imu_iter++;
        view_cam0 = new rosbag::View(*bag, rosbag::TopicQuery(topic_camera0), time_init, time_finish);
        view_cam0_iter = view_cam0->begin();
        view_cam1 = new rosbag::View(*bag, rosbag::TopicQuery(topic_camera1), time_init, time_finish);
        view_cam1_iter = view_cam1->begin();

    }

    void GenericDriver::startCamera(int cam_id) {
        return;
    }

    void GenericDriver::getImage(struct Generic_Image* img, int cam_id) {

        // Defaults
        img->data = nullptr;

        // copy into our new memory
        if (cam_id == 0) {
            ROS_INFO("Get image from cam0...");
            sensor_msgs::Image::ConstPtr msg = view_cam0_iter->instantiate<sensor_msgs::Image>();
            assert(msg != nullptr);
            cv::Mat imgdata = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
            img->timestamp = msg->header.stamp.toSec();
            last_left_time = msg->header.stamp.toSec();
            img->height = imgdata.rows;
            img->width = imgdata.cols;
            img->stride = img->width;
            img->data = (unsigned char*) malloc(img->width * img->height * sizeof(unsigned char));
            for(int i=0; i<img->height; i++) {
                for (int j = 0; j < img->width; j++) {
                    img->data[i*img->width+j] = imgdata.at<unsigned char>(i,j);
                }
            }
            view_cam0_iter++;
        } else if (cam_id == 1 && last_left_time != -1) {
            ROS_INFO("Get image from cam1...");

            // Ensure that our next image is greater than the current left timestamp
            // This ensures that the left and right images have simular timestamps
            rosbag::View::iterator view_cam1_iter_temp = view_cam1_iter;
            view_cam1_iter_temp++;
            sensor_msgs::Image::ConstPtr msg_next = view_cam1_iter_temp->instantiate<sensor_msgs::Image>();
            while(msg_next->header.stamp.toSec() < last_left_time) {
                ROS_INFO("moving right cam iter forward (dt = %.3f)",(last_left_time-msg_next->header.stamp.toSec()));
                view_cam1_iter_temp++;
                msg_next = view_cam1_iter_temp->instantiate<sensor_msgs::Image>();
                view_cam1_iter++;
            }

            // Now get the current message
            sensor_msgs::Image::ConstPtr msg = view_cam1_iter->instantiate<sensor_msgs::Image>();
            assert(msg != nullptr);
            cv::Mat imgdata = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8)->image;
            img->timestamp = last_left_time;
            img->height = imgdata.rows;
            img->width = imgdata.cols;
            img->stride = img->width;
            img->data = (unsigned char*) malloc(img->width * img->height * sizeof(unsigned char));
            for(int i=0; i<img->height; i++) {
                for (int j = 0; j < img->width; j++) {
                    // MONO: a black image so tracking fails on the right
                    //img->data[i*img->width+j] = 0.0;
                    // STEREO: use the actual pixel value
                    img->data[i*img->width+j] = imgdata.at<unsigned char>(i,j);
                }
            }
            view_cam1_iter++;
        } else {
            ROS_ERROR("Cannot find camera %d...", cam_id);
        }


    }

    void GenericDriver::startIMU() {
        return;
    }

    void GenericDriver::getIMU(Generic_IMU* imu) {

        ROS_INFO("Get imu...");

        // Get the next IMU msg.
        sensor_msgs::Imu::ConstPtr msg = view_imu_iter->instantiate<sensor_msgs::Imu>();
        assert(msg != nullptr);

        // Set the imu data.
        imu->timestamp = msg->header.stamp.toSec();
        imu->gyroX = msg->angular_velocity.x;
        imu->gyroY = msg->angular_velocity.y;
        imu->gyroZ = msg->angular_velocity.z;
        imu->accelX = msg->linear_acceleration.x;
        imu->accelY = msg->linear_acceleration.y;
        imu->accelZ = msg->linear_acceleration.z;

        // Move forward in time
        view_imu_iter++;

        return;
    }


} // end namespace MARS