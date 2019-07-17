

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <cv_bridge/cv_bridge.h>


#include <thread>
#include <fstream>
#include <boost/filesystem.hpp>

#include "ros_driver.h"
#include "core/mars-vins-facade.h"


using namespace MARS;


// Main function
int main(int argc, char** argv)
{

    // Launch our ros node
    ros::init(argc, argv, "mars_vins");
    ros::NodeHandle nh("~");

    // Get the path for calibration file and output file.
    std::string calib_file;
    std::string output_file;
    if (!nh.getParam("calib_file", calib_file))
        ROS_ERROR("Cannot get calibration file...");
    if (!nh.getParam("output_file", output_file))
        ROS_ERROR("Cannot get output file...");

    // Initialize the ros wrapper for vins.
    ROS_INFO("Initialize ros wrapper...");
    GenericDriver* driver = new MARS::GenericDriver(nh);
    std::unique_ptr<MARS::GenericDriver> driverptr = std::unique_ptr<MARS::GenericDriver>(driver);
    MARS::MARSVinsFacade mars_vins(std::move(driverptr), calib_file.c_str(), output_file.c_str());

    // Read in if we should be saving or not
    bool do_save;
    std::string filename;
    nh.param<bool>("do_save", do_save, false);
    nh.param<std::string>("record_file", filename, "output.txt");

    // If doing save open our file
    std::ofstream outfile;
    if(do_save) {
        // Create folder path to this location if not exists
        boost::filesystem::path dir(filename.c_str());
        if(boost::filesystem::create_directories(dir.parent_path())) {
            ROS_INFO("Created folder path to output file.");
            ROS_INFO("Path: %s",dir.parent_path().c_str());
        }
        // If it exists, then delete it
        if(boost::filesystem::exists(filename)) {
            ROS_WARN("Output file exists, deleting old file....");
            boost::filesystem::remove(filename);
        }
        // Open this file we want to write to
        outfile.open(filename.c_str());
        if(outfile.fail()) {
            ROS_ERROR("Unable to open output file!!");
            ROS_ERROR("Path: %s",filename.c_str());
            std::exit(EXIT_FAILURE);
        }
        outfile << "# timestamp(s) tx ty tz qx qy qz qw" << std::endl;
    }


    for (size_t i = 0; i < driver->num_images()-2 && ros::ok(); ++i) {

        // Process the next image
        mars_vins.RunSingleImage();

        // Get the current estimate
        MARS::Pose pose;
        mars_vins.GetPose(&pose);
        std::cout << "x: " << pose.global_P_imu[0] << " y: " << pose.global_P_imu[1] << " z: " << pose.global_P_imu[2] << std::endl;

        // Save the current estimate to file if we need to
        if(do_save) {

            // timestamp
            outfile.precision(5);
            outfile.setf(std::ios::fixed, std::ios::floatfield);
            outfile << driver->last_timestamp() << " ";

            // pose
            outfile.precision(6);
            outfile << pose.global_P_imu[0] << " " << pose.global_P_imu[1] << " " << pose.global_P_imu[2] << " "
                    << pose.imu_q_global[0] << " " << pose.imu_q_global[1] << " " << pose.imu_q_global[2] << " " << pose.imu_q_global[3] << std::endl;

        }

    }


    // Done!
    return EXIT_SUCCESS;

}


