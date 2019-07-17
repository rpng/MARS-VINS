/*
 * generic_driver.h
 *
 *  Created on: May 19, 2017
 *      Author: mrinalkanti
 */

#ifndef GENERIC_DRIVER_H_
#define GENERIC_DRIVER_H_

#include <iostream>
#include <string>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace MARS {

    struct Generic_Image {
        unsigned char *data;
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

        // CONSTRUCTOR FUNCTIONS
        GenericDriver(char *imu_path, char *left_image_path, char *right_image_path);
        ~GenericDriver();

        // API FUNCTIONS THAT ARE CALLED FROM MARS_CORE
        void startIMU();
        void startCamera(int cam_id);
        void getIMU(Generic_IMU *imu);
        void getImage(struct Generic_Image *img, int cam_id);

    private:

        // paths to txt files
        char *l_img_path_;
        char *r_img_path_;
        char *imu_path_;

        // file streams
        std::ifstream imu_file_;
        std::ifstream l_img_timestamp_file_;
        std::ifstream r_img_timestamp_file_;

        int l_index_, r_index_;


        // loads a pgm file from disk
        bool LoadPGMFile(struct Generic_Image *img, const std::string &file_name);



    }; // End of class: GenericDriver
} // End of namespace: MARS

#endif /* GENERIC_DRIVER_H_ */
