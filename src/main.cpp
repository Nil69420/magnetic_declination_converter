#include "magnetic_declination_converter/orientation_converter.hpp"
#include <thread>

int main(int argc, char** argv) {
    ros::init(argc, argv, "orientation_converter");

    // Instantiate the converter.
    OrientationConverter converter;

    std::thread rosThread([](){
        ros::spin();
    });


    rosThread.join();

    return 0;
}
