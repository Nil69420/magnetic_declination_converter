#ifndef ORIENTATION_CONVERTER_HPP
#define ORIENTATION_CONVERTER_HPP

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <GeographicLib/MagneticModel.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <robot_localization/navsat_conversions.h>
#include <cmath>
#include <memory>

class OrientationConverter {
public:
    OrientationConverter();
    ~OrientationConverter();

private:
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
    void quaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg);

    ros::NodeHandle nh_;
    ros::Subscriber gps_sub_;
    ros::Subscriber quat_sub_;
    ros::Publisher corrected_pub_;

    double current_lat_;
    double current_lon_;
    bool gps_valid_;
    std::unique_ptr<GeographicLib::MagneticModel> mag_model_;
};

#endif // ORIENTATION_CONVERTER_HPP
