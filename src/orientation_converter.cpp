#include "magnetic_declination_converter/orientation_converter.hpp"

OrientationConverter::OrientationConverter() : current_lat_(0.0), current_lon_(0.0), gps_valid_(false) {
    gps_sub_ = nh_.subscribe("/gps/filtered", 1, &OrientationConverter::gpsCallback, this);
    quat_sub_ = nh_.subscribe("/filter/quaternion", 10, &OrientationConverter::quaternionCallback, this);
    corrected_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>("/corrected_orientation", 10);

    try {
        mag_model_ = std::make_unique<GeographicLib::MagneticModel>("wmm2025");
    } catch (const GeographicLib::GeographicErr& e) {
        ROS_ERROR("Magnetic model init failed: %s", e.what());
        ros::shutdown();
    }
}

OrientationConverter::~OrientationConverter() {
}

void OrientationConverter::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (msg->status.status >= sensor_msgs::NavSatStatus::STATUS_FIX) {
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        gps_valid_ = true;
    }
}

void OrientationConverter::quaternionCallback(const geometry_msgs::QuaternionStamped::ConstPtr& msg) {
    if (!gps_valid_) {
        ROS_WARN_THROTTLE(5, "Waiting for valid GPS data...");
        return;
    }

    try {
        tf2::Quaternion tf_quat;
        tf2::fromMsg(msg->quaternion, tf_quat);
        double roll, pitch, yaw;
        tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);

        double Bx, By, Bz;
        double current_time_years = 2025.3; 
        mag_model_->operator()(current_time_years, current_lat_, current_lon_, 0, Bx, By, Bz);
        double declination_rad = std::atan2(By, Bx);

        double true_yaw = yaw + declination_rad;
        tf2::Quaternion corrected_quat;
        corrected_quat.setRPY(roll, pitch, true_yaw);

        geometry_msgs::QuaternionStamped corrected_msg;
        corrected_msg.header = msg->header;
        corrected_msg.quaternion = tf2::toMsg(corrected_quat);
        corrected_pub_.publish(corrected_msg);

    } catch (const GeographicLib::GeographicErr& e) {
        ROS_WARN_THROTTLE(60, "Magnetic model error: %s", e.what());
    }
}
