#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "imu_gps_localizer/imu_gps_localizer.h"
#include <GeographicLib/Geoid.hpp>
#include <GeographicLib/LocalCartesian.hpp>
class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);
    double ComputeHeightAboveGround(double latitude, double longitude, double ellipsoidal_height);
    
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Publisher state_pub_;

    std::ofstream file_state_;
    std::ofstream file_gps_;

    nav_msgs::Path ros_path_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
    
    ImuGpsLocalization::GpsPositionDataPtr previous_gps_data_ptr_; // 上一个 GPS 数据指针
    double accumulated_heading = 0.0; // 累积的朝向

};