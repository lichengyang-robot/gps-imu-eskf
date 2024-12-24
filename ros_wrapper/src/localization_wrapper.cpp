#include "localization_wrapper.h"

#include <iomanip>

#include <glog/logging.h>

#include "imu_gps_localizer/base_type.h"

#include <GeographicLib/Geoid.hpp>
    double height=0.0;
    Eigen::Vector3d origin_position;
    bool    is_first = true;
    bool first_orthometric_height=true;
    Eigen::Quaterniond origin_orientation;
    double init_orthometric_height;
void ConvertLLAToENU(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_lla, 
                            Eigen::Vector3d* point_enu) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), 
                            point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
}
LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    std::string log_folder = "/home";
    ros::param::get("log_folder", log_folder);

    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder +"/gps.csv");

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);

    // Subscribe topics.
    imu_sub_ = nh.subscribe("/imu/data", 10,  &LocalizationWrapper::ImuCallback, this);
    gps_position_sub_ = nh.subscribe("/gps/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);

    state_pub_ = nh.advertise<nav_msgs::Path>("/fused_path", 10);
}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    
    ImuGpsLocalization::State fused_state;
    const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
    if (!ok) {
        return;
    }

    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    state_pub_.publish(ros_path_);

    // Log fused state.
    LogState(fused_state);
}

double LocalizationWrapper::ComputeHeightAboveGround(double latitude, double longitude, double ellipsoidal_height) {
    try {
        // 使用GeographicLib库中的Geoid类来获取大地水准面高程
        // GeographicLib::Geoid geoid("egm96-5");  // 使用EGM96地球重力模型的文件
        GeographicLib::Geoid geoid("egm2008-1");  // 使用EGM2008模型
        
        // 验证输入的纬度和经度是否在合理范围内
        if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
            throw std::out_of_range("Latitude or longitude out of range.");
        }
        
        // 根据给定的经纬度获取大地水准面高度
        double geoid_height = geoid(latitude, longitude);

        // 计算正高（orthometric height），即相对于水准面的高度
        double orthometric_height = ellipsoidal_height - geoid_height;
        if(first_orthometric_height){
            init_orthometric_height = orthometric_height;
            std::cout << "Init Orthometric height: " << init_orthometric_height << " meters" << std::endl;
            first_orthometric_height=false;
        }

        // 输出Geoid height、Ellipsoidal height和Orthometric height
        std::cout << "Geoid height: " << geoid_height << " meters" << std::endl;
        std::cout << "Ellipsoidal height: " << ellipsoidal_height << " meters" << std::endl;
        std::cout << "Orthometric height: " << orthometric_height - init_orthometric_height<< " meters" << std::endl;
        std::cout << "Init Orthometric height: " << init_orthometric_height << " meters" << std::endl;
        // 返回正高
        return orthometric_height - init_orthometric_height;
    }
    catch (const std::exception& e) {  // 使用标准异常处理
        // 捕获所有异常并输出错误信息
        std::cerr << "Error computing orthometric height: " << e.what() << std::endl;
        return std::numeric_limits<double>::quiet_NaN();  // 返回NaN表示错误
    }
}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // // Check the gps_status.
    // if (gps_msg_ptr->status.status != 2) {
    //     LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
    //     return;
    // }

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    height = ComputeHeightAboveGround(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude);

    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    // 如果有上一个 GPS 数据，则计算速度和朝向
    if (previous_gps_data_ptr_) {
        double delta_time = gps_data_ptr->timestamp - previous_gps_data_ptr_->timestamp;

        // 将当前和上一个 GPS 数据转换为 ENU 坐标系
        Eigen::Vector3d current_enu, previous_enu;
        ConvertLLAToENU(previous_gps_data_ptr_->lla, gps_data_ptr->lla, &current_enu);
        ConvertLLAToENU(previous_gps_data_ptr_->lla, previous_gps_data_ptr_->lla, &previous_enu);

        // 计算 ENU 坐标系中的速度
        double vx = (current_enu(0) - previous_enu(0)) / delta_time; // 东向速度
        double vy = (current_enu(1) - previous_enu(1)) / delta_time; // 北向速度

        // 计算朝向（相对于东北方向）
        double heading = atan2(vy, vx); // 计算偏航角 (yaw)
        accumulated_heading += heading;  // 累加朝向
        gps_data_ptr->heading = heading;  // 可以选择将当前朝向存储在 gps_data_ptr 中
        std::cout <<"0°:面向北;90°:面向东;180°:面向南;270°:面向西;360°(或 0°):回到面向北"<<std::endl;
        std::cout << "gps_data_ptr->heading (ENU) = " << gps_data_ptr->heading * 180 / M_PI << " degrees" << std::endl; // 输出朝向（以度为单位）
    } else {
        // 初始化情况下，没有前一帧数据，默认朝向为0
        gps_data_ptr->heading = 0;
    }

    // 更新前一个 GPS 数据
    previous_gps_data_ptr_ = gps_data_ptr;

    imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);

    LogGps(gps_data_ptr);
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    ros_path_.header.frame_id = "camera_init";
    ros_path_.header.stamp = ros::Time().fromSec(state.timestamp); 

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    ros_path_.poses.push_back(pose);
}


// void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
//     if (is_first) {
//         // 如果是第一次调用，保存当前位姿和姿态作为原点
//         origin_position = Eigen::Vector3d(state.G_p_I[0], state.G_p_I[1], state.G_p_I[2]);
//         origin_orientation = Eigen::Quaterniond(state.G_R_I); // 保存原始的四元数
//         is_first = false;
//     }

//     ros_path_.header.frame_id = "camera_init";
//     ros_path_.header.stamp = ros::Time::now();  

//     geometry_msgs::PoseStamped pose;
//     pose.header = ros_path_.header;

//     // 计算相对位姿
//     Eigen::Vector3d current_position(state.G_p_I[0], state.G_p_I[1], state.G_p_I[2]);
//     Eigen::Vector3d relative_position = current_position - origin_position;
//     std::cout << "relative_position = " << relative_position.transpose() << std::endl;

//     pose.pose.position.x = relative_position[0];
//     pose.pose.position.y = relative_position[1];
//     pose.pose.position.z = relative_position[2];

//     // 当前的四元数
//     const Eigen::Quaterniond current_orientation(state.G_R_I);

//     // 计算相对旋转：用当前四元数乘以原始四元数的共轭，得到相对旋转
//     Eigen::Quaterniond relative_orientation = current_orientation * origin_orientation.inverse();

//     // 将相对旋转角度转换为四元数表示
//     pose.pose.orientation.x = relative_orientation.x();
//     pose.pose.orientation.y = relative_orientation.y();
//     pose.pose.orientation.z = relative_orientation.z();
//     pose.pose.orientation.w = relative_orientation.w();

//     // 输出相对旋转角度
//     Eigen::Vector3d relative_euler = relative_orientation.toRotationMatrix().eulerAngles(0, 1, 2); // roll, pitch, yaw
//     std::cout << "Relative Euler Angles (roll, pitch, yaw) = " << relative_euler.transpose() << std::endl;

//     // 将姿态推入路径
//     ros_path_.poses.push_back(pose);
// }
