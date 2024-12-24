# IMU & GPS localization
## Using EKF to fuse IMU and GPS data to achieve global localization.
### The code is implemented base on the book "Quaterniond kinematics for the error-state Kalman filter"

run:
roslaunch imu_gps_localization imu_gps_localization.launch

view:
sub topic: /fused_path ( nav_msgs::Path )
# gps-imu-eskf
