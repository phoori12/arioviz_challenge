#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#define MAX_LINEAR_SPEED 0.75   
#define MAX_ANGULAR_SPEED 0.5 

ros::Publisher cmd_vel_pub;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    yaw = -yaw; // Invert Rotation

    // if (fabs(yaw) >= 0.3) {
    //     yaw = 0;
    // } 

    // if (fabs(pitch) >= 0.3) {
    //     pitch = 0;
    // }

    // Map Pitch to Forward/Backward Motion (X-axis)
    double linear_x = MAX_LINEAR_SPEED * pitch; 
    linear_x = std::max(std::min(linear_x, MAX_LINEAR_SPEED), -MAX_LINEAR_SPEED);

    double linear_y = 0.0;  // Set to zero for TurtleBot (no strafing)

    // Map Yaw to Angular Velocity (Z-axis)
    double angular_z = MAX_ANGULAR_SPEED * yaw; 
    angular_z = std::max(std::min(angular_z, MAX_ANGULAR_SPEED), -MAX_ANGULAR_SPEED);

    geometry_msgs::Twist twist;
    twist.linear.x = linear_x;
    twist.linear.y = linear_y;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = angular_z;

    cmd_vel_pub.publish(twist);

    ROS_INFO("IMU -> cmd_vel | Pitch: %.2f | Roll: %.2f | Yaw: %.2f | X: %.2f | Z: %.2f",
             pitch, roll, yaw, linear_x, angular_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_to_cmdvel");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe("/imu/data", 1000, imuCallback);

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();
    return 0;
}
