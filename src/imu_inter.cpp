#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <iostream>

int main(int argc, char** argv) {
    std::cout << "Before ros::init()" << std::endl;
    ROS_INFO("ros::ok() status: %d", ros::ok());
    ROS_INFO("NODE STARTED");
    ros::init(argc, argv, "imu_interface");
    std::cout << "afgter ros::init()" << std::endl;
    ros::NodeHandle n;

    ros::Publisher imu_pub = n.advertise<std_msgs::String>("imu", 1000);
    
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;
    
        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();
    
        ROS_INFO("%s", msg.data.c_str());
    
        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        imu_pub.publish(msg);
    
        ros::spinOnce();
    
        loop_rate.sleep();
        ++count;
    }
  
    return 0;
}