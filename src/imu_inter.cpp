#include "ros/ros.h"
#include "serial/serial.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include "tf2/LinearMath/Quaternion.h"
#include <iostream>
#include <vector>
#include <cstring> 

#define PACKET_SIZE 28 

float alpha = 0.90; // Tuning parameter (0.95 - 0.99 works well)
float roll_filtered = 0.0, pitch_filtered = 0.0, yaw_filtered = 0.0;
float dt = 0.005;

float bytesToFloat(const uint8_t *bytes)
{
    float value;
    memcpy(&value, bytes, sizeof(float)); 
    return value;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imu_interface");
    ros::NodeHandle n;
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 1000);
    ros::Rate loop_rate(200);

    // Open Serial Port
    serial::Serial ser;
    try
    {
        ser.setPort("/dev/ttyUSB1");  
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR("Unable to open port");
        return -1;
    }

    if (ser.isOpen())
    {
        ROS_INFO("Serial Port initialized");
    }
    else
    {
        return -1;
    }   

    std::vector<uint8_t> buffer(PACKET_SIZE);
    int cmdIndex = 0;
    while (ros::ok())
    {
        if (ser.available() > 0)
        {
            uint8_t byte;
            ser.read(&byte, 1);

            if (byte == '#')  
            {
                buffer[0] = byte;
                ser.read(&buffer[1], PACKET_SIZE - 1); 

                // // Debug: Print raw packet
                // printf("Raw Packet: ");
                // for (int i = 0; i < PACKET_SIZE; i++)
                // {
                //     printf("%02X ", buffer[i]);
                // }
                // printf("\n");

                if (buffer[0] == '#' && buffer[1] == 's' && buffer[26] == '\r' && buffer[27] == '\n')
                {
                    float ax = bytesToFloat(&buffer[2]);
                    float ay = bytesToFloat(&buffer[6]);
                    float az = bytesToFloat(&buffer[10]);
                    float gy = bytesToFloat(&buffer[14]);
                    float gp = bytesToFloat(&buffer[18]);
                    float gr = bytesToFloat(&buffer[22]);

                    // TODO: FILTERS
                    roll_filtered = alpha * (roll_filtered + gy * dt) + (1 - alpha) * gr;
                    pitch_filtered = alpha * (pitch_filtered + gy * dt) + (1 - alpha) * gp;
                    yaw_filtered = alpha * yaw_filtered + (1 - alpha) * gy;

                    tf2::Quaternion q;
                    q.setRPY(roll_filtered * M_PI / 180.0, pitch_filtered * M_PI / 180.0, yaw_filtered * M_PI / 180.0); 
                    q.normalize();

                    // Publish IMU message
                    sensor_msgs::Imu imu_msg;
                    imu_msg.header.stamp = ros::Time::now();
                    imu_msg.header.frame_id = "imu_link";

                    imu_msg.orientation.x = q.x();
                    imu_msg.orientation.y = q.y();
                    imu_msg.orientation.z = q.z();
                    imu_msg.orientation.w = q.w();

                    imu_msg.linear_acceleration.x = ax;
                    imu_msg.linear_acceleration.y = ay;
                    imu_msg.linear_acceleration.z = az;

                    imu_pub.publish(imu_msg);

                    // ROS_INFO("IMU Data: ax=%.3f ay=%.3f az=%.3f gy=%.3f gp=%.3f gr=%.3f", ax, ay, az, gy, gp, gr);
                }
                else
                {
                    ROS_WARN("Packet misaligned, discarding...");
                }
            }
        }
       

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}