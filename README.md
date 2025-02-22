# arioviz_challenge

In this challenge, I decided to work with ROS Noetic on my Ubuntu 20.04 Virtual machine, because I already have them preinstalled.
I have picked the 1. Option as the challenge which is to control a simulated robot using the IMU’s motion data. I try to make everything simple by just making tilting forward/backward/left/right as the main control scheme. The simulated robot that I will be using is the TurtleBot3 in Gazebo.

The challenges are spiltted in to 6 Parts and are timestamped and documented as follows:
## 1. IMU Setup and Serial Communication (Sending)  13.10
   
![circuit](https://github.com/user-attachments/assets/552b00c2-2950-4a6a-87cf-357aae57403b)

I used an MPU6050 IMU connected to the ESP32 Node-MCU Module via an I2C connection on a 3.3v Voltage Supply. The code is straight-forward in which I chose to use the roll, pitch, yaw and linear acceleration of x, y and z values in this challenge due to the simplicity of implementation. The sensor fusion is already done internally by the DMP of the MPU6050 and it's already good enough to use for this purpose. I then send all of the relevant information via USB to the computer for further uses.

## 2. Serial Communication (Recieving) 15:30
   
I spent a quite a lot of time here fixing the buffer on this end and a bug causing packets send in big-edian format to not be usable.So I changed everything back to little-edian format including in the arduino side.

## 3. IMU INTERFACE (TO ROS MSG) 15:43
   
After getting the buffers and the recieved-values right, I wrote a simple ros node to process the data and publish it as a sensor message. Converted the roll, pitch, yaw to quaternions.

## 4. VISUAL OF IMU ON RVIZ 15:43
   
Just an imu topic visualization on RVIZ. Nothing special.
[IMU Visualization on RVIZ](https://www.youtube.com/watch?v=nJXcbCenSyE&feature=youtu.be)

## 5. IMU ROBOT CONTROL 16:52
   
I now wrote a seperated node to subscribe from the imu_inter node and translate the imu messages to the cmd_vel message for controlling the turtlebot. I also made a launch file for convenience in testing.
[IMU Robot Control](https://www.youtube.com/watch?v=q_BjnuRnBN0&ab_channel=zaldraxiz)

## 6. Playing around with filters 17.18
   
After everything is done, I went back to the imu_inter node and tried playing around with filters. I decided to use the complementary filter in this case because it was easy to implement but not too simple like the moving average. I know the EKF is likely the best option for this but if I were to use it, I have to change the packets in the arduino side to send angular velocity instead of roll, pitch, yaw. And I also have not much experience with EKF, so I decided to drop the idea to save time.

Problems encountered during the challenge
* The controls are a little bit sluggish and non-responsive. I think this is due to me using the DMP for sensor fusion, if I were to send the raw data and work on it on the ROS side, I would have more control and will fix it eventually. But it will take a lot of time.
