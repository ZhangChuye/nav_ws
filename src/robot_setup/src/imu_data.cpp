
#include <ros/ros.h> 
#include <sensor_msgs/Imu.h>
 
int main(int argc, char** argv)
{
    // Step 2: Initialization:
       ros::init(argc, argv, "imu");     
    ros::NodeHandle n;  
 
    ros::Publisher IMU_pub = n.advertise<sensor_msgs::Imu>("IMU_data", 20);  
    ros::Rate loop_rate(5);  
    int i=0;
    while(ros::ok())
    {
                   
            sensor_msgs::Imu imu_data;
            imu_data.header.stamp = ros::Time::now();
            imu_data.header.frame_id = "base_link";
            
            imu_data.orientation.x = 0 + sin(i);
            imu_data.orientation.y = -1 + cos(i);
            imu_data.orientation.z = -5 + sin(i);
            imu_data.orientation.w = 6 + cos(i);
            //线加速度
            imu_data.linear_acceleration.x = 0.01 + sin(i); 
            imu_data.linear_acceleration.y = 0.02 + sin(i);
            imu_data.linear_acceleration.z = 0.03 + sin(i);
            //角速度
            imu_data.angular_velocity.x = 0.05 + sin(i); 
            imu_data.angular_velocity.y = 0.06 + sin(i); 
            imu_data.angular_velocity.z = 0.07 + sin(i);
            i++;
 
            IMU_pub.publish(imu_data);
       
 
       
        ros::spinOnce();  
        loop_rate.sleep();  
    }
 
    return 0;
}