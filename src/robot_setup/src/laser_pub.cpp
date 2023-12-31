#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
// #include <math.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_scan_publisher");

  ros::NodeHandle n;
  ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 50);

  unsigned int num_readings = 1000;
  double laser_frequency = 40;
  double ranges[num_readings];
  double intensities[num_readings];

  int count = 10;
  ros::Rate r(1.0);
  while(n.ok()){
    //generate some fake data for our laser scan
    for(unsigned int i = 0; i < num_readings; ++i){
      if(i/num_readings > 0.5){
        ranges[i] = 10 * (1 - cos(2 * 3.14 * i/num_readings));  
      }else{
        ranges[i] = 3 * (1 - cos(2 * 3.14 * i/num_readings));
      }
      intensities[i] = 10*i;
    }
    ros::Time scan_time = ros::Time::now();

    //populate the LaserScan message
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "sensor_frame";
    scan.angle_min = -3.14;
    scan.angle_max = 3.14;
    scan.angle_increment = 6.28 / num_readings;
    scan.time_increment = (1 / laser_frequency) / (num_readings);
    scan.range_min = 0;
    scan.range_max = 200.0;

    scan.ranges.resize(num_readings);
    // ROS_INFO(scan.ranges);
    
    // std::count<< "list is: "<< str(scan.ranges) <<std::endl;

    scan.intensities.resize(num_readings);
    for(unsigned int i = 0; i < num_readings; ++i){
      // if( i < num_readings*.25){
      //   continue;
      // }
      scan.ranges[i] = ranges[i];
      scan.intensities[i] = intensities[i];
    }

    scan_pub.publish(scan);
    // ++count;
    ROS_WARN_STREAM("count"<<count);
    r.sleep();
  }
}