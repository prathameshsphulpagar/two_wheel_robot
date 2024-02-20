#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <fstream>

std::ofstream MyFile; // to open the CSV file to store the data

void storage_front_mid_callback(const geometry_msgs::Twist &msg);

void storage_front_mid_callback(const geometry_msgs::Twist &msg)
{
	//MyFile.open("lidar_storage_csv.log",std::ios::app);
	MyFile <<msg.linear.x <<"\t" <<msg.linear.y << "\t"<<msg.linear.z <<"\n";

		ROS_INFO("X %f",msg.linear.x);
			ROS_INFO("Y %f",msg.linear.y);
				ROS_INFO("Z %f",msg.linear.z );
			//		MyFile.close();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "storage_node"); // creating node
	ros::NodeHandle n;
	ROS_INFO("Storage code");
	ros::Rate loop_rate(10); // frequency rate
	
 	MyFile.open("lidar_storage_csv.log");//,std::ios::app);
	MyFile << "Angle default."<<"\t" << "Distance"<<"\t" <<"quality" <<"\n";	
	//MyFile.close();

ros::Subscriber sub_front_mid_lidar = n.subscribe ("/storage_front_mid_topic", 6000, storage_front_mid_callback);
loop_rate.sleep(); // sleep
ros::spin();
	return 0;
}
