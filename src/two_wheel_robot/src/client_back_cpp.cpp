#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <fstream>

#define RAD2DEG(x) ((x)*180. / M_PI)

float degree_max, degree, distance_avg = 0;
int degree_int, degree_compare = 0, count_angle = 0;
int flag_back = 0,flag_disp = 0;
time_t start, end; // creating global object
geometry_msgs::Twist lidar_flags;
std_msgs::Float32 degree_macx_f32, distance_f32, quality_f32;
std_msgs::Int32 obstacle_detection_flag;

ros::Publisher pub_sensing_stoping; 
	
void check_obstacle(); // this function check the obstacle came in 360 degree rotation
void find_path();	   // this function high the flags and check the condition where to go and what to do
void timer_function();
void all_conditions();

ros::Publisher sensing_stoping_pub; // to publish the sensing stoping data

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{ 
	int count = scan->scan_time / scan->time_increment;
	
	for (int i = 0; i < count; i++)
	{ // for loop for reading the angle and its value

		degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // this store that currently at which angle we are.

		distance_f32.data = scan->ranges[i];	 // this store the distance value at that angle
		quality_f32.data = scan->intensities[i]; // check the quality of that data

		if (scan->intensities[i] == 0)
		{						   // quality_f64.data = 0;
			distance_f32.data = 0; // if the intensity is zero then this convert distance from infinite to zero
		}
		
		all_conditions();
	}
//	find_path(); // find path after one lidar rotation which side obstacle is there
	time(&end);	  // end time
	lidar_flags.angular.x = flag_back;

	flag_back = 0; // flag to restart the lidar detected possition

}
void all_conditions()
{
if ((degree >= -10.0 && degree <= 0.00000) || (degree >= 0.000000 && degree <= 10.0))
		{
			if (distance_f32.data <= 2.5000 && distance_f32.data >= 1.0000)  // 2 feet distance
		//if (distance_f32.data <= 0.140 && distance_f32.data >= 0.010)
			{	
				ROS_WARN_STREAM("\033[1;31mVehical is comming take a side. \033[0m");
			}		

		}	


if ((degree >= -30.0 && degree <= 0.00000) || (degree >= 0.000000 && degree <= 30.0))
		{			
			if (distance_f32.data <= 1.00000 && distance_f32.data >= 0.3000)
			//if (distance_f32.data <= 0.14000 && distance_f32.data >= 0.0300) 
			{
				check_obstacle(); // if lidar distance cross sefty limit then it should check at which side the obstacle is there
			}
		}

	if ((degree >= 30.000 && degree <= 90.000) || (degree >= -90.000 && degree <= -30.000))
		{			
		if (distance_f32.data <= 0.85000 && distance_f32.data >= 0.7000)
			{
				if (degree >= 30.0 && degree <= 90.0)
				{
				//	if (flag_disp != 1)
				//	{
						ROS_INFO("Go carefully, Backside RIGHT obstacle");
				//		flag_disp = 1;
					//}
				}

				if (degree >= -90.0 && degree <= -30.0)
				{
				//	if (flag_disp != 2)
				//	{
						ROS_INFO("Go carefully, Backside LEFT obstacle");
				//		flag_disp = 2;
				//	}
				}
			}

	if (distance_f32.data <= 0.7000 && distance_f32.data >= 0.0100) 
			//if (distance_f32.data <= 0.1400 && distance_f32.data >= 0.0100) 
			{
				check_obstacle(); // if lidar distance cross sefty limit then it should check at which side the obstacle is there
			}
		}
}
void check_obstacle()
{
	// these are all angle dividation

	if (degree >= 30.0 && degree <= 90.0)
	{
	
		//ROS_INFO("in left");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		ROS_WARN_STREAM("Backside obstacle on RIGHT side take Left turn");
		flag_back = 1;
	}

if ((degree >= -30.0 && degree <= 0.00000) || (degree >= 0.000000 && degree <= 30.0))
	// if(degree_compare < 40  && degree_compare >3 )
	{
		// ROS_INFO("in front");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		ROS_WARN_STREAM("Backside obstacle on center");
		flag_back = 1;
	}

	if (degree >= -90.0 && degree <= -30.0)
	{
		// ROS_INFO("in right");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		ROS_WARN_STREAM("Backside obstacle on LEFT side take Right turn");
		flag_back = 1;
	}

}
/*void check_obstacle()
{
if ((degree >= -90.0 && degree <= 0.00000000000) || (degree >= 0.00000000000 && degree <= 90.0))
//	if (degree >= -90.0 && degree <= 90.0)
	{
		flag_back = 1;
	}
}*/

void find_path()
{
	//if (flag_back == 1)
	//{
		//ROS_WARN_STREAM("stop backside obstacle");
		//obstacle_detection_flag.data = 8;

	//}

	//ROS_INFO(" = %d", obstacle_detection_flag);
//	sensing_stoping_pub.publish(obstacle_detection_flag);
}
// timer function
void timer_function()
{
	double time_taken = double(end - start); // time difference
	if (time_taken == -5)
	{ // condition
		 ROS_INFO("lidar (main) is get killed");
		ROS_WARN_STREAM("lidar back is get killed");
		system("rosnode kill rplidar_node"); // killer command
		system("rosnode kill rplidar_client_node"); // killer command
		system("rosnode kill rplidar_client_front_mid_node"); // killer command
		system("rosnode kill teleop_twist_keyboard_node"); // killer command
		system("rosnode kill sensing_stoping_node"); // killer command
		system("rosnode kill combine_six_lidar_node"); // killer command
		system("rosnode kill wall_following_node"); // killer command
		system("rosnode kill node_restart_node"); // killer command
		exit(0);
	}
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_client_back_node"); // creating node
	ros::NodeHandle n;
	ROS_INFO("Lidar Client back code");
	ros::Rate loop_rate(10); // frequency rate

	ros::Subscriber sub = n.subscribe <sensor_msgs::LaserScan> ("/scan_back", 10, scanCallback); // taking scan lidar data from rplidar pakage

	pub_sensing_stoping = n.advertise <geometry_msgs::Twist> ("sensing_stoping_lidar_back_topic", 1);
	
	while (ros::ok())     
	{
		time(&start);	  // start time
		timer_function(); // call timer function
		pub_sensing_stoping.publish(lidar_flags);
		loop_rate.sleep(); // sleep	
		ros::spinOnce();
	}
	//ros::spin();
	return 0;
}
