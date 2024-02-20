#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <fstream>

#define RAD2DEG(x) ((x)*180. / M_PI)

float degree_max, degree, distance_avg = 0;
int degree_int, degree_compare = 0, count_angle = 0;
int flag_left = 0, flag_right = 0, flag_front = 0, flag_back = 0;
int flag_all_node_kill = 0; // Do 1 if want to kill all node if lidar gets off

std_msgs::Float32 degree_max_f32, distance_f32, quality_f32;
std_msgs::Int32 obstacle_detection_flag;
geometry_msgs::Point publish_data;
geometry_msgs::Twist lidar_flags,storage_lidar_values;
geometry_msgs::Twist wall_distance;
time_t start, end; // creating global object


ros::Publisher pub_storage;
ros::Publisher pub_wall_following;

void check_obstacle(); // this function check the obstacle came in 360 degree rotation
void find_path();	   // this function high the flags and check the condition where to go and what to do
void timer_function();

ros::Publisher sensing_stoping_pub; // to publish the sensing stoping data

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
	float avg_distance_l_wall = 0;
	float avg_distance_r_wall = 0;
	int count_avg_l = 0;
	int count_avg_r = 0;
	int count = scan->scan_time / scan->time_increment; // this v
	
	for (int i = 0; i < count; i++)
	{ // for loop for reading the angle and its value

		degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // this store that currently at which angle we are.
		degree_max = RAD2DEG(scan->angle_max - scan->angle_increment * i);
		degree_max =degree + 180.00;
		degree_int = int(degree_max);
		degree_max_f32.data = degree_max;

		distance_f32.data = scan->ranges[i];	 // this store the distance value at that angle
		quality_f32.data = scan->intensities[i]; // check the quality of that data

		if (scan->intensities[i] == 0)
		{						   // quality_f64.data = 0;
			distance_f32.data = 0; // if the intensity is zero then this convert distance from infinite to zero
		}
		
		storage_lidar_values.linear.x = degree;
		storage_lidar_values.linear.y = distance_f32.data;
		storage_lidar_values.linear.z = quality_f32.data;
	//	pub_storage.publish(storage_lidar_values);
		// ROS_INFO(": [%f,%f,%f]", degree,distance_f64.data,quality_f64.data);	
		
	//	if (distance_f32.data <= 0.850 && distance_f32.data >= 0.010)  // 2 feet distance
		if (distance_f32.data <= 0.60 && distance_f32.data >= 0.010)
		{
			check_obstacle(); // if lidar distance cross sefty limit then it should check at which side the obstacle is there
		}
/*
if ((degree >= -180.0 && degree <= -170.0) || (degree >= 170.0 && degree <= 180.0))
		{
			if (distance_f32.data <= 1.400 && distance_f32.data >= 0.010)  // 2 feet distance
			//if (distance_f32.data <= 0.140 && distance_f32.data >= 0.010)
			{
				check_obstacle(); // if lidar distance cross sefty limit then it should check at which side the obstacle is there
			}
		}
		if (degree <= -91.000 && degree >= -93.000)
		{
			avg_distance_l_wall = avg_distance_l_wall + distance_f32.data; 
			count_avg_l = count_avg_l + 1;
		}
		
		if (degree >= 91.000 && degree <= 93.000)
		{
			avg_distance_r_wall = avg_distance_r_wall + distance_f32.data;
			count_avg_r = count_avg_r + 1;
		}*/
	}
//	avg_distance_l_wall = (avg_distance_l_wall / count_avg_l );
//	avg_distance_r_wall = (avg_distance_r_wall / count_avg_r );
	//ROS_INFO(": left = [%f],right = [%f]", avg_distance_l_wall,avg_distance_r_wall);	
//	wall_distance.linear.x = avg_distance_l_wall;
//	wall_distance.angular.x = avg_distance_r_wall;
	
	//	find_path(); // find path after one lidar rotation which side obstacle is there
	lidar_flags.linear.x = flag_left;
	lidar_flags.linear.y = flag_front;
	lidar_flags.linear.z = flag_right;
	
	time(&end);	   // end time
	flag_left = 0;
	flag_right = 0;
	flag_front = 0;
	flag_back = 0; // flag to restart the lidar detected possition
}

void check_obstacle()
{
	// these are all angle dividation
	// if(degree_compare > 90 && degree_compare < 150 ) //&& distance_f64.data <=0.15)
	// if(degree_compare < 90 && degree_compare > 40 )
	if (degree >= 30.0 && degree <= 90.0)
	{
		// ROS_INFO("in left");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		flag_left = 1;
	}

	if (degree >= -30.0 && degree <= 30.0)
	{
		// ROS_INFO("in front");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		flag_front = 1;
	}

	if (degree >= -90.0 && degree <= -30.0)
	{
		// ROS_INFO("in right");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		flag_right = 1;
	}
/*
	// these are all angle dividation

	if (degree >= -150.0 && degree <= -90.0)
	{
		// ROS_INFO("in left side obstacele is there");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		flag_left = 1;
	}

	if ((degree >= -180.0 && degree <= -150.0) || (degree >= 150.0 && degree <= 180.0))
	// if(degree_compare < 40  && degree_compare >3 )
	{
		// ROS_INFO("in front");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		flag_front = 1;
	}

	if (degree >= 90.0 && degree <= 150.0)
	{
		// ROS_INFO("in right");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		flag_right = 1;
	}
*/
}
void find_path()
{
	if (flag_left == 1 && flag_front == 1 && flag_right == 1)
	{
		//	ROS_INFO("stop");
		obstacle_detection_flag.data = 7;
	}
	if (flag_left == 0 && flag_front == 1 && flag_right == 0)
	{
		//	ROS_INFO("stop front go left or right ");
		obstacle_detection_flag.data = 2;
	}
	if (flag_left == 1 && flag_front == 0 && flag_right == 0)
	{
		// ROS_INFO("slight go right");
		obstacle_detection_flag.data = 4;
	}
	if (flag_left == 1 && flag_front == 1 && flag_right == 0)
	{
		//	ROS_INFO("sharp go right ");
		obstacle_detection_flag.data = 6;
	}

	if (flag_left == 0 && flag_front == 0 && flag_right == 1)
	{
		//	ROS_INFO("slight go left");
		obstacle_detection_flag.data = 1;
	}
	if (flag_left == 0 && flag_front == 1 && flag_right == 1)
	{
		//	ROS_INFO("sharp go left");
		obstacle_detection_flag.data = 3;
	}

	if (flag_left == 0 && flag_front == 0 && flag_right == 0)
	{
		// ROS_INFO("go forward green");
		// ROS_INFO("\033[1;32mgo forward green \033[0m");
		obstacle_detection_flag.data = 0;
	}
	if (flag_left == 1 && flag_front == 0 && flag_right == 1)
	{
		//	ROS_INFO("carefully go ");
		obstacle_detection_flag.data = 5;
	}
	// ROS_INFO(" = %d", obstacle_detection_flag);
	// sensing_stoping_pub.publish(obstacle_detection_flag);
}

// timer function
void timer_function()
{
	double time_taken = double(end - start); // time difference
	if (time_taken == -5)
	{ // condition
	  // ROS_INFO("lidar (main) is get killed");
		ROS_WARN_STREAM("lidar front mid is get killed ");
		ROS_WARN_STREAM("ALL CODE IS GETTING KILLED ");

		
		if (flag_all_node_kill == 1)
		{		
			system("rosnode kill rplidar_client_front_mid_node"); // killer command
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
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_client_front_mid_node"); // creating node
	ros::NodeHandle n;
	ROS_INFO("Lidar Client front mid code");
	ros::Rate loop_rate(10); // frequency rate
	

	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan_front_mid1", 1, scanCallback); // taking scan lidar data from rplidar pakage

	ros::Publisher pub_sensing_stoping = n.advertise <geometry_msgs::Twist> ("sensing_stoping_lidar_front_mid_topic", 1);

pub_storage = n.advertise <geometry_msgs::Twist> ("storage_front_mid_topic", 1000);
pub_wall_following = n.advertise <geometry_msgs::Twist> ("distance_front_mid_topic", 1);
	
	while (ros::ok())
	{
		pub_sensing_stoping.publish(lidar_flags);
		pub_wall_following.publish(wall_distance);
		time(&start);	  // start time
		timer_function(); // call timer function
		ros::spinOnce();
		loop_rate.sleep(); // sleep
	}
//	ros::spin();
	return 0;
}
