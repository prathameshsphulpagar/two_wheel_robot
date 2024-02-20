#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <fstream>
#include <time.h>
// 192.168.0.14 pi ip adress
// farhan@1 password
#define RAD2DEG(x) ((x)*180. / M_PI)

float degree_max, degree, distance_avg = 0;
int degree_int, degree_compare = 0, count_angle = 0;
int flag_left = 0, flag_right = 0, flag_front = 0, flag_back = 0;

std_msgs::Float32 degree_max_f32, distance_f32, quality_f32;
std_msgs::Int32 obstacle_detection_flag;
geometry_msgs::Point publish_data;
geometry_msgs::Twist lidar_flags;
time_t start, end; // creating global object

void check_obstacle(); // this function check the obstacle came in 360 degree rotation
void find_path();	   // this function high the flags and check the condition where to go and what to do
void timer_function();

std::ofstream MyFile; // to open the CSV file to store the data
std::ofstream MyFile1;

ros::Publisher sensing_stoping_pub; // to publish the sensing stoping data



void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{ // MyFile.open("lidar_csv.ods",std::ios::app);
	// MyFile1.open("lidar_.ods",std::ios::app);eriable taking encoder count
	// ROS_INFO("Lidar code client");
	int count = scan->scan_time / scan->time_increment; // this v
	for (int i = 0; i < count; i++)
	{ // for loop for reading the angle and its value

		degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // this store that currently at which angle we are.

		distance_f32.data = scan->ranges[i];	 // this store the distance value at that angle
		quality_f32.data = scan->intensities[i]; // check the quality of that data

		if (scan->intensities[i] == 0)
		{						   // quality_f64.data = 0;
			distance_f32.data = 0; // if the intensity is zero then this convert distance from infinite to zero
		}

		// ROS_INFO(": [%f,%f,%f]", degree,distance_f64.data,quality_f64.data);

if (distance_f32.data <= 0.600 && distance_f32.data >= 0.0010) // 2 feet distance
		//if (distance_f32.data <= 0.140 && distance_f32.data >= 0.0010) // 2 feet distance
		{
			check_obstacle(); // if lidar distance cross sefty limit then it should check at which side the obstacle is there
		}

if ((degree >= -180.0 && degree <= -170.0) || (degree >= 170.0 && degree <= 180.0))
		{
if (distance_f32.data <= 1.000 && distance_f32.data >= 0.010)  
//		if (distance_f32.data <= 0.140 && distance_f32.data >= 0.010)
		{
			check_obstacle(); // if lidar distance cross sefty limit then it should check at which side the obstacle is there
		}
		}
	}

	//find_path(); // find path after one lidar rotation which side obstacle is there
	lidar_flags.linear.x = flag_left;
	lidar_flags.linear.y = flag_front;
	lidar_flags.linear.z = flag_right;
	
	flag_left = 0;
	flag_right = 0;
	flag_front = 0;
	flag_back = 0; // flag to restart the lidar detected possition
	time(&end);	   // end time

	// MyFile.close();
	// MyFile1.close();
	//  exit(0);
}

void check_obstacle()
{
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
/*	if (degree >= 30.0 && degree <= 90.0)
	{
		// ROS_INFO("in left side obstacele is there");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		flag_left = 1;
	}

	if (degree >= -30.0 && degree <= 30.0)
	// if(degree_compare < 40  && degree_compare >3 )
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
	}*/

}
void find_path()
{
	if (flag_left == 1 && flag_front == 1 && flag_right == 1)
	{
		// ROS_WARN_STREAM("stop");
		obstacle_detection_flag.data = 7;
	}
	if (flag_left == 0 && flag_front == 1 && flag_right == 0)
	{
		// ROS_WARN_STREAM("stop front go left or right ");
		obstacle_detection_flag.data = 2;
	}
	if (flag_left == 1 && flag_front == 0 && flag_right == 0)
	{
		// ROS_WARN_STREAM("slight go right");
		obstacle_detection_flag.data = 4;
	}
	if (flag_left == 1 && flag_front == 1 && flag_right == 0)
	{
		// ROS_WARN_STREAM("sharp go right ");
		obstacle_detection_flag.data = 6;
	}

	if (flag_left == 0 && flag_front == 0 && flag_right == 1)
	{
		// ROS_WARN_STREAM("slight go left");
		obstacle_detection_flag.data = 1;
	}
	if (flag_left == 0 && flag_front == 1 && flag_right == 1)
	{
		// ROS_WARN_STREAM("sharp go left");
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
		// ROS_WARN_STREAM("carefully go ");
		obstacle_detection_flag.data = 5;
	}
}

// timer function
void timer_function()
{
	double time_taken = double(end - start); // time difference
	if (time_taken == -5)
	{ // condition
		// ROS_INFO("lidar (main) is get killed");
		ROS_WARN_STREAM("lidar (main) is get killed");
		ROS_WARN_STREAM("ALL CODE IS GETTING KILLED ");
		
		//system("rosnode kill rplidar_node"); // killer command
		//system("rosnode kill rplidar_client_node"); // killer command
		//system("rosnode kill rplidar_client_front_mid_node"); // killer command
	//	system("rosnode kill teleop_twist_keyboard_node"); // killer command
		//system("rosnode kill sensing_stoping_node"); // killer command
		//system("rosnode kill combine_six_lidar_node"); // killer command
		//system("rosnode kill wall_following_node"); // killer command
		//system("rosnode kill node_restart_node"); // killer command
		//	exit(0);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_client_node"); // creating node
	ros::NodeHandle n;
	ros::Rate loop_rate(10); // frequency rate

	ROS_INFO("Lidar Client front bottom (main) code");
	// MyFile.open("lidar_csv.ods");// lidar_.ods
	// MyFile1.open("lidar_.ods");// lidar_.ods

	//  MyFile << "Angle default."<<"\t"<< "Angle convert"<<"\t"<< "0-360"<<"\t" << "Distance"<<"\t" <<"avg dis"<<"\t" <<"quality"<<"\t" <<"path" <<"\n";

	// MyFile1 << "0-360"<<"\t" <<"avg dis"<<"\t" <<"quality"<<"\t" <<"path" <<"\n";
	//   MyFile.close();
	//   MyFile1.close();
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback); // taking scan lidar data from rplidar pakage

//ros::Publisher pub_sensing_stoping = n.advertise<std_msgs::Int32> ("sensing_stoping_lidar_front_bottom_topic", 1);

ros::Publisher pub_sensing_stoping = n.advertise <geometry_msgs::Twist> ("sensing_stoping_lidar_front_bottom_topic", 10);

	while (ros::ok())
	{
	//	pub_sensing_stoping.publish(obstacle_detection_flag);
	pub_sensing_stoping.publish(lidar_flags);
		time(&start);	  // start time
		timer_function(); // call timer function
		ros::spinOnce();
		loop_rate.sleep(); // sleep
	}
	// ros::spin();
	return 0;
}
