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
time_t start, end; // creating global object

std_msgs::Float32 degree_max_f32, distance_f32, quality_f32;
std_msgs::Int32 obstacle_detection_flag;
geometry_msgs::Point publish_data;
geometry_msgs::Twist lidar_flags;

void check_obstacle(); // this function check the obstacle came in 360 degree rotation
void find_path();	   // this function high the flags and check the condition where to go and what to do
void timer_function();

std::ofstream MyFile; // to open the CSV file to store the data
std::ofstream MyFile1;

ros::Publisher sensing_stoping_pub; // to publish the sensing stoping data
/*
void scan_angle_data_quality_pub_function(ros::Publisher *pub,std_msgs::Float32 degree_max_fun,std_msgs::Float32 distance_fun,std_msgs::Float32 quality_fun)
{
	publish_data.x = degree_max_fun.data;
	publish_data.y = distance_fun.data;
	publish_data.z = quality_fun.data;
}*/

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{ // MyFile.open("lidar_csv.ods",std::ios::app);
	// MyFile1.open("lidar_.ods",std::ios::app);eriable taking encoder count
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

		if (distance_f32.data <= 0.600 && distance_f32.data >= 0.2000) // 2 feet distance
		// if(distance_avg <= 0.15)// && quality_f64.data == 1)
		{
			check_obstacle(); // if lidar distance cross sefty limit then it should check at which side the obstacle is there
		}
		
//if ((degree <= -0.0000 && degree >= -10.0) || (degree >= 0.0000 && degree <= 10.0))
	//	{
	//if (distance_f32.data <= 3.300 && distance_f32.data >= 0.200)  // 2 feet distance
		//if (distance_f32.data <= 0.140 && distance_f32.data >= 0.010)
	//	{
			//check_obstacle(); // if lidar distance cross sefty limit then it should check at which side the obstacle is there
	//	}
		//}
	}
	//find_path(); // find path after one lidar rotation which side obstacle is there
	lidar_flags.linear.x = flag_left;
	lidar_flags.linear.y = flag_front;
	lidar_flags.linear.z = flag_right;
	
	flag_left = 0;
	flag_right = 0;
	flag_front = 0;
	flag_back = 0; // flag to restart the lidar detected possition
				   // MyFile.close();
				   // MyFile1.close();
				   //  exit(0);
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

//	if (degree >= -30.0 && degree <= 30.0)
	// if(degree_compare < 40  && degree_compare >3 )
//	{
		// ROS_INFO("in front");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		//flag_front = 1;
//	}

	if (degree >= -90.0 && degree <= -30.0)
	{
		// ROS_INFO("in right");
		// ROS_INFO(": [%f, %f,%f]", degree_max_f64, distance_f64,quality_f64);
		flag_right = 1;
	}

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
		// ROS_INFO("stop front go left or right ");
		obstacle_detection_flag.data = 2;
	}
	if (flag_left == 1 && flag_front == 0 && flag_right == 0)
	{
		//	ROS_INFO("slight go right");
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
		//	ROS_INFO("\033[1;32mgo forward green \033[0m");
		obstacle_detection_flag.data = 0;
	}
	if (flag_left == 1 && flag_front == 0 && flag_right == 1)
	{
		//	ROS_INFO("carefully go ");
		obstacle_detection_flag.data = 5;
	}
	//	ROS_INFO(" = %d", obstacle_detection_flag);
	// sensing_stoping_pub.publish(obstacle_detection_flag);
}

// timer function
void timer_function()
{
	double time_taken = double(end - start); // time difference
	if (time_taken == -5)
	{ // condition
	  // ROS_INFO("lidar (main) is get killed");
		ROS_WARN_STREAM("lidar front top is get killed");
		system("rosnode kill rplidar_node"); // killer command
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "rplidar_client_front_top_node"); // creating node
	ros::NodeHandle n;
	ROS_INFO("Lidar Client front top code");
	ros::Rate loop_rate(10); // frequency rate
							 // MyFile.open("lidar_csv.ods");// lidar_.ods
							 // MyFile1.open("lidar_.ods");// lidar_.ods

	//  MyFile << "Angle default."<<"\t"<< "Angle convert"<<"\t"<< "0-360"<<"\t" << "Distance"<<"\t" <<"avg dis"<<"\t" <<"quality"<<"\t" <<"path" <<"\n";

	// MyFile1 << "0-360"<<"\t" <<"avg dis"<<"\t" <<"quality"<<"\t" <<"path" <<"\n";
	//   MyFile.close();
	//   MyFile1.close();

	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan_front_top", 1, scanCallback); // taking scan lidar data from rplidar pakage
	//  sensing_stoping_pub = n.advertise<std_msgs::Int32> ("sensing_stoping_lidar_front_top_topic", 1000);
	ros::Publisher pub_sensing_stoping = n.advertise<geometry_msgs::Twist>("sensing_stoping_lidar_front_top_topic", 1);

	while (ros::ok())
	{
		pub_sensing_stoping.publish(lidar_flags);
		time(&start);	  // start time
		timer_function(); // call timer function
		ros::spinOnce();
		loop_rate.sleep(); // sleep
	}
//	ros::spin();
	return 0;
}
