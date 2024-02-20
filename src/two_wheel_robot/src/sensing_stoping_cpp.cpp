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

std_msgs::Int8 obstacle_detection_flag;
int sensing_stoping_value = 11, flag_sensing_stoping = 0, flag_go_green = 0, flag_updated_mov = 0;
int flag_obstacle_stop = 0,new_flag_obstacle_stop = 2;
int flag_auto_mov = 0;
int flag_disp = 0;
std_msgs::Int32 flag_obstacle_stop_32, flag_sensing_stoping_32;
int encoder_zero = 0;
geometry_msgs::Twist updated_mov_value_twist, lidar_detaction;

int servo_brush_min_angle = 10;
int servo_brush_max_angle = 80;
geometry_msgs::Twist servo_brush_angle;

	
// std_msgs::Int32 key_value;
// creating global time object
time_t start, end ,start_auto , end_auto , start_disp , end_disp; 

ros::Publisher pub_obstacle_stop; // if any obstacle is getting detected then 
ros::Publisher pub_updated_twist_data; // this (pub_updated_twist_data) is publishing updated data comming from twist keyboard.
ros::Publisher pub_servo_brush; // this gives angle value to servo motor so that brush magnate should rotate.

ros::Subscriber sub_combine_six;  // subscribing to the front bottom lidar data
ros::Subscriber sub_teleop_input_valu; // taking other keyboard values to process some tasks.
ros::Subscriber sub_teleop_input; // this subcriber is taking value from teleop keyboard
ros::Subscriber sub_wall_mov_cmd; // this subcriber is taking value from l_wall_mov_cmd


void stop();
void flush();
void find_path();
void obstacle_stop();
void timer_function();
void sensing_stoping();
void timer_function_updated();
void sensing_stoping_display();
void teleop_input_callback(const geometry_msgs::Twist &msg);
void wall_mov_cmd_callback(const geometry_msgs::Twist &msg);
void sensing_stoping_callback(const geometry_msgs::Twist &scan);
void teleop_key_value_input_callback(const std_msgs::Int32 &msg);

// void sensing_stoping_callback(const std_msgs::Int32::ConstPtr &scan);


void stop() 
{
// to make mov command value zero.
	updated_mov_value_twist.linear.x = 0;
	updated_mov_value_twist.linear.y = 0;
	updated_mov_value_twist.linear.z = 0;

	updated_mov_value_twist.angular.x = 0;
	updated_mov_value_twist.angular.y = 0; // for without encoder robot
	updated_mov_value_twist.angular.z = 0;
	flag_auto_mov = 0;
	
}
void obstacle_stop()
{
// this stop function is used when there is an obstacle is detected then function make all the value zero.
	stop();
	flag_auto_mov = 0;
	flag_updated_mov = 0;
	flag_obstacle_stop = 1;
	flag_obstacle_stop_32.data = flag_obstacle_stop;
	
	pub_obstacle_stop.publish(flag_obstacle_stop_32);
	pub_updated_twist_data.publish(updated_mov_value_twist);
}

void wall_mov_cmd_callback(const geometry_msgs::Twist &msg) // wall following
{
	if (flag_obstacle_stop == 0)
	{
		updated_mov_value_twist.linear.x = msg.linear.x;
		updated_mov_value_twist.linear.y = msg.linear.y;
		updated_mov_value_twist.linear.z = 0;

		updated_mov_value_twist.angular.x = encoder_zero + msg.angular.x;
		updated_mov_value_twist.angular.y = 2; // for without encoder robot
		updated_mov_value_twist.angular.z = msg.angular.z;
	
		flag_auto_mov = 1;
		flag_updated_mov = 0;
	}	
}
void sensing_stoping_callback(const geometry_msgs::Twist &scan)
{

// this function is reding topic data coming from combined six lidar  in this we are taking lidar obstacle in geometry format linear (x, y, z) represent all the binary numbers of obstacles we will; process this data and make a design according to it.

	lidar_detaction = scan;
	if (flag_sensing_stoping == 0)
	{
	
		// this find path function is used to convert the binary logic into a fixed number so that we can easily process that data. ex = "000" as 0" & "101" as "5" etc.
		
		find_path();
		
		// Sensing_stoping function we are using to mark exactly what that binary converted number means and exactly what type of action should we perform on that. we can use this function for further navigation as well as decision-taking purposes. 
		
		sensing_stoping();
		
		// this function sensing_stoping_display(); is used to just display the process to the terminal this is exactly same as sensing_stoping();
		
		sensing_stoping_display();
	}
else
	{
		//  in this situation (else condition) there is no processing is happing just display the situation in the terminal when the robot is stopped due to an obstacle. 
		
		sensing_stoping_display();
	}
}

void teleop_key_value_input_callback(const std_msgs::Int32 &msg)
{	
	// the value coming from teleop to operate a few functionalities of teleop.
	if (msg.data == 113) // 113 is small "q" ascii value robot complete stopd
	{
		stop();
	}

	if (msg.data == 101) // 101 is small "e" ascii value for enable the sensing stoping
	{
		flag_sensing_stoping = 0;
	}

	if (msg.data == 114) // 101 is small "r" ascii value for disable the sensing stoping
	{
		flag_sensing_stoping = 1;
		flag_obstacle_stop = 0;
		time(&end); // end time
	}
}

void flush() // for clearing the detected value
{
	//  once after every loop we clear this variable which stores the combined lidar process data
	lidar_detaction.linear.x = 0;
	lidar_detaction.linear.y = 0;
	lidar_detaction.linear.z = 0;
	
	lidar_detaction.angular.x = 0; // for back lidar value
}

void timer_function()
{
	// this timer function besically used to automatically enable sensing stoping if someone desable it.
	double time_taken = double(end - start); // time difference
	if (time_taken == -3)
	{ // condition if timming is exceeded 
		flag_sensing_stoping = 0;
	}
	
	double time_taken_auto = double(end_auto - start_auto); // time difference
	if (time_taken_auto == -2 && flag_auto_mov == 1 && flag_obstacle_stop == 0)
	{ // condition if timming is exceeded 
		ROS_INFO("\033[1;32mAutomatically data is sent to teensy.\033[0m");
		flag_updated_mov = 0;

	}
	
	double time_disp = double(end_disp - start_disp); // time difference
	if (time_disp == -2)
	{ // condition if timming is exceeded 
		flag_disp = 30;
	}
}

void teleop_input_callback(const geometry_msgs::Twist &msg)
{ 
	int staring_angle = msg.angular.z;
	int encoder_pulse = msg.angular.x;
// this function is taking key value like(W,S,A,D) only direction command
	if (flag_obstacle_stop == 0)
	{
		// (if function) if there is no obstacle then the telop value is directly get pass to the teensy
		
		updated_mov_value_twist = msg;
		flag_auto_mov = 0;
	}
	
	if (flag_obstacle_stop == 1)
	{
		// (if function) if there is obstacle then the only steering value get pass to teency
		
		updated_mov_value_twist.linear.x = 0;
		updated_mov_value_twist.linear.y = 0;
		updated_mov_value_twist.linear.z = 0;

		updated_mov_value_twist.angular.x = 0;
		updated_mov_value_twist.angular.y = 0; // for without encoder robot
		updated_mov_value_twist.angular.z = 0;
		
		pub_updated_twist_data.publish(updated_mov_value_twist);

	}
	
	// this flag (flag_obstacle_stop_32) is used to publish a topic to teleop if any obstacle comes then the teleop node should stop updating the value.

	flag_obstacle_stop_32.data = flag_obstacle_stop;
	timer_function();
	
	// this variable (flag_updated_mov) is just used as a flag to identify that the value is updated and now we can publish new value to the tenancy. this avoid publishing the same value multiple times.

	flag_updated_mov = 0;
	
	if (flag_sensing_stoping == 0)
	{
		sensing_stoping();
	}
	//else
	//{
		//sensing_stoping_display();
	//}
	
	if(staring_angle > 0) // to find the zero possition encoder value
	{

		for ( int i = staring_angle; i > 0 ; i = i - 1)
		{
			staring_angle = staring_angle - 1;//10;
			//encoder_pulse = encoder_pulse + 200;
			encoder_pulse = encoder_pulse + 10;
		}
		encoder_zero = encoder_pulse;
	}
	else{
		for ( int i = staring_angle; i < 0 ; i = i + 1)
		{
			staring_angle = staring_angle + 1;//10;
			//encoder_pulse = encoder_pulse - 200;
			encoder_pulse = encoder_pulse - 10;
		}
		encoder_zero = encoder_pulse; 
	}
	//ROS_INFO("zero value %d,%d",encoder_zero,staring_angle);
	flag_auto_mov = 0;
}

void find_path()
{
	if (lidar_detaction.linear.x == 1 && lidar_detaction.linear.y == 1 && lidar_detaction.linear.z == 1)
	{
		//	ROS_INFO("stop");
		sensing_stoping_value = 7;
	}

	if (lidar_detaction.linear.x == 0 && lidar_detaction.linear.y == 1 && lidar_detaction.linear.z == 0)
	{
		//	ROS_INFO("stop front go left or right ");
		sensing_stoping_value = 2;
	}

	if (lidar_detaction.linear.x == 1 && lidar_detaction.linear.y == 0 && lidar_detaction.linear.z == 0)
	{
		// ROS_INFO("slight go right");1
		sensing_stoping_value = 4;
	}

	if (lidar_detaction.linear.x == 1 && lidar_detaction.linear.y == 1 && lidar_detaction.linear.z == 0)
	{
		//	ROS_INFO("sharp go right ");
		sensing_stoping_value = 6;
	}

	if (lidar_detaction.linear.x == 0 && lidar_detaction.linear.y == 0 && lidar_detaction.linear.z == 1)
	{
		//	ROS_INFO("slight go left");
		sensing_stoping_value = 1;
	}

	if (lidar_detaction.linear.x == 0 && lidar_detaction.linear.y == 1 && lidar_detaction.linear.z == 1)
	{
		//	ROS_INFO("sharp go left");
		sensing_stoping_value = 3;
	}


	if (lidar_detaction.linear.x == 1 && lidar_detaction.linear.y == 0 && lidar_detaction.linear.z == 1)
	{
		//	ROS_INFO("carefully go ");
		sensing_stoping_value = 5;
	}
	
	if (lidar_detaction.linear.x == 0 && lidar_detaction.linear.y == 0 && lidar_detaction.linear.z == 0 && lidar_detaction.angular.x == 0)
	{
		// ROS_INFO("go forward green");
		// ROS_INFO("\033[1;32mgo forward green \033[0m");
		sensing_stoping_value = 0;
	}
	
	if (lidar_detaction.linear.x == 0 && lidar_detaction.linear.y == 0 && lidar_detaction.linear.z == 0 && lidar_detaction.angular.x == 1)
	{
		//ROS_INFO("go forward green");
		// ROS_INFO("\033[1;32mgo forward green \033[0m");
		sensing_stoping_value = 8;
	}
}

void sensing_stoping_display()
{
	if (sensing_stoping_value == 7)
	{
		if (flag_disp != 7)
		{
			ROS_WARN_STREAM("\033[1;31mStop \033[0m");
			flag_go_green = 0;
			flag_disp = 7;
			// sensing_stoping_value.data = 7;
			time(&end_disp);
		}
	}
	
	if (sensing_stoping_value == 2)
	{
		if (flag_disp != 2)
		{
			ROS_WARN_STREAM("Stop front go left or right ");
			flag_go_green = 0;
			flag_disp = 2;
			// obstacle_detection_flag.data = 2;
			time(&end_disp);
		}
	}
	
	if (sensing_stoping_value == 4)
	{
		if (flag_disp != 4)
		{
			ROS_WARN_STREAM("Slight go right");
			flag_go_green = 0;
			flag_disp = 4;
			// obstacle_detection_flag.data = 4;
			time(&end_disp);
		}
	}
	
	if (sensing_stoping_value == 6)
	{
		if (flag_disp != 6)
		{
			ROS_WARN_STREAM("Sharp go right ");
			flag_go_green = 0;
			flag_disp = 6;
			// obstacle_detection_flag.data = 6;
			time(&end_disp);
		}
	}

	if (sensing_stoping_value == 1)
	{
		if (flag_disp != 1)
		{
			ROS_WARN_STREAM("Slight go left");
			flag_go_green = 0;
			flag_disp = 1;
			// obstacle_detection_flag.data = 1;
			time(&end_disp);
		}
	}
	
	if (sensing_stoping_value == 3)
	{
		if (flag_disp != 3)
		{
			ROS_WARN_STREAM("sharp go left");
			flag_go_green = 0;
			flag_disp = 3;
			// obstacle_detection_flag.data = 3;
			time(&end_disp);
		}
	}

	if (sensing_stoping_value == 6)
	{
		if (flag_disp != 6)
		{
			ROS_WARN_STREAM("Sharp go right");
			flag_go_green = 0;
			flag_disp = 6;
			// obstacle_detection_flag.data = 0;
			time(&end_disp);
		}
	}
	
	if (sensing_stoping_value == 5)
	{
		if (flag_disp != 5)
		{
			ROS_WARN_STREAM("Carefully go, left & right obstacle");
			// obstacle_detection_flag.data = 5;
			flag_go_green = 0;
			flag_disp = 5;
			time(&end_disp);
		}
		
	}
	
	if (sensing_stoping_value == 8)
	{
		if (flag_disp != 8)
		{
			ROS_WARN_STREAM("\033[1;31mBack side obstacle is there \033[0m");
			// obstacle_detection_flag.data = 5;
			flag_go_green = 0;
			flag_disp = 8;
			time(&end_disp);
		}
	}
	
	if (sensing_stoping_value == 0)
	{
		if (flag_go_green == 0)
		{
			ROS_INFO("\033[1;32mgo forward green \033[0m");
			// obstacle_detection_flag.data = 0;
			flag_go_green = 1;
			flag_disp = 0;
		}
	}
}
void sensing_stoping()
{
// this function is the important function in this function for each lidar obstacle value there is particular task is assigned
	if (sensing_stoping_value == 7)
	{
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("\033[1;31mStop \033[0m");
			flag_go_green = 0;

			obstacle_stop();
			// sensing_stoping_value.data = 7;
			// updated_mov_value_twist.linear.x = 0;
			// updated_mov_value_twist.linear.y = 0;
			// updated_mov_value_twist.linear.z = 0;

			// updated_mov_value_twist.angular.x = 0;
			// updated_mov_value_twist.angular.y = 0;
			// updated_mov_value_twist.angular.z = 0;

			// flag_updated_mov = 0;
		}
	}
	if (sensing_stoping_value == 2)
	{
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("stop front go left or right ");
			flag_go_green = 0;
			obstacle_stop();
		}
	}
	if (sensing_stoping_value == 4)
	{
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("slight go right");
			flag_go_green = 0;
			obstacle_stop();
		}
	}
	if (sensing_stoping_value == 6)
	{
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("sharp go right ");
			flag_go_green = 0;
			obstacle_stop();
		}
	}

	if (sensing_stoping_value == 1)
	{
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("slight go left");
			flag_go_green = 0;
			obstacle_stop();
		}
	}
	if (sensing_stoping_value == 3)
	{
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("sharp go left");
			flag_go_green = 0;
			obstacle_stop();
		}
	}

	if (sensing_stoping_value == 6)
	{
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("sharp go right");
			flag_go_green = 0;
			obstacle_stop();
		}
	}
	
	if (sensing_stoping_value == 5)
	{
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("Carefully go, left & right obstacle");
			flag_go_green = 0;
			obstacle_stop();
		}
	}
	
	if (sensing_stoping_value == 8)
	{			
	//ROS_WARN_STREAM("stop backside if loop");
		if (flag_obstacle_stop == 0)
		{
			ROS_WARN_STREAM("\033[1;31mBack side obstacle is there \033[0m");
			//ROS_WARN_STREAM("Back side obstacle is there");
			ROS_WARN_STREAM("stop backside");
			flag_go_green = 0;
			obstacle_stop();

		}
	}
	if (sensing_stoping_value == 0)
	{
		if (flag_go_green == 0)
		{
			ROS_INFO("\033[1;32mgo forward green \033[0m");

			flag_go_green = 1;
			flag_obstacle_stop = 0;
			flag_obstacle_stop_32.data = flag_obstacle_stop;
			pub_obstacle_stop.publish(flag_obstacle_stop_32);
			pub_updated_twist_data.publish(updated_mov_value_twist);
		}
		
	}
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "sensing_stoping_node"); // node name
	ros::NodeHandle n;
	ROS_INFO("sensing stoping code");

 
	pub_updated_twist_data = n.advertise <geometry_msgs::Twist> ("updated_mov_cmd_topic", 1); 

	pub_obstacle_stop = n.advertise <std_msgs::Int32> ("obstacle_stop_topic", 1); 

	pub_servo_brush = n.advertise <geometry_msgs::Twist> ("servo_brush_topic", 1);

	sub_teleop_input = n.subscribe("/mov_cmd_topic", 1, teleop_input_callback); 
	
	sub_teleop_input_valu = n.subscribe("/key_value_topic", 1, teleop_key_value_input_callback); 
	
	sub_combine_six = n.subscribe("/combine_six_twist_topic", 1, sensing_stoping_callback);

	sub_wall_mov_cmd = n.subscribe("/wall_mov_cmd_topic", 1, wall_mov_cmd_callback); 
	
	ros::Rate loop_rate(10);
	
	
	while (ros::ok())
	{
		time(&start_disp); // start time
		time(&start_auto); // start time
		time(&start); // start time 
		timer_function();
		servo_brush_angle.linear.x = servo_brush_min_angle;
servo_brush_angle.linear.y = servo_brush_max_angle;
		if (flag_updated_mov == 0 && flag_obstacle_stop == 0)
		{
			pub_updated_twist_data.publish(updated_mov_value_twist);
			pub_servo_brush.publish(servo_brush_angle);
			flag_updated_mov = 1;
			if (flag_auto_mov == 1 )
			{
				time(&end_auto); // end time  
				timer_function();
			}
		} 
		

		flush();
		ros::spinOnce();
	}
	return 0;
}
