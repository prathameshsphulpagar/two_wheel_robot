//ASCII value of 1 is :  49
//ASCII value of 2 is :  50
//ASCII value of 3 is :  51
//ASCII value of 4 is :  52

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

ros::Publisher pub_wall_mov_cmd;
ros::Publisher pub_wall_sensing_stopping;
ros::Subscriber sub_front_mid; // subscribing to the front mid lidar data
ros::Subscriber sub_teleop_input_valu;

geometry_msgs::Twist wall_mov_value_twist;
//int flag_wall_sensing_stopping = 0;
int flag_wall = 0;
int flag_dont_repeat = 0;
float wall_set_distance = 0.800; //1.000;//1.500; //2.500;
float wall_distance = 0;
int flag_old_value = 1;
int speed = 1600;
int direction = 10;
std_msgs::Int8 flag_wall_sensing_stopping;

void teleop_key_value_input_callback(const std_msgs::Int32 &msg);
void wall_following_lidar_front_mid_callback(const geometry_msgs::Twist &msg);

void teleop_key_value_input_callback(const std_msgs::Int32 &msg)
{	
float wall_inc_dec = 0.80000; //1.000; //0.500;//1.000;
// =============================== wall following sides======================
	if (msg.data == 105) // 105 is small "i" ascii value robot start left side wall following
	{		
		flag_wall = 1;
	//	wall_mov_value_twist.linear.x = 10; // forward direction
	//	wall_mov_value_twist.linear.y = 1500; // safe maximum voltage 
	wall_distance = 0;
	}
	
	if (msg.data == 123) // 123 is small "w,q,s,d,a" ascii value robot start left side wall following
	{
		flag_wall = 0;
		flag_dont_repeat = 1;
		wall_distance = 0;
		wall_mov_value_twist.linear.x = 0;
		wall_mov_value_twist.linear.y = 0;
		wall_mov_value_twist.linear.z = 0;

		wall_mov_value_twist.angular.x = 0;
		wall_mov_value_twist.angular.y = 2; // for without encoder robot
		// 	wall_mov_value_twist.angular.z = 0;
		flag_wall_sensing_stopping.data = 0;
		pub_wall_sensing_stopping.publish(flag_wall_sensing_stopping);
	}
	
	if (msg.data == 112) // 112 is small "p" ascii value robot start right side wall following
	{
		flag_wall = 2;
		wall_distance = 0;
	//	ROS_INFO("following.");
	//	wall_mov_value_twist.linear.x = 10; // forward direction
	//	wall_mov_value_twist.linear.y = 2000; // safe maximum voltage 
	}
	
// =============================== wall following distance ===================

	if (msg.data == 49)  //1
	{
		wall_distance = wall_set_distance ;
		if(flag_wall == 1)
		{
			flag_wall_sensing_stopping.data = 1;
		}
		
		if(flag_wall == 2)
		{
			flag_wall_sensing_stopping.data = 2;
		}
		pub_wall_sensing_stopping.publish(flag_wall_sensing_stopping);
	}
	
	if (msg.data == 50) //2
	{
		flag_wall_sensing_stopping.data = 0;
		wall_distance = wall_set_distance + wall_inc_dec;
		pub_wall_sensing_stopping.publish(flag_wall_sensing_stopping);
	}
	
	if (msg.data == 51)  //3
	{
		flag_wall_sensing_stopping.data = 0;
		wall_distance = wall_set_distance + (2 * wall_inc_dec );
		pub_wall_sensing_stopping.publish(flag_wall_sensing_stopping);
	}
	
	if (msg.data == 52)   //4
	{
		flag_wall_sensing_stopping.data = 0;
		wall_distance = wall_set_distance + (3 * wall_inc_dec );
		pub_wall_sensing_stopping.publish(flag_wall_sensing_stopping);
	}
	
	if (msg.data == 53)  //5
	{
		flag_wall_sensing_stopping.data = 0;
		wall_distance = wall_set_distance + (4 * wall_inc_dec );
		pub_wall_sensing_stopping.publish(flag_wall_sensing_stopping);
	}
	/*
		if (msg.data == 54)  //6
	{
		wall_distance = wall_set_distance + (5 * wall_inc_dec );
	}
	
		if (msg.data == 55)  //7
	{
		wall_distance = wall_set_distance + (6 * wall_inc_dec );
	}
	
		if (msg.data == 56)  //8
	{
		wall_distance = wall_set_distance + (7 * wall_inc_dec );
	}
	
		if (msg.data == 57)  //9
	{
		wall_distance = wall_set_distance + (8 * wall_inc_dec );
	}*/
		//ROS_INFO("distance = '%f' = for wall following ", wall_distance);
}

void wall_following_lidar_front_mid_callback(const geometry_msgs::Twist &msg)
{
	int encoder_value = 0;
	int staring_angle_in_de = 0;
	int speed_local = 0;
	int direction_local = 0;
	
	if (flag_wall == 1)
	{
		// run the left side wall following.

	/*if(msg.linear.x >(wall_distance + 0.700) && msg.linear.x <(wall_distance + 0.800))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = 1900;
			//staring_angle_in_de = -4;
		}

//if(msg.linear.x >(wall_distance + 0.0350) && msg.linear.x <(wall_distance + 0.0500))
	if(msg.linear.x >(wall_distance + 0.600) && msg.linear.x <(wall_distance + 0.700))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = 1650;
			//staring_angle_in_de = -3;
		}
	
//if(msg.linear.x >(wall_distance + 0.0200) && msg.linear.x <(wall_distance + 0.0350))
	if(msg.linear.x >(wall_distance + 0.500) && msg.linear.x <(wall_distance + 0.600))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = 1400;
			//staring_angle_in_de = -2;
		}*/
		
	if(msg.linear.x >(wall_distance + 0.400) && msg.linear.x <(wall_distance + 0.500))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 1150; //original
			encoder_value = 1900;
		//	staring_angle_in_de = -1;
		}
		
	if(msg.linear.x >(wall_distance + 0.300) && msg.linear.x <(wall_distance + 0.400))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 900;
			encoder_value = 1520;
			//staring_angle_in_de = -1;
		}
		
	if(msg.linear.x >(wall_distance + 0.200) && msg.linear.x <(wall_distance + 0.300))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 650;
			encoder_value = 1140;
			//staring_angle_in_de = -1;
		}

	if(msg.linear.x >(wall_distance + 0.100) && msg.linear.x <(wall_distance + 0.200))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 400;
			encoder_value = 760;
			//staring_angle_in_de = -1;
		}

	if(msg.linear.x >(wall_distance + 0.020) && msg.linear.x <(wall_distance + 0.100))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 150;
			encoder_value = 380;
			//staring_angle_in_de = -1;
		}
//if(msg.linear.x <(wall_distance + 0.0100) && msg.linear.x >(wall_distance - 0.0100))
	if(msg.linear.x <(wall_distance + 0.020) && msg.linear.x >(wall_distance - 0.020))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = 0; // center possition ////============================
			//staring_angle_in_de = 0;
			flag_old_value = 1;
		}
		
	if(msg.linear.x <(wall_distance - 0.020) && msg.linear.x >(wall_distance - 0.100))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -150;
			encoder_value = -380;
			//staring_angle_in_de = 1;
		}
		
//if(msg.linear.x <(wall_distance - 0.0100) && msg.linear.x >(wall_distance - 0.0200))
	if(msg.linear.x <(wall_distance - 0.100) && msg.linear.x >(wall_distance - 0.200))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -400;
			encoder_value = -760;
			//staring_angle_in_de = 1;
		}

//if(msg.linear.x <(wall_distance - 0.0200) && msg.linear.x >(wall_distance - 0.0350))
	if(msg.linear.x <(wall_distance - 0.200) && msg.linear.x >(wall_distance - 0.300))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -650;
			encoder_value = -1140;
			//staring_angle_in_de = 2;
		}
		
//if(msg.linear.x <(wall_distance - 0.0350) && msg.linear.x >(wall_distance - 0.0500))
	if(msg.linear.x <(wall_distance - 0.300) && msg.linear.x >(wall_distance - 0.400))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -900;
			encoder_value = -1520;
			//staring_angle_in_de = 3;
		}
		
	if(msg.linear.x <(wall_distance - 0.400) && msg.linear.x >(wall_distance - 0.500))
		{
			direction_local = direction;
			speed_local = speed;
		//	encoder_value = -1150; //main
		encoder_value = -1900;
		//	staring_angle_in_de = 4;
		}
	/*	
	if(msg.linear.x <(wall_distance - 0.500) && msg.linear.x >(wall_distance - 0.600))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = -1400;
		//	staring_angle_in_de = 4;
		}

	i f(msg.linear.x <(wall_distance - 0.600) && msg.linear.x >(wall_distance - 0.700))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = -1650;
		//	staring_angle_in_de = 4;
		}

	if(msg.linear.x <(wall_distance - 0.700) && msg.linear.x >(wall_distance - 0.800))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = -1900;
		//	staring_angle_in_de = 4;
		}*/
	}
	
	
	
// ============================= run the right side wall following ================
	
	
	
	if (flag_wall == 2)
	{
		// run the right side wall following
		
		//ROS_INFO("run the left side wall following.");
/*if(msg.angular.x >(wall_distance + 0.700) && msg.angular.x <(wall_distance + 0.800))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -1900;
			encoder_value = 70;
			//staring_angle_in_de = -4;
		}

//if(msg.linear.x >(wall_distance + 0.0350) && msg.linear.x <(wall_distance + 0.0500))
	if(msg.angular.x >(wall_distance + 0.600) && msg.angular.x <(wall_distance + 0.700))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -1650;
			encoder_value = 60;
			//staring_angle_in_de = -3;
		}
	
//if(msg.linear.x >(wall_distance + 0.0200) && msg.linear.x <(wall_distance + 0.0350))
	if(msg.angular.x >(wall_distance + 0.500) && msg.angular.x <(wall_distance + 0.600))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -1400;
			//encoder_value = -1900;
			//staring_angle_in_de = -2;
		}*/
		
	if(msg.angular.x >(wall_distance + 0.400) && msg.angular.x <(wall_distance + 0.500))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -1150;
			encoder_value = -1900;
		//	staring_angle_in_de = -1;
		}
		
	if(msg.angular.x >(wall_distance + 0.300) && msg.angular.x <(wall_distance + 0.400))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -900;
			encoder_value = -1520;
			//staring_angle_in_de = -1;
		}
		
	if(msg.angular.x >(wall_distance + 0.200) && msg.angular.x <(wall_distance + 0.300))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -650;
			encoder_value = -1140;
			//staring_angle_in_de = -1;
		}

	if(msg.angular.x >(wall_distance + 0.100) && msg.angular.x <(wall_distance + 0.200))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -400;
			encoder_value = -760;
			//staring_angle_in_de = -1;
		}

	if(msg.angular.x >(wall_distance + 0.020) && msg.angular.x <(wall_distance + 0.100))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = -150;
			encoder_value = -380;
			//staring_angle_in_de = -1;
		}
//if(msg.linear.x <(wall_distance + 0.0100) && msg.linear.x >(wall_distance - 0.0100))
	if(msg.angular.x <(wall_distance + 0.020) && msg.angular.x >(wall_distance - 0.020))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = 0; // center possition ////============================
			//staring_angle_in_de = 0;
			flag_old_value = 2;
		}
		
	if(msg.angular.x <(wall_distance - 0.020) && msg.angular.x >(wall_distance - 0.100))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 150;
			encoder_value = 380;
			//staring_angle_in_de = 1;
		}
		
//if(msg.linear.x <(wall_distance - 0.0100) && msg.linear.x >(wall_distance - 0.0200))
	if(msg.angular.x <(wall_distance - 0.100) && msg.angular.x >(wall_distance - 0.200))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 400;
			encoder_value = 760;
			//staring_angle_in_de = 1;
		}

//if(msg.linear.x <(wall_distance - 0.0200) && msg.linear.x >(wall_distance - 0.0350))
	if(msg.angular.x <(wall_distance - 0.200) && msg.angular.x >(wall_distance - 0.300))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 650;
			encoder_value = 1140;
			//staring_angle_in_de = 2;
		}
		
//if(msg.linear.x <(wall_distance - 0.0350) && msg.linear.x >(wall_distance - 0.0500))
	if(msg.angular.x <(wall_distance - 0.300) && msg.angular.x >(wall_distance - 0.400))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 900;
			encoder_value = 1520;
			//staring_angle_in_de = 3;
		}
		
	if(msg.angular.x <(wall_distance - 0.400) && msg.angular.x >(wall_distance - 0.500))
		{
			direction_local = direction;
			speed_local = speed;
			//encoder_value = 1150;
			encoder_value = 1900;
		//	staring_angle_in_de = 4;
		}
		
/*if(msg.angular.x <(wall_distance - 0.500) && msg.angular.x >(wall_distance - 0.600))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = 1400;
			//encoder_value = 50;
		//	staring_angle_in_de = 4;
		}

	if(msg.angular.x <(wall_distance - 0.600) && msg.angular.x >(wall_distance - 0.700))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = 1650;
			//encoder_value = 60;
		//	staring_angle_in_de = 4;
		}

	if(msg.angular.x <(wall_distance - 0.700) && msg.angular.x >(wall_distance - 0.800))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = 1900;
			//encoder_value = 70;
		//	staring_angle_in_de = 4;
		}*/
	}
	
//
	/*if(msg.angular.x >(wall_distance + 0.500) && msg.angular.x <(wall_distance + 0.800))
		{
			direction_local = direction;
			speed_local = speed;
			encoder_value = -1700;
			//staring_angle_in_de = -4;
		} */

	if (encoder_value == flag_old_value)
	{
		flag_dont_repeat = 1;
	}	else
	{
		flag_dont_repeat = 0;
		flag_old_value = encoder_value;
	}
	
	wall_mov_value_twist.linear.x = direction_local; // forward direction
	wall_mov_value_twist.linear.y = speed_local; // safe maximum voltage 
	
	wall_mov_value_twist.angular.x = encoder_value;
	wall_mov_value_twist.angular.z = 0;//staring_angle_in_de;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "wall_following_node"); // node name
	ros::NodeHandle n;
	ROS_INFO("wall following code");
	
	pub_wall_mov_cmd = n.advertise <geometry_msgs::Twist> ("wall_mov_cmd_topic", 1);
	pub_wall_sensing_stopping = n.advertise<std_msgs::Int8>("wall_sensing_stopping", 1);
	
	sub_teleop_input_valu = n.subscribe("/key_value_topic", 1, teleop_key_value_input_callback); 
	sub_front_mid = n.subscribe ("/distance_front_mid_topic", 1, wall_following_lidar_front_mid_callback);
	
	while (ros::ok())
	{
		if (flag_wall != 0 && flag_dont_repeat == 0)
		{
			pub_wall_mov_cmd.publish(wall_mov_value_twist);
			flag_dont_repeat = 1;
		}
		ros::spinOnce();
	}
	
	return 0;
}
