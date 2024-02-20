//   4   ROS_DEBUG_ONCE("This message will only print once");
//   3   ROS_DEBUG_THROTTLE(60, "This message will print every 60 seconds");

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <map>

// =============================== veriable declaration ===============================
float speed_KMPA = 109;
float speed_KMPA_change = speed_KMPA * 0.10; // P% * X = Y

// integer veriable declaration
char key(' ');
int speed_changing = 0, staring_angle = 0, flag_direction = 2;
int encoder_pulse = 0 /*400*/, encoder_puls_tendegree = 240 /*900*/ /*600*/ /*400*/ /*200*/ ;
int stop_update = 2, angle_inc_dec = 1;
int max_steering_angle = /*1*/ /*2*/ /*3*/ 8;
int flag_slag = 2,slag = 200;
int flag_arrow = 2;
float key_x, key_z, flag_x = 0, flag_z = 0, flag_y = 0, rpm_value_base_motor = 0;

//=========================== two wheel veriable =========================
int left_pwm = 150 , right_pwm = 150 , max_pwm = 200 , left_pwm_pass = 0,right_pwm_pass = 0; 
int flag_left_inc_dec_pwm = 0 , flag_right_inc_dec_pwm = 0 , pwm_change = 1;
int left_dac = (left_pwm * 12) , right_dac = (right_pwm * 12) , left_dac_pass = 0, right_dac_pass = 0;  ;

geometry_msgs::Twist value_twist, update_teleop;
std_msgs::Int8 dirr, dirr_x, dirr_y;
std_msgs::Int32 key_value, speed_value;

ros::Publisher twist_data;
ros::Publisher pub_key_value;
ros::Publisher pub_voltage_value;
ros::Publisher pub_absolute_giving_angle;
ros::Publisher pub_normal_encoder_giving_value;

ros::Subscriber sub_obstacle_stop;
ros::Subscriber sub_teleop_final_update;
ros::Subscriber sub_absolute_encoder_angle;

// =============================== function declaration ===============================
void obstacle_stop_callback(const std_msgs::Int32 &msg);
void teleop_final_update_callback(const geometry_msgs::Twist &msg);
void normal_encoder_position_callback(const std_msgs::Int32::ConstPtr &msg);
void absolute_encoder_angle_callback(const geometry_msgs::Twist::ConstPtr &msg);


// =============================== function defination ===============================
void obstacle_stop_callback(const std_msgs::Int32 &msg)
{
	stop_update= msg.data;
	//ROS_INFO(" = '%d' = robot ", stop_update);
	//ROS_INFO(" = '%d' = robot 1 ", msg->data);
	//ROS_INFO(" = robot  2");
}
void teleop_final_update_callback(const geometry_msgs::Twist &msg)
{
	update_teleop = msg;
	
/*	key_x = update_teleop.linear.x;
	speed_changing = update_teleop.linear.y;
	
	encoder_pulse = update_teleop.angular.x;
	flag_direction = update_teleop.angular.y;
	staring_angle = update_teleop.angular.z;
	
	value_twist.linear.x = key_x;
	value_twist.linear.y = speed_changing;//0; passing the voltage to move the base motor
	value_twist.linear.z = 0;

	value_twist.angular.x = encoder_pulse; // steering motor encoder redding 
	value_twist.angular.y = flag_direction; // 0; //giving motor direction
	value_twist.angular.z = staring_angle;	// key_z; // main base steering motor actual angle
*/
}

void normal_encoder_position_callback(const std_msgs::Int32::ConstPtr &msg)
{
}

void absolute_encoder_angle_callback(const geometry_msgs::Twist::ConstPtr &msg)
{
}

void process_data(char key_input) // this function checking the value and assigning the value to it
{
//if (stop_update == 0) // for updating the obstacle value 
//{
	flag_direction = 2;
	if (key_input == 'w' || key_input == 'W') // this W,S act as acclarator pedel to the 3 weel robot
	{
	key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite

		if (flag_x == 0)
		{
			key_x = 10;
			flag_y = 1;
			
			left_pwm_pass = left_pwm;
			right_pwm_pass = right_pwm;	
			
		//	left_dac = left_pwm * 12;
			//right_dac = right_pwm * 12;
			ROS_INFO("Forward direction");
		}
		else
		{
			key_x = 0;
			flag_x = 0;
			flag_y = 0;
			left_pwm_pass = 0;
			right_pwm_pass = 0;
			
			//left_dac = left_pwm * 12;
		//right_dac = right_pwm * 12;
			ROS_INFO("Robot is stoped");
		}
		//ROS_INFO("key = '%c', voltage = '%d' = giving direct high speed", key_input, speed_changing);

	//	ROS_INFO("key = '%c' = machine is moving forward direction", key_input);
	}

	if (key_input == 's' || key_input == 'S')
	{
	key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite
		if (flag_y == 0)
		{
			key_x =	-10;
			flag_x = 1;
			left_pwm_pass = left_pwm;
			right_pwm_pass = right_pwm;
			
		//	left_dac = left_pwm * 12;
		//	right_dac = right_pwm * 12;
				
	    ROS_INFO("Backward direction");
		}
		else
		{
			key_x = 0;
			flag_x = 0;
			flag_y = 0;
			left_pwm_pass = 0;
			right_pwm_pass = 0;
						
			//left_dac = left_pwm * 12;
			//right_dac = right_pwm * 12;
			ROS_INFO("Robot is stoped");
		}

//		ROS_INFO("key = '%c' = machine is moving backward direction", key_input);
	}

	if ( (key_input == 'a' || key_input == 'A') && flag_arrow != 1) 
	// || key_input == 27) // this A,D will act as staring for the robot
	{
	key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite
		flag_direction = 0;

			key_x = 20;
			left_pwm_pass = 0;
			right_pwm_pass = right_pwm;
			
		//	left_dac = left_pwm * 12;
		//	right_dac = right_pwm * 12;
			ROS_INFO("Taking left turn");		

	}
	
	if ( (key_input == 'd' || key_input == 'D') && flag_arrow != 1) 
	{
	key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite
		flag_direction = 1;

			key_x = 30;
			left_pwm_pass = left_pwm;
			right_pwm_pass = 0;
			
			//left_dac = left_pwm * 12;
		//	right_dac = right_pwm * 12;
			ROS_INFO("Taking right turn");	
	}

// ========================== WALL FOLLOWING KEYS ============================
/*
	if (key_input == 'i' || key_input == 'I') // for left side wall following
	{
		key_value.data = 105; // "i" ascii value
		ROS_INFO("key = '%c' = Left side wall following enabled", key_input);
	}
	
	if (key_input == 'p' || key_input == 'P') // for sensing stoping disabling
	{
		key_value.data = 112; // "p" ascii value
		ROS_INFO("key = '%c' = Right side wall following enabled", key_input);
	}

	if (key_input == '1' )
	{
		key_value.data = 49; 
		//ROS_INFO("key = '%c' = wall following distance 1.5m", key_input);
	}

	if (key_input == '2' )
	{
		key_value.data = 50; 
		//ROS_INFO("key = '%c' = wall following distance 2m", key_input);
	}
	
	if (key_input == '3' )
	{
		key_value.data = 51; 
		//ROS_INFO("key = '%c' = wall following distance 2.5m", key_input);
	}
	
	if (key_input == '4' )//|| key_input == 'Q') // for stoping the robot
	{
		key_value.data = 52; // 52 use to wall following distance 5.5m
		//ROS_INFO("key = '%c' = wall following distance 3m", key_input);
	}

	if (key_input == '5' )//|| key_input == 'Q') // for stoping the robot
	{
		key_value.data = 53; // 52 use to wall following distance 5.5m
		//ROS_INFO("key = '%c' = wall following distance 3.5m", key_input);
	}
*/
// ========================== SENSING STOPPING KEYS ============================

	if (key_input == 'e' || key_input == 'E') // for sensing stoping disabling
	{
		key_value.data = 114; // "r" ascii value
		ROS_INFO("key = '%c' = sensing stoping disabled", key_input);
	}

	if (key_input == 'r' || key_input == 'R') // for sensing stoping enabling
	{
	
		key_value.data = 101; // "e" ascii value
		ROS_INFO("key = '%c' = sensing stoping enabled", key_input);
	}


// ========================== SPEED/VOLTAGE ADJUSTMENT KEYS ==========================

	if ( (key_input == 'b' || key_input == 'B') && flag_arrow != 1)
	// for giving incrmental speed act as potentiometer
	{	key_x = 0;
			if (right_pwm > (max_pwm + (pwm_change - max_pwm)))
			{
				right_pwm = right_pwm - pwm_change;
				ROS_INFO("Decreasing right wheel pwm = '%d' ", right_pwm);
			}
	}

	if (key_input == 'v' || key_input == 'V') // for giving decremental speed act as potentiometer
	{ 		key_x = 0;
			if (left_pwm > (max_pwm + (pwm_change - max_pwm)))
			{
				left_pwm = left_pwm - pwm_change;
				ROS_INFO("Decreasing left wheel pwm = '%d' ", left_pwm);
			}
	}

	if (key_input == 'm' || key_input == 'M') // for giving incremental high speed
	{			key_x = 0;
		if (right_pwm <= (max_pwm - pwm_change))
		{
			right_pwm = right_pwm + pwm_change;
			ROS_INFO("Increasing right wheel pwm = '%d' ", right_pwm);
		}
	}
	
	if (key_input == 'n' || key_input == 'N') // for giving decremental pwm speed
	{			key_x = 0;
			if (left_pwm <= (max_pwm - pwm_change))
			{
				left_pwm = left_pwm + pwm_change;
				ROS_INFO("Increasing left wheel pwm = '%d' ", left_pwm);
			}	
	}

	if (key_input == 'q' || key_input == 'Q') // for stoping the robot
	{
		key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite
		key_x = 0;
		//staring_angle = 0;
		ROS_INFO("Robot is stoped");
	} 
	// ======================== HORN =====================================
	
	if (key_input == 'g' || key_input == 'G') 
	{ 		
		key_value.data = 103;
	}
	
	if (key_input == 'h' || key_input == 'H') 
	{ 		
		key_value.data = 104;
	}
// ========================== SLAG ADDITION  =================================

	if (flag_slag != flag_direction && flag_direction != 2)
	{
		if(flag_direction > 0)
		{
			encoder_pulse = encoder_pulse - slag;
		} else {
					encoder_pulse = encoder_pulse + slag;
				}
	flag_slag = flag_direction;
	}
	
// ================================= Arrow keys =======================================

	if ( (key_input == 'd' || key_input == 'D') && flag_arrow == 1)  // this is key "A"code
	// || key_input == 27) // this A,D will act as staring for the robot
	{
	key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite
		flag_direction = 0;
			key_x = 20;
			left_pwm_pass = 0;
			right_pwm_pass = right_pwm;
			
			//left_dac = left_pwm * 12;
		//	right_dac = right_pwm * 12;
			ROS_INFO("Taking left turn");		

	flag_arrow = 3;
	}
	
	if ( (key_input == 'c' || key_input == 'C') && flag_arrow == 1) // this is key "D" code
	{
	key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite
		flag_direction = 1;
			key_x = 30;
			left_pwm_pass = left_pwm;
			right_pwm_pass = 0;
		
		//	left_dac = left_pwm * 12;
		//	right_dac = right_pwm * 12;
			ROS_INFO("Taking right turn");	
	flag_arrow = 3;
	}
	
	
if ( (key_input == 'a' || key_input == 'A') && flag_arrow == 1) // key "W" code for arrow
 // this W,S act as acclarator pedel to the 3 weel robot
	{
	key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite

		if (flag_x == 0)
		{
			key_x = 10;
			flag_y = 1;
			left_pwm_pass = left_pwm;
			right_pwm_pass = right_pwm;	
			
			//left_dac = left_pwm * 12;
		//	right_dac = right_pwm * 12;
			ROS_INFO("Forward direction");
		}
		else
		{
			key_x = 0;
			flag_x = 0;
		//	flag_y = 0;	
			left_pwm_pass = 0;
			right_pwm_pass = 0;	
			
		//	left_dac = left_pwm * 12;
		//	right_dac = right_pwm * 12;

		}
	///	ROS_INFO("Forward arrow voltage = '%d' = giving direct high speed", key_input, speed_changing);
		ROS_INFO("Forward direction");
		flag_arrow = 3;
	}
	
	if ( (key_input == 'b' || key_input == 'B') && flag_arrow == 1)//key "Q" code for arrow
	// for stoping the robot
	{
		key_value.data = 123; // 123 use to disable wall following mode or any automated mode exite
		key_x = 0;
		//staring_angle = 0;
		left_pwm_pass = 0;
		right_pwm_pass = 0;	
		ROS_INFO("Robot is Stoped");
		flag_arrow = 3;
	} 
// ========================== STORE THE VERIABLE  ============================

	left_dac = left_pwm_pass * 12;
	right_dac = right_pwm_pass * 12;
	speed_value.data = speed_changing; // voltage MCP4725 DAC 
	
	value_twist.linear.x = key_x;
	value_twist.linear.y = left_pwm_pass;//0; passing the voltage to move the base motor
	value_twist.linear.z = right_pwm_pass;

	value_twist.angular.x = left_dac; // left dac value
	value_twist.angular.y = right_dac; // 0; //giving motor direction
	value_twist.angular.z = 0;	// key_z; // main base steering motor actual angle
	
}
// For non-blocking keyboard inputs
int getch(void)
{
	int ch;
	struct termios oldt;
	struct termios newt;

	// Store old settings, and copy to new settings
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;

	// Make required changes and apply the settings
	
	newt.c_lflag &= ~(ICANON | ECHO);
	newt.c_iflag |= IGNBRK;
	newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
	newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
	newt.c_cc[VMIN] = 1;
	newt.c_cc[VTIME] = 0;
	tcsetattr(fileno(stdin), TCSANOW, &newt);

	// Get the current character
	ch = getchar();

	// Reapply old settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	return ch;
}

// =============================== main function ======================================
int main(int argc, char **argv) // main function
{
	// Init ROS node
	ros::init(argc, argv, "teleop_twist_keyboard_node");

	ros::NodeHandle n;
	ROS_INFO("Teleop keyboard info");
	///	ROS_INFO_GREEN("Teleop keyboard info");
	ROS_INFO("\033[1;32mTeleop keyboard info \033[0m");

	geometry_msgs::Twist twist; // Create Twist message

	ROS_WARN_STREAM("warn");
	ROS_FATAL_STREAM("fatale");
	ROS_ERROR_STREAM("error");
	ROS_DEBUG_STREAM("debug");
	
	sub_obstacle_stop = n.subscribe("/obstacle_stop_topic", 1, obstacle_stop_callback); 

	//=================================== Publisher ===================================

	twist_data = n.advertise<geometry_msgs::Twist>("mov_cmd_topic", 1);
	pub_key_value = n.advertise<std_msgs::Int32>("key_value_topic", 1);
	pub_voltage_value = n.advertise<std_msgs::Int32>("voltage_value_topic", 1);

	//pub_absolute_giving_angle = n.advertise <geometry_msgs::Twist> ("absolute_encoder_angle_giving_topic", 1);
	//pub_normal_encoder_giving_value = n.advertise<geometry_msgs::Twist> ("normal_encoder_positioon_giving_topic", 1);

	//=================================== Subcriber ===================================

	sub_teleop_final_update = n.subscribe ("/updated_mov_cmd_topic", 1, teleop_final_update_callback);

	// ros::Subscriber sub_normal_encoder_position = n.subscribe("/normal_encoder_positioon_topic", 1, normal_encoder_position_callback); // it

	sub_absolute_encoder_angle = n.subscribe<geometry_msgs::Twist>("/absolute_encoder_angle_topic", 1, absolute_encoder_angle_callback); // it

	while (true)
	{

		// Get the pressed key
		key = getch();
		if (key == 3) // attached control c to kill the running node
		{
			exit(0);
		}
		// std::cout << "ASCII value of " << key << " is :  " << (int)key << " \n";
		//ROS_INFO("%c", key); // DISPLAY THE WHICH KEY IS PRESSED
		
		if(key == 27)
		{
		//	ROS_INFO("Arrow key is pressed"); 
			flag_arrow = 1;
		}
		process_data(key);

		pub_key_value.publish(key_value);
		key_value.data = 0;
		twist_data.publish(value_twist); // publish the geometry value
		pub_voltage_value.publish(speed_value);
		
		ros::spinOnce();
	}
			//ros::spinOnce();
	return 0;
}
/*
// ========================== ENCODER KEYS ============================
	if (key_input == 'o' || key_input == 'O') // for restart the encoder value
	{
		key_value.data = 79; // "o" ascii value
		//staring_angle = 0;
		//ROS_INFO("key = '%c' = encoder value set as 0th possition = '%d' degree ", key_input, staring_angle);
		
		staring_angle = 0;
		ROS_INFO("key = '%c' = encoder value set as 0th possition = '%d' degree ", key_input, staring_angle);
	}
	
	if (key_input == 'y' || key_input == 'Y') // for restart the encoder value
	{
	//	key_value.data = 79; // "o" ascii value
		//staring_angle = 0;
		//ROS_INFO("key = '%c' = encoder value set as 0th possition = '%d' degree ", key_input, staring_angle);
		
		staring_angle = -(max_steering_angle + 1);
		ROS_INFO("key = '%c' = encoder value set as -max possition = '%d' degree ", key_input, staring_angle);
	}
	
	if (key_input == 'u' || key_input == 'U') // for restart the encoder value
	{
	//	key_value.data = 79; // "o" ascii value
		//staring_angle = 0;
		//ROS_INFO("key = '%c' = encoder value set as 0th possition = '%d' degree ", key_input, staring_angle);
		
		staring_angle = (max_steering_angle + 1);
		ROS_INFO("key = '%c' = encoder value set as maxd possition = '%d' degree ", key_input, staring_angle);
	}
	
	if (key_input == 'k' || key_input == 'K' || key_input == 'x' || key_input == 'X' ) // for restart the encoder value
	{
		if(staring_angle > 0)
		{
			for ( int i=staring_angle; i > 0 ; i = i - angle_inc_dec)
			{
				flag_direction = 0;	
				staring_angle = staring_angle - angle_inc_dec;//10;
				encoder_pulse = encoder_pulse + encoder_puls_tendegree;
			}
		}else {
			for ( int i=staring_angle; i < 0 ; i = i + angle_inc_dec)
			{
				flag_direction = 1;
				staring_angle = staring_angle + angle_inc_dec;//10;
				encoder_pulse = encoder_pulse - encoder_puls_tendegree;
			}
		}

		ROS_INFO("key = '%c' = machine is taking = '%d' degree turn", key_input, staring_angle);
	}
	
	if (key_input == 'j' || key_input == 'J' || key_input == 'z' || key_input == 'Z' ) // for giving left minimum angle full turn
	{
		flag_direction = 0;	
		for ( int i=staring_angle; i > - max_steering_angle ; i = i - angle_inc_dec)//80 ; i = i - angle_inc_dec)
		{
			staring_angle = staring_angle - angle_inc_dec;//10;
			encoder_pulse = encoder_pulse + encoder_puls_tendegree;
		}
	ROS_INFO("key = '%c' = machine is taking = '%d' degree turn", key_input, staring_angle);
	}
	
	if ( (key_input == 'l' || key_input == 'L' || key_input == 'c' || key_input == 'C' ) && flag_arrow != 1) 
	// for giving right maximum angle full turn
	{
		flag_direction = 1;	
		for ( int i=staring_angle; i < max_steering_angle ; i = i + angle_inc_dec) // 80 ; i = i + angle_inc_dec)
		{
			staring_angle = staring_angle + angle_inc_dec;//10;
			encoder_pulse = encoder_pulse - encoder_puls_tendegree;
		}
	ROS_INFO("key = '%c' = machine is taking = '%d' degree turn", key_input, staring_angle);
	}
	*/
