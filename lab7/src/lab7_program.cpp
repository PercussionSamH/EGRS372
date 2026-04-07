// FILENAME:
// draw_poly.cpp
#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include "tf/tfMessage.h"
#include "tf/tf.h"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/BatteryState.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sstream>               // LIBRARY THAT PROVIDES STRING CLASSES
#include "geometry_msgs/Twist.h" 
#include "tf/tfMessage.h" 


#define INITIALIZE_VALUE -1
#define PI 3.14159265359

// ---------------- GLOBAL VARIABLES ----------------
double current_angle = INITIALIZE_VALUE;
double pos_x = 0.0;
double pos_y = 0.0;
double pos_z = 0.0;
double q_x = 0.0;
double q_y = 0.0;
double q_z = 0.0;
double q_w = 0.0;
double input_angle = 0.0; //in deg
int button_flag = 0;
bool batt_OK = false;
bool has_item = true;
bool backup_done = false;
bool goal_reached_flag;
int items_placed = 0;
std::string frame_selection;


//std::string string_frame;

tf2::Quaternion q;


// ---------------- GOAL REACHED CALLBACK ----------------
void goal_callback(const move_base_msgs::MoveBaseActionResult goal_result)
{
    goal_reached_flag = 0;
    if (goal_result.status.status == 3)
    {
	std::cout << "goal reached\n";
	goal_reached_flag = 1;
    }

}


// ---------------- BUMPER CALLBACK ----------------
void bumper_callback(const std_msgs::Byte bumper_byte)
{
    button_flag = 0;
    if (bumper_byte.data == 1 && goal_reached_flag == 0)
    {
	std::cout << "BUMPER triggered\n";
	button_flag = 1;
	//backup_done = false;
    }

}

// ---------------- BATTERY CALLBACK ----------------
void batt_callback(const sensor_msgs::BatteryState batt_result)
{
    batt_OK = 1;
	
    if (batt_result.voltage <= 11.0)
    {
	std::cout << "batt triggered\n";
	batt_OK = 0;
    }

}
// ---------------- MAIN -------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab7_program");
    ros::NodeHandle n;

    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);   
    geometry_msgs::PoseStamped pstamp;

    ros::Publisher led_screen_pub =  n.advertise<std_msgs::Int32>("/LED_node",50);
    ros::Subscriber button_press = n.subscribe("/bumper_node", 1, bumper_callback);    
    ros::Subscriber goal_reached = n.subscribe("/move_base/result",1, goal_callback);
    ros::Subscriber battery_status = n.subscribe("/battery_state",1, batt_callback);
    std_msgs::Int32 items_msg;

    ros::Time go;

    //geometry_msgs::Twist c;

    ros::Rate loop_rate(500);
    //wait for initialize
    go = ros::Time::now();
    while(ros::Time::now() - go < ros::Duration(0.5) && ros::ok())
    {
    	ros::spinOnce();
        loop_rate.sleep();
    }
    

    ros::spinOnce();
    loop_rate.sleep(); 

    items_placed = 0;
    items_msg.data = items_placed;
    led_screen_pub.publish(items_msg);
    std::cout << "items placed = " << items_placed << "\n";

    ros::spinOnce();
    loop_rate.sleep(); 

    std::cout << "Program Started....\n";
    
    //Move Home
	goal_reached_flag = 0;	
	pos_x = 0;
	pos_y = 0;

        //convert to rad
    	input_angle = 0 * PI / 180;

    	//Convert to quaterions
    	q.setRPY( 0, 0, input_angle);
    	q.normalize();
    	q_x = q[0];
    	q_y = q[1];
    	q_z = q[2];
    	q_w = q[3];
    	//set var data
    	pstamp.header.frame_id = "map"; // Set the reference frame ID
    	pstamp.pose.position.x = pos_x;
    	pstamp.pose.position.y = pos_y;
    	pstamp.pose.position.z = pos_z;
    	pstamp.pose.orientation.x = q_x;
    	pstamp.pose.orientation.y = q_y;
    	pstamp.pose.orientation.z = q_z;
    	pstamp.pose.orientation.w = q_w;
	
    	//publish
    	pose_pub.publish(pstamp);
    	std::cout << "published new pos\n";
	
	while (goal_reached_flag == 0 && ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep(); 
	}


    // -------- MAIN LOOP --------
    while(ros::ok())
    {
	
	//loop
	while (batt_OK == 1 && ros::ok())
	{
	    if (has_item)
	    {
	    	//pick
	    	goal_reached_flag = 0;	
	    	pos_x = -2;
	    	pos_y = 0.5;
	    
            	//convert to rad
    	    	input_angle = 0 * PI / 180;
	    	//Convert to quaterions
	    	q.setRPY( 0, 0, input_angle);
	    	q.normalize();
	    	q_x = q[0];
 	    	q_y = q[1];
	    	q_z = q[2];
	    	q_w = q[3];
	    	//set var data
	    	pstamp.header.frame_id = "map"; // Set the reference frame ID	
	    	pstamp.pose.position.x = pos_x;
	    	pstamp.pose.position.y = pos_y;
	    	pstamp.pose.position.z = pos_z;
	    	pstamp.pose.orientation.x = q_x;
	    	pstamp.pose.orientation.y = q_y;
	    	pstamp.pose.orientation.z = q_z;
 	    	pstamp.pose.orientation.w = q_w;
	
 	    	//publish
	    	pose_pub.publish(pstamp);
    	    	std::cout << "published new pos\n";
	
	    	while (goal_reached_flag == 0 && batt_OK == 1 && ros::ok())
	    	{
			ros::spinOnce();
			loop_rate.sleep(); 
	    	}
	    	goal_reached_flag = 0;
	    	while (button_flag == 0 && batt_OK == 1 && ros::ok())
	    	{
	    		ros::spinOnce();
	   	 	loop_rate.sleep(); 
	    	}
	    }

	    //place
	    if (batt_OK == 1)
	    {
		has_item = false;
	    	goal_reached_flag = 0;		    
	    	pos_x = -2;
	    	pos_y = -1.75;

            	//convert to rad
    	    	input_angle = 90 * PI / 180;
	    	//Convert to quaterions
	    	q.setRPY( 0, 0, input_angle);
	    	q.normalize();
	    	q_x = q[0];
 	    	q_y = q[1];
	    	q_z = q[2];
	    	q_w = q[3];
	    	//set var data
	    	pstamp.header.frame_id = "map"; // Set the reference frame ID	
	    	pstamp.pose.position.x = pos_x;
	    	pstamp.pose.position.y = pos_y;
	    	pstamp.pose.position.z = pos_z;
	    	pstamp.pose.orientation.x = q_x;
	    	pstamp.pose.orientation.y = q_y;
	    	pstamp.pose.orientation.z = q_z;
 	    	pstamp.pose.orientation.w = q_w;
	
 	    	//publish
	    	pose_pub.publish(pstamp);
    	    	std::cout << "published new pos\n";
	    }
	    while (goal_reached_flag == 0 && batt_OK == 1 && ros::ok())
	    {
		ros::spinOnce();
		loop_rate.sleep(); 
	    }
	    goal_reached_flag = 0;
	    while (button_flag == 0 && batt_OK == 1 && ros::ok())
	    {
	    	ros::spinOnce();
	    	loop_rate.sleep(); 
	    }
	    if (batt_OK == 1)
	    {
	    	items_placed = items_placed +1;
	    	items_msg.data = items_placed;
	    	led_screen_pub.publish(items_msg);
		has_item = true;
    	    }

	    
	}
//Move Home
	goal_reached_flag = 0;	
	pos_x = 0;
	pos_y = 0;

        //convert to rad
    	input_angle = 0 * PI / 180;

    	//Convert to quaterions
    	q.setRPY( 0, 0, input_angle);
    	q.normalize();
    	q_x = q[0];
    	q_y = q[1];
    	q_z = q[2];
    	q_w = q[3];
    	//set var data
    	pstamp.header.frame_id = "map"; // Set the reference frame ID
    	pstamp.pose.position.x = pos_x;
    	pstamp.pose.position.y = pos_y;
    	pstamp.pose.position.z = pos_z;
    	pstamp.pose.orientation.x = q_x;
    	pstamp.pose.orientation.y = q_y;
    	pstamp.pose.orientation.z = q_z;
    	pstamp.pose.orientation.w = q_w;
	
    	//publish
    	pose_pub.publish(pstamp);
    	std::cout << "published new pos\n";
	
	while (goal_reached_flag == 0 && ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep(); 
	}
///Add wait for batt to be high
	int i = 0;
	while ((batt_OK == 0 || i<5) && ros::ok())
	{
	    if (batt_OK == 0)
	    {
		i=0;
	    }
	    i++;
	    ros::spinOnce();
	    loop_rate.sleep();
	}

    }//end while ros::OK
return 0;

}//end main

