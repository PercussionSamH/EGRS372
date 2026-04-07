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
int button = 0;
bool backup_done = false;
bool robot_moving;
int bumper_count = 0;

bool flag; 				// 'FLAG' = FALSE WHEN CONDITIONS ARE MET

double target_speed=0.0; 	  	// ROBOT TRAVELLING SPEED (METERS/SECOND)
double target_angle=INITIALIZE_VALUE; 	// DESIRED ANGLE OF ROBOT PATH (THETA)
double target_forward=0.5;            	// DISTANCE THE ROBOT IS TRYING TO MOVE
double moved=0; 			// DISTANCE TRAVELLED (FORWARD, IN METERS) 
double initialx=INITIALIZE_VALUE;     	// INITIAL X COORDINATE OF A MOVE
double initialy=INITIALIZE_VALUE;     	// INITIAL Y COORDINATE OF A MOVE

std::string string_frame;

tf2::Quaternion q;

// ---------------- FORWARD CALLBACK ----------------
void forwardprog(const tf::tfMessage cvalue)
{
  double dx, dy; // VARIABLES FOR X,Y COORDINATES

  // SETS THE INITIAL X AND Y COORDINATES
  if(initialx==INITIALIZE_VALUE || initialy==INITIALIZE_VALUE)
  {
    initialx=cvalue.transforms[0].transform.translation.x;
    initialy=cvalue.transforms[0].transform.translation.y;
  }

  // CALCULATES DISTANCE IN X AND Y TRAVELED (ONLY CARES ABOUT FORWARD MOVEMENT)
  dx = std::abs(cvalue.transforms[0].transform.translation.x-initialx);
  dy = std::abs(cvalue.transforms[0].transform.translation.y-initialy);
  
  // CALCULATES TOTAL DISTANCE THE ROBOT HAS TRAVELLED
  moved = sqrt(dx*dx+dy*dy);
  
  // CALCULATES DISTANCE (METERS) THE ROBOT HAS YET TO TRAVEL
  if(moved>target_forward)
  {
    flag=false;
  }

  // SETS A SPEED PROPORTIONAL TO THE DISTANCE YET TO BE TRAVELED
  // PLUS AN OFFSET TO ACCOUNT FOR FRICTION
  // SPEED IS IN M/S
  target_speed = std::abs(target_forward - moved)/4+0.1;
}



// ---------------- CALLBACK ----------------
void bumper_callback(const std_msgs::Byte bumper_byte)
{
    button = 0;
    if (bumper_byte.data == 1)
    {
	std::cout << "BUMPER triggered\n";
	button = 1;
	//backup_done = false;
    }

}

// ---------------- MAIN -------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab6_program");
    ros::NodeHandle n;

    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);    
    ros::Publisher led_pub =  n.advertise<std_msgs::Int32>("/LED_node",50);
    ros::Subscriber tf; // SUBSCRIBER DECLARATION TO GET THE TRANSFORMATION MATRICES OF THE ROBOT
    ros::Subscriber button1 = n.subscribe("/bumper_node", 1, bumper_callback);    

    geometry_msgs::PoseStamped pstamp;
    std_msgs::Int32 bump_msg;


    ros::Time go;
    //reset bumper count
    bumper_count = 0;
    bump_msg.data = bumper_count;
    led_pub.publish(bump_msg);

    geometry_msgs::Twist c;
    c.linear.x=0.0;
    c.linear.y=0.0;
    c.linear.z=0.0;
    c.angular.x=0.0;
    c.angular.y=0.0;
    c.angular.z=0.0;

    ros::Rate loop_rate(500);
    
// DECLARE SUBSCRIBER TO THE TF TOPIC (MR POSE) WITH THE FUNCTION FORWARDPROG
    tf = n.subscribe("/tf", 1, forwardprog);

    std::cout << "Program Started....\n";

    while(ros::ok())
    {
	while (button == 0 && ros::ok())
	{
 		//PUBLISHES THE TARGET SPEED, CALCULATED WHEN THE ROBOT POSITION IS SUBSCRIBED (WHEN ros::spinOnce(); IS RUN)
		c.linear.x=target_speed;
		c.angular.z=0;
		vel_pub.publish(c);

		//INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
		ros::spinOnce();
		loop_rate.sleep(); 
	}
    	// SHUTS DOWN THE SUBSCRIBER TO CHANGE ITS RELATED FUNCTION LATER
    	
   	// STOPS THE ROBOT FOR A LITTLE TO BE SAFE
   	go = ros::Time::now();
   	while(ros::Time::now() - go < ros::Duration(1) && ros::ok())
   	{
 	    //PUBLISHES 0 VELOCITY TO STOP THE ROBOT;
      	    c.linear.x=-target_speed;
            c.angular.z=0;
      	    vel_pub.publish(c);
      	    //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
      	    ros::spinOnce();
      	    loop_rate.sleep();
    	}
   	go = ros::Time::now();
   	while(ros::Time::now() - go < ros::Duration(0.1) && ros::ok())
   	{
 	    //PUBLISHES 0 VELOCITY TO STOP THE ROBOT;
      	    c.linear.x=0.0;
            c.angular.z=0;
      	    vel_pub.publish(c);
      	    //INVOKES ALL SUBSCRIBERS CALLBACKFUNCTIONS
      	    ros::spinOnce();
      	    loop_rate.sleep();
    	}
	bumper_count = bumper_count +1;
	bump_msg.data = bumper_count;
	
	led_pub.publish(bump_msg);
	std::cout << "bumper count = " << bumper_count << "\n";

	pos_x = -0.4;
	pos_y = 0;
	input_angle = 90;
	string_frame = "base_link";
	
	
	//convert to deg
	input_angle = input_angle * PI / 180;
	

    	//Convert to quaterions
    	q.setRPY( 0, 0, input_angle);

    	q.normalize();
    	q_x = q[0];
    	q_y = q[1];
    	q_z = q[2];
    	q_w = q[3];


	//set var data
	//pstamp.header.frame_id = "map"; // Set the reference frame ID
	pstamp.header.frame_id = string_frame;
	
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
	
	while (robot_moving && ros::ok())
	{
	    //////SET THE ROBOT_MOVING VARIBALE SOMEHOW
	    //maybe use cmd_vel == 0? if the navigator uses same vars
	    ros::spinOnce();
	}

	ros::spinOnce();
	loop_rate.sleep();

    
    }
return 0;

}//end main

