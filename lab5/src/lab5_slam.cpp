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

std::string string_frame;

tf2::Quaternion q;

// ---------------- FORWARD CALLBACK ----------------
void forwardprog(const tf::tfMessage& cvalue)
{
}


// ---------------- MAIN -------------------
int main(int argc, char **argv)
{

    ros::init(argc, argv, "lab5_slam");
    ros::NodeHandle n;

    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);
    geometry_msgs::PoseStamped pstamp;
    ros::Rate loop_rate(500);

    // ---------------- USER INPUT ----------------
    while(ros::ok())
    {
	while (true)
    	{
            std::cout << "Enter desired x position: ";
            std::cin >> pos_x;
            if (std::cin.fail())
            {
            	std::cin.clear();
            	std::cin.ignore(10000, '\n');
            	std::cout << "Invalid input.\n";
            }
            else break;
	}

	while (true)
    	{
            std::cout << "Enter desired y position: ";
            std::cin >> pos_y;
            if (std::cin.fail())
            {
            	std::cin.clear();
            	std::cin.ignore(10000, '\n');
            	std::cout << "Invalid input.\n";
            }
            else break;
    	}

	while (true)
    	{
            std::cout << "Enter desired orientation (psi) in deg: ";
            std::cin >> input_angle;
            if (std::cin.fail())
            {
            	std::cin.clear();
            	std::cin.ignore(10000, '\n');
            	std::cout << "Invalid input.\n";
            }
            else break;
	}

	while (true)
	{
	    std::cout << "Enter desired Frame \"map\" or \"base_link\": ";
            std::cin >> string_frame;
            if (std::cin.fail())
            {
            	std::cin.clear();
            	std::cin.ignore(10000, '\n');
            	std::cout << "Invalid input.\n";
            }
            else break;
	}
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
	ros::spinOnce();
	loop_rate.sleep();
    
    }
return 0;

}//end main

