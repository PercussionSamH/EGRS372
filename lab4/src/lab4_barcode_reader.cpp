//

#include "pluginlib/class_list_macros.h"
#include "std_msgs/String.h"

#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "tf/tfMessage.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"

#include "std_msgs/UInt64.h"
#include <string>

#define INITIALIZE_VALUE -1
#define PI 3.14159265359



// ---------------- GLOBAL VARIABLES ----------------
std::string starting_barcode = "0";
    std::string b1 = "705632441947";
    std::string b2 = "051111407592";
    std::string b3 = "123456789012";


int num_success = 0;
int barcode_num =0;

//  DRAW_POLY VARIABLES ----------------
bool flag;
double current_angle = INITIALIZE_VALUE;
double target_speed = 0.0;
double target_forward = 0.5;
double moved = 0;
double initialx = INITIALIZE_VALUE;
double initialy = INITIALIZE_VALUE;
// ODOMETRY DATA (circle mode)
double current_x = 0.0;
double current_y = 0.0;
double current_yaw = 0.0;
// POLYGON CONTROLS
int g_sides = 4;
double g_side_length = 0.5;
double g_step_angle = PI / 2;



// ---------------- Barcode CALLBACK ----------------
void BarcodeCallback(const std_msgs::String::ConstPtr& barcode)
{
    if (num_success == 0)
    {
       	starting_barcode = barcode->data;
    }
    if (starting_barcode == barcode->data)
    {
	num_success += 1;
    }
    else
    {
	if (num_success != 0)
	{
	     std::cout << "Barcode changed\n";	
	}
	num_success = 0;
    } 
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab4_barcode_reader");
    ros::NodeHandle n;

    ros::Subscriber barcode = n.subscribe("/barcode", 1, BarcodeCallback);

    ros::Rate loop_rate(1000);
    unsigned barcode_output;
    std_msgs::UInt64 barcode_msg;
    ros::Publisher barcode_publish = n.advertise<std_msgs::UInt64>("barcode_topic",1000);
    
    

    num_success = 0;


    while(ros::ok())
    {
	ros::spinOnce();

  	if (num_success >= 5)
	{
	    num_success = 0;
	    if (starting_barcode == b1)//barcode 1
		{
		    //publish
		    barcode_msg.data = 1;
		    barcode_publish.publish(barcode_msg);
		    
		}
	    else if (starting_barcode == b2)//barcode 2
		{
		    //publish
		    barcode_msg.data = 2;
		    barcode_publish.publish(barcode_msg);
		}

	    else if (starting_barcode == b3)//barcode 3
		{
		    //publish
		    barcode_msg.data = 3;
		    barcode_publish.publish(barcode_msg);
		}
	    else
		{
		     std::cout << "Barcode read error: invalid barcode\n";
		}
	}
    }    
}

