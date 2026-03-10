// FILENAME:
// draw_poly.cpp
#include "ros/ros.h"
#include <iostream>
#include <math.h>
#include "geometry_msgs/Twist.h"
#include "tf/tfMessage.h"
#include "nav_msgs/Odometry.h"
#include "tf/tf.h"
#include "std_msgs/UInt64.h"

#define INITIALIZE_VALUE -1
#define PI 3.14159265359

// ---------------- GLOBAL VARIABLES ----------------
bool flag;
double current_angle = INITIALIZE_VALUE;
double target_speed = 0.0;
double target_forward = 0.5;
double moved = 0;
double initialx = INITIALIZE_VALUE;
double initialy = INITIALIZE_VALUE;
unsigned barcode_num  = 0;
bool direction;
// ODOMETRY DATA (circle mode)
double current_x = 0.0;
double current_y = 0.0;
double current_yaw = 0.0;

// POLYGON CONTROLS
int g_sides = 4;
double g_side_length = 0.5;
double g_step_angle = PI / 2;

// ---------------- ODOM CALLBACK ----------------
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w
    );

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw = yaw;
}

// ---------------- FORWARD CALLBACK ----------------
void forwardprog(const tf::tfMessage& cvalue)
{
    double dx, dy;

    if (initialx == INITIALIZE_VALUE || initialy == INITIALIZE_VALUE)
    {
        initialx = cvalue.transforms[0].transform.translation.x;
        initialy = cvalue.transforms[0].transform.translation.y;
    }

    dx = cvalue.transforms[0].transform.translation.x - initialx;
    dy = cvalue.transforms[0].transform.translation.y - initialy;

    moved = sqrt(dx * dx + dy * dy);

    if (moved >= target_forward)
        flag = false;

    target_speed = fabs(target_forward - moved) / 4 + 0.1;
}

// ---------------- TURN CALLBACK ----------------
void Turnprog(const tf::tfMessage& cvalue)
{
    double z = cvalue.transforms[0].transform.rotation.z;
    double w = cvalue.transforms[0].transform.rotation.w;

    current_angle = 2 * atan2(z, w);
    if (current_angle < 0) current_angle += 2 * PI;
    if (current_angle >= 2 * PI) current_angle -= 2 * PI;
}

// ---------------- ANGLE WRAP ----------------
double wrapToPi(double angle)
{
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

// ---------------- BARCODE ----------------
void barcodeCallback(const std_msgs::UInt64 barcode_msg)
{
    barcode_num = barcode_msg.data;
    std::cout << "Barcode recieved as: #"<< barcode_num << std::endl;
}

// ---------------- MAIN -------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "draw_poly");
    ros::NodeHandle n;

    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Subscriber tf;
    ros::Subscriber odom_sub = n.subscribe("/odom", 10, odomCallback);
    ros::Subscriber barcode_sub = n.subscribe("barcode_topic",1,barcodeCallback);

    ros::Rate loop_rate(50);

    // ---------------- USER INPUT ----------------
 while(ros::ok())
 {
    ros::spinOnce();
    if (barcode_num == 1)
    {
	std::cout << "setting param #1\n";
    	g_sides = 4;
   	g_side_length = 1;
	direction = 1;
    }
    if (barcode_num == 2)
    {
	std::cout << "setting param #2\n";
    	g_sides = 4;
   	g_side_length = 1;
	direction = 0;
    }
    if (barcode_num == 3)
    {
	std::cout << "setting param #3\n";
    	g_sides = 2;
   	g_side_length = 2;
	direction = 1;
    }


    //char input = 0;
    

    target_forward = g_side_length;
    g_step_angle = 0.93*(2.0 * PI) / g_sides;
    if (direction == 1) g_step_angle = 0.90*(2.0 * PI) / g_sides;
    if (direction == 0) g_step_angle = 0.85*(2.0 * PI) / g_sides;

    geometry_msgs::Twist c;
    c.linear.x = 0;
    c.angular.z = 0;

   
    // ================= POLYGON MODE =================
    if (barcode_num==3)
    {
	barcode_num=0; //reset barcode
	std::cout << " doing path 3\n";
	for (int i = 0; i < 2; i++)
    	{
            flag = true;
            moved = 0;
            initialx = INITIALIZE_VALUE;
            initialy = INITIALIZE_VALUE;
	    
   	    tf = n.subscribe("/tf", 1, Turnprog);
            ros::spinOnce();
            loop_rate.sleep();

	//rotate
            current_angle = INITIALIZE_VALUE;

            while (current_angle == INITIALIZE_VALUE && ros::ok())
            {
            	ros::spinOnce();
            	loop_rate.sleep();
            }

            double target_angle = current_angle - g_step_angle;
            if (target_angle < 0) target_angle += 2 * PI;

            while (ros::ok())
            {
               	double diff = wrapToPi(target_angle - current_angle);
                if (fabs(diff) < 0.05) break;

            	c.linear.x = 0;
            	c.angular.z = -fabs(diff) / PI - 0.5;
            	vel_pub.publish(c);

            	ros::spinOnce();
            	loop_rate.sleep();
            }
            tf.shutdown();

	//stop
            ros::Time go = ros::Time::now();
            while ((ros::Time::now() - go) < ros::Duration(0.1) && ros::ok())
            {
                c.linear.x = 0;
                c.angular.z = 0;
                vel_pub.publish(c);
                ros::spinOnce();
                loop_rate.sleep();
            }

	//drive
	   tf = n.subscribe("/tf", 1, forwardprog);
           while (flag && ros::ok())
            {
                c.linear.x = target_speed;
                c.angular.z = 0;
                vel_pub.publish(c);
                ros::spinOnce();
                loop_rate.sleep();
            }
            tf.shutdown();
    	}
    }
    else
    {
    	if (barcode_num == 1 || barcode_num == 2)
	{
	    barcode_num=0; //reset barcode
	    std::cout << "doing path 1/2\n";
	    for (int i = 0; i < g_sides; i++)
            {
           	flag = true;
           	moved = 0;
            	initialx = INITIALIZE_VALUE;
            	initialy = INITIALIZE_VALUE;
		//drive
            	tf = n.subscribe("/tf", 1, forwardprog);
            	ros::spinOnce();
            	loop_rate.sleep();

           	while (flag && ros::ok())
            	{
                	c.linear.x = target_speed;
                	c.angular.z = 0;
                	vel_pub.publish(c);
                	ros::spinOnce();
                	loop_rate.sleep();
            	}
            	tf.shutdown();
		//stop
            	ros::Time go = ros::Time::now();
            	while ((ros::Time::now() - go) < ros::Duration(0.1) && ros::ok())
            	{
                	c.linear.x = 0;
                	c.angular.z = 0;
                	vel_pub.publish(c);
                	ros::spinOnce();
                	loop_rate.sleep();
            	}
		//rotate
            	tf = n.subscribe("/tf", 1, Turnprog);
            	current_angle = INITIALIZE_VALUE;
		bool flag = true;
            	while (current_angle == INITIALIZE_VALUE && ros::ok())
            	{
            		ros::spinOnce();
            		loop_rate.sleep();
            	}

            	double target_angle;
		

		if (direction == 1)            	
		{			
			target_angle = current_angle - g_step_angle;
			
		}		
		if (direction == 0)
		{
			target_angle = current_angle + g_step_angle;
			
		}
		if (target_angle > 2*PI) target_angle -= 2*PI;
		if (target_angle < 0) target_angle += 2 * PI;
		double diff;
            	while (ros::ok())
            	{
               		if (direction == 1) diff = wrapToPi(target_angle - current_angle);
			if (direction == 0) diff = wrapToPi(current_angle - target_angle);
			//std::cout << "Diff ="<< diff << std::endl;

			//diff = wrapToPi(target_angle - current_angle);
                	if (fabs(diff) < 0.05) break;
			
            		c.linear.x = 0;
		        if (direction == 1) c.angular.z = -fabs(diff) / PI - (0.5);
			if (direction == 0) c.angular.z = (fabs(diff) / PI) + (0.5);
            		//c.angular.z = -fabs(diff) / PI + (0.5 * direction);
			//std::cout << "direction ="<< direction << std::endl;			
			//std::cout << "anglular ="<< c.angular.z << std::endl;
            		vel_pub.publish(c);

            		ros::spinOnce();
            		loop_rate.sleep();
            	}

            	tf.shutdown();
	    }//end for
    	   c.linear.x = 0;
    	   c.angular.z = 0;
    	   vel_pub.publish(c);
   	 }//end if


    }//end else

  }//end while ros
  return 0;
}//end main

