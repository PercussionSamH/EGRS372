//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>


#include <stdio.h>
#include <stdlib.h>

//Params from Lab 8
#include <iostream>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <std_msgs/Byte.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/BatteryState.h>
#include <move_base_msgs/MoveBaseAction.h> 

//These libraries are needed for the dynamic reconfiguring
//DoubleParameter could be substituted for IntParameter if ints needed to be set instead of doubles, this applies for all variables
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ros/service.h>


//Define consts
#define INITIALIZE_VALUE -1
#define PI 3.14159265359

//global vars
bool goal_reached_flag;
bool slow_flag = false;

double current_angle = INITIALIZE_VALUE;
double pos_z = 0.0;

double goal_x;
double goal_y;
double goal_theta;

double q_x = 0.0;
double q_y = 0.0;
double q_z = 0.0;
double q_w = 0.0;
tf2::Quaternion q;

double input_angle = 0.0; //in deg
int button_flag = 0;


//param read in vars
double yaw = 0;
double max_vel_lin = 0;
double min_vel_lin = 0;
char isBackAllow [1] = "";
double max_rot_vel = 0;
char isAdjustOrient[1] = "";
//params to track max velocities through a safety stop
double old_max_vel_lin =0;
double old_min_vel_lin =0;
double old_max_rot_vel =0;

//Human position (x,y,z)
int human_x = 0;
int human_y = 0;
int human_z = 0;
double radius_human = 0.5; //radius of the human represented as a circle

//Crosswalk location (rect box)
int x_cross_min = 0;
int x_cross_max = 1;
int y_cross_min = -4;
int y_cross_max = 0;

//Position of the mobile robot
double xMR = 0;
double yMR = 0;
double zMR = 0;


std_msgs::Bool motor_msg;



//---------------- Callbacks ----------------


// ---------------- HUMAN CALLBACK ----------------
//fetches the position of the simulated human when a new position is published
void human_pos_callback(const geometry_msgs::Point position)
{
    human_x = position.x;
    human_y = position.y;
    human_z = position.z;
}


// ---------------- GOAL REACHED CALLBACK ----------------
//updates the goal reached flag to notify the robot when it reaches its goal
void goal_callback(const move_base_msgs::MoveBaseActionResult goal_result)
{
    goal_reached_flag = 0;
    if (goal_result.status.status == 3) //checks if the robot has reached its desired goal (within set tolerance)
    {
	    std::cout << "goal reached\n"; //print statement
	    goal_reached_flag = 1;
    }
}


// ---------------- MR Position CALLBACK ----------------
//gets the position of the mobile robot in xyz coords (currently only used to check if the robot is too close to a human)
void pos_feed_callback(const move_base_msgs::MoveBaseActionFeedback feedback_result)
{
    xMR = feedback_result.feedback.base_position.pose.position.x;
    yMR = feedback_result.feedback.base_position.pose.position.y;
    zMR = feedback_result.feedback.base_position.pose.position.z;
}



//---------------- Main ----------------
int main(int argc, char **argv)
{

  //File management
  FILE* file_pt; //makes a pointer to a file
  file_pt = fopen("/home/egrs372/catkin_ws/src/lab10/Parameters.txt","r"); //assigns the pointer to parameters

  //names the programs node
  ros::init(argc, argv, "Lab10");
  ros::NodeHandle n;

  //Publishers 
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);  
  ros::Publisher motor_pub = n.advertise<std_msgs::Bool>("/motor_power",1); //publishing false on this will cut motor power

  //Subscribers
  ros::Subscriber goal_reached = n.subscribe("/move_base/result",1, goal_callback);  //fetches goal data
  ros::Subscriber human_pos = n.subscribe("human",10,human_pos_callback); //subscribes to human position from human_sim
  ros::Subscriber MR_pos = n.subscribe("/move_base/feedback",10, pos_feed_callback); //fetches position data

  //Msgs
  geometry_msgs::PoseStamped pstamp;

  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  std::cout << "Starting wait...\n";
  for (int i=1; i<20; i++) //additional wait to ensure the other launch programs have time to boot up
  {
     loop_rate.sleep();
  }
  
  //send power to the motors
  motor_msg.data = true; 
  motor_pub.publish(motor_msg);
  
  //wait for a localization goal to be sent before the robot starts its code
  std::cout << "waiting for localization goal reached...\n"; 
  goal_reached_flag = 0;
  while (goal_reached_flag == 0 && ros::ok()) //wait until manually set goal is reached
  {
	ros::spinOnce();
	loop_rate.sleep(); 
  }
  std::cout << "Done waiting\n";

  //for loop to iterate through all 6 points
int j =1;
for (j=1; j<6; j++)
{
    std::cout << "-----\n\nj = " << j << "\n";
	//pull corresponding point data from the params file
    if (j==1)
    {
	std::cout << "pulling params\n";
    	//pull goal location 1 from launch file
    	ros::param::get("/pnt_1/x",goal_x);
    	ros::param::get("/pnt_1/y",goal_y);
    	ros::param::get("/pnt_1/theta",goal_theta);
	std::cout << "pulled params\n";
    }
    if (j==2)
    {
    	//pull goal location 2 from launch file     
    	ros::param::get("/pnt_2/x",goal_x);
    	ros::param::get("/pnt_2/y",goal_y);
    	ros::param::get("/pnt_2/theta",goal_theta);
    }
    if (j==3)
    {
    	//pull goal location 3 from launch file    
    	ros::param::get("/pnt_3/x",goal_x);
    	ros::param::get("/pnt_3/y",goal_y);
    	ros::param::get("/pnt_3/theta",goal_theta);
    }
    if (j==4)
    {
    	//pull goal location 4 from launch file    
    	ros::param::get("/pnt_4/x",goal_x);
    	ros::param::get("/pnt_4/y",goal_y);
    	ros::param::get("/pnt_4/theta",goal_theta);
    }
    if (j==5)
    {
    	//pull goal location 5 from launch file    
    	ros::param::get("/pnt_5/x",goal_x);
    	ros::param::get("/pnt_5/y",goal_y);
    	ros::param::get("/pnt_5/theta",goal_theta);
    }
	
	//read data from file file_pt is pointing at
    std::cout << "Reading data from file...\n";
    fscanf(file_pt,"%lf,", &max_vel_lin); //pull max velocity
    std::cout << "max vel read as:  " << max_vel_lin << "\n";
    fscanf(file_pt,"%c,", isBackAllow);  // check if the robot is allowed to reverse to reach this point
    std::cout << "isBackAllow read as:  " << isBackAllow << "\n";
    fscanf(file_pt,"%lf,", &max_rot_vel); //pull max rotation speed
    std::cout << "max_rot_vel read as:  " << max_rot_vel << "\n";
    fscanf(file_pt,"%c,", isAdjustOrient); //check if the robot needs to reach the set orientation for the given point
    std::cout << "isAdjustOrient read as:  " << isAdjustOrient << "\n";

 
//Setting new params
  //Creating the dynamic reconfigure variables 
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  //setting max linear velocity (forward)
  double_param.name = "max_vel_x";
  double_param.value = max_vel_lin;
  conf.doubles.push_back(double_param);
  
  //setting max rotational velocity
  double_param.name = "max_rot_vel";
  double_param.value = max_rot_vel;
  conf.doubles.push_back(double_param);

  //setting max linear velocity (backwards)
  min_vel_lin = -max_vel_lin;
  if (isBackAllow[0] == 'f') //if backing up is not allowed, then set backup speed to 0
  {
      min_vel_lin = 0.0;
  }
  std::cout << "min_vel_lin set to:  " << min_vel_lin << "\n";
  double_param.name = "min_vel_x";
  double_param.value = min_vel_lin;
  conf.doubles.push_back(double_param);

  //Setting allowable yaw range
  yaw = 10; //Allow for any orientation to reach a goal
  if (isAdjustOrient[0] == 't') //if the point requires you to match desired orientation
  {
      yaw = 0.17; //ensure the MR adjusts orientation to match desired angle
  }
  std::cout << "yaw set to:  " << yaw << "\n";
  double_param.name = "yaw_goal_tolerance";
  double_param.value = yaw;
  conf.doubles.push_back(double_param);

  //srv_req is whats actually sent to the service and is set to be equal to conf which should contain the parameters
  srv_req.config = conf;
  //calling the service to set the parameters (apply changes)
  ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
  //clearing conf so that it can be used later to set the parameters
  //if this is not done the values for max_vel_x and min_vel_x are still saved in conf
  conf.doubles.clear();



//move to point
    goal_reached_flag = 0;	//clear goal reached flag
	
    //convert to rad
    input_angle = goal_theta * PI / 180;

    //Convert angle to quaterions
    q.setRPY( 0, 0, input_angle);
    q.normalize();
    q_x = q[0];
    q_y = q[1];
    q_z = q[2];
    q_w = q[3];
    //set var data
    pstamp.header.frame_id = "map"; // Set the reference frame ID to use map (not MR frame)
    pstamp.pose.position.x = goal_x; //set up goal positional data
    pstamp.pose.position.y = goal_y;
    pstamp.pose.position.z = pos_z;
    pstamp.pose.orientation.x = q_x;
    pstamp.pose.orientation.y = q_y;
    pstamp.pose.orientation.z = q_z;
    pstamp.pose.orientation.w = q_w;
    //publish pose of goal
    pose_pub.publish(pstamp);
    std::cout << "published new pos\n";

    //Move to goal
    while (goal_reached_flag == 0 && ros::ok())
    {
	//Check position
	double dist_x = (xMR-human_x); //dist to human in x
	double dist_y = (yMR-human_y); //dist to human in y
	std::cout << "dist_x = " << dist_x << "\n";
	std::cout << "dist_y = " << dist_y << "\n";
    std::cout << "dist = " << ((dist_x*dist_x)+(dist_y*dist_y)) << "\n";
	
	if(sqrt((dist_x*dist_x)+(dist_y*dist_y))<(radius_human)) //if the diagonal distance to the center of the human is less than the acceptable distance (radius_human) then cut motor power
	{
	    while ((sqrt((dist_x*dist_x)+(dist_y*dist_y))<(radius_human)) && ros::ok())
	    {
		//Cut motor power until the human moves away

		motor_msg.data = false; //set motor power to false
		motor_pub.publish(motor_msg); //publish to cut power
		std::cout << "Human detected\n"; //print that a human is present
 		dist_x = (xMR-human_x); //recalc distance to human 
	 	dist_y = (yMR-human_y);
		std::cout << "dist_x = " << dist_x << "\n";
		std::cout << "dist_y = " << dist_y << "\n";
        std::cout << "dist = " << sqrt((dist_x*dist_x)+(dist_y*dist_y)) << "\n";
		//setting oscillation dist
  		double_param.name = "oscillation_distance";
  		double_param.value = 0.0; //this ensures the robot does not time out trying to reach its goal
  		conf.doubles.push_back(double_param);
		
		//Apply params
	    	//srv_req is whats actually sent to the service and is set to be equal to conf which should contain the parameters
  	    	srv_req.config = conf;
  	    	//calling the service to set the parameters (apply changes)
  	    	ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
  	    	//clearing conf so that it can be used later to set the parameters
  	    	//if this is not done the values for max_vel_x and min_vel_x are still saved in conf
  	    	conf.doubles.clear();
	
		ros::spinOnce();
		loop_rate.sleep();
		
	    } //end while
	    //allow for motion again
	    motor_msg.data = true;
	    motor_pub.publish(motor_msg);
	    std::cout << "human gone: resuming\n";
 	    
	}
	//check for crosswalk
	if( xMR > x_cross_min && xMR < x_cross_max && yMR > y_cross_min && yMR < y_cross_max )
	{
            std::cout << "slow!\n";
	    slow_flag = true;
	//---slow speed---
	    //setting max_vel_x
	    old_max_vel_lin = max_vel_lin;
  	    double_param.name = "max_vel_x";
	    if (max_vel_lin > 0.13)
	    {
  	    	double_param.value = 0.13;
	    }
  	    conf.doubles.push_back(double_param);

	    //setting min_vel_x
	    old_min_vel_lin = min_vel_lin;
	    if (min_vel_lin < -0.13)
	    {
  	    	min_vel_lin = -0.13;
	    }

  	    if (isBackAllow[0] == 'f')
  	    {
      		min_vel_lin = 0.0;
  	    }
  	    double_param.name = "min_vel_x";
  	    double_param.value = min_vel_lin;
  	    conf.doubles.push_back(double_param);

	    //store max angular speed
	    old_max_rot_vel = max_rot_vel;
	    if (max_rot_vel > 1)
	    {
		max_rot_vel = 1;
	    }
  	    double_param.name = "max_rot_vel";
  	    double_param.value = max_rot_vel;
  	    conf.doubles.push_back(double_param);
	    
	    //Apply params
	    //srv_req is whats actually sent to the service and is set to be equal to conf which should contain the parameters
  	    srv_req.config = conf;
  	    //calling the service to set the parameters (apply changes)
  	    ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
  	    //clearing conf so that it can be used later to set the parameters
  	    //if this is not done the values for max_vel_x and min_vel_x are still saved in conf
  	    conf.doubles.clear();

	    
	}
	else //nothing detected
	{
 	    std::cout << "nothing!\n";
	    if (slow_flag == true)
	    {
		//resume motion (potentially resend goal)
		std::cout << "RESUME!\n";
		slow_flag = false;
		
	    //-----restore motion params-----
	 	//setting max_vel_x
	    	max_vel_lin = old_max_vel_lin;
  	    	double_param.name = "max_vel_x";
		double_param.value = max_vel_lin;
  	    	conf.doubles.push_back(double_param);
	
		//min x
		min_vel_lin = old_min_vel_lin;
  	    	double_param.name = "min_vel_x";
  	    	double_param.value = min_vel_lin;
  	    	conf.doubles.push_back(double_param);

		//rotation
		max_rot_vel = old_max_rot_vel;
		double_param.name = "max_rot_vel";
  	    	double_param.value = max_rot_vel;
  	    	conf.doubles.push_back(double_param);
		
	        //Apply params
	    	//srv_req is whats actually sent to the service and is set to be equal to conf which should contain the parameters
  	    	srv_req.config = conf;
  	    	//calling the service to set the parameters (apply changes)
  	   	ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
  	    	//clearing conf so that it can be used later to set the parameters
  	    	//if this is not done the values for max_vel_x and min_vel_x are still saved in conf
  	   	conf.doubles.clear();
		
	    }
	}
	//Refresh vars and sleep
	ros::spinOnce();
	loop_rate.sleep(); 
    }
}

//--------

  ros::spinOnce();
  loop_rate.sleep();
  return 0;
}




