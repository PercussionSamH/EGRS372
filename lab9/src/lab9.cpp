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



//---------------- Callbacks ----------------



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





//---------------- Main ----------------
int main(int argc, char **argv)
{

  //File management
  FILE* file_pt; //makes a pointer to a file
  file_pt = fopen("/home/egrs372/catkin_ws/src/lab9/Parameters.txt","r"); //assigns the pointer to parameters

  //names the programs node
  ros::init(argc, argv, "Lab9");
  ros::NodeHandle n;

  //Publishers 
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);  

  //Subscribers
  ros::Subscriber goal_reached = n.subscribe("/move_base/result",1, goal_callback); 
  
  //Msgs
  geometry_msgs::PoseStamped pstamp;

  
  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  std::cout << "Starting wait...\n";
  for (int i=1; i<20; i++)
  {
     loop_rate.sleep();
  }
  std::cout << "waiting for localization goal reached...\n";
  goal_reached_flag = 0;
  while (goal_reached_flag == 0 && ros::ok())
  {
	ros::spinOnce();
	loop_rate.sleep(); 
  }


  std::cout << "Done waiting\n";



for (int i=1; i<6; i++)
{
    std::cout << "-----\n\ni = " << i << "\n";
    if (i==1)
    {
    	//pull goal location 1 from launch file    
    	ros::param::get("/pnt_1/x",goal_x);
    	ros::param::get("/pnt_1/y",goal_y);
    	ros::param::get("/pnt_1/theta",goal_theta);
    }
    if (i==2)
    {
    	//pull goal location 2 from launch file     
    	ros::param::get("/pnt_2/x",goal_x);
    	ros::param::get("/pnt_2/y",goal_y);
    	ros::param::get("/pnt_2/theta",goal_theta);
    }
    if (i==3)
    {
    	//pull goal location 3 from launch file    
    	ros::param::get("/pnt_3/x",goal_x);
    	ros::param::get("/pnt_3/y",goal_y);
    	ros::param::get("/pnt_3/theta",goal_theta);
    }
    if (i==4)
    {
    	//pull goal location 4 from launch file    
    	ros::param::get("/pnt_4/x",goal_x);
    	ros::param::get("/pnt_4/y",goal_y);
    	ros::param::get("/pnt_4/theta",goal_theta);
    }
    if (i==5)
    {
    	//pull goal location 5 from launch file    
    	ros::param::get("/pnt_5/x",goal_x);
    	ros::param::get("/pnt_5/y",goal_y);
    	ros::param::get("/pnt_5/theta",goal_theta);
    }

    //read data from file file_pt is pointing at
    fscanf(file_pt,"%lf,", &max_vel_lin);
    std::cout << "max vel read as:  " << max_vel_lin << "\n";
    fscanf(file_pt,"%c,", isBackAllow);
    std::cout << "isBackAllow read as:  " << isBackAllow << "\n";
    fscanf(file_pt,"%lf,", &max_rot_vel);
    std::cout << "max_rot_vel read as:  " << max_rot_vel << "\n";
    fscanf(file_pt,"%c,", isAdjustOrient);
    std::cout << "isAdjustOrient read as:  " << isAdjustOrient << "\n";

 
//Setting new params
  //creating the dynamic reconfigure variables
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter double_param;
  dynamic_reconfigure::Config conf;

  //setting max_vel_x
  double_param.name = "max_vel_x";
  double_param.value = max_vel_lin;
  conf.doubles.push_back(double_param);
  
  //setting max_vel_angular
  double_param.name = "max_rot_vel";
  double_param.value = max_rot_vel;
  conf.doubles.push_back(double_param);

  //setting min_vel_x
  min_vel_lin = -max_vel_lin;
  if (isBackAllow[0] == 'f')
  {
      min_vel_lin = 0.0;
  }
  std::cout << "min_vel_lin set to:  " << min_vel_lin << "\n";
  double_param.name = "min_vel_x";
  double_param.value = min_vel_lin;
  conf.doubles.push_back(double_param);


  //Setting allowable yaw range
  yaw = 10; //Allow for any orientation
  if (isAdjustOrient[0] == 't')
  {
      yaw = 0.17; //ensure the MR adjusts orientation
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

    goal_reached_flag = 0;	

    //convert to rad
    input_angle = goal_theta * PI / 180;

    //Convert to quaterions
    q.setRPY( 0, 0, input_angle);
    q.normalize();
    q_x = q[0];
    q_y = q[1];
    q_z = q[2];
    q_w = q[3];
    //set var data
    pstamp.header.frame_id = "map"; // Set the reference frame ID to use map (not MR frame)
    pstamp.pose.position.x = goal_x;
    pstamp.pose.position.y = goal_y;
    pstamp.pose.position.z = pos_z;
    pstamp.pose.orientation.x = q_x;
    pstamp.pose.orientation.y = q_y;
    pstamp.pose.orientation.z = q_z;
    pstamp.pose.orientation.w = q_w;
	
    //publish pose
    pose_pub.publish(pstamp);
    std::cout << "published new pos\n";
    //loop until goal reached
    while (goal_reached_flag == 0 && ros::ok())
    {
	ros::spinOnce();
	loop_rate.sleep(); 
    }

}

//--------

  ros::spinOnce();
  loop_rate.sleep();


  return 0;
}




