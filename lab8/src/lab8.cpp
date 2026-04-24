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


//custom msgs
#include "lab8/turtlebot_status.h"
//custom services
#include "lab8/go_home.h"
#include "lab8/return_to_work.h"
#include "lab8/update_count.h"

#define INITIALIZE_VALUE -1
#define PI 3.14159265359

// ---------------- GLOBAL VARIABLES ----------------
double current_angle = INITIALIZE_VALUE;
double pos_z = 0.0;

double pick_x;
double pick_y;
double pick_theta;

double place_x;
double place_y;
double place_theta;

double home_x;
double home_y;
double home_theta;

double q_x = 0.0;
double q_y = 0.0;
double q_z = 0.0;
double q_w = 0.0;
double input_angle = 0.0; //in deg
int button_flag = 0;

double battery_high_value = 0.0;
double battery_low_value = 0.0;

float batt_voltage = 0.0;
bool batt_OK = false;
bool backup_done = false;
bool goal_reached_flag;

bool update_count(lab8::update_count::Request &req,lab8::update_count::Response &res);
bool go_home(lab8::go_home::Request &req,lab8::go_home::Response &res);
bool return_to_work(lab8::return_to_work::Request &req,lab8::return_to_work::Response &res);

int places = 0;
std::string new_job = "";
std::string old_job = "";

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
    if (batt_result.voltage > battery_high_value)
	batt_OK = 1;
	
    if (batt_result.voltage <= battery_low_value)
    {
	std::cout << "low batt triggered\n";
	batt_OK = 0;
    }
    batt_voltage = batt_result.voltage;
}


//function for the update count service
//the service requests a new count for the LED
//the service responds with the old LED count
bool update_count(lab8::update_count::Request  &req, lab8::update_count::Response &res)
{
  //creates a new Node handler to publish the data inside this service
  ros::NodeHandle m;

  //creates the publisher for the LED data
  ros::Publisher led_screen_pub = m.advertise<std_msgs::Int32>("LED_node", 1);

  //creates the message variable
  std_msgs::Int32 LED_update;

  //sets the response to the old number of places
  res.old_count = places;

  //updates the places with the request
  places = req.new_count;

  //updates the message variable with the new number
  LED_update.data=places;

  //publishes the new LED data
  led_screen_pub.publish(LED_update);
  return true;
}


//function for the go_home
//the service requests nothing
//the service responds with the previous job
bool go_home(lab8::go_home::Request  &req, lab8::go_home::Response &res)
{
  //creates a new Node handler to publish the data inside this service
  ros::NodeHandle m;

  //creates the publisher for the LED data
  ros::Publisher turtle_job_pub = m.advertise<std_msgs::String>("turtle_job", 0);

  //creates the message variable
  std_msgs::String job_update;

  //records the previous non-home job
  if (new_job != "home")
  {
      old_job = new_job;
  }
  res.old_job = old_job;
  
  //updates job home
  new_job = "home";

  return true;
}


//function for the return_to_work
//the service requests nothing
//the service responds with the previous job that is restored
bool return_to_work(lab8::return_to_work::Request  &req, lab8::return_to_work::Response &res)
{

  if (new_job == "home")
  {
     new_job = old_job; //return to old job before going to home
     res.old_job = old_job;
  }
  return true;
}



// ---------------- MAIN -------------------
int main(int argc, char **argv)
{
    ros::init(argc, argv, "lab8_program");
    ros::NodeHandle n;
    ros::param::get("/battery_low_value",battery_low_value);
    ros::param::get("/battery_high_value",battery_high_value);
    
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",50);   
    geometry_msgs::PoseStamped pstamp;

    ros::ServiceServer count_service = n.advertiseService("update_count",update_count);
    ros::ServiceServer go_home_service = n.advertiseService("go_home",go_home);
    ros::ServiceServer return_to_work_service = n.advertiseService("return_to_work",return_to_work);

    ros::Publisher led_screen_pub =  n.advertise<std_msgs::Int32>("/LED_node",50);
    ros::Publisher status_pub = n.advertise<lab8::turtlebot_status>("TurtleBot_Status",1);

    ros::Subscriber button_press = n.subscribe("/bumper_node", 1, bumper_callback);    
    ros::Subscriber goal_reached = n.subscribe("/move_base/result",1, goal_callback);
    ros::Subscriber battery_status = n.subscribe("/battery_state",1, batt_callback);

    std_msgs::Int32 items_msg;
    lab8::turtlebot_status job_msg;

    
    job_msg.current_job = "Initializing...";
    job_msg.place_count = places;
    job_msg.battery = batt_voltage;
    status_pub.publish(job_msg);

    ros::Time go;

    ros::Rate loop_rate(500);

    //wait for all functions to initialize
    go = ros::Time::now();
    while(ros::Time::now() - go < ros::Duration(0.5) && ros::ok())
    {
    	ros::spinOnce();
        loop_rate.sleep();
    }
    
    ros::spinOnce();
    loop_rate.sleep(); 

    places = 0; //reset # of items placed
    items_msg.data = places;
    led_screen_pub.publish(items_msg);


    std::cout << "items placed = " << items_placed << "\n";

    ros::spinOnce();
    loop_rate.sleep(); 

    //pull home location from param file    
    ros::param::get("/home_location/x",home_x);
    ros::param::get("/home_location/y",home_y);
    ros::param::get("/home_location/theta",home_theta);
    
    std::cout << "Program Started....\n";
    
    job_msg.current_job = "home";
    job_msg.place_count = places;
    job_msg.battery = batt_voltage;   
    status_pub.publish(job_msg);

//-----Start by moving to home -----
    goal_reached_flag = 0;	

    //convert to rad
    input_angle = home_theta * PI / 180;

    //Convert to quaterions
    q.setRPY( 0, 0, input_angle);
    q.normalize();
    q_x = q[0];
    q_y = q[1];
    q_z = q[2];
    q_w = q[3];
    //set var data
    pstamp.header.frame_id = "map"; // Set the reference frame ID
    pstamp.pose.position.x = home_x;
    pstamp.pose.position.y = home_y;
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
	
    //default to pick task
    new_job = "pick";



// -------- MAIN LOOP --------
    while(ros::ok())
    {
	//loop
	while (batt_OK == 1 && ros::ok() && new_job != "home")
	{
	    while(batt_OK == 1 && ros::ok() && (new_job != "pick" && new_job != "place"))
	    {
            	ros::spinOnce();
	    	loop_rate.sleep();

	    }    
		
// ----- PICK ROUTINE -----
	    if (batt_OK == 1 && new_job == "pick")
	    {
		//publish status
            	job_msg.current_job = "pick";
    	   	job_msg.place_count = places;
    	    	job_msg.battery = batt_voltage;
            	status_pub.publish(job_msg);
	        //fetch params
	    	ros::param::get("/pick_location/x",pick_x);
    	    	ros::param::get("/pick_location/y",pick_y);
            	ros::param::get("/pick_location/theta",pick_theta);

            	ros::spinOnce();
	    	loop_rate.sleep();
		goal_reached_flag = 0;	
	    
            	//convert to rad
    	    	input_angle = pick_theta * PI / 180;
	    	//Convert to quaterions
	    	q.setRPY( 0, 0, input_angle);
	    	q.normalize();
	    	q_x = q[0];
 	    	q_y = q[1];
	    	q_z = q[2];
	    	q_w = q[3];
	    	//set var data
	    	pstamp.header.frame_id = "map"; // Set the reference frame ID	
	    	pstamp.pose.position.x = pick_x;
	    	pstamp.pose.position.y = pick_y;
	    	pstamp.pose.position.z = pos_z;
	    	pstamp.pose.orientation.x = q_x;
	    	pstamp.pose.orientation.y = q_y;
	    	pstamp.pose.orientation.z = q_z;
 	    	pstamp.pose.orientation.w = q_w;
	
 	    	//publish
	    	pose_pub.publish(pstamp);
    	    	std::cout << "published new pos\n";

		//wait for goal reached or job changed
	    	while (goal_reached_flag == 0 && batt_OK == 1 && new_job=="pick" && ros::ok())
	    	{
		    ros::spinOnce();
		    loop_rate.sleep(); 
	    	}
	    	goal_reached_flag = 0;
		//wait for button or job change
	    	while (button_flag == 0 && batt_OK == 1 && new_job=="pick" && ros::ok())
	    	{
	    	    ros::spinOnce();
	   	    loop_rate.sleep(); 
	    	}
		//if job is completed: set job to place
		if (new_job == "pick") 
		{
		    new_job = "place";
		}

	    }

// ----- PLACE ROUTINE -----
	    if (batt_OK == 1 && new_job == "place")
	    {		
		//update status
		job_msg.current_job = "place";
    	    	job_msg.place_count = places;
    	    	job_msg.battery = batt_voltage;
           	status_pub.publish(job_msg);

		//fetch place location from params    
    		ros::param::get("/place_location/x",place_x);
    		ros::param::get("/place_location/y",place_y);
    		ros::param::get("/place_location/theta",place_theta);
		ros::spinOnce();
		loop_rate.sleep();


	    	goal_reached_flag = 0;		

            	//Convert angle to rad
    	    	input_angle = place_theta * PI / 180;
	    	//Convert orientation to quaterions
	    	q.setRPY( 0, 0, input_angle);
	    	q.normalize();
	    	q_x = q[0];
 	    	q_y = q[1];
	    	q_z = q[2];
	    	q_w = q[3];
	    	//set location data
	    	pstamp.header.frame_id = "map"; // Set the reference frame ID	
	    	pstamp.pose.position.x = place_x;
	    	pstamp.pose.position.y = place_y;
	    	pstamp.pose.position.z = pos_z;
	    	pstamp.pose.orientation.x = q_x;
	    	pstamp.pose.orientation.y = q_y;
	    	pstamp.pose.orientation.z = q_z;
 	    	pstamp.pose.orientation.w = q_w;
	
 	    	//publish pose goal
	    	pose_pub.publish(pstamp);
    	    	std::cout << "published new pos\n";
	    
		//wait for position reached or new job
	    	while (goal_reached_flag == 0 && batt_OK == 1 && new_job=="place" && ros::ok())
	    	{
			ros::spinOnce();
			loop_rate.sleep(); 
	    	}
	    	goal_reached_flag = 0;
		//wait for button or new job
	    	while (button_flag == 0 && batt_OK == 1 && new_job=="place" && ros::ok())
	    	{
	    		ros::spinOnce();
	    		loop_rate.sleep(); 
	    	}
		//if job was completed (didn't change jobs)
	    	if (new_job == "place")
	    	{
		    //increment # of items placed, publish new count
	    	    places = places +1;
	    	    items_msg.data = places;
	    	    led_screen_pub.publish(items_msg);
		    //set task to pick
		    new_job = "pick";
    	    	}
	    }
	}
// ----- HOME ROUTINE -----

	job_msg.current_job = "home";
    	job_msg.place_count = places;
    	job_msg.battery = batt_voltage;
        status_pub.publish(job_msg);

	goal_reached_flag = 0;	
	
	ros::param::get("/home_location/x",home_x);
    	ros::param::get("/home_location/y",home_y);
    	ros::param::get("/home_location/theta",home_theta);

        //convert angle to rad
    	input_angle = home_theta * PI / 180;

    	//Convert orientation to quaterions
    	q.setRPY( 0, 0, input_angle);
    	q.normalize();
    	q_x = q[0];
    	q_y = q[1];
    	q_z = q[2];
    	q_w = q[3];
    	//set var data
    	pstamp.header.frame_id = "map"; // Set the reference frame ID
    	pstamp.pose.position.x = home_x;
    	pstamp.pose.position.y = home_y;
    	pstamp.pose.position.z = pos_z;
    	pstamp.pose.orientation.x = q_x;
    	pstamp.pose.orientation.y = q_y;
    	pstamp.pose.orientation.z = q_z;
    	pstamp.pose.orientation.w = q_w;
	
    	//publish
    	pose_pub.publish(pstamp);
    	std::cout << "published new pos\n";
	
	//wait for home reached or job change
	while (goal_reached_flag == 0 && new_job=="home" && ros::ok())
	{
	    ros::spinOnce();
	    loop_rate.sleep(); 
	}
	//wait for batt ok or job change
	int i = 0;
	while ((batt_OK == 0 || i<5) && new_job=="home" && ros::ok())
	{
	    if (batt_OK == 0)
	    {
		i=0;
	    }
	    i++;
	    ros::spinOnce();
	    loop_rate.sleep();
	}
	// wait for new job
	while (new_job=="home" && ros::ok())
	{
	    ros::spinOnce();
	    loop_rate.sleep();
	}

    }//end while ros::OK
return 0;

}//end main

