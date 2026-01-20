//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include "std_msgs/String.h"

void string_function(const std_msgs::String string_msg)
{
  //declare the variables
  std::string string_out;
  int str_len;
  //set the variable to the squared number
  string_out = string_msg.data;
  str_len = string_msg.data.size();


  //output the data
  std::cout << "The string " << string_out << " was published with length: " << str_len <<std::endl;
}

int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_string_listener");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  ros::Subscriber octogone = n.subscribe("octostring", 1, string_function);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {

    //looks for data
    ros::spin();
  }

  return 0;
}



