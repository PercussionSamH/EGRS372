//Includes all of the ROS libraries needed
#include "ros/ros.h"
#include <sstream>
#include "math.h"
#include <iostream>
#include "std_msgs/String.h"


int main(int argc, char **argv)
{

  //names the program for visual purposes
  ros::init(argc, argv, "Lab2_string_input");
  ros::NodeHandle n;

  //sets the frequency for which the program sleeps at. 10=1/10 second
  ros::Rate loop_rate(10);

  //declare variables
  std::string string_output;
  std_msgs::String string_msg;


  //declare publisher "octostring" is the name of the node
  //1 is the number of values to keep stored until they are overwritten
  ros::Publisher octogone = n.advertise<std_msgs::String>("octostring", 1);

  //rosk::ok() will stop when the user inputs Ctrl+C
  while(ros::ok())
  {
    //clear the input buffer
    std::fflush;

    //prompt the user for an input
    std::cout << "Enter a string to publish: ";
    //get the input from the user
    std::getline(std::cin, string_output, '\n');
    //std::getline >> string_output;

     //confirm the number is being sent
    std::cout << "Sending the string " << string_output << std::endl;

    //set the message value
    string_msg.data=string_output;
    //publish the data
    octogone.publish(string_msg);
    

    //sends out any data necessary then waits based on the loop rate
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}




