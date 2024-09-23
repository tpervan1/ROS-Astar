#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

 std::vector<std::pair<double, double>> goalPoints = {
      {-1.95, 0.5},
      {-0.8, 0.55},
      {-0.45, -0.6}, 
      {-1.85, -0.7}
  };
  
int counter = 0;
bool nextGoal = 1;
int count = 0;
std_msgs::String msg;
  
void listenerCallback(const std_msgs::String::ConstPtr& msg1) {
  std::string message = msg1->data;
  if (message == "Next goal?") {
    nextGoal = 1;
    count = 0;
    counter++;
    if(counter == goalPoints.size()) {
      ROS_WARN("Time to end");
      msg.data = "End";
    }
    else {
      std::stringstream ss_next;
      ss_next << "Goal: " << goalPoints[counter].first <<","<<goalPoints[counter].second;
      msg.data = ss_next.str();
    }
    
  }
}
  

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Subscriber sub = n.subscribe("listener", 1000, listenerCallback);

  ros::Rate loop_rate(5);
  
  std::stringstream ss_next;
  ss_next << "Goal: " << goalPoints[counter].first <<","<<goalPoints[counter].second;
  msg.data = ss_next.str();
  
  while (ros::ok())
  {    
    if (nextGoal && count < 5) {
      count++;
      if (count == 5) nextGoal = 0;  // Publish 5 times
      

      ROS_INFO("%s", msg.data.c_str());
      chatter_pub.publish(msg);
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

