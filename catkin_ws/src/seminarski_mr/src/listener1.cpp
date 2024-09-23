#include "astar.hpp"

#include <cmath>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/tf.h>

#define PI 3.14159265

cv::Point goal(104, 150);


int counter = 0;

bool astarFlag = 0;
vector<cv::Point> pathToTake;
nav_msgs::Path path1;

ros::Publisher pubControl;
ros::Publisher path_pub;
ros::Publisher chatterPub;

bool aligned = 0;
bool arrived = 0;
bool endReceived = 0;
bool goalReceived = 0;
double goal_x = -0.7;
double goal_y = -2.1;

void chatterTalkerCallback(const std_msgs::String::ConstPtr& msg) {
  std::string message = msg->data;
  
  // Check if message starts with "Goal:"
  if (message.compare(0, 5, "Goal:") == 0) {
    int commaPos = message.find(',');
    
    // Extract values if the comma is found
    if (commaPos != -1) {
      double goalX = std::stod(message.substr(5, commaPos - 5));  // Extract the first double
      double goalY = std::stod(message.substr(commaPos + 1));      // Extract the second double
      
      goal.x = 150 - goalY*20;
      goal.y = 150 - goalX*20;
      goalReceived = 1;
      ROS_INFO("Goal received: x = %f, y = %f", goalX, goalY);
    } else {
      ROS_WARN("Invalid Goal format. Comma not found.");
    }
  }
  else if (message == "End") {
    endReceived = 1;
  } 

}




void movingCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {
     double current_x=msg->pose.pose.position.x;
     double current_y=msg->pose.pose.position.y;
     
     
     tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
     tf::Matrix3x3 m(q);
     double roll, pitch, yaw;
     m.getRPY(roll, pitch, yaw);
 
     double current_theta=yaw;
     double goal_angle = atan2(goal_y - current_y, goal_x - current_x);
     double delta_theta = (current_theta - goal_angle) * 180 / PI;
     
     ROS_INFO("Goal angle: %lf, current angle: %lf, current x position: %lf, current y position: %lf, GOAL X: %lf, GOAL Y: %lf", goal_angle*180 / PI, current_theta*180 / PI, current_x, current_y, goal_x, goal_y);
     
     delta_theta = goal_angle - current_theta;
     delta_theta = atan2(sin(delta_theta), cos(delta_theta));

     if(fabs(delta_theta)>0.15){
        aligned = 0;
     }else{
        aligned = 1;
     }
     
     geometry_msgs::Twist control;
     if(!aligned && delta_theta > 0){
     
     	if(delta_theta > 0.4) control.angular.z = 0.3;
     	else if (delta_theta > 0.2) control.angular.z = 0.25;
     	else control.angular.z = 0.15;
     	
        //control.angular.z = (delta_theta > 0.2) ? 0.2 : 0.1;
     }
     else if(!aligned && delta_theta < 0){
       if(delta_theta < -0.4) control.angular.z = -0.3;
     	else if (delta_theta < -0.2) control.angular.z = -0.25;
     	else control.angular.z = -0.15;
        //control.angular.z = (delta_theta < -0.2) ? -0.2 : -0.1;
     }
     else{
        control.angular.z = 0;
     }
     
     double udaljenost = sqrt((goal_x - current_x)*(goal_x - current_x) + (goal_y - current_y)*(goal_y - current_y));

     if(abs(udaljenost > 0.025)){
        arrived = 0;
     }
     else{
        arrived = 1;
     }

     if(aligned && !arrived){
        control.linear.x = 0.15;
     }
     else if(arrived){
        control.linear.x = 0;
        //control.angular.z = 0;
        //goal_x=0;
        //goal_y=0;
     }
     
     pubControl.publish(control);
     
     path1.header.frame_id = "odom";
     path1.header.stamp = ros::Time::now();
	     
     geometry_msgs::PoseStamped current_pose;
     current_pose.header.stamp = ros::Time::now();
     current_pose.header.frame_id = "odom";  // The frame where path is visualized

     current_pose.pose.position.x = current_x;  
     current_pose.pose.position.y = current_y;
     current_pose.pose.position.z = 0;

     current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
     path1.poses.push_back(current_pose);
     path_pub.publish(path1);

    }




 void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
 {

     if(goalReceived){
       double current_x=msg->pose.pose.position.x;
       double current_y=msg->pose.pose.position.y;
     
     ROS_INFO("x pozicija: %lf, y pozicija: %lf",current_x, current_y);
     
     if(astarFlag == 0) {
       astarFlag = 1;
       Point start(150-current_y*20, 150-current_x*20);
       ROS_INFO("\n\n Current starting position - x position: %d, y position: %d",start.x, start.y);
       pathToTake = runAll(start);
       if (pathToTake.empty()) {  
         ROS_WARN("No path found! Shutting down listener.");
         ros::shutdown();  // Stop the ROS node and exit listener
         return;
         }
         
       ROS_INFO("Path from the start to the goal: ");
       for (const auto &p : pathToTake) {
          ROS_INFO("Image coordinates: (%d , %d), map coordinates: (%.3f, %.3f)", p.x, p.y, (150-p.y)/20., (150-p.x)/20.);
        }
     }
     else {
       goal_x = (150-pathToTake[counter].y)/20.;
       goal_y = (150-pathToTake[counter].x)/20.;
       ROS_INFO("\n Current goal %lf %lf %d",goal_x, goal_y, pathToTake[counter].x);
       movingCallback(msg);
       if(arrived) {
         arrived = 0;
         if(counter < pathToTake.size()-1) counter++;
         else {
           ROS_INFO("Reached goal.");
           goalReceived = 0;
           astarFlag = 0;
           counter = 0;
           std_msgs::String msg1;
           msg1.data = "Next goal?";
           chatterPub.publish(msg1);
         }
       }
     }
     }
     
     else if (endReceived) {
       ROS_WARN("Reached final goal. Shuting down...");
       ros::shutdown();
     }
     else ROS_INFO("Talker didn't send a goal!");
     
 }
 


int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  pubControl = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  path_pub = n.advertise<nav_msgs::Path>("robot_path", 100);
  ros::Subscriber sub1 = n.subscribe("chatter", 1000, chatterTalkerCallback);
  chatterPub = n.advertise<std_msgs::String>("listener", 1000);

  ros::spin();

  return 0;
}

