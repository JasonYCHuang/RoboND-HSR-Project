#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <string>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void go_to_target(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal, double pos_x, double pos_y, double ori_w, const std::string& target) {
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pos_x;
  goal.target_pose.pose.position.y = pos_y;
  goal.target_pose.pose.orientation.w = ori_w;

   // Send the goal position and orientation for the robot to reach
  const std::string info_action = std::string("Moving to ") + target;
  ROS_INFO(info_action.c_str());
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  const std::string info_success = std::string("Hooray, the base moved to ") + target;
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO(info_success.c_str());
  else
    ROS_INFO("Fails!!!");
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects_node");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  go_to_target(ac, goal, 4.0, 6.0, 1.0, "pickup");

  ros::Duration five_sec(5.0);
  five_sec.sleep();

  go_to_target(ac, goal, -4.0, 3.0, -1.0, "dropoff");

  return 0;
}