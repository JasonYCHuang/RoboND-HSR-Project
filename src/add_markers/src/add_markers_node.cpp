#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <cmath> 

struct pose {
  float x;
  float y;
  float z;
  float ox;
  float oy;
  float oz;
  float ow;
};

pose POSE_PICKUP = {4.0, 6.0, 0.0, 0.0, 0.0, 0.0, 1.0};

pose POSE_DROPOFF = {-4.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0};

bool REACH_PICKUP = false;
bool REACH_DROPOFF = false;

void set_color(visualization_msgs::Marker& marker) {
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;
}

void set_scale(visualization_msgs::Marker& marker) {
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
}

void init_marker(visualization_msgs::Marker& marker) {
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "add_markers_ns";
  marker.id = 1;

  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.type = shape;

  marker.lifetime = ros::Duration();

  set_scale(marker);
  set_color(marker);
}

void set_pose(visualization_msgs::Marker& marker, pose& target) {
  marker.pose.position.x = target.x;
  marker.pose.position.y = target.y;
  marker.pose.position.z = target.z;
  marker.pose.orientation.x = target.ox;
  marker.pose.orientation.y = target.oy;
  marker.pose.orientation.z = target.oz;
  marker.pose.orientation.w = target.ow;
}

void sleep_till_pubisher_ready(ros::Publisher& marker_pub) {
  while (marker_pub.getNumSubscribers() < 1) {
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
}

void odom_callback(const nav_msgs::Odometry& odom) {
  double x = odom.pose.pose.position.x;
  double y = odom.pose.pose.position.y;
  double dpx = abs(x - POSE_PICKUP.x);
  double dpy = abs(y - POSE_PICKUP.y);

  const double tolerance = 0.00001;
  
  if(!REACH_PICKUP && dpx < tolerance && dpy < tolerance){
    ROS_INFO(">>> - - - Reach Pickup - - - <<<");
    REACH_PICKUP = true;
  }

  double ddx = abs(x - POSE_DROPOFF.x);
  double ddy = abs(y - POSE_DROPOFF.y);

  if(!REACH_DROPOFF && ddx < tolerance && ddy < tolerance){
    ROS_INFO(">>> - - - Reach Dropoff - - - <<<");
    REACH_DROPOFF = true;
  }
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "add_markers_node");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 1000, odom_callback);

  if (!ros::ok()) {
    return 0;
  }

  visualization_msgs::Marker marker;
  init_marker(marker);
  sleep_till_pubisher_ready(marker_pub);

  // - - - - - - - - - - - - - - - - - - - - 
  ROS_INFO("Show marker @ pick-up");
  marker.action = visualization_msgs::Marker::ADD;
  set_pose(marker, POSE_PICKUP);
  marker_pub.publish(marker);

  while (!REACH_PICKUP) {
    ros::spinOnce();
    sleep(1);
  }
  sleep(5);

  // - - - - - - - - - - - - - - - - - - - - 
  ROS_INFO("Hide marker");
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  while (!REACH_DROPOFF) {
    ros::spinOnce();
    sleep(1);
  }
  sleep(5);

  // - - - - - - - - - - - - - - - - - - - - 
  ROS_INFO("Show marker @ drop-off");
  marker.action = visualization_msgs::Marker::ADD;
  set_pose(marker, POSE_DROPOFF);
  marker_pub.publish(marker);

}
