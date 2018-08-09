#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

struct pose {
  float x;
  float y;
  float z;
  float ox;
  float oy;
  float oz;
  float ow;
};

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

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_node");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Duration fivs_sec(5.0);

  visualization_msgs::Marker marker;
  init_marker(marker);
  set_scale(marker);
  set_color(marker);

  while (marker_pub.getNumSubscribers() < 1) {
    if (!ros::ok()) {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  while (ros::ok())
  {
    ROS_INFO("Show marker @ pick-up...");
    marker.action = visualization_msgs::Marker::ADD;
    pose target_pickup = {4.0, 6.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    set_pose(marker, target_pickup);
    marker_pub.publish(marker);

    fivs_sec.sleep();

    ROS_INFO("Hide marker...");
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);

    fivs_sec.sleep();

    ROS_INFO("Show marker @ drop-off...");
    marker.action = visualization_msgs::Marker::ADD;
    pose target_dropoff = {-4.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    set_pose(marker, target_dropoff);
    marker_pub.publish(marker);

    fivs_sec.sleep();
  }
}
