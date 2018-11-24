#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

float goal_x = 0.0;
float goal_y = 0.0;
bool near_goal = false;

void odom_callback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  const float threshold = 0.4;

  if (sqrt(pow(goal_x - odom_msg->pose.pose.position.x, 2) + pow(goal_y - odom_msg->pose.pose.position.y, 2)) < threshold)
    near_goal = true;
  else
    near_goal = false;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odometry_sub = n.subscribe("odom", 1000, odom_callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "cube";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 2.5;
  marker.pose.position.y = -0.5;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
  
  goal_x = marker.pose.position.x;
  goal_y = marker.pose.position.y;
  
  ros::Duration time_between_ros_wakeups(0.001);
  while (ros::ok() && !near_goal) {
    ros::spinOnce();
    time_between_ros_wakeups.sleep();
  }

  ROS_INFO("Object has been picked up");
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);

  marker.pose.position.x = -4.0;
  marker.pose.position.y = 5.0;
  near_goal = false;
  goal_x = marker.pose.position.x;
  goal_y = marker.pose.position.y;
  
  ROS_INFO("Delivering object");
  
  while (ros::ok() && !near_goal) {
    ros::spinOnce();
    time_between_ros_wakeups.sleep();
  }

  ROS_INFO("Object has been delivered");
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  
  sleep(5);
  
  r.sleep();
}