#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

double pickupX = -1.0;
double pickupY = -7.2;
double dropX = 6.5;
double dropY = -7.2;

float x_cor = 0.0;
float y_cor = 0.0;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // Extract the position from the odometry message
    // geometry_msgs::Pose pose = msg->pose.pose;
    x_cor = msg->pose.pose.position.x;
    y_cor = msg->pose.pose.position.y;

    // Process the position data here
    // You can use the x, y, and z variables to access the position values

    // ROS_INFO("Robot Position: x_cor = %f, x_cor = %f ", x_cor, x_cor);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle nh;
  ros::Rate r(1); // 1 Hz

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Subscribe to the odometry topic
  ros::Subscriber odomSubscriber = nh.subscribe("/odom", 10, odometryCallback);

  // Create a marker message
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "add_markers";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  ROS_INFO("add_markers: active");

  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();

  // Wait for 5 seconds
  ros::Duration(1.0).sleep();


    // Publish the marker initially at the pickup zone
    marker.pose.position.x = pickupX;
    marker.pose.position.y = pickupY;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    // ROS_INFO("Marker published at the pickup zone");
    // ROS_INFO("Robot Position: pickupX = %f, pickupY = %f ", pickupX, pickupY);
    // ROS_INFO("Robot Position: x_cor = %f, x_cor = %f ", x_cor, x_cor);

  
  // Wait for 5 seconds
  ros::Duration(2.0).sleep();

  if ( (abs(pickupX - x_cor)<0.5) && (abs(pickupY - y_cor )<0.5 ))
  {
  // Hide the marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Marker hidden");
  }
  // Wait for 5 seconds
  ros::Duration(5.0).sleep();

  if ( (abs(dropX - x_cor)<0.5) && (abs( dropY - y_cor)<0.5) )
  {
    // Publish the marker at the drop-off zone
    marker.pose.position.x = dropX;
    marker.pose.position.y = dropY;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);
    ROS_INFO("Marker published at the drop-off zone");
  }
  
  // Wait for 5 seconds
  ros::Duration(5.0).sleep();

  ros::spin();

  return 0;
}