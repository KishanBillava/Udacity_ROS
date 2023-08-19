#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "markers_time");
  ros::NodeHandle nh;
  ros::Rate r(1); // 1 Hz

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Create a marker message
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "markers_time";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  ROS_INFO("markers_time: active");

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
  ros::Duration(5.0).sleep();

  // Publish the marker initially at the pickup zone
  marker.pose.position.x = -1.0;
  marker.pose.position.y = -7.2;
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ROS_INFO("Marker published at the pickup zone");

  // Wait for 5 seconds
  ros::Duration(5.0).sleep();

  // Hide the marker
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
  ROS_INFO("Marker hidden");

  // Wait for 5 seconds
  ros::Duration(5.0).sleep();

  // Publish the marker at the drop-off zone
  marker.pose.position.x = 6.5;
  marker.pose.position.y = -7.2;
  marker.action = visualization_msgs::Marker::ADD;
  marker_pub.publish(marker);
  ROS_INFO("Marker published at the drop-off zone");

  // Wait for 5 seconds
  ros::Duration(5.0).sleep();

  ros::spin();

  return 0;
}
