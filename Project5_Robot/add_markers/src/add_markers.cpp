#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <cmath>

double pickupX = -1.0;
double pickupY = -7.2;
double dropX = 6.5;
double dropY = -7.2;

float x_cor = 0.0;
float y_cor = 0.0;

bool isPickedUp = false;

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    x_cor = msg->pose.pose.position.x;
    y_cor = msg->pose.pose.position.y;

    // Check if robot is close to pickup or drop-off zones
    double distance_to_pickup = std::hypot(pickupX - x_cor, pickupY - y_cor);
    double distance_to_dropoff = std::hypot(dropX - x_cor, dropY - y_cor);

    if (!isPickedUp && distance_to_pickup < 0.5) {
        ROS_INFO("Robot is close to the pickup zone.");
        isPickedUp = true;
    }

    if (isPickedUp && distance_to_dropoff < 0.5) {
        ROS_INFO("Robot is close to the drop-off zone.");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "add_markers");
    ros::NodeHandle nh;
    ros::Rate r(1); // 1 Hz

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber odomSubscriber = nh.subscribe("/odom", 1000, odometryCallback);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;

    marker.pose.position.x = pickupX;
    marker.pose.position.y = pickupY;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration();

    ros::Duration(2.0).sleep();

    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);

    ros::Duration(1.0).sleep();

    while (ros::ok()) {
        if (isPickedUp) {
            marker.action = visualization_msgs::Marker::DELETE;
            marker_pub.publish(marker);
        } else {
            marker.action = visualization_msgs::Marker::ADD;
            marker_pub.publish(marker);
        }

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
