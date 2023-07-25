#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z; 
    // call the command_robot service and pass the requested linear_x, angular_z
    if (!client.call(srv))
        ROS_ERROR("failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void  process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // width * 3 = step --> move from one row to the next 

    for(int i=0; i<img.height * img.step; i +=3)
    {
        if(img.data[i] == white_pixel && img.data[i+1] == white_pixel && img.data[i+2] == white_pixel)
        {
            int column = (i % img.step)/3; 

            if (column < img.width/3 )
                drive_robot(0.06, 0.06); // left
            else if (column < 2* img.width/3 )
                drive_robot(0.06, 0.0); // forward
            else 
                drive_robot(0.06, -0.06); // right
            
            return;
            
        }
        
    }
    drive_robot(0.0, 0.0);

}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback); 


    // Handle ROS communication events
    ros::spin();

    return 0;


}