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

    if (!client.call(srv)) {
      ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    ROS_INFO("Img received - height: %d, width: %d, step: %d", img.height, img.width, img.step);

    int ball_left = -1;
    int ball_right = -1;
    bool ball_found = false;
    for (int i = 0; i < img.height; i++) {
      for (int j = 0; j < img.width; j++) {
        if (img.data[(i * img.width + j)*3] == white_pixel && 
            img.data[(i * img.width + j)*3 + 1] == white_pixel && 
            img.data[(i * img.width + j)*3 + 2] == white_pixel) {
          ball_found = true;
          if (ball_left == -1) {
            ball_left = j;
          }
        } else if (ball_found && ball_right == -1 && 
            (img.data[(i * img.width + j) * 3] != white_pixel ||
            img.data[(i * img.width + j) * 3 + 1] != white_pixel ||
            img.data[(i * img.width + j) * 3 + 2] != white_pixel)) {
          ball_right = j;
        }
      }
      if (ball_found && ball_right == -1) {
        ball_right = img.width - 1;
      }
      if (ball_found) {
        break;
      }
    }

    if (ball_found) {
      int ball_center = (ball_left + ball_right) / 2;
      ROS_INFO("Ball found at %d (%d, %d)", ball_center, ball_left, ball_right);
      if (ball_center < (img.width / 3)) {
        ROS_INFO("Turning right");
        drive_robot(0.0, -0.15); // turn right
      } else if (ball_center < (2 * img.width / 3)) {
        ROS_INFO("Moving forward");
        drive_robot(-0.7, 0.0); // forward
      } else {
        ROS_INFO("Turning left");
        drive_robot(0.0, 0.15); // turn left
      }
    } else {
      // Rotate to find the ball
      ROS_INFO("Rotating");
      drive_robot(0.0, 0.2);
    }
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
