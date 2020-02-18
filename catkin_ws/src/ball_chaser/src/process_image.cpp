#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget service;
    service.request.linear_x = lin_x;
    service.request.angular_z = ang_z;

    if(client.call(service) == false) {
        ROS_ERROR("Error: process_image was not able to call DriveToTarget service.");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    // minimum number of white pixels to make a match 
    static const int min_pixels = 1;

    // threshold value for while pixel
    static const int white_pixel = 255;

    // number of bins cols to sort in pixels -- needs to be odd!
    static int number_bins = 7;

    // ensure odd bins 
    if((number_bins % 2) == 0)
        number_bins += 1;

    // angular velocity per bin (the farer the bin, the faster we turn)
    static const float angular_vel_per_bin = 0.1;

    // vector for bins to count number of white pixels in
    std::vector<int> bin(number_bins, 0);

    // number of coloumns per bin to insert pixel 
    int col_per_bin = img.step / number_bins;

    // loop through all pixels and sort them into the col vector
    for(int px = 0; px < img.height * img.step; ++px) {

        // only insert pixels above threshold into the according bin
        if(img.data[px] >= white_pixel) {

            // increment the corresponding bin
            auto bin_index = (px % img.step) / col_per_bin;

            if(bin_index < bin.size())
                bin.at(bin_index)++;
        }
    }

    // get max bin
    auto max_bin = std::max_element(bin.begin(), bin.end());

    // only on active bin, drive robot, else stop 
    if((max_bin == bin.end()) || (*max_bin <= min_pixels)) {
        drive_robot(0.0F, 0.0F);
    } else {
        // keep the linear speed low and the angular speed based on the bin
        int factor = std::distance(bin.begin(), max_bin) - (number_bins / 2);
        ROS_INFO("Selected bin %ld factor %d", std::distance(bin.begin(), max_bin), factor);
        drive_robot(0.1F, -factor * angular_vel_per_bin);
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
