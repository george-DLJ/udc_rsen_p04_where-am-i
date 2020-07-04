//This server node provides a ball_chaser/command_robot service to drive the robot around by setting its linear x and angular z velocities. The service server publishes messages containing the velocities for the wheel joints. 
//contain a ROS publisher and service. But this time, instead of publishing messages to the arm joint angles, you have to publish messages to the wheels joint angles. 

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

#include "ball_chaser/DriveToTarget.h" //DONE: Include the ball_chaser "DriveToTarget" header file

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// This callback function executes whenever a drive_request service is requested
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req,
    ball_chaser::DriveToTarget::Response& res)
{
	ROS_INFO("DriveToTarget Request received - j1:%1.2f, j2:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = req.linear_x; //0.5;
    motor_command.angular.z = req.angular_z; // 0.0;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);
	
	// Return a response message
    res.msg_feedback = "Wheel velocities set - linear_x: " + std::to_string(motor_command.linear.x) + " , angular_z: " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // DONE: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
	ros::ServiceServer service = n.advertiseService("ball_chaser/command_robot", handle_drive_request);
	ROS_INFO("Ready to send ball_chaser commands");

	// DONE: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish the requested velocities instead of constant values
	
	
    // DONE: Handle ROS communication events
    ros::spin();

    return 0;
}


