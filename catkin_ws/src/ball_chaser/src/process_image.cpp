#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>


enum class PixelLocation
{
	Left,
	Mid,
	Right
};

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
//ROS_INFO("process_image.cpp: drive_robot(): lin_x:%1.2f, ang_z:%1.2f", (float)lin_x, (float)ang_z);
	
	// DONE: Request a service and pass the velocities to it to drive the robot
	//client.call(srv);    // request a service 
	ball_chaser::DriveToTarget driveToTargetSrv; 
	driveToTargetSrv.request.linear_x = lin_x;
	driveToTargetSrv.request.angular_z = ang_z;
	client.call(driveToTargetSrv);

}

PixelLocation getLocation(int column, int maxColumn)
{
	

	if(column < (maxColumn / 3))
	{
		 ROS_INFO("process_image.cpp: getLocation(): column:%d, maxColumn: %d", (int)column,(int)maxColumn);	
		return PixelLocation::Left;
	}
	if (column > (2 * maxColumn / 3))
    	{
		return PixelLocation::Right;
	}
	return PixelLocation::Mid;
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
//	ROS_INFO("process_image.cpp::process_image_callback(): img.width:%d, img.height:%d", (int)img.height, (int)img.width);
    int white_pixel = 255;
	
    // DONE: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	int maxColLeft = img.width/4;
	int minColRight = 3 * img.width/4;
	int pixel;
	for (int row = 0; row < img.height; row+=5)
	{
		// search left section:
		for(int col = 0; col < maxColLeft; col+=5)
		{ 
			pixel = img.data[img.step * row + col];			
			if (pixel == white_pixel){
				ROS_INFO("process_image.cpp::process_image_callback(): Found->Left row:%d, col:%d", (int)row, (int)col);
				drive_robot(0.0, 0.1);
				return;
			}
		}
		// search right section: 
		for(int col = img.width - 1 ; col > minColRight; col-=5)
		{ 
			pixel = img.data[img.step * row + col];			
			if (pixel == white_pixel){
				ROS_INFO("process_image.cpp::process_image_callback(): found->Right row:%d, col:%d", (int)row, (int)col);
				// turn right:
				drive_robot(0.0, -0.1);
				return;

			
			}
		}
		// search mid section:
		for(int col = maxColLeft ; col < minColRight; col+=5)
		{ 
			pixel = img.data[img.step * row + col];			
			if (pixel == white_pixel){
				ROS_INFO("process_image.cpp::process_image_callback(): found->Midt row:%d, col:%d", (int)row, (int)col);				
				// drive forward:
				drive_robot(0.1, 0.0);
				return;
			
			}
		}		
		
	}
	// pixel not found --> stop
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
