#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

enum class PixelLocation
{
	Left,
	Mid,
	Right
};

const int SIDES_SIZE_PERCENTAGE = 25; //percentage of the image width used for left and right sections.
const float YAW_SPEED = 0.15; //angular_x speed
const float FWD_SPEED = 0.1; //linear_x speed

const int RGB8_DATA_SIZE = 3; // number of bytes of data per pixel on data array.
const int RGB8_R = 0;	// Offset of R data byte
const int RGB8_G = 1; 	// Offset of G data byte
const int RGB8_B = 2; 	// Offset of B data byte

struct RGB8_Pixel{
	uint8_t r;
	uint8_t g;
	uint8_t b;
};


bool isPixelWhite(sensor_msgs::Image img, int row, int col)
{
	return img.data[(row * img.step + col * RGB8_DATA_SIZE) + 0] == 255 && 
	       img.data[(row * img.step + col * RGB8_DATA_SIZE) + 1] == 255 &&
		   img.data[(row * img.step + col * RGB8_DATA_SIZE) + 2] == 255;
}

bool isPixelWhite(uint8_t r, uint8_t g, uint8_t b)
{
	return (r == 255) && 
		   (g == 255) && 
		   (b == 255);
}

/// Determine the location of the pixel based on given pixel column position and the 
/// limits of the areas left and right (determined at execution, function of image size)
PixelLocation identifyPixelLocation(int pixelColumn, int maxColLeft, int minColRight)
{
ROS_INFO("process_image.cpp::identifyPixelLocation(): pixelColumn: %d; maxColLeft:%d; minColRight: %d", (int)pixelColumn, (int) maxColLeft, (int) minColRight);
	if (pixelColumn < maxColLeft)
	{
		return PixelLocation::Left;
	}
	if (pixelColumn > minColRight)
	{
		return PixelLocation::Right;
	}
	return PixelLocation::Mid;
}


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

// This callback function continuously executes and reads the image data
// New approach: to prioritize steering over moving, the pixel search loop checks first sides,
//               then the middle part.
void process_image_callback(const sensor_msgs::Image img)
{
    //int white_pixel = 255; //only valid on MONO8
	int maxColLeft = (img.width * SIDES_SIZE_PERCENTAGE) / 100;
	int minColRight = (img.width * (100-SIDES_SIZE_PERCENTAGE))/100;

	
	//ROS_INFO("process_image.cpp::process_image_callback(): image info: height: %d; width: %d; step: %d; encoding: %s", img.height, img.width, img.step, img.encoding.c_str());

	if (img.encoding != sensor_msgs::image_encodings::RGB8){
		ROS_INFO("process_image.cpp::process_image_callback(): ERROR: Unexpected image encoding: %s found 'RGB8' expected", img.encoding.c_str());
		return;
	}
    // DONE: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
	int pixel;
	for (int row = 0; row < img.height; row++)
	{
		for(int col = 0; col < img.width; col++)
		{ 
			//pixel = img.data[img.step * row + col*RGB8_DATA_SIZE];			
			if (isPixelWhite(img.data[(row * img.step + col * RGB8_DATA_SIZE) + 0], 
			                 img.data[(row * img.step + col * RGB8_DATA_SIZE) + 1],
							 img.data[(row * img.step + col * RGB8_DATA_SIZE) + 2]))
			{
				ROS_INFO("process_image.cpp::process_image_callback(): Pixel Found! row:%d, col:%d", (int)row, (int)col);
				PixelLocation pixelLocation = identifyPixelLocation(col, maxColLeft, minColRight);
				
				switch (pixelLocation)
				{
					case PixelLocation::Left:
						drive_robot(0.0, YAW_SPEED);
						return;
					case PixelLocation::Right:
						drive_robot(0.0, (-1.0 * YAW_SPEED) );
						return;
					case PixelLocation::Mid:
						drive_robot(FWD_SPEED, 0.0);
						return;
				}
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
