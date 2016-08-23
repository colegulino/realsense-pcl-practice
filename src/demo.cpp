// Personal Libraries
#include <realsense_utils/realsense_utils.h>
#include <realsense_device/realsense_device.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Librealsense
#include <librealsense/rs.hpp>

// Standard Libraries
#include <iostream>

int main(int argc, const char * argv[]) 
{
	// Create a RealSense context object
	realsense_device dev;

	if(dev.is_connected())
	{
		std::cout << "There is a device connected." << std::endl;
		dev.print_info();
	}
	else
	{
		std::cout << "There are no realsense device connected." << std::endl;
	}

    return 0;
}
