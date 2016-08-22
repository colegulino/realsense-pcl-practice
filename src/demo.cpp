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
	rs::context ctx;
	std::cout << "There are " << ctx.get_device_count() << " connected RealSense devices."
			  << std::endl;
    return 0;
}
