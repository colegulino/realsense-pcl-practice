// Personal Libraries
#include <realsense_utils/realsense_utils.h>
#include <realsense_device/realsense_device.h>
#include <render_utils/render_utils.h>

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

		// Wait for frames and get the point cloud
		dev.wait_for_frames();
		auto point_cloud = rs_utils::get_point_cloud(dev);

		// Visualize the point cloud
		auto viewer = render_utils::visualize_rgb_pc(point_cloud);
		render_utils::loop_viewer(viewer);
	}
	else
	{
		std::cout << "There are no realsense device connected." << std::endl;
	}

    return 0;
}
