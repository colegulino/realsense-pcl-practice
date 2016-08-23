// 
// Utilities for realsense usage
// 

#include <realsense_utils/realsense_utils.h>

#include <GLFW/glfw3.h>

namespace rs_utils
{

// Inspiration and code from: http://www.joshmcculloch.nz/#!realsense
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const std::vector<uint16_t>& depth_image,
													   const std::vector<uint8_t>& color_image,
													   const rs::extrinsics& depth_to_color,
													   const rs::intrinsics& depth_intrin,
													   const rs::intrinsics& color_intrin,
													   const double scale)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	const auto d_height = depth_intrin.height;
	const auto d_width = depth_intrin.width;
	const auto c_height = color_intrin.height;
	const auto c_width = color_intrin.width;

	for(int dy = 0; dy < d_height; ++dy)
	{
		for(int dx = 0; dx < d_width; ++dx)
		{
			// Get the depoth value of the current pixel in meters
			// Skip those with no depth
			const auto depth_val = depth_image[dy * d_width + dx];
			const auto depth_m = depth_val * scale;
			if(depth_m == 0) continue;

			// Map pixel coordinates in depth image with pixel coordinates in the color image
			rs::float2 depth_pixel = {static_cast<float>(dx), static_cast<float>(dy)};
			rs::float3 depth_point = depth_intrin.deproject(depth_pixel, depth_m);
			rs::float3 color_point = depth_to_color.transform(depth_point);
			rs::float2 color_pixel = color_intrin.project(color_point);

			// Create a point at location of the depth point
			pcl::PointXYZRGB point;
			point.x = depth_point.x;
			point.y = depth_point.y;
			point.z = depth_point.z;

			// Make the color at the new point to be the color of the closest color
			// point or white if it is out of bounds of the color image
			const auto cx = static_cast<int>(std::round(color_pixel.x));
			const auto cy = static_cast<int>(std::round(color_pixel.y));
			if(cx < 0 or cx > c_width or cy < 0 or cy > c_height) // Out of bounds
			{
				point.r = 255;
				point.g = 255;
				point.b = 255;
			}
			else
			{
				const auto c_it = std::begin(color_image) + 3 * (cy * c_width + cx);
				const auto c_idx = std::distance(color_image.begin(), c_it);

				point.r = color_image[c_idx];
				point.g = color_image[c_idx + 1];
				point.b = color_image[c_idx + 2];
			}

			point_cloud->push_back(point);
		}
	}

	return point_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const realsense_device& dev)
{
	return get_point_cloud(dev.depth_image(),
						   dev.color_image(),
						   dev.depth_to_color(),
						   dev.depth_intrin(),
						   dev.color_intrin(),
						   dev.scale());
}

}