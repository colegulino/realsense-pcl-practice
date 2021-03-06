// 
// Utilities for realsense usage
// 
#pragma once

// Personal libs
#include <realsense_device/realsense_device.h>

// Librealsense
#include <librealsense/rs.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace rs_utils
{

// 
// Generates a PCL point cloud from the depth and color image of a realsense camera
// 
// @param depth_image Depth image from the realsense camera
// @param color_image Color image from the realsense camera
// @param depth_to_color Color extrinsics that convert from depth image to color image
// @param depth_intrin Depth intrinsics of the realsense camera
// @param color_intrin Color intrinsics of the realsense camera
// @param scale Scale of the depth image that can convert it to meters
// @return Point cloud based on the realsense camera information
// 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const std::vector<uint16_t>& depth_image,
													   const std::vector<uint8_t>& color_image,
													   const rs::extrinsics& depth_to_color,
													   const rs::intrinsics& depth_intrin,
													   const rs::intrinsics& color_intrin,
													   const double scale);

// 
// Generates a PCL point cloud from the depth and color image of a realsense camera
// 
// @param dev Realsense device wrapper
// @return Point cloud based on the realsense camera information
// 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const realsense_device& dev);

}
