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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const std::vector<uint16_t>& depth_image,
													   const std::vector<uint8_t>& color_image,
													   const rs::extrinsics& depth_to_color,
													   const rs::intrinsics& depth_intrin,
													   const rs::intrinsics& color_intrin,
													   const double scale);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr get_point_cloud(const realsense_device& dev);

}
