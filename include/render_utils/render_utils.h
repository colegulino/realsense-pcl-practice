// 
// Utilities for rendering point cloud data
// 
#pragma once

// Rendering library
#include <GLFW/glfw3.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace render_utils
{

// Globals
double yaw, pitch, lastX, lastY;
int ml;

class point_cloud_window
{
private:
	// Point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_;

	// // Camera and mouse states
	// double yaw_;
	// double pitch_;
	// double lastX_;
	// double lastY_;
	// int ml_;

	// GLFW window
	GLFWwindow* win_;

public:
	// Constructor
	point_cloud_window(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& point_cloud);


	// Rendering loop
	void render();
};

// Event handlers
static void on_mouse_button(GLFWwindow* win, int button, int action, int mods);
static double clamp(double val, double lo, double hi);
static void on_cursor_pos(GLFWwindow* win, double x, double y);

}