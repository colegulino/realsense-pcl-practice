// 
// Wrapper for a realsense device
// 
#pragma once

// Librealsense
#include <librealsense/rs.hpp>

// Standard library functions
#include <memory>
#include <vector>

class realsense_device : public rs::context
{
private:
	rs::context ctx_;

	rs::device* dev_;

	rs::intrinsics depth_intrin_;

	rs::intrinsics color_intrin_;

	rs::extrinsics depth_to_color_;

	double scale_;

public:
	// 
	// General constructor
	// 
	realsense_device();

	// 
	// Function that determines if there is a realsense camera connected
	// 
	// @return Boolean that is true if a realsense device is connected
	// 
	bool is_connected() const;

	// 
	// Returns the name of the device connected
	// 
	// @return A string that represents the name of the realsense device connected
	// @exception Throws a runtime_error if there is no device connected
	// 
	std::string name() const;

	// 
	// Returns the serial number of the device connected
	// 
	// @return A string that represents the serial number of the realsense device connected
	// @exception Throws a runtime_error if there is no device connected
	// 
	std::string serial() const;

	// 
	// Returns the depth intrinsics of the realsense camera
	// The depth intrinsics can be used to project from a 2D to 3D point and get the 
	// width and height of the depth image.
	//
	// @return Depth intrinsics of the realsense camera
	// @exception Throws a runtime_error if there is no device connected
	// 
	rs::intrinsics depth_intrin() const;

	// 
	// Returns the color intrinsics of the realsense camera
	// The color intrinsics can be used to project from a 2D to 3D point and get the 
	// width and height of the color image.
	//
	// @return Color intrinsics of the realsense camera
	// @exception Throws a runtime_error if there is no device connected
	// 
	rs::intrinsics color_intrin() const;

	// 
	// Returns the scale of the realsense camera used to convert a depth point to meters
	// 
	// @return Scale of the realsense camera 
	// @exception Throws a runtime_error if there is no device connected
	// 
	double scale() const;

	// 
	// Prints the camera info to the console
	// 
	void print_info() const;

	// 
	// Generates and returns the depth image from the realsense camera with type uint16
	// 
	// @return Depth image from the realsense camera
	// @exception Throws a runtime_error if there is no device connected
	// 
	std::vector<uint16_t> depth_image() const;

	// 
	// Generates and returns the color image from the realsense camera with type uint8
	// 
	// @return Color image from the realsense camera
	// @exception Throws a runtime_error if there is no device connected
	// 
	std::vector<uint8_t> color_image() const;

	// 
	// Returns the extrinsics of the realsense camera
	// The extrinsics can be used to convert from a depth image to a color image
	//
	// @return Extrinsics of the realsense camera
	// @exception Throws a runtime_error if there is no device connected
	// 
	rs::extrinsics depth_to_color() const;

	// 
	// Logs the debug messages for the realsense camera
	// 
	// @param log_file A string representing the log file name
	// 
	void log_to_file(const std::string& log_file = "/logs/librealsense.log") const;

	// 
	// Hold until frames have been gotten from the device
	// 
	void wait_for_frames() const;
};










