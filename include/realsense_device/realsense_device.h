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
	realsense_device();

	bool is_connected() const;

	std::string name() const;

	std::string serial() const;

	rs::intrinsics depth_intrin() const;

	rs::intrinsics color_intrin() const;

	double scale() const;

	void print_info() const;

	std::vector<uint16_t> depth_image() const;

	std::vector<uint8_t> color_image() const;

	rs::extrinsics depth_to_color() const;
};