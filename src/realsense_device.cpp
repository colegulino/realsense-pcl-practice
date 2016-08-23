// 
// Wrapper for a realsense device
// 
#include <realsense_device/realsense_device.h>

// Standard library functions
#include <stdexcept>
#include <iostream>
#include <iterator>

realsense_device::realsense_device()
{
	dev_ = ctx_.get_device(0);

	dev_->enable_stream(rs::stream::depth, rs::preset::best_quality);
	dev_->enable_stream(rs::stream::color, rs::preset::best_quality);
	dev_->start();

	dev_->get_extrinsics(rs::stream::depth, rs::stream::color);

	depth_intrin_ = dev_->get_stream_intrinsics(rs::stream::depth);

	color_intrin_ = dev_->get_stream_intrinsics(rs::stream::color);

	scale_ = dev_->get_depth_scale(); 
}

bool realsense_device::is_connected() const
{
	return ctx_.get_device_count() > 0;	
}

std::string realsense_device::name() const
{
	if(is_connected())
	{
		return dev_->get_name();
	}
	else
	{
		throw std::runtime_error("Cannot get device name. No device connected.");
	}
}

std::string realsense_device::serial() const
{
	if(is_connected())
	{
		return dev_->get_serial();
	}
	else
	{
		throw std::runtime_error("Cannot get serial number. No device connected.");
	}
}

rs::intrinsics realsense_device::depth_intrin() const 
{
	return depth_intrin_;
}

rs::intrinsics realsense_device::color_intrin() const
{
	return color_intrin_;
}

double realsense_device::scale() const
{
	return scale_;
}

void realsense_device::print_info() const
{
	std::cout << "========================================" << std::endl;
	std::cout << "Device Name: " << name() << std::endl;
	std::cout << "Serial Number: " << serial() << std::endl;
	std::cout << "Scale: " << scale() << std::endl;
	std::cout << "========================================" << std::endl;
}

std::vector<uint16_t> realsense_device::depth_image() const
{
	const uint16_t* d_image = 
		static_cast<const uint16_t*>(dev_->get_frame_data(rs::stream::depth));

	return std::vector<uint16_t>(d_image, d_image + (sizeof(d_image) / sizeof(d_image[0])));
}

std::vector<uint8_t> realsense_device::color_image() const
{
	const uint8_t* c_image =
		static_cast<const uint8_t*>(dev_->get_frame_data(rs::stream::color));

	return std::vector<uint8_t>(c_image, c_image + (sizeof(c_image) / sizeof(c_image[0])));
}
rs::extrinsics realsense_device::depth_to_color() const
{
	return depth_to_color_;
}


