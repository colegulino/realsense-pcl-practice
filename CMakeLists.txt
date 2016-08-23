# CMake files for demo.cpp

cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Set the project info
project(Demo)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
add_executable(demo ${PROJECT_SOURCE_DIR}/src/demo.cpp)
include_directories(${PROJECT_SOURCE_DIR}/include)

# Find PCL
find_package(PCL 1.8 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find realsense
link_directories(${Project_SOURCE_DIR}/lib/librealsense/lib)
find_library(REALSENSE realsense ${PROJECT_SOURCE_DIR}/lib/librealsense/lib)

# Add library for realsense device wrapper
add_library(realsense_device
			SHARED
			 src/realsense_device.cpp
			)
target_link_libraries(realsense_device realsense)

# Add library for realsense_utils
add_library(realsense_utils
			SHARED
			 src/realsense_utils.cpp
			)
target_link_libraries(realsense_utils realsense realsense_device)


target_link_libraries(demo 
					  ${PCL_COMMON_LIBRARIES} 
					  ${PCL_IO_LIBRARIES} 
					  realsense 
					  realsense_utils
					  realsense_device
					  )