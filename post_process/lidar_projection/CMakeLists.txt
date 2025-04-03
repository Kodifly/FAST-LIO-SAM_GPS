cmake_minimum_required(VERSION 3.10)
project(LidarProjection)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "OpenCV found: ${OpenCV_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "OpenCV not found!")
endif()

# Find PCL
find_package(PCL 1.10 REQUIRED COMPONENTS common io)
if (PCL_FOUND)
    message(STATUS "PCL found: ${PCL_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "PCL not found!")
endif()

# Include directories
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

# Add executable
add_executable(lidar_projection src/lidar_projection.cpp)

# Link libraries
target_link_libraries(lidar_projection ${OpenCV_LIBS} ${PCL_LIBRARIES})