cmake_minimum_required(VERSION 3.10)
project(fast_lio_sam)

### set compiler
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fexceptions -g -ggdb") #c++17, for parallel programming and for Eigen (see https://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra ${CMAKE_CXX_FLAGS}")

### get packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rosbag
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  tf
  tf_conversions
  pcl_ros
  pcl_conversions
  message_filters
  cv_bridge
  image_transport
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GeographicLib REQUIRED)
if(PCL_FOUND)
  message(WARNING "PCL_VER: ${PCL_VERSION}, and PCL_DIR: ${PCL_INCLUDE_DIRS}")
endif()
if(Eigen3_FOUND)
  message(WARNING "EIGEN_VER: ${EIGEN3_VERSION_STRING}, and Eigen DIR: ${EIGEN3_INCLUDE_DIR}")
endif()
if(GTSAM_FOUND)
  message(WARNING "GTSAM_VER: ${GTSAM_VERSION}, and GTSAM DIR: ${GTSAM_INCLUDE_DIR}")
endif()
if(GeographicLib_FOUND)
  message(WARNING "GeographicLib_VER: ${GeographicLib_VERSION}, and GeographicLib DIR: ${GeographicLib_INCLUDE_DIR}")
endif()

catkin_package() #this automatically adds/installs the results of build

### get packages' headers
include_directories(
  include
	${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
)

###########
## Build ##
###########
### main
set(COMMON_LIBS ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${EIGEN3_LIBS} gtsam)
add_library(${PROJECT_NAME}_modules src/loop_closure.cpp src/fast_lio_sam.cpp)
target_link_libraries(${PROJECT_NAME}_modules ${COMMON_LIBS} ${GeographicLib_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${COMMON_LIBS} ${PROJECT_NAME}_modules)

add_executable(${PROJECT_NAME}_sensor_recorder src/sensor_recorder.cpp)
target_link_libraries(${PROJECT_NAME}_sensor_recorder ${COMMON_LIBS} ${PROJECT_NAME}_modules)