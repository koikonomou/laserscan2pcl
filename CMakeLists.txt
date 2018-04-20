cmake_minimum_required(VERSION 2.8.3)
project(laserscan2pcl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_filters
  laser_geometry
  my_new_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros
)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})




catkin_package()


###########
## Build ##
###########


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(pointcloud src/pointcloud.cpp)
target_link_libraries(pointcloud ${catkin_LIBRARIES})

add_executable(pointn src/pointn.cpp)
target_link_libraries(pointn ${catkin_LIBRARIES})


add_executable(points src/points.cpp)
target_link_libraries(points ${catkin_LIBRARIES})

add_executable(laserscan_topcl src/laserscan_topcl.cpp)
target_link_libraries(laserscan_topcl ${catkin_LIBRARIES})

add_executable(pointcloud_layers src/pointcloud_layers.cpp)
target_link_libraries(pointcloud_layers ${catkin_LIBRARIES})


