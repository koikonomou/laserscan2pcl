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
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser2pcl
#  CATKIN_DEPENDS message_filters laser_geometry roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#add_executable(laser2pc src/laser2pc.cpp)
#target_link_libraries(laser2pc ${catkin_LIBRARIES})


#add_executable(pclmap src/pclmap.cpp)
#target_link_libraries(pclmap ${catkin_LIBRARIES})


add_executable(pointcloud src/pointcloud.cpp)
target_link_libraries(pointcloud ${catkin_LIBRARIES})
#add_dependencies(pointcloud laser2pcl_generate_messages_cpp)

add_executable(pointn src/pointn.cpp)
target_link_libraries(pointn ${catkin_LIBRARIES})

#add_executable(pointn src/pointn.cpp)
#target_link_libraries(pointn ${catkin_LIBRARIES})

#add_executable(pointxyz src/pointxyz.cpp)
#target_link_libraries(pointxyz ${catkin_LIBRARIES})



