cmake_minimum_required(VERSION 3.0.2)
project(lab5)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lab5
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)




add_executable(lab5_slam src/lab5_slam.cpp)

target_link_libraries(lab5_slam ${catkin_LIBRARIES})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)





