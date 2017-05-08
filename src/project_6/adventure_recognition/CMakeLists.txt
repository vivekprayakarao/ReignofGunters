cmake_minimum_required(VERSION 2.8.3)
project(adventure_recognition)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  rospy
  roscpp
)

find_package(OpenCV REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})

add_executable(objrec src/object_recognition.cpp)
add_executable(objectrecog src/ObjectRecognition.cpp)

target_link_libraries(objrec ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(objectrecog ${catkin_LIBRARIES} ${OpenCV_LIBS})
