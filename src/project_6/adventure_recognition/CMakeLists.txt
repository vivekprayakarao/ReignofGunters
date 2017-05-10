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
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)

#add_executable(objrec src/object_recognition.cpp)
#target_link_libraries(objrec ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_library(objrec
  src/ObjectRecognition.cpp
)

target_link_libraries(objrec ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(adventurerecog src/AdventureRecognition.cpp)
target_link_libraries(adventurerecog objrec ${catkin_LIBRARIES})

add_executable(simple_navigation_goals src/simple_navigation_goals.cpp)
target_link_libraries(simple_navigation_goals ${catkin_LIBRARIES})
