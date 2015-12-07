cmake_minimum_required(VERSION 2.8.3)
project(image_listener)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  tf
)


catkin_package(
  INCLUDE_DIRS #include
#  LIBRARIES image_listener
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_definitions(-std=c++11)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(image_listener_node src/image_listener.cpp)

add_dependencies(image_listener_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(image_listener_node
   ${catkin_LIBRARIES}
 )
