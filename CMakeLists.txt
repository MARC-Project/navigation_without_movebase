cmake_minimum_required(VERSION 2.8.3)
project(navigation_without_movebase)

## Compile as C++11, supported in ROS Kinetic and newer

set(OpenCV_DIR /usr/local/share/OpenCV/)
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

find_package(catkin REQUIRED COMPONENTS
  OpenCV REQUIRED
  cv_bridge
  image_transport
  roscpp
  rospy
  serial
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
sensor_msgs tf move_base_msgs
)


add_message_files(FILES suduxifen.msg square_and_position.msg cmdvel_control.msg )

generate_messages(DEPENDENCIES actionlib_msgs std_msgs )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES learning_communication
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
    "/usr/include/eigen3/"
)


## Template for adding a node:
#add_executable(NODE_NAME
#  src/SRC_FILE_NAME
#  src/HEAD_FILE_NAME
#)
#add_dependencies(NODE_NAME ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(NODE_NAME
#  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
#)



add_executable( simple_serial src/my_serial_node.cpp)
target_link_libraries( simple_serial
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable(bezier_node src/bezier_path.cpp)
add_dependencies(bezier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bezier_node
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

add_executable( suduxifen_cmdvel src/suduxifen_cmdvel.cpp)
add_dependencies(suduxifen_cmdvel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( suduxifen_cmdvel
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

