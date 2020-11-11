cmake_minimum_required(VERSION 2.8.3)
project(ros_bno055)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
)

add_message_files(
  FILES
  OrientationEuler.msg
  Gravity.msg
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/bno055_driver.cpp
)

add_executable(bno055_node src/bno055_node.cpp src/bno055_driver.cpp)

target_link_libraries(bno055_node
  ${catkin_LIBRARIES}
)

add_dependencies(bno055_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
