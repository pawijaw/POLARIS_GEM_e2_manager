cmake_minimum_required(VERSION 3.0.2)

set(NAME v2x_mock)
project(${NAME})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   V2xMsg.msg
 )
 
set(PROJECT_SOURCES
        include/V2xMockStrings.h
        src/V2xModel.h
        src/V2xModel.cpp
        src/v2x_main.cpp
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS sensor_mock_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${NAME} ${PROJECT_SOURCES})
target_link_libraries(${NAME} ${catkin_LIBRARIES} sensor_mock_lib)
add_dependencies(${NAME} ${NAME}_generate_messages_cpp)


