cmake_minimum_required(VERSION 3.0.2)

set(NAME estop_mock)
project(${NAME})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   EstopMsg.msg
 )
 
set(PROJECT_SOURCES
        include/EstopMockStrings.h
        src/EstopModel.h
        src/EstopModel.cpp
        src/estop_main.cpp
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


