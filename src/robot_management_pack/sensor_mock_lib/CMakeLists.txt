cmake_minimum_required(VERSION 3.0.2)

set(NAME sensor_mock_lib)
project(${NAME})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_service_files(
   FILES
   SensorMockSrv.srv
)
 
set(PROJECT_SOURCES
    include/DataProfile.h
    src/DataProfile.cpp
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

add_library(${NAME} ${PROJECT_SOURCES})
target_link_libraries(${NAME} ${catkin_LIBRARIES})
add_dependencies(${NAME} ${NAME}_generate_messages_cpp)


