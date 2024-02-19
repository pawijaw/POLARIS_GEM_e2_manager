cmake_minimum_required(VERSION 3.0.2)

set(NAME battery_mock)
project(${NAME})

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
   FILES
   BatteryMsg.msg
 )
 
set(PROJECT_SOURCES
        include/BatteryMockStrings.h
        src/BatteryModel.h
        src/BatteryModel.cpp
        src/battery_main.cpp
)

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
  CATKIN_DEPENDS sensor_mock_lib
#  DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${NAME} ${PROJECT_SOURCES})
target_link_libraries(${NAME} ${catkin_LIBRARIES} sensor_mock_lib)
add_dependencies(${NAME} ${NAME}_generate_messages_cpp)


