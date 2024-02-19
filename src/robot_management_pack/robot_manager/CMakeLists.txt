cmake_minimum_required(VERSION 3.0.2)

set(NAME robot_manager)
project(${NAME})


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5Widgets)
find_package(Qt5Charts)

get_target_property(QtCore_location Qt5::Core LOCATION)
message(STATUS "QT: " ${QtCore_location})
 
add_service_files(
   FILES
   SensorMockReq.srv
 )
 
set(PROJECT_SOURCES
        src/VehicleData.h
        src/RobotManagerDefs.h
        src/SensorDataManager.h
        src/SensorDataManager.cpp
        src/RobotManager.h
        src/RobotManager.cpp
        src/RobotStateMachine.h
        src/RobotStateMachine.cpp
        src/SensorThreshold.h
        src/MockControl.h
        src/MockControl.cpp
        src/UserInterface.cpp
        src/UserInterface.h
        src/UserInterface.ui
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

message(FATAL "CATKIN INC " ${catkin_INCLUDE_DIRS})

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(${NAME}_lib ${PROJECT_SOURCES} ${UI_FILES})
add_executable(${NAME} src/robot_manager_main.cpp)
target_link_libraries(${NAME}_lib ${catkin_LIBRARIES} Qt5::Widgets Qt5::Charts)
target_link_libraries(${NAME} ${NAME}_lib)
add_dependencies(${NAME}_lib robot_manager_generate_messages_cpp ${catkin_EXPORTED_TARGETS})



catkin_add_gtest(${NAME}_utest src/unit_tests.cpp)
target_link_libraries(${NAME}_utest ${NAME}_lib ${catkin_LIBRARIES})



