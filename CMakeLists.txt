cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  tf
)

add_service_files(
  FILES
  SetOutputString.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES beginner_tutorials
 CATKIN_DEPENDS roscpp std_msgs message_runtime tf
 DEPENDS
)

catkin_add_gtest(talker_test test/talker_tests.cpp test/talker_test_class.cpp src/talker_class.cpp)
target_link_libraries(talker_test ${catkin_LIBRARIES})

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/include
)

add_executable(talker src/talker.cpp src/talker_class.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp src/listener_class.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})