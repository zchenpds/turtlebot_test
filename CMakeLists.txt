cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_test)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs std_msgs genmsg)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(turtlebot_test src/test.cpp)
target_link_libraries(turtlebot_test ${catkin_LIBRARIES})
# install(TARGETS test DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
