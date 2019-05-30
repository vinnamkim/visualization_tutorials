cmake_minimum_required(VERSION 3.5)
project(interactive_marker_tutorials)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${interactive_markers_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(WARNING "dir='${dir}'")
endforeach()
message(WARNING ${Boost_INCLUDE_DIRS})

#add_executable(simple_marker src/simple_marker.cpp)
#target_link_libraries(simple_marker
#   ${catkin_LIBRARIES}
#)

add_executable(basic_controls src/basic_controls.cpp)
ament_target_dependencies(basic_controls
   interactive_markers
   rclcpp
   visualization_msgs
   tf2
   tf2_ros
)

#add_executable(selection src/selection.cpp)
#target_link_libraries(selection
#   ${catkin_LIBRARIES}
#)

#add_executable(pong src/pong.cpp)
#target_link_libraries(pong
#   ${catkin_LIBRARIES}
#)

#add_executable(cube src/cube.cpp)
#target_link_libraries(cube
#   ${catkin_LIBRARIES}
#)

#add_executable(menu src/menu.cpp)
#target_link_libraries(menu
#   ${catkin_LIBRARIES}
#)

#add_executable(point_cloud src/point_cloud.cpp)
#target_link_libraries(point_cloud
#   ${catkin_LIBRARIES}
#)
#############
## Install ##
#############

#install(PROGRAMS
#  scripts/basic_controls.py
#  scripts/cube.py
#  scripts/menu.py
#  scripts/simple_marker.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS
#  simple_marker
  basic_controls
#  selection
#  pong
#  cube
#  menu
#  point_cloud
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
