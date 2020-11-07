cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Aravis REQUIRED)
find_package(GLIB2 REQUIRED)

#include_directories(${catkin_INCLUDE_DIRS} ${ARAVIS_INCLUDE_PATH} ${GLIB2_INCLUDE_DIRS})
include_directories(${ARAVIS_INCLUDE_PATH} ${GLIB2_INCLUDE_DIRS})

add_executable(camnode src/camnode.cpp)
#add_executable(camnode src/camnode2_glib.cpp)
ament_target_dependencies(camnode rclcpp std_msgs sensor_msgs)

install(TARGETS
  camnode
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(camnode ${ARAVIS_LIBRARY} glib-2.0 gmodule-2.0 gobject-2.0)
ament_package()
