cmake_minimum_required(VERSION 3.0.2)
project(magnetic_declination_converter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

find_package(GeographicLib REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES magnetic_declination_converter
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf2 tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

add_executable(orientation_converter_main
  src/main.cpp
  src/orientation_converter.cpp
)

target_link_libraries(orientation_converter_main
  ${catkin_LIBRARIES}
  GeographicLib::GeographicLib
)

install(TARGETS orientation_converter_main
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)