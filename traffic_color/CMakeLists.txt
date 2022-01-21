cmake_minimum_required(VERSION 3.0.2)
project(traffic_color)


### Use C++ 17 compiler
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  dynamic_reconfigure
  image_transport
  image_geometry
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  dataspeed_ulc_msgs
)
find_package(OpenCV REQUIRED)


generate_dynamic_reconfigure_options(
  cfg/traffic_color.cfg
)


catkin_package(
  DEPENDS OpenCV
)



include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



 add_executable(tcd_project
  src/Tcd.cpp
  src/tcd_node.cpp
 )


add_dependencies(tcd_project
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(tcd_project
  ${catkin_LIBRARIES}
)
