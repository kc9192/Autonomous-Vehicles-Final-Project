cmake_minimum_required(VERSION 3.0.2)
project(group5_sim_launch)


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


catkin_package(
  DEPENDS OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)



 add_executable(tld_project
  
  src/Tldp.cpp
  src/tld_node.cpp
 )


add_dependencies(tld_project
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(tld_project
  ${catkin_LIBRARIES}
)
