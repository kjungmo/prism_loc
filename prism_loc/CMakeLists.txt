cmake_minimum_required(VERSION 3.16)
project(prism_loc VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(prism_loc_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(deps
  prism_loc_core rclcpp rclcpp_components tf2 tf2_ros tf2_geometry_msgs
  sensor_msgs nav_msgs geometry_msgs std_msgs std_srvs nav2_msgs pcl_conversions)

add_library(prism_loc_node SHARED
  src/localization_node.cpp
  src/laser_adapter.cpp
  src/pointcloud_adapter.cpp)
target_include_directories(prism_loc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(prism_loc_node ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(prism_loc_node ${deps})

add_executable(prism_loc_node_main src/main.cpp)
target_link_libraries(prism_loc_node_main prism_loc_node)
ament_target_dependencies(prism_loc_node_main rclcpp)

install(TARGETS prism_loc_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS prism_loc_node_main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch params rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf_math test/test_tf_math.cpp)
  target_link_libraries(test_tf_math prism_loc_core::prism_loc_core)
  ament_add_gtest(test_laser_adapter test/test_laser_adapter.cpp)
  target_link_libraries(test_laser_adapter prism_loc_node)
  ament_target_dependencies(test_laser_adapter ${deps})
endif()

ament_package()
