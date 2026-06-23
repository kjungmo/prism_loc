cmake_minimum_required(VERSION 3.16)
project(prism_loc_fusion_ros VERSION 0.1.0 LANGUAGES CXX)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(prism_loc_fusion REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(deps prism_loc_fusion rclcpp tf2 tf2_ros sensor_msgs nav_msgs geometry_msgs pcl_conversions)

add_library(fusion_node SHARED
  src/ndt_registration.cpp
  src/fusion_localization_node.cpp)
target_include_directories(fusion_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_link_libraries(fusion_node ${PCL_LIBRARIES} Eigen3::Eigen)
ament_target_dependencies(fusion_node ${deps})

add_executable(fusion_node_main src/fusion_main.cpp)
target_link_libraries(fusion_node_main fusion_node)
ament_target_dependencies(fusion_node_main rclcpp)

install(TARGETS fusion_node ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(TARGETS fusion_node_main DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf_compose test/test_tf_compose.cpp)
  target_include_directories(test_tf_compose PRIVATE include)
  target_link_libraries(test_tf_compose Eigen3::Eigen)
  ament_add_gtest(test_ndt_registration test/test_ndt_registration.cpp)
  target_link_libraries(test_ndt_registration fusion_node)
  ament_target_dependencies(test_ndt_registration ${deps})
endif()

ament_package()
