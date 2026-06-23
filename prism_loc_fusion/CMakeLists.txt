cmake_minimum_required(VERSION 3.16)
project(prism_loc_fusion VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)

add_library(prism_loc_fusion
  src/so3.cpp
  src/geodetic.cpp
  src/eskf.cpp)
target_include_directories(prism_loc_fusion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(prism_loc_fusion PUBLIC Eigen3::Eigen)
target_compile_features(prism_loc_fusion PUBLIC cxx_std_17)

option(PRISM_LOC_FUSION_BUILD_TESTS "Build prism_loc_fusion tests" OFF)
# Default BUILD_TESTING ON so the core unit tests are configured under a plain
# `colcon build` (which does not define BUILD_TESTING) and then run by `colcon test`.
if(NOT DEFINED BUILD_TESTING)
  set(BUILD_TESTING ON)
endif()
if(BUILD_TESTING OR PRISM_LOC_FUSION_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  set(PRISM_LOC_FUSION_TESTS
    test/test_smoke.cpp
    test/test_so3.cpp
    test/test_geodetic.cpp
    test/test_eskf.cpp
    test/test_eskf_integration.cpp)
  add_executable(prism_loc_fusion_tests ${PRISM_LOC_FUSION_TESTS})
  target_link_libraries(prism_loc_fusion_tests prism_loc_fusion GTest::gtest GTest::gtest_main)
  add_test(NAME prism_loc_fusion_tests COMMAND prism_loc_fusion_tests)
endif()

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS prism_loc_fusion EXPORT export_prism_loc_fusion
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin INCLUDES DESTINATION include)
  ament_export_targets(export_prism_loc_fusion HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)
  ament_package()
endif()
