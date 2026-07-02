cmake_minimum_required(VERSION 3.16)
project(prism_loc_core VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(Eigen3 REQUIRED)

# Library — sources are added as tasks land. Start with types/occupancy/ndt/motion/measurement/pf.
add_library(prism_loc_core
  src/types.cpp
  src/occupancy_grid.cpp
  src/ndt_map.cpp
  src/motion_model.cpp
  src/measurement_model.cpp
  src/particle_filter.cpp
  src/bbs.cpp
)
target_include_directories(prism_loc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(prism_loc_core PUBLIC Eigen3::Eigen)
target_compile_features(prism_loc_core PUBLIC cxx_std_17)
# The ROS node packages link this static archive into their shared node
# libraries, which requires position-independent code.
set_target_properties(prism_loc_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

option(PRISM_LOC_CORE_BUILD_TESTS "Build prism_loc_core tests" OFF)

# Dual-mode test wiring: ament when available, plain CTest+system gtest otherwise.
if(DEFINED ENV{AMENT_PREFIX_PATH} AND COMMAND ament_package)
  set(_use_ament ON)
endif()

if(BUILD_TESTING OR PRISM_LOC_CORE_BUILD_TESTS)
  enable_testing()
  find_package(GTest REQUIRED)
  set(PRISM_LOC_CORE_TESTS
    test/test_smoke.cpp
    test/test_types.cpp
    test/test_rng.cpp
    test/test_occupancy_grid.cpp
    test/test_ndt_map.cpp
    test/test_motion_model.cpp
    test/test_measurement_laser.cpp
    test/test_measurement_ndt.cpp
    test/test_particle_filter.cpp
    test/test_integration.cpp
    test/test_bbs.cpp)
  add_executable(prism_loc_core_tests ${PRISM_LOC_CORE_TESTS})
  target_link_libraries(prism_loc_core_tests prism_loc_core GTest::gtest GTest::gtest_main)
  add_test(NAME prism_loc_core_tests COMMAND prism_loc_core_tests)
endif()

# ament install/export only when built inside a ROS 2 workspace.
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  install(DIRECTORY include/ DESTINATION include)
  install(TARGETS prism_loc_core EXPORT export_prism_loc_core
    ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin INCLUDES DESTINATION include)
  ament_export_targets(export_prism_loc_core HAS_LIBRARY_TARGET)
  ament_export_dependencies(Eigen3)
  ament_package()
endif()
