cmake_minimum_required(VERSION 3.16)
project(cbgl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(CBGL_EXACT_FP "Forbid floating point reassociation and contraction" OFF)
option(CBGL_DETERMINISTIC_FFT "Select FFTW plans by heuristic, not by timing" OFF)

if(CBGL_EXACT_FP)
  add_compile_options(-O2 -fno-fast-math -ffp-contract=off)
else()
  add_compile_options(-Ofast -fno-finite-math-only)
endif()

find_package(ament_cmake REQUIRED)
find_package(fsm_localisation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(cbgl_core SHARED
  src/cbgl/cbgl.cpp)

target_include_directories(cbgl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(cbgl_core PUBLIC
  fsm_localisation::fsm_localisation_core)

if(CBGL_DETERMINISTIC_FFT)
  target_compile_definitions(cbgl_core PUBLIC FSM_LOCALISATION_FFTW_ESTIMATE)
endif()

add_library(cbgl_grid SHARED
  src/cbgl/free_space.cpp)

target_link_libraries(cbgl_grid PUBLIC
  cbgl_core
  fsm_localisation::fsm_localisation_grid)

add_library(cbgl_interface SHARED
  src/cbgl/cbgl_interface.cpp)

target_link_libraries(cbgl_interface PUBLIC
  cbgl_grid
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS})

add_executable(cbgl_interface_node
  src/cbgl/cbgl_interface_node.cpp)

target_link_libraries(cbgl_interface_node cbgl_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  # The vendored casting, the particle filter, the second ray tracer and the
  # ROS 1 node are not this package's code to hold to its own conventions, and
  # they are on their way out of it. The exclusion below leaves with them. It
  # names the files one by one because a linter excludes a path it can match
  # against a file it found, and a directory matches nothing.
  file(GLOB_RECURSE cbgl_unlinted
    ${CMAKE_CURRENT_SOURCE_DIR}/include/cbgl_node/*
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utils/*
    ${CMAKE_CURRENT_SOURCE_DIR}/src/cbgl_node/*
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils/*)

  set(AMENT_LINT_AUTO_FILE_EXCLUDE ${cbgl_unlinted})

  # Two of the linters take no exclusion and walk the whole package instead, so
  # their automatic pass is held back and each is pointed at this package's own
  # files. Neither is switched off: the vendored trees' own CMake and the two
  # plotting scripts that came with the vendored casting are simply not what
  # this package is linted on.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_lint_cmake ament_cmake_pep257)

  ament_lint_auto_find_test_dependencies()

  ament_lint_cmake(CMakeLists.txt cmake)
  ament_pep257(test)

  find_package(ament_cmake_gtest REQUIRED)

  add_library(cbgl_core_exact OBJECT
    src/cbgl/cbgl.cpp)

  target_include_directories(cbgl_core_exact PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  target_link_libraries(cbgl_core_exact PUBLIC
    fsm_localisation::fsm_localisation_core)

  target_compile_options(cbgl_core_exact PUBLIC
    -O2 -fno-fast-math -ffp-contract=off)

  target_compile_definitions(cbgl_core_exact PUBLIC
    FSM_LOCALISATION_FFTW_ESTIMATE)

  add_library(cbgl_grid_exact OBJECT
    src/cbgl/free_space.cpp)

  target_link_libraries(cbgl_grid_exact PUBLIC
    cbgl_core_exact
    fsm_localisation::fsm_localisation_grid)

  ament_add_gtest(test_cbgl_caer
    test/test_caer.cpp)

  ament_add_gtest(test_cbgl_dispersal
    test/test_dispersal.cpp)

  ament_add_gtest(test_cbgl_free_space
    test/test_free_space.cpp)

  ament_add_gtest(test_cbgl_localiser
    test/test_localiser.cpp
    TIMEOUT 300)
  target_compile_definitions(test_cbgl_localiser PRIVATE
    CBGL_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  ament_add_gtest(test_cbgl_parameters
    test/test_parameters.cpp)

  ament_add_gtest(test_cbgl_rotation_labelling
    test/test_rotation_labelling.cpp
    TIMEOUT 120)

  ament_add_gtest(test_cbgl_sift
    test/test_sift.cpp
    TIMEOUT 300)

  foreach(test_target test_cbgl_caer test_cbgl_dispersal test_cbgl_parameters)
    if(TARGET ${test_target})
      target_link_libraries(${test_target} cbgl_core_exact)
    endif()
  endforeach()

  foreach(test_target test_cbgl_free_space test_cbgl_localiser
                      test_cbgl_rotation_labelling test_cbgl_sift)
    if(TARGET ${test_target})
      target_link_libraries(${test_target}
        cbgl_core_exact
        cbgl_grid_exact)
    endif()
  endforeach()
endif()

install(TARGETS
  cbgl_core
  cbgl_grid
  cbgl_interface
  EXPORT export_cbgl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

install(TARGETS
  cbgl_interface_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/cbgl
  DESTINATION include)

ament_export_targets(export_cbgl HAS_LIBRARY_TARGET)

ament_export_dependencies(
  fsm_localisation
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs)

ament_package(CONFIG_EXTRAS cmake/cbgl-extras.cmake)
