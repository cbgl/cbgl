cmake_minimum_required(VERSION 3.16)
project(cbgl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(fsm_localisation REQUIRED)

# The arithmetic is fsm_localisation's to choose and this package follows it.
#
# It is not a choice this package can make. The matcher is a header this
# package compiles for itself, and fsm_localisation compiled the same header
# when it was built. Two arithmetics over one header put two definitions of one
# inline function into a single process, and which survives linking is the
# linker's choice rather than anybody's intention. Where fsm_localisation
# arrives already built, from a package manager, its arithmetic is settled and
# nothing here can revisit it; where the two are built together, whoever asked
# for that build has already said what they want. Either way the answer is read
# rather than made, so this package offers no switch of its own.
#
# The transform planning choice needs nothing here at all. It travels as a
# compile definition on the exported library target and reaches this package's
# own translation units through the link.
if(NOT DEFINED FSM_LOCALISATION_BUILT_COMPILE_OPTIONS)
  message(FATAL_ERROR
    "the fsm_localisation found here does not record the arithmetic it was "
    "built with, so this package cannot match it. Every released "
    "fsm_localisation records it. A tree that does not is older than the "
    "first release and has to be updated before this package is built "
    "against it.")
endif()

add_compile_options(${FSM_LOCALISATION_BUILT_COMPILE_OPTIONS})
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(CSM REQUIRED IMPORTED_TARGET csm)

# csm is LGPL v3 and is linked dynamically, which is what satisfies that
# licence's relinking requirement. Nothing about the lookup above guarantees
# that. csm's own build file declares its library with no type, so a csm built
# without BUILD_SHARED_LIBS produces an archive, and the lookup resolves to
# that archive with nothing said by anything. What comes out links, runs,
# passes every test here, and quietly puts whoever redistributes it in the
# position choosing this variant was meant to avoid. Reading a linker command
# is the only way to notice after the fact, so it is refused here instead.
# The same refusal stands in cmake/cbgl-extras.cmake, which is what a package
# depending on this one resolves csm through.
if(CSM_LINK_LIBRARIES MATCHES "\\.a(;|$)")
  message(FATAL_ERROR
    "csm resolved to a static archive (${CSM_LINK_LIBRARIES}). cbgl links csm "
    "dynamically because csm is LGPL v3 and a static link does not satisfy "
    "that licence's relinking requirement. Rebuild csm with "
    "-DBUILD_SHARED_LIBS=ON. The installation section of cbgl's README carries "
    "the whole recipe, including three patches csm needs.")
endif()

add_library(cbgl_core SHARED
  src/cbgl/cbgl.cpp)

target_include_directories(cbgl_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# cbgl_core refines a survivor by one of two algorithms and the second of them
# is cbgl_icp_refinement, so csm travels with the core rather than only with
# whoever asked for that refinement. csm is LGPL v3 and the note above says what
# that costs; it now costs it of everything that links the core.
target_link_libraries(cbgl_core PUBLIC
  cbgl_icp_refinement
  fsm_localisation::fsm_localisation_core)

add_library(cbgl_grid SHARED
  src/cbgl/free_space.cpp)

target_link_libraries(cbgl_grid PUBLIC
  cbgl_core
  fsm_localisation::fsm_localisation_grid)

add_library(cbgl_icp_refinement SHARED
  src/cbgl/icp_refinement.cpp)

target_include_directories(cbgl_icp_refinement PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(cbgl_icp_refinement
  PUBLIC fsm_localisation::fsm_localisation_core
  PRIVATE PkgConfig::CSM)

add_library(cbgl_map_file SHARED
  src/cbgl/map_file.cpp)

target_include_directories(cbgl_map_file PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(cbgl_map_file
  PUBLIC ${nav_msgs_TARGETS}
  PRIVATE yaml-cpp::yaml-cpp)

add_library(cbgl_interface SHARED
  src/cbgl/cbgl_interface.cpp)

target_link_libraries(cbgl_interface PUBLIC
  cbgl_grid
  cbgl_map_file
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
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

  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # The refinement is compiled into this object library rather than linked in
  # as the shared one. The shared one carries the package's default fast
  # arithmetic, so a test that asks two builds to agree bit for bit would
  # otherwise have one translation unit in the middle of it where the compiler
  # may reassociate and contract: the scan conversion, the settings copy and the
  # composition of the solver's answer onto the estimate. The solver underneath
  # is a separate library built once and not by these options, so that one file
  # is the whole of the gap and compiling it here closes it.
  # It is a source of this library rather than a second object library because
  # the objects of an object library reach an executable only where that
  # executable links it directly, and what the tests link is this one.
  add_library(cbgl_core_exact OBJECT
    src/cbgl/cbgl.cpp
    src/cbgl/icp_refinement.cpp)

  target_include_directories(cbgl_core_exact PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

  target_link_libraries(cbgl_core_exact
    PUBLIC fsm_localisation::fsm_localisation_core
    PRIVATE PkgConfig::CSM)

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

  ament_add_gtest(test_cbgl_icp_refinement
    test/test_icp_refinement.cpp)
  target_link_libraries(test_cbgl_icp_refinement cbgl_icp_refinement)

  ament_add_gtest(test_cbgl_localiser
    test/test_localiser.cpp
    TIMEOUT 300)
  target_compile_definitions(test_cbgl_localiser PRIVATE
    CBGL_TEST_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")

  ament_add_gtest(test_cbgl_map_file
    test/test_map_file.cpp)
  target_link_libraries(test_cbgl_map_file cbgl_map_file)

  ament_add_gtest(test_cbgl_node_params
    test/test_node_params.cpp
    ENV ROS_DOMAIN_ID=81)
  target_link_libraries(test_cbgl_node_params cbgl_interface)
  target_compile_definitions(test_cbgl_node_params PRIVATE
    CBGL_README="${CMAKE_CURRENT_SOURCE_DIR}/../README.md")

  ament_add_gtest(test_cbgl_parameters
    test/test_parameters.cpp)

  ament_add_gtest(test_cbgl_rotation_labelling
    test/test_rotation_labelling.cpp
    TIMEOUT 120)

  ament_add_gtest(test_cbgl_sift
    test/test_sift.cpp
    TIMEOUT 300)

  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/test_map_from_file.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=88
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_node_behaviour.py TIMEOUT 400
    ENV ROS_DOMAIN_ID=82
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_node_startup.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=83
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_optional_topics.py TIMEOUT 400
    ENV ROS_DOMAIN_ID=84
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_refusals.py TIMEOUT 600
    ENV ROS_DOMAIN_ID=85
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_refuses_to_start.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=86
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

  add_launch_test(test/test_scan_qos.py TIMEOUT 300
    ENV ROS_DOMAIN_ID=87
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test)

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
  cbgl_icp_refinement
  cbgl_interface
  cbgl_map_file
  EXPORT export_cbgl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include)

install(TARGETS
  cbgl_interface_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/cbgl
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

# The licence texts are installed rather than left in the repository alone.
# What a redistributor ships is this tree, not the sources, and LGPL v3 asks
# for its own text to travel with the work. csm reaches every exported target
# through cbgl_core, so the obligation reaches every one of them too.
install(FILES LICENSE COPYING COPYING.LESSER
  DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_cbgl HAS_LIBRARY_TARGET)

ament_export_dependencies(
  fsm_localisation
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  tf2
  tf2_ros)

ament_package(CONFIG_EXTRAS cmake/cbgl-extras.cmake)
