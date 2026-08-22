add_library(cbgl_lib cbgl.cpp)

# Range_libc required library
set(BASEPATH "${CMAKE_SOURCE_DIR}")
set(RANGELIB_DIR ${BASEPATH}/include/utils/range_libc)
add_library(cbgl_lodepng_lib
  ${RANGELIB_DIR}/vendor/lodepng/lodepng.cpp)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
  /usr/include/pcl-1.7/
  /usr/include/eigen3/
)

target_link_libraries(cbgl_lib
  fftw3
  CGAL::CGAL_Core
  cbgl_lodepng_lib
  ${catkin_LIBRARIES}
  ${csm_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  cbgl_og_utils_lib
  cbgl_pf_utils_lib)

add_dependencies(cbgl_lib
  ${csm_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(cbgl_node
  cbgl_node.cpp)

target_link_libraries(cbgl_node
  cbgl_lib
  ${catkin_LIBRARIES}
)

add_dependencies(cbgl_node
  ${catkin_EXPORTED_TARGETS}
)
