cmake_minimum_required(VERSION 3.5.0)
project(descartes_samplers VERSION 0.1.0 LANGUAGES CXX)

find_package(descartes_light REQUIRED)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/evaluators/distance_edge_evaluator.cpp
  src/evaluators/euclidean_distance_edge_evaluator.cpp
  src/evaluators/gantry_euclidean_distance_edge_evaluator.cpp
  src/samplers/axial_symmetric_sampler.cpp
  src/samplers/cartesian_point_sampler.cpp
  src/samplers/external_axis_sampler.cpp
  src/samplers/fixed_joint_pose_sampler.cpp
  src/samplers/railed_axial_symmetric_sampler.cpp
  src/samplers/railed_cartesian_point_sampler.cpp
  src/samplers/axial_symmetric_variable_offset_sampler.cpp
)
target_link_libraries(${PROJECT_NAME} descartes::descartes_light)
descartes_target_compile_options(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC)

descartes_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )
