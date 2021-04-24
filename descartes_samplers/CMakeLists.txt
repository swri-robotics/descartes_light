cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

find_package(descartes_light REQUIRED)
find_package(Eigen3 REQUIRED)

descartes_variables()
initialize_code_coverage(ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
add_code_coverage_all_targets(ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/evaluators/timing_edge_evaluator.cpp
  src/evaluators/euclidean_distance_edge_evaluator.cpp
  src/evaluators/compound_edge_evaluator.cpp
  src/samplers/fixed_joint_pose_sampler.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen descartes::descartes_light_core)
target_cxx_version(${PROJECT_NAME} PUBLIC VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME} PUBLIC ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME} ENABLE ${DESCARTES_ENABLE_CLANG_TIDY} ARGUMENTS ${DESCARTES_CLANG_TIDY_ARGS})
target_code_coverage(${PROJECT_NAME} ALL ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE descartes TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  PATTERN ".svn" EXCLUDE
 )
