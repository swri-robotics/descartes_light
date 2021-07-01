cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

include(cmake/descartes_light_macros.cmake)

descartes_variables()
initialize_code_coverage(ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
add_code_coverage_all_targets(ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})

add_subdirectory(core)
add_subdirectory(edge_evaluators)
add_subdirectory(samplers)
add_subdirectory(solvers)
add_subdirectory(state_evaluators)

if(DESCARTES_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${DESCARTES_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Package configuration
install(EXPORT ${PROJECT_NAME}-targets NAMESPACE "descartes::" DESTINATION lib/cmake/${PROJECT_NAME})
export(EXPORT ${PROJECT_NAME}-targets NAMESPACE "descartes::"
       FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)
install(FILES "${CMAKE_CURRENT_LIST_DIR}/cmake/descartes_light_macros.cmake" DESTINATION lib/cmake/${PROJECT_NAME})
configure_package()
