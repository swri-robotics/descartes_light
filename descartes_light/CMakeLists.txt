cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

include(cmake/descartes_light_macros.cmake)

# Find dependency packages
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(console_bridge)

# Create targets if necessary
if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if (NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

descartes_variables()
initialize_code_coverage(ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
add_code_coverage_all_targets(ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})

# Interface Library
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE Eigen3::Eigen console_bridge::console_bridge)
target_cxx_version(${PROJECT_NAME}_core INTERFACE VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_core INTERFACE ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_core INTERFACE ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_core ENABLE ${DESCARTES_ENABLE_CLANG_TIDY} ARGUMENTS ${DESCARTES_CLANG_TIDY_ARGS})
target_code_coverage(${PROJECT_NAME}_core INTERFACE ALL ENABLE ${DESCARTES_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

# Core Library
add_library(${PROJECT_NAME}_ladder_graph
  src/ladder_graph/ladder_graph_solver.cpp
  src/ladder_graph/ladder_graph.cpp
  src/ladder_graph/ladder_graph_dag_search.cpp
)
target_link_libraries(${PROJECT_NAME}_ladder_graph PUBLIC ${PROJECT_NAME}_core console_bridge::console_bridge OpenMP::OpenMP_CXX)
target_cxx_version(${PROJECT_NAME}_ladder_graph PUBLIC VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_ladder_graph PUBLIC ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_ladder_graph PUBLIC ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_ladder_graph ENABLE ${DESCARTES_ENABLE_CLANG_TIDY} ARGUMENTS ${DESCARTES_CLANG_TIDY_ARGS})
target_code_coverage(${PROJECT_NAME}_ladder_graph ALL ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})

configure_package(NAMESPACE descartes TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ladder_graph)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
 )

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/descartes_light_macros.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

if(DESCARTES_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${DESCARTES_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()
