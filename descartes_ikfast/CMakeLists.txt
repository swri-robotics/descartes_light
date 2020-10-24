cmake_minimum_required(VERSION 3.5.0)
project(descartes_ikfast VERSION 0.1.0 LANGUAGES CXX)

find_package(descartes_light REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LAPACK REQUIRED)
find_package(console_bridge REQUIRED)
find_package(cmake_common_scripts REQUIRED)

set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)

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

if(NOT MSVC)
  add_code_coverage_all_targets(ENABLE ${DESCARTES_ENABLE_TESTING})
endif()

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE console_bridge::console_bridge descartes::descartes_light_core ${LAPACK_LIBRARIES})
target_cxx_version(${PROJECT_NAME} INTERFACE VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME} INTERFACE ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME} INTERFACE ${DESCARTES_COMPILE_DEFINITIONS})
if(NOT MSVC)
  target_clang_tidy(${PROJECT_NAME} ENABLE ${DESCARTES_ENABLE_CLANG_TIDY} ARGUMENTS ${DESCARTES_CLANG_TIDY_ARGS})
  target_code_coverage(${PROJECT_NAME} INTERFACE ALL ENABLE ${DESCARTES_ENABLE_TESTING})
endif()
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${LAPACK_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

configure_package(NAMESPACE descartes TARGETS ${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  PATTERN ".svn" EXCLUDE
 )

if (DESCARTES_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${DESCARTES_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()
