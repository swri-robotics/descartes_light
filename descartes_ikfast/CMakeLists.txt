cmake_minimum_required(VERSION 3.5.0)
project(descartes_ikfast VERSION 0.1.0 LANGUAGES CXX)

find_package(descartes_light REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LAPACK REQUIRED)
find_package(console_bridge REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${console_bridge_INCLUDE_DIRS}")
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES "${console_bridge_LIBRARIES}")
endif()

add_library(${PROJECT_NAME} INTERFACE)
target_link_libraries(${PROJECT_NAME} INTERFACE console_bridge::console_bridge descartes::descartes_light_core ${LAPACK_LIBRARIES})
descartes_target_compile_options(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE
    ${LAPACK_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS})

descartes_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

if (ENABLE_TESTS)
  enable_testing()
  add_custom_target(run_tests ALL
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMAND ${CMAKE_CTEST_COMMAND} -V -C $<CONFIGURATION>)

  add_subdirectory(test)
endif()
