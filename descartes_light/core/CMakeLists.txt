# Find dependency packages
find_package(Eigen3 REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)

# Create targets if necessary
if(NOT TARGET Eigen3::Eigen)
  find_package(Threads REQUIRED)
  add_library(Eigen3::Eigen IMPORTED INTERFACE)
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_COMPILE_DEFINITIONS ${EIGEN3_DEFINITIONS})
  set_property(TARGET Eigen3::Eigen PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${EIGEN3_INCLUDE_DIRS})
endif()

# Interface Library
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE Eigen3::Eigen)
target_cxx_version(${PROJECT_NAME}_core INTERFACE VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_core INTERFACE ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_core INTERFACE ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(
  ${PROJECT_NAME}_core
  ENABLE ${DESCARTES_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${DESCARTES_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(
  ${PROJECT_NAME}_core
  INTERFACE
  ALL
  ENABLE ${DESCARTES_ENABLE_TESTING})
target_include_directories(${PROJECT_NAME}_core INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

install(
  TARGETS ${PROJECT_NAME}_core
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
