find_package(ompl REQUIRED)

add_library(${PROJECT_NAME}_ompl src/ompl_solver.cpp)
target_link_libraries(${PROJECT_NAME}_ompl PUBLIC ${PROJECT_NAME} ${OMPL_LIBRARIES})
target_cxx_version(${PROJECT_NAME}_ompl PUBLIC VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_ompl PUBLIC ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_ompl PUBLIC ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(
  ${PROJECT_NAME}_ompl
  ENABLE ${DESCARTES_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${DESCARTES_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME}_ompl ALL ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_ompl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>"
                                                          ${OMPL_INCLUDE_DIRS})

install(
  TARGETS ${PROJECT_NAME}_ompl
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Install headers
install(DIRECTORY include/ DESTINATION include/)
