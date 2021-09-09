find_package(Boost REQUIRED COMPONENTS graph)

add_library(${PROJECT_NAME}_bgl src/bgl_solver.cpp)
target_link_libraries(${PROJECT_NAME}_bgl PUBLIC ${PROJECT_NAME} Boost::graph)
target_cxx_version(${PROJECT_NAME}_bgl PUBLIC VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_bgl PUBLIC ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_bgl PUBLIC ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(
  ${PROJECT_NAME}_bgl
  ENABLE ${DESCARTES_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${DESCARTES_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME}_bgl ALL ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_bgl PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                      "$<INSTALL_INTERFACE:include>")

install(
  TARGETS ${PROJECT_NAME}_bgl
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Install headers
install(DIRECTORY include/ DESTINATION include/)
