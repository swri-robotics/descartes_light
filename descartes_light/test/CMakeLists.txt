# Utilities Library
add_library(${PROJECT_NAME}_test_utils src/utils.cpp src/solver_factory.cpp)
target_link_libraries(${PROJECT_NAME}_test_utils PUBLIC ${PROJECT_NAME} ${PROJECT_NAME}_bgl)
target_cxx_version(${PROJECT_NAME}_test_utils PUBLIC VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_test_utils PUBLIC ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_test_utils PUBLIC ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(
  ${PROJECT_NAME}_test_utils
  ENABLE ${DESCARTES_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${DESCARTES_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_include_directories(${PROJECT_NAME}_test_utils PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                             "$<INSTALL_INTERFACE:include>")

install(
  TARGETS ${PROJECT_NAME}_test_utils
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# Install headers
install(DIRECTORY include/ DESTINATION include/)

# Unit Tests
find_gtest()
add_executable(${PROJECT_NAME}_utest utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_utest
  PRIVATE ${PROJECT_NAME}
          ${PROJECT_NAME}_bgl
          ${PROJECT_NAME}_test_utils
          GTest::GTest
          GTest::Main)
target_clang_tidy(
  ${PROJECT_NAME}_utest
  ENABLE ${DESCARTES_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${DESCARTES_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME}_utest ALL ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
add_gtest_discover_tests(${PROJECT_NAME}_utest)
add_dependencies(run_tests ${PROJECT_NAME}_utest)

install(
  TARGETS ${PROJECT_NAME}_test_utils ${PROJECT_NAME}_utest
  RUNTIME DESTINATION bin/tests
  LIBRARY DESTINATION lib/test
  ARCHIVE DESTINATION lib/test)

# Benchmarks
add_subdirectory(benchmarks)
