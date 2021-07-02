find_gtest()
add_executable(${PROJECT_NAME}_utest utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_utest
  PRIVATE ${PROJECT_NAME}_edge_evaluators
          ${PROJECT_NAME}_solvers
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
  TARGETS ${PROJECT_NAME}_utest
  RUNTIME DESTINATION bin/tests
  LIBRARY DESTINATION lib/tests
  ARCHIVE DESTINATION lib/tests)
