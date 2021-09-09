add_executable(${PROJECT_NAME}_benchmarks benchmarks.cpp)
target_link_libraries(
  ${PROJECT_NAME}_benchmarks
  PRIVATE ${PROJECT_NAME}
          ${PROJECT_NAME}_bgl
          ${PROJECT_NAME}_test_utils
          GTest::GTest
          GTest::Main)
target_clang_tidy(
  ${PROJECT_NAME}_benchmarks
  ENABLE ${DESCARTES_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${DESCARTES_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})

install(
  TARGETS ${PROJECT_NAME}_benchmarks
  RUNTIME DESTINATION bin/benchmarks
  LIBRARY DESTINATION lib/benchmarks
  ARCHIVE DESTINATION lib/benchmarks)
