find_gtest()
add_executable(${PROJECT_NAME}_utest utest.cpp)
target_link_libraries(${PROJECT_NAME}_utest PRIVATE ${PROJECT_NAME} GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_utest)
add_dependencies(run_tests ${PROJECT_NAME}_utest)

install(TARGETS ${PROJECT_NAME}_utest
  RUNTIME DESTINATION bin/tests
  LIBRARY DESTINATION lib/tests
  ARCHIVE DESTINATION lib/tests
)
