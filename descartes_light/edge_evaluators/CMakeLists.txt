add_library(${PROJECT_NAME}_edge_evaluators
  src/timing_edge_evaluator.cpp
  src/euclidean_distance_edge_evaluator.cpp
  src/compound_edge_evaluator.cpp
)
target_link_libraries(${PROJECT_NAME}_edge_evaluators PUBLIC ${PROJECT_NAME}_core)
target_cxx_version(${PROJECT_NAME}_edge_evaluators PUBLIC VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_edge_evaluators PUBLIC ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_edge_evaluators PUBLIC ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(${PROJECT_NAME}_edge_evaluators ENABLE ${DESCARTES_ENABLE_CLANG_TIDY} ARGUMENTS ${DESCARTES_CLANG_TIDY_ARGS})
target_code_coverage(${PROJECT_NAME}_edge_evaluators ALL ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_edge_evaluators PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

install(
  TARGETS
    ${PROJECT_NAME}_edge_evaluators
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)
