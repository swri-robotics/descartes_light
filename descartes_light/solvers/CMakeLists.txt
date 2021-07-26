find_package(OpenMP REQUIRED)
find_package(console_bridge)
find_package(Boost REQUIRED COMPONENTS graph)

if(NOT TARGET OpenMP::OpenMP_CXX)
  find_package(Threads REQUIRED)
  add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
  # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
  set_property(TARGET OpenMP::OpenMP_CXX PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)
endif()

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                  ${console_bridge_INCLUDE_DIRS})
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES ${console_bridge_LIBRARIES})
else()
  get_target_property(CHECK_INCLUDE_DIRECTORIES console_bridge::console_bridge INTERFACE_INCLUDE_DIRECTORIES)
  if(NOT ${CHECK_INCLUDE_DIRECTORIES})
    set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
                                                                    ${console_bridge_INCLUDE_DIRS})
  endif()
endif()

add_library(
  ${PROJECT_NAME}_solvers
  src/bgl/bgl_ladder_graph_solver.cpp
  src/ladder_graph/ladder_graph_solver.cpp
  src/ladder_graph/ladder_graph.cpp
  src/ladder_graph/ladder_graph_dag_search.cpp)
target_link_libraries(
  ${PROJECT_NAME}_solvers
  PUBLIC ${PROJECT_NAME}_core
         OpenMP::OpenMP_CXX
         console_bridge::console_bridge
         Boost::graph)
target_cxx_version(${PROJECT_NAME}_solvers PUBLIC VERSION ${DESCARTES_CXX_VERSION})
target_compile_options(${PROJECT_NAME}_solvers PUBLIC ${DESCARTES_COMPILE_OPTIONS})
target_compile_definitions(${PROJECT_NAME}_solvers PUBLIC ${DESCARTES_COMPILE_DEFINITIONS})
target_clang_tidy(
  ${PROJECT_NAME}_solvers
  ENABLE ${DESCARTES_ENABLE_CLANG_TIDY}
  WARNINGS_AS_ERRORS ${DESCARTES_ENABLE_TESTING}
  CHECKS ${DEFAULT_CLANG_TIDY_CHECKS})
target_code_coverage(${PROJECT_NAME}_solvers ALL ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
target_include_directories(${PROJECT_NAME}_solvers PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                          "$<INSTALL_INTERFACE:include>")

install(
  TARGETS ${PROJECT_NAME}_solvers
  EXPORT ${PROJECT_NAME}-targets
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
