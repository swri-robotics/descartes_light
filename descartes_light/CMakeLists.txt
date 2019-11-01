cmake_minimum_required(VERSION 3.5.0)
project(descartes_light VERSION 0.1.0 LANGUAGES CXX)
include(cmake/descartes_light_macros.cmake)

find_package(OpenMP REQUIRED)

if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

find_package(Eigen3 REQUIRED)
find_package(console_bridge)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${console_bridge_INCLUDE_DIRS}")
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES "${console_bridge_LIBRARIES}")
endif()

# Interface Library
add_library(${PROJECT_NAME}_core INTERFACE)
target_link_libraries(${PROJECT_NAME}_core INTERFACE console_bridge::console_bridge)
descartes_target_compile_options(${PROJECT_NAME}_core INTERFACE)
target_include_directories(${PROJECT_NAME}_core INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_core SYSTEM INTERFACE
    ${EIGEN3_INCLUDE_DIRS})

# Core Library
add_library(${PROJECT_NAME} SHARED
  src/descartes_light.cpp
  src/ladder_graph.cpp
  src/ladder_graph_dag_search.cpp
)
target_link_libraries(${PROJECT_NAME} PUBLIC console_bridge::console_bridge OpenMP::OpenMP_CXX)
descartes_target_compile_options(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    ${EIGEN3_INCLUDE_DIRS})

# Declare a C++ library
add_library(${PROJECT_NAME}_gantry SHARED src/gantry_kinematics.cpp)
target_link_libraries(${PROJECT_NAME}_gantry PUBLIC console_bridge::console_bridge ${PROJECT_NAME}_core)
descartes_target_compile_options(${PROJECT_NAME}_gantry PUBLIC)
target_include_directories(${PROJECT_NAME}_gantry PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_gantry SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIRS})

descartes_configure_package(${PROJECT_NAME}_core ${PROJECT_NAME} ${PROJECT_NAME}_gantry)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h*"
  PATTERN ".svn" EXCLUDE
 )

install(FILES
  "${CMAKE_CURRENT_LIST_DIR}/cmake/descartes_light_macros.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})
