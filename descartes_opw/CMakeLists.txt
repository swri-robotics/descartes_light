cmake_minimum_required(VERSION 3.5.0)
project(descartes_opw VERSION 0.1.0 LANGUAGES CXX)

find_package(descartes_light REQUIRED)
find_package(opw_kinematics REQUIRED)
find_package(console_bridge REQUIRED)

if(NOT TARGET console_bridge::console_bridge)
  add_library(console_bridge::console_bridge INTERFACE IMPORTED)
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${console_bridge_INCLUDE_DIRS}")
  set_target_properties(console_bridge::console_bridge PROPERTIES INTERFACE_LINK_LIBRARIES "${console_bridge_LIBRARIES}")
endif()

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED src/descartes_opw_kinematics.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC console_bridge::console_bridge opw_kinematics::opw_kinematics descartes::descartes_light)
descartes_target_compile_options(${PROJECT_NAME} PUBLIC)
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC)

descartes_configure_package(${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )
