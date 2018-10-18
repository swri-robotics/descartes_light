cmake_minimum_required(VERSION 2.8.3)
project(workcell2_demos)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  hybrid_planning_common
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Demo
add_executable(${PROJECT_NAME} src/workcell2_demo.cpp)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME workcell2_demo PREFIX "")

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
 ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
