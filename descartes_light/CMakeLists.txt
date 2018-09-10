cmake_minimum_required(VERSION 2.8.3)
project(descartes_light)

add_compile_options(-std=c++11 -Wall -Wextra -fno-omit-frame-pointer)

find_package(catkin REQUIRED COMPONENTS
  opw_kinematics
  control_msgs
  roscpp # Demo
  actionlib # Demo
  tesseract_ros
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    descartes_light
  CATKIN_DEPENDS
    opw_kinematics
    tesseract_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Core Library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/collision_checker.cpp
  src/${PROJECT_NAME}/descartes_light.cpp
  src/${PROJECT_NAME}/edge_sampler.cpp
  src/${PROJECT_NAME}/kinematic_interface.cpp
  src/${PROJECT_NAME}/position_sampler.cpp
  src/${PROJECT_NAME}/ladder_graph_dag_search.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Demonstration

add_executable(${PROJECT_NAME}_node src/descartes_light_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME demonstration PREFIX "")

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_descartes_light.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
