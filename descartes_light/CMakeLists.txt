cmake_minimum_required(VERSION 2.8.3)
project(descartes_light)

add_compile_options(-std=c++11 -Wall -Wextra -fopenmp)

find_package(Eigen3 REQUIRED)

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
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    descartes_light
  CATKIN_DEPENDS
    opw_kinematics
    tesseract_ros
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM ${EIGEN3_INCLUDE_DIRS}
)

# Core Library
add_library(${PROJECT_NAME}_core
  src/core/descartes_light.cpp
  src/core/ladder_graph_dag_search.cpp
)

add_dependencies(${PROJECT_NAME}_core ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES} gomp)

# Collision Interface Implementations
add_library(${PROJECT_NAME}_collision
  src/collision/tesseract_collision_checker.cpp
)

add_dependencies(${PROJECT_NAME}_collision ${${PROJECT_NAME}_collision_EXPORTED_TARGETS} ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_collision ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_kinematics
  src/kinematics/opw_kinematics.cpp
  src/kinematics/opw_railed_kinematics.cpp
)

add_dependencies(${PROJECT_NAME}_kinematics ${${PROJECT_NAME}_kinematics_EXPORTED_TARGETS} ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_kinematics ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_evaluators
  src/evaluators/distance_edge_evaluator.cpp
)

add_dependencies(${PROJECT_NAME}_evaluators ${${PROJECT_NAME}_evaluators_EXPORTED_TARGETS} ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_evaluators ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_samplers
  src/samplers/axial_symmetric_sampler.cpp
  src/samplers/cartesian_point_sampler.cpp
  src/samplers/external_axis_sampler.cpp
  src/samplers/railed_axial_symmetric_sampler.cpp
  src/samplers/railed_cartesian_point_sampler.cpp
)

add_dependencies(${PROJECT_NAME}_samplers ${${PROJECT_NAME}_samplers_EXPORTED_TARGETS} ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_samplers ${PROJECT_NAME}_core ${catkin_LIBRARIES})

# Demonstration

add_executable(${PROJECT_NAME}_node src/descartes_light_node.cpp)

set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME demonstration PREFIX "")

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_core_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_collision_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_kinematics_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_evaluators_EXPORTED_TARGETS}
    ${${PROJECT_NAME}_samplers_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_collision
  ${PROJECT_NAME}_kinematics
  ${PROJECT_NAME}_evaluators
  ${PROJECT_NAME}_samplers
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_collision ${PROJECT_NAME}_kinematics ${PROJECT_NAME}_evaluators ${PROJECT_NAME}_samplers ${PROJECT_NAME}_node
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
