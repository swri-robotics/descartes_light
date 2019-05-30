cmake_minimum_required(VERSION 2.8.3)
project(descartes_light)

add_compile_options(-std=c++11 -Wall -Wextra -fopenmp)

find_package(Eigen3 REQUIRED)
find_package(console_bridge)
find_package(tesseract REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  opw_kinematics
  control_msgs
  roscpp # Demo
  actionlib # Demo
)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_collision
    ${PROJECT_NAME}_kinematics
    ${PROJECT_NAME}_evaluators
    ${PROJECT_NAME}_samplers
  CATKIN_DEPENDS
    opw_kinematics
  DEPENDS
    EIGEN3
    tesseract
    console_bridge
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
)

# Core Library
add_library(${PROJECT_NAME}_core
  src/descartes_light.cpp
  src/ladder_graph.cpp
  src/ladder_graph_dag_search.cpp
)
add_dependencies(${PROJECT_NAME}_core ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES} gomp)

# Collision Interface Implementations
add_library(${PROJECT_NAME}_collision
  src/impl/collision/tesseract_collision_checker.cpp
)

add_dependencies(${PROJECT_NAME}_collision ${${PROJECT_NAME}_collision_EXPORTED_TARGETS} ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(${PROJECT_NAME}_collision ${PROJECT_NAME}_core ${catkin_LIBRARIES} tesseract::tesseract)

add_library(${PROJECT_NAME}_kinematics
  src/impl/kinematics/opw_kinematics.cpp
  src/impl/kinematics/opw_railed_kinematics.cpp
)

add_dependencies(${PROJECT_NAME}_kinematics ${${PROJECT_NAME}_kinematics_EXPORTED_TARGETS} ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_kinematics ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_evaluators
  src/impl/evaluators/distance_edge_evaluator.cpp
)

add_dependencies(${PROJECT_NAME}_evaluators ${${PROJECT_NAME}_evaluators_EXPORTED_TARGETS} ${${PROJECT_NAME}_core_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_evaluators ${PROJECT_NAME}_core ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_samplers
  src/impl/samplers/axial_symmetric_sampler.cpp
  src/impl/samplers/cartesian_point_sampler.cpp
  src/impl/samplers/external_axis_sampler.cpp
  src/impl/samplers/fixed_joint_pose_sampler.cpp
  src/impl/samplers/railed_axial_symmetric_sampler.cpp
  src/impl/samplers/railed_cartesian_point_sampler.cpp
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

install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_descartes_light.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
