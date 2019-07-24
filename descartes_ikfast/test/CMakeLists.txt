find_package(GTest REQUIRED)

add_library(${PROJECT_NAME}_robot SHARED descartes_ikfast_fanuc_m20ia10l_manipulator.cpp)
target_link_libraries(${PROJECT_NAME}_robot PUBLIC ${PROJECT_NAME})
target_compile_options(${PROJECT_NAME}_robot PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_robot PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_robot PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_robot PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include>")
add_dependencies(${PROJECT_NAME}_robot ${PACKAGE_LIBRARIES})

add_executable(${PROJECT_NAME}_unit descartes_ikfast_unit.cpp)
target_link_libraries(${PROJECT_NAME}_unit PRIVATE ${GTEST_LIBRARIES} ${PROJECT_NAME}_robot)
target_compile_options(${PROJECT_NAME}_unit PRIVATE -Wsuggest-override -Wconversion -Wsign-conversion)
if(CXX_FEATURE_FOUND EQUAL "-1")
    target_compile_options(${PROJECT_NAME}_unit PUBLIC -std=c++11)
else()
    target_compile_features(${PROJECT_NAME}_unit PRIVATE cxx_std_11)
endif()
target_include_directories(${PROJECT_NAME}_unit PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_unit SYSTEM PRIVATE
    ${GTEST_INCLUDE_DIRS})
if(${CMAKE_VERSION} VERSION_LESS "3.10.0")
    gtest_add_tests(${PROJECT_NAME}_unit "" AUTO)
else()
    gtest_discover_tests(${PROJECT_NAME}_unit)
endif()
add_dependencies(${PROJECT_NAME}_unit ${PACKAGE_LIBRARIES})
add_dependencies(run_tests ${PROJECT_NAME}_unit)
