cmake_minimum_required(VERSION 3.5.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

include(core/cmake/core-macros.cmake)

descartes_variables()
initialize_code_coverage(ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})
add_code_coverage_all_targets(ENABLE ${DESCARTES_ENABLE_CODE_COVERAGE})

set(SUPPORTED_COMPONENTS core)
add_subdirectory(core)

option(DESCARTES_BUILD_BGL "Build Boost graph library based components" ON)
if(DESCARTES_BUILD_BGL)
  message("Building Boost graph library implementations")
  list(APPEND SUPPORTED_COMPONENTS bgl)
  add_subdirectory(bgl)
endif()

if(DESCARTES_ENABLE_TESTING)
  enable_testing()
  add_run_tests_target(ENABLE ${DESCARTES_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Package configuration
configure_package(COMPONENT core SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})

if(DESCARTES_PACKAGE)
  cpack_component_package(
    VERSION ${pkg_extracted_version}
    MAINTAINER_NAME ${pkg_extracted_maintainer_name}
    MAINTAINER_EMAIL ${pkg_extracted_maintainer_email}
    DESCRIPTION ${pkg_extracted_description}
    LICENSE_FILE ${CMAKE_CURRENT_LIST_DIR}/../LICENSE.Apache-2.0
    README_FILE ${CMAKE_CURRENT_LIST_DIR}/../README.md
    COMPONENT_DEPENDS ${SUPPORTED_COMPONENTS})

  if(UNIX AND DESCARTES_PACKAGE_SOURCE)
    cpack_debian_source_package(
      CHANGLELOG ${CMAKE_CURRENT_LIST_DIR}/CHANGELOG.rst
      UPLOAD ${DESCARTES_PACKAGE_SOURCE_UPLOAD}
      DPUT_HOST ${DESCARTES_PACKAGE_SOURCE_DPUT_HOST}
      DEBIAN_INCREMENT ${DESCARTES_PACKAGE_SOURCE_DEBIAN_INCREMENT}
      DISTRIBUTIONS ${DESCARTES_PACKAGE_SOURCE_DISTRIBUTIONS})
  endif()
endif()
