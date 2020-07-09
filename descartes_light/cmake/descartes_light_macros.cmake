#
# @file descartes_light_macros.cmake
# @brief Common Tesseract CMake Macros
#
# @author Levi Armstrong
# @date October 15, 2019
# @version TODO
# @bug No known bugs
#
# @copyright Copyright (c) 2019, Southwest Research Institute
#
# @par License
# Software License Agreement (Apache License)
# @par
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# @par
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# This macro will add tesseract standard compiler option to a target
# Usage: descartes_target_compile_options(target <INTERFACE|PUBLIC|PRIVATE>)
#    * c++11
#    * Warning (-Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
#    * disable avx due to eigen issues
macro(descartes_target_compile_options target)
  cmake_parse_arguments(ARG "INTERFACE;PUBLIC;PRIVATE" "" "" ${ARGN})

  if(ARG_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR "tesseract_target_compile_options() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
  endif()

  if (ARG_INTERFACE)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	target_compile_options(${target} INTERFACE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
      else()
	target_compile_options(${target} INTERFACE -Wall -Wextra -Winconsistent-missing-override -Wconversion -Wsign-conversion)
      endif()
      target_compile_options(${target} INTERFACE -mno-avx)
      target_compile_definitions(${target} INTERFACE DESCARTES_BUILDING_LIBRARY=ON)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      target_compile_definitions(${target} INTERFACE _USE_MATH_DEFINES=ON DESCARTES_BUILDING_LIBRARY=ON)
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PUBLIC)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	target_compile_options(${target} PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
      else()
	target_compile_options(${target} PRIVATE -Wall -Wextra -Winconsistent-missing-override -Wconversion -Wsign-conversion)
      endif()
      target_compile_options(${target} PUBLIC -mno-avx)
      target_compile_definitions(${target} PRIVATE DESCARTES_BUILDING_LIBRARY=ON)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      target_compile_definitions(${target} PUBLIC _USE_MATH_DEFINES=ON)
      target_compile_definitions(${target} PRIVATE DESCARTES_BUILDING_LIBRARY=ON)
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  elseif(ARG_PRIVATE)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
      if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	target_compile_options(${target} PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
      else()
	target_compile_options(${target} PRIVATE -Wall -Wextra -Winconsistent-missing-override -Wconversion -Wsign-conversion)
      endif()
      target_compile_options(${target} PRIVATE -mno-avx)
      target_compile_definitions(${target} PRIVATE DESCARTES_BUILDING_LIBRARY=ON)
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
      target_compile_definitions(${target} PRIVATE _USE_MATH_DEFINES=ON DESCARTES_BUILDING_LIBRARY=ON)
    else()
      message(WARNING "${CMAKE_CXX_COMPILER_ID} compiler detected. If using AVX instructions, Eigen alignment issues may result.")
    endif()
  endif()
endmacro()
