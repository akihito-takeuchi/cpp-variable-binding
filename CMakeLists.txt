#
# DINO project
#
# Root CMake file
#

project(cpp-variable-binding)

cmake_minimum_required(VERSION 3.12)

set(CMAKE_CXX_FLAGS "-Wall -W")

set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Setup boost
find_package(Boost 1.67 REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Setup fmtlib
set(BUILD_SHARED_LIBS true)
set(FMT_ROOT "${CMAKE_CURRENT_LIST_DIR}/third_party/fmt-5.2.1")
add_subdirectory(${FMT_ROOT})
include_directories(${FMT_ROOT}/include)

# vbd
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_subdirectory(vbd)

# For test
set(with-test true)

# unit tests
if(with-test)
  enable_testing()
  include_directories("${GTEST_ROOT}/include")
  add_subdirectory(tests)
endif(with-test)
