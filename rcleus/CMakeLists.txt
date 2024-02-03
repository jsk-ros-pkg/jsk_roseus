cmake_minimum_required(VERSION 3.8)
project(rcleus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Check EusLisp exists
set(ENV_EUSDIR $ENV{EUSDIR})
if(ENV_EUSDIR)
  set(EUSDIR $ENV_EUSDIR)
else()
  set(EUSDIR /usr/share/euslisp) # for debian/ubuntu
endif()
if(EXISTS ${EUSDIR})
  message(STATUS "Found EusLisp: ${EUSDIR}")
else()
  message(FATAL_ERROR "EusLisp not found")
endif()

# Set EusLisp include directory
set(euslisp_INCLUDE_DIRS ${EUSDIR}/include)

include_directories(include)

ament_auto_add_library(rcleus SHARED
  src/rcleus.cpp)
target_compile_options(rcleus
  PRIVATE -O2 -DNDEBUG -DLinux -D_REENTRANT -DTHREADED -DPTHREAD -DX11R6_1 -Dx86_64 -Wno-comment)
target_include_directories(rcleus
  PUBLIC ${euslisp_INCLUDE_DIRS})

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
