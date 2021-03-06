cmake_minimum_required(VERSION 2.8)
project(argos3_examples)

# Set the path where CMake will find additional scripts
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

# Find the ARGoS package
find_package(PkgConfig)
pkg_check_modules(ARGOS REQUIRED argos3_simulator)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ARGOS_PREFIX}/share/argos3/cmake)
message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")

# Check whether all the necessary libs have been installed to compile the
# code that depends on Qt and OpenGL
# include(ARGoSCheckQTOpenGL)

# Look for the (optional) galib library
find_package(GALIB)
if(GALIB_FOUND)
  include_directories(${GALIB_INCLUDE_DIRS})
endif(GALIB_FOUND)

# Find the GSL package
find_package(GSL REQUIRED)

# Find Lua
find_package(Lua52 REQUIRED)

# Set ARGoS include dir
include_directories(${CMAKE_SOURCE_DIR} ${ARGOS_INCLUDE_DIRS} ${GSL_INCLUDE_DIR} ${LUA_INCLUDE_DIR})

# Set ARGoS link dir
link_directories(${ARGOS_LIBRARY_DIRS})

# Descend into the controllers directory
add_subdirectory(controllers)

# Descend into the loop_functions directory
add_subdirectory(loop_functions)

# Set flag for Release compilation
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} - Wall")

# Set flag for Debug compilation
#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} - Wall")
