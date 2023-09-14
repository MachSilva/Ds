cmake_minimum_required(VERSION 3.16)

option(USE_CUDA "Build projects with CUDA support" OFF)

set(DS_LANGUAGES CXX)
if(${USE_CUDA})
  set(DS_LANGUAGES ${DS_LANGUAGES} CUDA)
endif()

message(STATUS "CUDA support: " ${USE_CUDA})

project(Ds VERSION 1.1 LANGUAGES ${DS_LANGUAGES})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)

if(UNIX)
  set(CMAKE_EXECUTABLE_SUFFIX ".out")
endif()

if(USE_CUDA)
  add_definitions(-D_USE_CUDA)
  include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
endif()

# Projects where CUDA is optional
add_subdirectory(cg)
add_subdirectory(cgvis)
add_subdirectory(apps/cgdemo)
add_subdirectory(apps/cgvisdemo)
add_subdirectory(apps/template)

if(USE_CUDA)
  # CUDA projects
  add_subdirectory(cgcuda)
else()
  # Non CUDA projects (or incompatible with CUDA)
endif()
