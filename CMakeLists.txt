cmake_minimum_required(VERSION 3.18)

project(Ds VERSION 1.1 LANGUAGES CXX)

option(USE_CUDA "Build projects with CUDA support" OFF)

message(STATUS "CUDA support: " ${USE_CUDA})

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CUDA_STANDARD 20)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)

if(UNIX)
  set(CMAKE_EXECUTABLE_SUFFIX ".out")
endif()

if(USE_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
  set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)
  set(CMAKE_POSITION_INDEPENDENT_CODE ON)
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
