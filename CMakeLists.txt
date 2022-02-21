cmake_minimum_required(VERSION 3.16)

option(USE_CUDA "Build projects with CUDA support" OFF)

set(GPGPU_LANGUAGES CXX)
if (${USE_CUDA})
  set(GPGPU_LANGUAGES ${GPGPU_LANGUAGES} CUDA)
endif()

message(STATUS "CUDA support: " ${USE_CUDA})

project(gpgpu VERSION 1.0 LANGUAGES ${GPGPU_LANGUAGES})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CUDA_STANDARD 17)
set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)

if(WIN32)
  set(CMAKE_SHARED_LIBRARY_PREFIX "")
endif()

if(USE_CUDA)
  add_definitions(-D_USE_CUDA)
  include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})
endif()

add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GL3W)

# Projects where CUDA is optional
add_subdirectory(cg)
add_subdirectory(cgvis)
add_subdirectory(apps/cgdemo)
add_subdirectory(apps/cgvisdemo)

if(USE_CUDA)
  # CUDA projects
  add_subdirectory(cgcuda)
else()
  # Non CUDA projects (or incompatible with CUDA)
endif()
