cmake_minimum_required(VERSION 3.10)

project(cgcuda)

set(CGCUDA_SOURCES src/CUDAHelper.cpp)

add_library(${PROJECT_NAME} STATIC ${CGCUDA_SOURCES})

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
)

target_link_libraries(cgcuda cg)
