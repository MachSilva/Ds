add_library(cgcuda STATIC
    src/CUDAHelper.cpp
)

target_include_directories(cgcuda PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
)

target_link_libraries(cgcuda cg)
