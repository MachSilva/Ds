cmake_minimum_required(VERSION 3.16)

project(cgvisdemo VERSION 0.1 LANGUAGES CXX)

add_executable(cgvisdemo
    Demo.cpp
    Main.cpp
    MainWindow.cpp
)

target_link_libraries(cgvisdemo cgvis)

# Symlink assets directory
execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink 
        ${CMAKE_CURRENT_LIST_DIR}/assets ${CMAKE_CURRENT_BINARY_DIR}/assets
    COMMAND ${CMAKE_COMMAND} -E create_symlink 
        ${CMAKE_CURRENT_LIST_DIR}/demo.txt ${CMAKE_CURRENT_BINARY_DIR}/demo.txt
)
