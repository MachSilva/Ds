cmake_minimum_required(VERSION 3.16)

project(template VERSION 0.1 LANGUAGES CXX)

add_executable(template
    Main.cpp
    MainWindow.cpp
)

target_link_libraries(template cg)

set_target_properties(template PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)
