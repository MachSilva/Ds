cmake_minimum_required(VERSION 3.16)

project(cgvisdemo VERSION 0.1 LANGUAGES CXX)

add_executable(cgvisdemo
    Demo.cpp
    Main.cpp
    MainWindow.cpp
)

target_link_libraries(cgvisdemo cgvis)
