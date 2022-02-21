cmake_minimum_required(VERSION 3.16)

project(cgdemo VERSION 0.1 LANGUAGES CXX)

add_executable(cgdemo
    Main.cpp
    MainWindow.cpp
    RayTracer.cpp
    reader/AbstractParser.cpp
    reader/Buffer.cpp
    reader/ErrorHandler.cpp
    reader/Expression.cpp
    reader/FileBuffer.cpp
    reader/ReaderBase.cpp
    reader/SceneReader.cpp
    reader/Scope.cpp
)

target_link_libraries(cgdemo cg)
