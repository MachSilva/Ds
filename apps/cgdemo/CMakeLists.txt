add_executable(cgdemo
    Main.cpp
    MainWindow.cpp
    RayTracer.cpp
    SceneWriter.cpp
    Writer.cpp
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

set_target_properties(cgdemo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)
