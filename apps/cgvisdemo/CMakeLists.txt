add_executable(cgvisdemo
    Demo.cpp
    Main.cpp
    MainWindow.cpp
)

target_link_libraries(cgvisdemo cgvis)

set_target_properties(cgvisdemo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)
