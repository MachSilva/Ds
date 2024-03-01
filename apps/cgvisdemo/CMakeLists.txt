add_executable(cgvisdemo
    Demo.cpp
    Main.cpp
    MainWindow.cpp
)

target_link_libraries(cgvisdemo cgvis)

set_target_properties(cgdemo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)

# Symlink assets directory
#execute_process(
#    COMMAND ${CMAKE_COMMAND} -E create_symlink 
#        ${CMAKE_CURRENT_LIST_DIR}/assets ${CMAKE_CURRENT_BINARY_DIR}/assets
#    COMMAND ${CMAKE_COMMAND} -E create_symlink 
#        ${CMAKE_CURRENT_LIST_DIR}/demo.txt ${CMAKE_CURRENT_BINARY_DIR}/demo.txt
#)
