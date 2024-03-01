add_library(cgvis STATIC
    src/TriCellMesh.cpp
    src/CylinderSource.cpp
    src/PolyMeshBuilder.cpp
    src/SceneWindow.cpp
    src/TriCellMeshMapper.cpp
    src/ConeSource.cpp
    src/BoxSource.cpp
    src/TriCell.cpp
    src/Mapper.cpp
    src/Scene.cpp
    src/SphereSource.cpp
    src/Transform.cpp
    src/Glyph3.cpp
    src/LookupTable.cpp
    src/TriCellMeshReader.cpp
    src/Scalars.cpp
    src/TriCellMeshNormals.cpp
    src/PolyMeshMapper.cpp
    src/PolyData.cpp
    src/PolyMesh.cpp
    src/Source.cpp
    src/PolyDataMapper.cpp
)

target_include_directories(cgvis
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
)

target_link_libraries(cgvis cg)

set_target_properties(cgvis PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)
