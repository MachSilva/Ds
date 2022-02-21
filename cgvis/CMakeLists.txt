cmake_minimum_required(VERSION 3.16)

project(cgvis VERSION 1.0)

add_library(${PROJECT_NAME} STATIC
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

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
)

target_link_libraries(${PROJECT_NAME} cg)
