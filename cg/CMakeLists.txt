cmake_minimum_required(VERSION 3.16)

project(cg VERSION 1.0)

if(POLICY CMP0072)
    cmake_policy(SET CMP0072 NEW) # Use vendor neutral OpenGL library.
endif()

add_library(${PROJECT_NAME} STATIC
    externals/src/gl3w.c
    externals/src/imgui_draw.cpp
    externals/src/imgui_impl_glfw.cpp
    externals/src/imgui_impl_opengl3.cpp
    externals/src/imgui_tables.cpp
    externals/src/imgui_widgets.cpp
    externals/src/imgui.cpp
    src/core/BlockAllocator.cpp
    src/core/Exception.cpp
    src/core/NameableObject.cpp
    src/debug/AnimatedAlgorithm.cpp
    src/geometry/BVH.cpp
    src/geometry/MeshSweeper.cpp
    src/geometry/TriangleMesh.cpp
    src/geometry/TriangleMeshBVH.cpp
    src/graph/CameraProxy.cpp
    src/graph/Component.cpp
    src/graph/LightProxy.cpp
    src/graph/PrimitiveProxy.cpp
    src/graph/SceneObject.cpp
    src/graph/SceneObjectBuilder.cpp
    src/graph/SceneWindow.cpp
    src/graph/Transform.cpp
    src/graphics/Application.cpp
    src/graphics/Assets.cpp
    src/graphics/AssetFolder.cpp
    src/graphics/Camera.cpp
    src/graphics/Color.cpp
    src/graphics/GLFramebuffer.cpp
    src/graphics/GLGraphics2.cpp
    src/graphics/GLGraphics3.cpp
    src/graphics/GLGraphicsBase.cpp
    src/graphics/GLImage.cpp
    src/graphics/GLMesh.cpp
    src/graphics/GLMeshRenderer.cpp
    src/graphics/GLProgram.cpp
    src/graphics/GLRenderer.cpp
    src/graphics/GLRendererBase.cpp
    src/graphics/GLRenderWindow2.cpp
    src/graphics/GLRenderWindow3.cpp
    src/graphics/GLTextureFramebuffer.cpp
    src/graphics/GLWindow.cpp
    src/graphics/Image.cpp
    src/graphics/Light.cpp
    src/graphics/Primitive.cpp
    src/graphics/PrimitiveBVH.cpp
    src/graphics/PrimitiveMapper.cpp
    src/graphics/Renderer.cpp
    src/graphics/SceneEditor.cpp
    src/graphics/SceneWindowBase.cpp
    src/graphics/Shape.cpp
    src/graphics/TransformableObject.cpp
    src/graphics/TriangleMeshMapper.cpp
    src/graphics/TriangleMeshShape.cpp
    src/utils/MeshReader.cpp
)

find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})

if(MSVC)
    find_library(glfw3_LIB glfw3 ${CMAKE_CURRENT_LIST_DIR}/externals/lib)
    target_link_libraries(${PROJECT_NAME} ${glfw3_LIB})
else()
    add_compile_options(-Wno-narrowing)
    find_package(glfw3 REQUIRED)
    include_directories(${glfw3_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} dl glfw)
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${PROJECT_SOURCE_DIR}/include/
        ${PROJECT_SOURCE_DIR}/externals/include/
)

target_link_libraries(${PROJECT_NAME} OpenGL::GL)
