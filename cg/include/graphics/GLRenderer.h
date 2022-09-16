//[]---------------------------------------------------------------[]
//|                                                                 |
//| Copyright (C) 2018, 2022 Paulo Pagliosa.                        |
//|                                                                 |
//| This software is provided 'as-is', without any express or       |
//| implied warranty. In no event will the authors be held liable   |
//| for any damages arising from the use of this software.          |
//|                                                                 |
//| Permission is granted to anyone to use this software for any    |
//| purpose, including commercial applications, and to alter it and |
//| redistribute it freely, subject to the following restrictions:  |
//|                                                                 |
//| 1. The origin of this software must not be misrepresented; you  |
//| must not claim that you wrote the original software. If you use |
//| this software in a product, an acknowledgment in the product    |
//| documentation would be appreciated but is not required.         |
//|                                                                 |
//| 2. Altered source versions must be plainly marked as such, and  |
//| must not be misrepresented as being the original software.      |
//|                                                                 |
//| 3. This notice may not be removed or altered from any source    |
//| distribution.                                                   |
//|                                                                 |
//[]---------------------------------------------------------------[]
//
// OVERVIEW: GLRenderer.h
// ========
// Class definition for OpenGL Renderer.
//
// Author: Paulo Pagliosa
// Modified by: Felipe Machado
// Last revision: 16/09/2022

#ifndef __GLRenderer_h
#define __GLRenderer_h

#include "graphics/GLGraphics3.h"
#include "graphics/GLRendererBase.h"

#include <sstream>

namespace cg
{ // begin namespace cg

namespace GLSL
{

struct alignas(16) mat3
{
  vec3f x;
  float _x;
  vec3f y;
  float _y;
  vec3f z;
  float _z;

  mat3() = default;
  mat3(mat3&&) = default;
  mat3(const mat3&) = default;

  mat3(const mat3f& m) { x = m[0]; y = m[1]; z = m[2]; }
  operator mat3f () const { return mat3f(x, y, z); }

  mat3& operator = (mat3&&) = default;
  mat3& operator = (const mat3&) = default;
};

// IMPORTANT: KEEP the data structures defined here in SYNC with the ones used in the shaders.
// Also, do not forget to use std140 layout and align the fields within 16 bytes.
// Prefer declaring fields with bigger storage needs earlier. (mat4 > mat3 > vec4 > vec3 > int)

struct alignas(16) LightProps
{
  vec4f color; // color
  alignas(16) vec3f position; // VRC position
  alignas(16) vec3f direction; // VRC direction
  int type; // DIRECTIONAL/POINT/SPOT
  int falloff; // CONSTANT/LINEAR/QUADRATIC
  float range; // range (== 0 INFINITE)
  float angle; // spot angle
};

struct alignas(16) MaterialProps
{
  vec4f Oa; // ambient color
  vec4f Od; // diffuse color
  vec4f Os; // specular spot color
  float shine; // specular shininess exponent
};

struct alignas(16) LineProps
{
  vec4f color;
  float width;
};

struct LightingBlock
{
  static constexpr auto MAX_LIGHTS = 8U;

  LightProps lights[MAX_LIGHTS];
  alignas(16) vec4f ambientLight;
  int lightCount;
};

struct MatrixBlock
{
  mat4f viewportMatrix;
  mat4f mvMatrix;
  mat4f mvpMatrix;
  mat3 normalMatrix;
};

struct ConfigBlock
{
  MaterialProps material;
  LineProps line;
  int projectionType; // PERSPECTIVE/PARALLEL
};

enum BindingLocations
{
  LightingBlockBindingLoc = 0,
  MatrixBlockBindingLoc = 1,
  ConfigBlockBindingLoc = 2
};

extern const char* VERSION;
extern const char* PROPS_DECLARATIONS;
extern const char* LIGHTING_BLOCK_DECLARATION;
extern const char* MATRIX_BLOCK_DECLARATION;
extern const char* CONFIG_BLOCK_DECLARATION;

class ShaderProgram : public SharedObject
{
public:
  ShaderProgram() = default;

  ShaderProgram(GLuint stage, const char* source) : ShaderProgram(stage, 1, &source) {}

  ShaderProgram(GLuint stage, uint32_t count, const char* const sources[])
    : _stage{stage}
  {
    _handle = glCreateShaderProgramv(stage, count, sources);
    glGetProgramiv(_handle, GL_LINK_STATUS, &_link);
  }

  ShaderProgram(GLuint stage, std::initializer_list<const char*> sources)
    : ShaderProgram(stage, sources.size(), sources.begin()) {}

  ShaderProgram(const ShaderProgram&) = delete;
  ShaderProgram& operator = (const ShaderProgram&) = delete;

  // ShaderProgram(ShaderProgram&&);
  // ShaderProgram& operator = (ShaderProgram&&);

  ~ShaderProgram();

  GLuint stage() const { return _stage; }
  operator GLuint () const { return _handle; }
  operator bool () const { return ok(); }
  bool ok() const { return _handle != 0 && _link == GL_TRUE; }

  GLuint subroutineIndex(const char* name) const
  {
    return glGetSubroutineIndex(_handle, _stage, name);
  }

  std::string infoLog() const
  {
    std::string log;
    GLint length;
    glGetProgramiv(_handle, GL_INFO_LOG_LENGTH, &length);
    
    log.resize(length);
    glGetProgramInfoLog(_handle, log.size(), &length, log.data());

    return log;
  }

private:
  GLuint _stage = 0;
  GLuint _handle = 0;
  GLint _link = 0;
};

} // namespace GLSL

//////////////////////////////////////////////////////////
//
// GLRenderer: OpenGL renderer class
// ==========
class GLRenderer: public GLRendererBase, public GLGraphics3
{
public:
  using RenderFunction = void (*)(GLRenderer&);
  using GLGraphics3::drawMesh;

  /// Constructs a GL renderer object.
  GLRenderer(SceneBase& scene, Camera& camera);

  /// Destructor.
  ~GLRenderer();

  using GLGraphics3::drawAxes;
  
  void update();
  void render();
  bool drawMesh(const Primitive& primitive) final;
  float pixelsLength(float d) const;
  void renderMaterial(const Material& material) final;
  void setBasePoint(const vec3f& p);

  void setRenderFunction(RenderFunction f)
  {
    _renderFunction = f;
  }

  GLBuffer<GLSL::LightingBlock>& lightingBlock() { return *_gl.lightingBlock; }

protected:
  RenderFunction _renderFunction{};
  vec3f _basePoint;
  float _basePointZ;
  float _windowViewportRatio;

  virtual void beginRender();
  virtual void endRender();
  virtual void renderActors();
  virtual void renderLights();

  void drawAxes(const mat4f&, float);

  void renderDefaultLights();

  struct GLData
  {
    GLuint pipeline; // Default pipeline
    Reference<GLSL::ShaderProgram> vertex, geometry, fragment;

    Reference<GLBuffer<GLSL::LightingBlock>> lightingBlock;
    Reference<GLBuffer<GLSL::MatrixBlock>> matrixBlock;
    Reference<GLBuffer<GLSL::ConfigBlock>> configBlock;

    mat4f viewportMatrix;

    GLuint noMixIdx;
    GLuint lineColorMixIdx;
    GLuint modelMaterialIdx;
    GLuint colorMapMaterialIdx;
  } _gl;

}; // GLRenderer

} // end namespace cg

#endif // __GLRenderer_h
