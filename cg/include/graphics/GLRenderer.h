//[]---------------------------------------------------------------[]
//|                                                                 |
//| Copyright (C) 2018, 2023 Paulo Pagliosa.                        |
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
// Last revision: 02/09/2023
// Altered by: Felipe Machado
// Altered version last revision: 13/10/2023

#ifndef __GLRenderer_h
#define __GLRenderer_h

#include "graphics/GLGraphics3.h"
#include "graphics/GLRendererBase.h"
#include <cstring>
#include <functional>
#include <memory>
#include <span>
#include <sstream>
#include <unordered_map>

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

// IMPORTANT: KEEP the data structures defined here in SYNC with the ones used
// in the shaders. Also, do not forget to use std140 layout and align the fields
// within 16 bytes.
// Prefer declaring fields with bigger storage needs earlier.
// (mat4 > mat3 > vec4 > vec3 > int)

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
  vec4f Od; // diffuse color (Phong) or base color (PBR)
  vec4f Os; // specular spot color
  float shine; // specular shininess exponent
  // PBR
  float metalness; // [0,1]
  float roughness; // [0,1]
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
  // Texture presence
  int hasEnvironmentTexture;
  int hasDiffuseTexture;
  int hasSpecularTexture;
  int hasMetalRoughTexture; // green channel: metalness; blue channel: roughness
};

enum BindingPoints
{
  LightingBlockBindingPoint = 0,
  MatrixBlockBindingPoint = 1,
  ConfigBlockBindingPoint = 2
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

  ShaderProgram(GLuint stage, const char* source)
    : ShaderProgram(stage, 1, &source) {}

  ShaderProgram(GLuint stage, uint32_t count, const char* const sources[])
    : _stage{stage}
  {
    _handle = glCreateShaderProgramv(stage, count, sources);
    glGetProgramiv(_handle, GL_LINK_STATUS, &_link);
    auto n = activeSubroutineUniformLocations();
    _subroutineUniformsCount = n;
    if (n > 0)
    {
      _subroutineUniforms = std::make_unique<GLuint[]>(n);
      memset(_subroutineUniforms.get(), 0, n * sizeof (GLuint));
    }
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

  GLuint subroutineUniformLocation(const char* name) const
  {
    return glGetSubroutineUniformLocation(_handle, _stage, name);
  }

  GLint activeSubroutineUniformLocations() const
  {
    GLint e;
    glGetProgramStageiv(_handle,
      _stage,
      GL_ACTIVE_SUBROUTINE_UNIFORM_LOCATIONS,
      &e);
    return e;
  }

  std::span<GLuint> subroutineUniforms()
  {
    return { _subroutineUniforms.get(), _subroutineUniformsCount };
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

protected:
  GLuint _stage = 0;
  GLuint _handle = 0;
  GLint _link = 0;
  uint32_t _subroutineUniformsCount = 0;
  std::unique_ptr<GLuint[]> _subroutineUniforms = nullptr;
};

} // namespace GLSL

template<std::size_t N>
struct _string8_id
{
  static_assert(N <= 8, "Your ID string must have at most 8 characters.");
  constexpr _string8_id(const char s[N+1]) noexcept
  {
    id = 0;
    for (auto i = 0u; i < N; i++)
      id |= uint64_t(s[i]) << (8*i);
  }
  uint64_t id;
};

#define REDEFINE_ID8(str8,value) template<> \
  struct _pipeline_id<str8> { static constexpr const uint64_t id = value; };

template<std::size_t N>
_string8_id(const char (&s)[N]) -> _string8_id<N-1>;

template<_string8_id S>
struct _pipeline_id { static constexpr const auto id = S.id; };

template<_string8_id S>
constexpr uint64_t operator""_ID8() { return _pipeline_id<S>::id; }

REDEFINE_ID8("Default",  0)
REDEFINE_ID8("Mesh",     0)
REDEFINE_ID8("Sphere",   0x01)
REDEFINE_ID8("Surface",  0x02)
REDEFINE_ID8("Particle", 0x03)

//////////////////////////////////////////////////////////
//
// GLRenderer: OpenGL renderer class
// ==========
class GLRenderer: public GLRendererBase, public GLGraphics3
{
public:
  using RenderFunction = std::function<void(GLRenderer&)>;
  using enum RenderMode;

  using GLGraphics3::drawAxes;
  using GLGraphics3::drawMesh;
  using GLGraphics3::drawSubMesh;

  class FragmentShader;
  class EnvironmentProgram;

  /// Constructs a GL renderer object.
  GLRenderer(SceneBase& scene, Camera& camera);

  /// Destructor.
  ~GLRenderer();
  
  void update();
  void render();

  void renderMaterial(const Material& material) final;
  // void renderTextures(GLuint diffuse, GLuint specular, GLuint metal_rough)
  bool drawMesh(const Primitive& primitive) final;

  bool drawSubMesh(const TriangleMesh& mesh,
    int count,
    int offset,
    const Material& material,
    const mat4f& t,
    const mat3f& n);

  void setRenderFunction(RenderFunction f)
  {
    _renderFunction = f;
  }
  
  void setBasePoint(const vec3f& p);
  float pixelsLength(float d) const;

  // Any negative value disables the environment texture fetching.
  void setEnvironmentTextureUnit(int texture)
  {
    _environmentTextureUnit = texture;
  }

  /**
   * An uniform buffer containing the current lights of the scene.
   */
  GLBuffer<GLSL::LightingBlock>& lightingBlock();

  /**
   * An uniform buffer containing transformation matrices.
   */
  GLBuffer<GLSL::MatrixBlock>& matrixBlock();

  /**
   * An uniform buffer containing the material and data for the fragment shader.
   */
  GLBuffer<GLSL::ConfigBlock>& configBlock();

  /**
   * The vertex shader program used in this renderer.
   * You may use it as the vertex stage in your custom pipeline.
   * 
   * @warning Be careful to match the required interface of the input/output
   * variables in the other stages of your pipeline, otherwise your pipeline
   * may end in an undefined behavior.
   * 
   * The required interface of the vertex attributes:
   * 
   * layout(location = 0) in vec4 position;
   * layout(location = 1) in vec3 normal;
   * layout(location = 2) in vec4 color;
   * 
   * The output interface:
   * 
   * layout(location = 0) out vec3 vPosition;
   * layout(location = 1) out vec3 vNormal;
   * layout(location = 2) out vec4 vColor;
   * 
   * @return GLSL::ShaderProgram* Vertex shader program.
   */
  GLSL::ShaderProgram* vertexShader();

  /**
   * The geometry shader program used in this renderer.
   * You may use it as the vertex stage in your custom pipeline.
   * 
   * @warning Be careful to match the required interface of the input/output
   * variables in the other stages of your pipeline, otherwise your pipeline
   * may end in an undefined behavior.
   * 
   * The input interface:
   * 
   * layout(location = 0) in vec3 vPosition[];
   * layout(location = 1) in vec3 vNormal[];
   * layout(location = 2) in vec4 vColor[];
   * 
   * The output interface:
   * 
   * layout(location = 0) out vec3 gPosition;
   * layout(location = 1) out vec3 gNormal;
   * layout(location = 2) out vec4 gColor;
   * layout(location = 3) noperspective out vec3 gEdgeDistance;
   * 
   * @return GLSL::ShaderProgram* Geometry shader program.
   */
  GLSL::ShaderProgram* geometryShader();

  /**
   * The fragment shader program used in this renderer.
   * You may use it as the vertex stage in your custom pipeline.
   * 
   * @warning Be careful to match the required interface of the input/output
   * variables in the other stages of your pipeline, otherwise your pipeline
   * may end in an undefined behavior.
   * 
   * The input interface:
   * 
   * layout(location = 0) in vec3 gPosition;
   * layout(location = 1) in vec3 gNormal;
   * layout(location = 2) in vec4 gColor;
   * layout(location = 3) noperspective in vec3 gEdgeDistance;
   * 
   * The required framebuffer output interface:
   * 
   * layout(location = 0) out vec4 fragmentColor;
   * 
   * Also, this shader contains two subroutines that must be set before every
   * draw call.
   * 
   * @return GLRenderer::FragmentShader* Fragment shader program.
   */
  FragmentShader* fragmentShader();

  /**
   * Helper member function to assign default values for fragment shader
   * subroutine uniforms based on current `GLRenderer` context.
   * 
   * @note Every time a bound program/pipeline is changed, the subroutine
   * uniforms' states are lost. So, if you are willing to use `GLRenderer`'s
   * fragment shader in your custom pipeline, you'll need to reassign subroutine
   * uniforms every time by calling `glUniformSubroutinesuiv` or you may use
   * this member function.
   */
  void defaultFragmentSubroutines();

  class Pipeline;
  class MeshPipeline;

  enum PipelineId : std::uint64_t
  {
    Default  = "Default"_ID8,
    Mesh     = "Mesh"_ID8,
    Sphere   = "Sphere"_ID8,
    Surface  = "Surface"_ID8,
    Particle = "Particle"_ID8
  };

  Pipeline* pipeline(uint64_t id) const;

  void setPipeline(uint64_t id, Pipeline* p);

  // TODO Remove
  EnvironmentProgram* __environmentProg() { return _environmentProgram; }

protected:
  RenderFunction _renderFunction{};
  mat4f _viewportMatrix;
  vec3f _basePoint;
  float _basePointZ;
  float _windowViewportRatio;
  int _environmentTextureUnit {-1};

  virtual void beginRender();
  virtual void endRender();
  virtual void renderActors();
  virtual void renderLights();

  void drawAxes(const mat4f&, float);
  void drawMesh(const TriangleMesh&,
    const Material&,
    const mat4f&,
    const mat3f&,
    int,
    int);

  void renderDefaultLights();

  std::unordered_map<uint64_t,Reference<Pipeline>> _pipelines {0x10};
  Reference<MeshPipeline> _gl; // Default pipeline. Listed in _pipelines[0].
  Reference<EnvironmentProgram> _environmentProgram {nullptr};

}; // GLRenderer

class GLRenderer::Pipeline : public SharedObject
{
public:
  ~Pipeline() override;

  operator GLuint () const { return _pipeline; }

  void use() const
  {
    glUseProgram(0);
    glBindProgramPipeline(_pipeline);
  }

  /**
   * @brief Callback function that should be called just before a draw call.
   * 
   * When binding a pipeline for drawing an object, call `beforeDrawing`
   * before a draw call to set some default uniform values needed by shaders.
   * As matter of fact, the reason why this function member was created is for
   * setting values for subroutine uniforms, since their state are always lost
   * when the currently bound program/pipeline is changed.
   * 
   * You may assign your own function to change the pipeline behavior.
   * 
   * @warning This variable may be null.
   */
  std::function<void(GLRenderer&)> beforeDrawing {};

protected:
  Pipeline();

  GLuint _pipeline;
};

struct SamplerData
{
  GLint location;
  GLint textureUnit;

  void set(int textureUnit)
  {
    glUniform1i(location, this->textureUnit = (GLint)textureUnit);
  }
};

class GLRenderer::FragmentShader : public GLSL::ShaderProgram
{
public:
  FragmentShader(std::initializer_list<const char*> sources);
  ~FragmentShader() override;

  const auto& mixColor() const { return _mixColor; }
  const auto& matProps() const { return _matProps; }
  const auto& shade() const { return _shade; }
  const auto& samplers() const { return _samplers; }

  void set_sEnvironment(int textureUnit)
  {
    _samplers.sEnvironment.set(textureUnit);
  }

  void set_sDiffuse(int textureUnit)
  {
    _samplers.sDiffuse.set(textureUnit);
  }

  void set_sSpecular(int textureUnit)
  {
    _samplers.sSpecular.set(textureUnit);
  }

  void set_sMetalRough(int textureUnit)
  {
    _samplers.sMetalRough.set(textureUnit);
  }

protected:
  // `mixColor` subroutine type
  struct MixColorSRType
  {
    GLuint location;
    GLuint noMixIdx;
    GLuint lineColorMixIdx;
  } _mixColor;

  // `matProps` subroutine type
  struct MatPropsSRType
  {
    GLuint location;
    GLuint colorMapMaterialIdx;
    GLuint modelMaterialIdx;
  } _matProps;

  // `shading` subroutine type
  struct ShadingSRType
  {
    GLuint location;
    GLuint phongIdx;
    GLuint pbrIdx;
  } _shade;

  struct Samplers
  {
    SamplerData sEnvironment;
    SamplerData sDiffuse;
    SamplerData sSpecular;
    SamplerData sMetalRough;
  } _samplers;
};

class GLRenderer::EnvironmentProgram : public GLSL::Program, public SharedObject
{
public:
  EnvironmentProgram();
  ~EnvironmentProgram();

  // Direction order: top left, top right, bottom left, bottom right.
  void draw(const vec3f directions[4]) const;

  void setDepth(float value)
  {
    setUniform(_uDepthLoc, value);
  }

  void set_sEnvironment(int textureUnit)
  {
    _sEnvironment.set(textureUnit);
  }

private:
  GLuint _uDepthLoc;
  SamplerData _sEnvironment;
  GLuint _vao;
  GLuint _vertexBuffer;
};

class GLRenderer::MeshPipeline : public GLRenderer::Pipeline
{
public:
  MeshPipeline();
  ~MeshPipeline() override;

  GLSL::ShaderProgram* vertex() { return _vertex; }
  GLSL::ShaderProgram* geometry() { return _geometry; }
  FragmentShader* fragment() { return _fragment; }

  GLBuffer<GLSL::LightingBlock>& lightingBlock() const
  { return *_lightingBlock; }

  GLBuffer<GLSL::MatrixBlock>& matrixBlock() const
  { return *_matrixBlock; }

  GLBuffer<GLSL::ConfigBlock>& configBlock() const
  { return *_configBlock; }

protected:
  Reference<GLSL::ShaderProgram> _vertex, _geometry;
  Reference<FragmentShader> _fragment;

  Reference<GLBuffer<GLSL::LightingBlock>> _lightingBlock;
  Reference<GLBuffer<GLSL::MatrixBlock>> _matrixBlock;
  Reference<GLBuffer<GLSL::ConfigBlock>> _configBlock;
};

inline GLBuffer<GLSL::LightingBlock>&
GLRenderer::lightingBlock() { return _gl->lightingBlock(); }

inline GLBuffer<GLSL::MatrixBlock>&
GLRenderer::matrixBlock() { return _gl->matrixBlock(); }

inline GLBuffer<GLSL::ConfigBlock>&
GLRenderer::configBlock() { return _gl->configBlock(); }

inline GLSL::ShaderProgram*
GLRenderer::vertexShader() { return _gl->vertex(); }

inline GLSL::ShaderProgram*
GLRenderer::geometryShader() { return _gl->geometry(); }

inline GLRenderer::FragmentShader*
GLRenderer::fragmentShader() { return _gl->fragment(); }

} // end namespace cg

#endif // __GLRenderer_h
