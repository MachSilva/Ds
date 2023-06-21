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
// OVERVIEW: GLRenderer.cpp
// ========
// Source file for OpenGL renderer.
//
// Author: Paulo Pagliosa
// Modified by: Felipe Machado
// Last revision: 30/03/2023

#include "graphics/GLRenderer.h"

#define STRINGIFY(A) #A

namespace cg
{ // begin namespace cg

namespace GLSL
{

ShaderProgram::~ShaderProgram()
{
  // virtual destructor
}

const char* VERSION = "#version 430 core\n";

const char* PROPS_DECLARATIONS = STRINGIFY(
  struct LightProps
  {
    vec4 color; // color
    vec3 position; // VRC position
    vec3 direction; // VRC direction
    int type; // DIRECTIONAL/POINT/SPOT
    int falloff; // CONSTANT/LINEAR/QUADRATIC
    float range; // range (== 0 INFINITE)
    float angle; // spot angle
  };

  struct MaterialProps
  {
    vec4 Oa; // ambient color
    vec4 Od; // diffuse color
    vec4 Os; // specular spot color
    float shine; // specular shininess exponent
  };

  struct LineProps
  {
    vec4 color;
    float width;
  };
);

// Depends on PROPS_DECLARATIONS.
const char* LIGHTING_BLOCK_DECLARATION = STRINGIFY(
  layout(binding = 0, std140) uniform LightingBlock
  {
    LightProps lights[8];
    vec4 ambientLight;
    int lightCount;
  };
);

const char* MATRIX_BLOCK_DECLARATION = STRINGIFY(
  layout(binding = 1, std140) uniform MatrixBlock
  {
    mat4 viewportMatrix;
    mat4 mvMatrix;
    mat4 mvpMatrix;
    mat3 normalMatrix;
  };
);

// Depends on PROPS_DECLARATIONS.
const char* CONFIG_BLOCK_DECLARATION = STRINGIFY(
  layout(binding = 2, std140) uniform ConfigBlock
  {
    MaterialProps material;
    LineProps line;
    int projectionType; // PERSPECTIVE/PARALLEL
    //vec4 backFaceColor = vec4(1, 0, 1, 1);
  };
);

} // namespace GLSL

static const char* gVertexShader = STRINGIFY(
  layout(location = 0) in vec4 position;
  layout(location = 1) in vec3 normal;
  layout(location = 2) in vec4 color;

  layout(location = 0) out vec3 vPosition;
  layout(location = 1) out vec3 vNormal;
  layout(location = 2) out vec4 vColor;

  void main()
  {
    vPosition = vec3(mvMatrix * position);
    vNormal = normalize(normalMatrix * normal);
    gl_Position = mvpMatrix * position;
    vColor = color;
  }
);

static const char* gGeometryShader = STRINGIFY(
  layout(triangles) in;
  layout(triangle_strip, max_vertices = 3) out;

  layout(location = 0) in vec3 vPosition[];
  layout(location = 1) in vec3 vNormal[];
  layout(location = 2) in vec4 vColor[];

  layout(location = 0) out vec3 gPosition;
  layout(location = 1) out vec3 gNormal;
  layout(location = 2) out vec4 gColor;
  layout(location = 3) noperspective out vec3 gEdgeDistance;

  void main()
  {
    vec2 p0 = vec2(viewportMatrix *
      (gl_in[0].gl_Position / gl_in[0].gl_Position.w));
    vec2 p1 = vec2(viewportMatrix *
      (gl_in[1].gl_Position / gl_in[1].gl_Position.w));
    vec2 p2 = vec2(viewportMatrix *
      (gl_in[2].gl_Position / gl_in[2].gl_Position.w));
    float a = length(p1 - p2);
    float b = length(p2 - p0);
    float c = length(p1 - p0);
    float alpha = acos((b * b + c * c - a * a) / (2 * b * c));
    float delta = acos((a * a + c * c - b * b) / (2 * a * c));
    float ha = abs(c * sin(delta));
    float hb = abs(c * sin(alpha));
    float hc = abs(b * sin(alpha));

    gEdgeDistance = vec3(ha, 0, 0);
    gPosition = vPosition[0];
    gNormal = vNormal[0];
    gColor = vColor[0];
    gl_Position = gl_in[0].gl_Position;
    EmitVertex();
    gEdgeDistance = vec3(0, hb, 0);
    gPosition = vPosition[1];
    gNormal = vNormal[1];
    gColor = vColor[1];
    gl_Position = gl_in[1].gl_Position;
    EmitVertex();
    gEdgeDistance = vec3(0, 0, hc);
    gPosition = vPosition[2];
    gNormal = vNormal[2];
    gColor = vColor[2];
    gl_Position = gl_in[2].gl_Position;
    EmitVertex();
    EndPrimitive();
  }
);

static const char* gFragmentShader = STRINGIFY(
  subroutine vec4 mixColorType(vec4 color);
  subroutine void matPropsType(out MaterialProps m);

  layout(location = 0) in vec3 gPosition;
  layout(location = 1) in vec3 gNormal;
  layout(location = 2) in vec4 gColor;
  layout(location = 3) noperspective in vec3 gEdgeDistance;

  subroutine uniform mixColorType mixColor;
  subroutine uniform matPropsType matProps;  

  layout(location = 0) out vec4 fragmentColor;

  subroutine(matPropsType)
  void modelMaterial(out MaterialProps m)
  {
    m = material;
  }

  subroutine(matPropsType)
  void colorMapMaterial(out MaterialProps m)
  {
    const float Oa = 0.4;
    const float Od = 0.6;

    m = MaterialProps(gColor * Oa, gColor * Od, vec4(1), material.shine);
  }

  bool lightVector(int i, vec3 P, out vec3 L, out float d)
  {
    int type = lights[i].type;

    // DIRECTIONAL
    if (type == 0)
    {
      L = -lights[i].direction;
      return true;
    }
    L = lights[i].position - P;
    d = length(L);

    float range = lights[i].range;

    if (d == 0 || (range > 0 && d > range))
      return false;
    L /= d;
    // POINT
    if (type == 1)
      return true;

    // SPOT
    float DL = dot(lights[i].direction, L);

    return DL < 0 && lights[i].angle > radians(acos(DL));
  }

  vec4 lightColor(int i, float d)
  {
    int falloff = lights[i].falloff;

    // directional light or constant falloff
    if (lights[i].type == 0  || falloff == 0)
      return lights[i].color;

    float range = lights[i].range;
    float f;

    if (range == 0) // infinite range
    {
      f = 1 / d;
      if (falloff == 2) // quadratic falloff
        f *= f;
    }
    else
    {
      f = d / range;
      f = falloff == 2 ? 1 + f * (f - 2) : 1 - f;
    }
    return lights[i].color * f;
  }

  vec4 phong(vec3 P, vec3 N)
  {
    MaterialProps m;
    vec4 color;

    matProps(m);
    color = ambientLight * m.Oa;

    vec3 V = projectionType == 0 ?
      // PERSPECTIVE
      normalize(P) :
      // PARALLEL
      vec3(0, 0, -1);

    if (dot(N, V) > 0)
      //return backFaceColor;
      N *= -1;

    vec3 R = reflect(V, N);

    for (int i = 0; i < lightCount; i++)
    {
      vec3 L; float d;
 
      if (lightVector(i, P, L, d))
      {
        vec4 I = lightColor(i, d);

        color += I * m.Od * max(dot(N, L), 0);
        color += I * m.Os * pow(max(dot(R, L), 0), m.shine);
      }
    }
    return min(color, vec4(1));
  }

  subroutine(mixColorType)
  vec4 noMix(vec4 color)
  {
    return color;
  }

  subroutine(mixColorType)
  vec4 lineColorMix(vec4 color)
  {
    float d = min(min(gEdgeDistance.x, gEdgeDistance.y), gEdgeDistance.z);
    float mixVal;

    if (d < line.width - 1)
      mixVal = 1;
    else if (d > line.width + 1)
      mixVal = 0;
    else
    {
      float x = d - (line.width - 1);
      mixVal = exp2(-2 * (x * x));
    }
    return mix(color, line.color, mixVal);
  }

  void main()
  {
    fragmentColor = mixColor(phong(gPosition, normalize(gNormal)));
  }
);

GLRenderer::Pipeline::Pipeline()
{
  glCreateProgramPipelines(1, &_pipeline);
}

GLRenderer::Pipeline::~Pipeline()
{
  glDeleteProgramPipelines(1, &_pipeline);
}

GLRenderer::MeshPipeline::MeshPipeline()
{
  // Create shader programs
  _vertex = new GLSL::ShaderProgram(GL_VERTEX_SHADER, {
    GLSL::VERSION,
    GLSL::MATRIX_BLOCK_DECLARATION,
    gVertexShader
  });

  if (!_vertex->ok())
    throw new std::runtime_error("failed to create vertex shader program"
      + _vertex->infoLog());

  _geometry = new GLSL::ShaderProgram(GL_GEOMETRY_SHADER, {
    GLSL::VERSION,
    GLSL::MATRIX_BLOCK_DECLARATION,
    gGeometryShader
  });

  if (!_geometry->ok())
    throw new std::runtime_error("failed to create geometry shader program\n"
      + _geometry->infoLog());

  _fragment = new GLSL::ShaderProgram(GL_FRAGMENT_SHADER, {
    GLSL::VERSION,
    GLSL::PROPS_DECLARATIONS,
    GLSL::LIGHTING_BLOCK_DECLARATION,
    GLSL::CONFIG_BLOCK_DECLARATION,
    gFragmentShader
  });

  if (!_fragment->ok())
    throw new std::runtime_error("failed to create fragment shader program"
      + _fragment->infoLog());

  _noMixIdx = _fragment->subroutineIndex("noMix");
  _lineColorMixIdx = _fragment->subroutineIndex("lineColorMix");
  _modelMaterialIdx = _fragment->subroutineIndex("modelMaterial");
  _colorMapMaterialIdx = _fragment->subroutineIndex("colorMapMaterial");

  // Create uniform buffers
  _lightingBlock = new GLBuffer<GLSL::LightingBlock>(1, GL_UNIFORM_BUFFER);
  _matrixBlock = new GLBuffer<GLSL::MatrixBlock>(1, GL_UNIFORM_BUFFER);
  _configBlock = new GLBuffer<GLSL::ConfigBlock>(1, GL_UNIFORM_BUFFER);

  // Create pipeline
  glUseProgramStages(_pipeline, GL_VERTEX_SHADER_BIT, *_vertex);
  glUseProgramStages(_pipeline, GL_GEOMETRY_SHADER_BIT, *_geometry);
  glUseProgramStages(_pipeline, GL_FRAGMENT_SHADER_BIT, *_fragment);

  // Default target for glUniform* calls, including uniform subroutines
  glActiveShaderProgram(_pipeline, *_fragment);
}

GLRenderer::MeshPipeline::~MeshPipeline() { /* RAII */ }

GLRenderer::GLRenderer(SceneBase& scene, Camera& camera):
  GLRendererBase{scene, camera}
{
  _pipelines[0] = _gl = new MeshPipeline();
}

GLRenderer::~GLRenderer() {}

inline void
GLRenderer::renderDefaultLights()
{
  GLSL::LightingBlock b;

  b.ambientLight = _scene->ambientLight;
  b.lights[0].type = 1; // POINT
  b.lights[0].color = vec4f{1, 1, 1, 0};
  b.lights[0].position = vec3f{0, 0, 0};
  b.lights[0].range = 0.0f;
  b.lightCount = 1;

  glNamedBufferSubData(
    *_gl->lightingBlock(),
    0,
    sizeof (GLSL::LightingBlock),
    (void*) &b
  );
  glBindBufferBase(GL_UNIFORM_BUFFER, GLSL::LightingBlockBindingPoint, GLuint(*_gl->lightingBlock()));
}

void
GLRenderer::update()
{
  // TODO: do it only when camera or viewport has been changed
  GLGraphics3::setView(_camera->position(), vpMatrix(_camera));
  glViewport(0, 0, _viewport.w, _viewport.h);

  auto w = _viewport.w / 2.0f;
  auto h = _viewport.h / 2.0f;

  _viewportMatrix[0].set(w, 0, 0, 0);
  _viewportMatrix[1].set(0, h, 0, 0);
  _viewportMatrix[2].set(0, 0, 1, 0);
  _viewportMatrix[3].set(w, h, 0, 0);
  _windowViewportRatio = _camera->windowHeight() / _viewport.h;
}

void
GLRenderer::setBasePoint(const vec3f& p)
{
  _basePointZ = -_camera->worldToCamera(_basePoint = p).z;
}

float
GLRenderer::pixelsLength(float d) const
{
  if (_camera->projectionType() == Camera::Perspective)
    d *= _basePointZ / _camera->nearPlane();
  return _windowViewportRatio * d;
}

void
GLRenderer::renderLights()
{
  if (_scene->lightCount() == 0)
  {
    renderDefaultLights();
    return;
  }

  auto block = static_cast<GLSL::LightingBlock*>(
    glMapNamedBuffer(*_gl->lightingBlock(), GL_WRITE_ONLY));

  block->ambientLight = _scene->ambientLight;

  const auto& vm = _camera->worldToCameraMatrix();
  int nl{0};

  for (auto& light : _scene->lights())
  {
    if (!light->isTurnedOn())
      continue;
    block->lights[nl].type = (int)light->type();
    block->lights[nl].color = light->color;
    {
      const auto p = vm.transform3x4(light->position());
      block->lights[nl].position = p;
    }
    {
      const auto d = vm.transformVector(light->direction()).versor();
      block->lights[nl].direction = d;
    }
    block->lights[nl].falloff = (int)light->falloff;
    block->lights[nl].range = light->range();
    block->lights[nl].angle = light->spotAngle();
    if (++nl == GLSL::LightingBlock::MAX_LIGHTS)
      break;
  }
  block->lightCount = nl;

  glUnmapNamedBuffer(*_gl->lightingBlock());
  glBindBufferBase(GL_UNIFORM_BUFFER, GLSL::LightingBlockBindingPoint, GLuint(*_gl->lightingBlock()));
}

void
GLRenderer::renderActors()
{
  for (auto& actor : _scene->actors())
  {
    if (!actor->isVisible())
      continue;

    auto mapper = actor->mapper();

    assert(mapper != nullptr);
    mapper->update();
    if (!mapper->render(*this))
    {
      if (auto primitive = mapper->primitive())
        drawMesh(*primitive);
    }
    else if (flags.isSet(DrawBounds))
    {
      setLineColor(boundsColor);
      drawBounds(mapper->bounds());
    }
  }
}

void
GLRenderer::beginRender()
{
  const auto& bc = _scene->backgroundColor;

  glClearColor((float)bc.r, (float)bc.g, (float)bc.b, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glUseProgram(0);
  glBindProgramPipeline(GLuint(*_gl));

  glNamedBufferSubData(
    *_gl->matrixBlock(),
    offsetof (GLSL::MatrixBlock, viewportMatrix),
    sizeof (_viewportMatrix),
    &_viewportMatrix
  );

  int projectionType = _camera->projectionType();
  glNamedBufferSubData(
    *_gl->configBlock(),
    offsetof (GLSL::ConfigBlock, projectionType),
    sizeof (projectionType),
    &projectionType
  );

  glBindBufferBase(GL_UNIFORM_BUFFER, GLSL::MatrixBlockBindingPoint, *_gl->matrixBlock());
  glBindBufferBase(GL_UNIFORM_BUFFER, GLSL::ConfigBlockBindingPoint, *_gl->configBlock());

  glPolygonMode(GL_FRONT_AND_BACK,
    (renderMode != RenderMode::Wireframe) + GL_LINE);
}

void
GLRenderer::render()
{
  update();
  beginRender();
  renderLights();
  renderActors();
  endRender();
}

void
GLRenderer::endRender()
{
  if (_renderFunction != nullptr)
    _renderFunction(*this);
  glFlush();
}

void
GLRenderer::renderMaterial(const Material& material)
{
  auto block = static_cast<GLSL::ConfigBlock*>(glMapNamedBuffer(*_gl->configBlock(), GL_WRITE_ONLY));

  block->material.Oa = material.ambient;
  block->material.Od = material.diffuse;
  block->material.Os = material.spot;
  block->material.shine = material.shine;
  // block->line.width = material.lineWidth;
  // block->line.color = material.lineColor;

  glUnmapNamedBuffer(*_gl->configBlock());
}

inline mat4f
mvMatrix(const Camera& c, const Primitive& p)
{
  return c.worldToCameraMatrix() * p.localToWorldMatrix();
}

inline mat4f
mvpMatrix(const mat4f& mvm, const Camera& c)
{
  return c.projectionMatrix() * mvm;
}

inline mat3f
normalMatrix(const Camera& c, const Primitive& p)
{
  return mat3f{c.worldToCameraMatrix()} * p.normalMatrix();
}

bool
GLRenderer::drawMesh(const Primitive& primitive)
{
  auto mesh = primitive.tesselate();

  if (nullptr == mesh)
    return false;

  auto block = static_cast<GLSL::MatrixBlock*>(glMapNamedBuffer(*_gl->matrixBlock(), GL_WRITE_ONLY));

  auto mvm = mvMatrix(*_camera, primitive);
  block->mvMatrix = mvm;
  block->mvpMatrix = mvpMatrix(mvm, *_camera);
  block->normalMatrix = normalMatrix(*_camera, primitive);

  glUnmapNamedBuffer(*_gl->matrixBlock());

  GLuint subIds[2];

  subIds[0] = renderMode == RenderMode::HiddenLines ?
    _gl->lineColorMixIdx() :
    _gl->noMixIdx();
  subIds[1] = flags.isSet(UseVertexColors) ?
    _gl->colorMapMaterialIdx() :
    _gl->modelMaterialIdx();
  glUniformSubroutinesuiv(GL_FRAGMENT_SHADER, 2, subIds);
  renderMaterial(*primitive.material());
  if (auto m = glMesh(mesh))
  {
    m->bind();
    glDrawElements(GL_TRIANGLES, m->vertexCount(), GL_UNSIGNED_INT, 0);
    if (flags.isSet(DrawBounds))
    {
      setLineColor(boundsColor);
      drawBounds(primitive.bounds());
    }
    /*
    setVectorColor(Color::white);
    drawNormals(*mesh,
      primitive.localToWorldMatrix(),
      primitive.normalMatrix());
    */
  }
  return true;
}

void
GLRenderer::drawAxes(const mat4f& m, float s)
{
  mat3f r{m};

  r[0].normalize();
  r[1].normalize();
  r[2].normalize();
  GLGraphics3::drawAxes(vec3f{m[3]}, r, s);
}

void
GLRenderer::setPipeline(uint32_t code, Pipeline* p)
{
  assert(code < PipelineCode::Max);
  if (code == 0)
  {
    if (auto q = dynamic_cast<MeshPipeline*>(p))
      _pipelines[0] = _gl = q;
  }
  else
  {
    _pipelines[code] = p;
  }
}

} // end namespace cg
