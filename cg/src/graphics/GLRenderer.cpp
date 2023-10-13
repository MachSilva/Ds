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
// OVERVIEW: GLRenderer.cpp
// ========
// Source file for OpenGL renderer.
//
// Author: Paulo Pagliosa
// Last revision: 02/09/2023
// Altered by: Felipe Machado
// Altered version last revision: 13/10/2023

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
    vec4 Od; // diffuse color or albedo (PBR)
    vec4 Os; // specular spot color (Phong) or media refraction ratio (PBR)
    float shine; // specular shininess exponent
    float roughness; // [PBR]
    float metalness; // [PBR]
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
  out gl_PerVertex
  {
    vec4 gl_Position;
  };

  layout(location = 0) in vec4 position;
  layout(location = 1) in vec3 normal;
  layout(location = 2) in vec4 color;

  layout(location = 0) out vec3 vPosition;
  layout(location = 1) out vec3 vNormal;
  layout(location = 2) out vec4 vColor;

  void main()
  {
    gl_Position = mvpMatrix * position;
    vPosition = vec3(mvMatrix * position);
    vNormal = normalize(normalMatrix * normal);
    vColor = color;
  }
);

static const char* gGeometryShader = STRINGIFY(
  in gl_PerVertex
  {
    vec4 gl_Position;
  } gl_in[];

  out gl_PerVertex
  {
    vec4 gl_Position;
  };

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
    vec2 v0 = p2 - p1;
    vec2 v1 = p2 - p0;
    vec2 v2 = p1 - p0;
    float a = v2.x * v1.y - v1.x * v2.y;

    gl_Position = gl_in[0].gl_Position;
    gEdgeDistance = vec3(abs(a / length(v0)), 0, 0);
    gPosition = vPosition[0];
    gNormal = vNormal[0];
    gColor = vColor[0];
    EmitVertex();
    gl_Position = gl_in[1].gl_Position;
    gEdgeDistance = vec3(0, abs(a / length(v1)), 0);
    gPosition = vPosition[1];
    gNormal = vNormal[1];
    gColor = vColor[1];
    EmitVertex();
    gl_Position = gl_in[2].gl_Position;
    gEdgeDistance = vec3(0, 0, abs(a / length(v2)));
    gPosition = vPosition[2];
    gNormal = vNormal[2];
    gColor = vColor[2];
    EmitVertex();
    EndPrimitive();
  }
);

static const char* gFragmentShader = STRINGIFY(
  const float M_PI = 3.14159265358979323846;

  subroutine vec4 shadingType(vec3 P, vec3 N);
  subroutine vec4 mixColorType(vec4 color);
  subroutine void matPropsType(out MaterialProps m);

  layout(location = 0) in vec3 gPosition;
  layout(location = 1) in vec3 gNormal;
  layout(location = 2) in vec4 gColor;
  layout(location = 3) noperspective in vec3 gEdgeDistance;

  subroutine uniform shadingType shade;
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

    m = MaterialProps(gColor * Oa, gColor * Od, vec4(1), material.shine, material.roughness, material.metalness);
  }

  /**
   * @brief Light to point vector.
   * 
   * @param i Light index in uniform block
   * @param P The point
   * @param L A vector from the point P to the light L
   * @param d Light intensity
   * @return bool If point P is illuminated by the light
   */
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

  vec3 cameraToPoint(vec3 P)
  {
    return projectionType == 0 ?
      // PERSPECTIVE
      normalize(P) :
      // PARALLEL
      vec3(0, 0, -1);
  }

  subroutine(shadingType)
  vec4 phong(vec3 P, vec3 N)
  {
    MaterialProps m;
    vec4 color;

    matProps(m);
    color = ambientLight * m.Oa;

    vec3 V = cameraToPoint(P);

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

  float G1(float dotNX, float k)
  {
    return dotNX / (dotNX * (1 - k) + k);
  }

  // Smith's geometry shadow-mask function
  float G(float dotNL, float dotNV, float r)
  {
    const float r1 = r + 1;
    const float k = r1 * r1 * 0.125; // .125 = 1/8
    return G1(dotNL, k) * G1(dotNV, k);
  }

  // Microfacet normal distribution function
  float D(float dotNH, float r)
  {
    dotNH = max(dotNH, 0);
    float a2 = pow(max(r, 1e-3), 4); // a = r^2; a^2 = r^4
    float d = dotNH * dotNH * (a2 - 1) + 1; // dot(H,N)^2 * (a^2 - 1) + 1
    return a2 / (M_PI * d*d);
  }

  // Schlick's approximation for Fresnel reflectance for each wavelength
  vec4 schlick(vec4 R0, float dotLH)
  {
    return R0 + (vec4(1) - R0) * pow(1 - dotLH, 5);
  }

  subroutine(shadingType)
  vec4 pbr(vec3 P, vec3 N)
  {
    MaterialProps m;
    vec4 color = vec4(0,0,0,1);

    matProps(m);
    m.Os.xyz = max(m.Os.xyz, vec3(0.04));

    // V = camera_pos - point; from point to camera
    vec3 V = - cameraToPoint(P);
    float dotNV = dot(N, V);

    if (dotNV < 0)
    {
      N = -N;
      dotNV = -dotNV;
    }

    for (int i = 0; i < lightCount; i++)
    {
      vec3 L;
      float d;
 
      if (lightVector(i, P, L, d))
      {
        const float dotNL = dot(N, L);
        if (dotNL > 1e-7) // Avoid division by zero.
        {
          vec4 I = lightColor(i, d);
          vec3 H = normalize(L + V);
          vec4 specular = (
            schlick(m.Os, dot(L, H))
            * G(dotNL, dotNV, m.roughness)
            * D(dot(H, N), m.roughness)
          ) / (4 * dotNL * dotNV);
          vec4 diffuse = m.Od / M_PI; // Lambertian diffuse
          color += I * mix(diffuse, specular, m.metalness) * dotNL;
        }
      }
    }
    return min(M_PI * color, vec4(1));
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
    fragmentColor = mixColor(shade(gPosition, normalize(gNormal)));
  }
);

GLRenderer::FragmentShader::FragmentShader(
  std::initializer_list<const char*> sources
) : GLSL::ShaderProgram{GL_FRAGMENT_SHADER, sources}
{
  _mixColor.noMixIdx = subroutineIndex("noMix");
  _mixColor.lineColorMixIdx = subroutineIndex("lineColorMix");
  _matProps.modelMaterialIdx = subroutineIndex("modelMaterial");
  _matProps.colorMapMaterialIdx = subroutineIndex("colorMapMaterial");
  _shade.phongIdx = subroutineIndex("phong");
  _shade.pbrIdx = subroutineIndex("pbr");

  _mixColor.location = subroutineUniformLocation("mixColor");
  _matProps.location = subroutineUniformLocation("matProps");
  _shade.location = subroutineUniformLocation("shade");
}

GLRenderer::FragmentShader::~FragmentShader()
{
  // virtual destructor
}

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
    throw std::runtime_error("failed to create vertex shader program\n"
      + _vertex->infoLog());

  _geometry = new GLSL::ShaderProgram(GL_GEOMETRY_SHADER, {
    GLSL::VERSION,
    GLSL::MATRIX_BLOCK_DECLARATION,
    gGeometryShader
  });

  if (!_geometry->ok())
    throw std::runtime_error("failed to create geometry shader program\n"
      + _geometry->infoLog());

  _fragment = new GLRenderer::FragmentShader({
    GLSL::VERSION,
    GLSL::PROPS_DECLARATIONS,
    GLSL::LIGHTING_BLOCK_DECLARATION,
    GLSL::CONFIG_BLOCK_DECLARATION,
    gFragmentShader
  });

  if (!_fragment->ok())
    throw std::runtime_error("failed to create fragment shader program\n"
      + _fragment->infoLog());

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

  beforeDrawing = [](GLRenderer& ctx) { ctx.defaultFragmentSubroutines(); };
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
    _gl->lightingBlock(),
    0,
    sizeof (GLSL::LightingBlock),
    (void*) &b
  );
  glBindBufferBase(GL_UNIFORM_BUFFER,
    GLSL::LightingBlockBindingPoint, _gl->lightingBlock());
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
    glMapNamedBuffer(_gl->lightingBlock(), GL_WRITE_ONLY));

  block->ambientLight = _scene->ambientLight;

  const auto& vm = _camera->worldToCameraMatrix();
  int nl{0};

  for (auto light : _scene->lights())
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

  glUnmapNamedBuffer(_gl->lightingBlock());
  glBindBufferBase(GL_UNIFORM_BUFFER,
    GLSL::LightingBlockBindingPoint, _gl->lightingBlock());
}

void
GLRenderer::renderActors()
{
  for (auto actor : _scene->actors())
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

  _gl->use();

  glNamedBufferSubData(
    _gl->matrixBlock(),
    offsetof (GLSL::MatrixBlock, viewportMatrix),
    sizeof (_viewportMatrix),
    &_viewportMatrix
  );

  int projectionType = _camera->projectionType();
  glNamedBufferSubData(
    _gl->configBlock(),
    offsetof (GLSL::ConfigBlock, projectionType),
    sizeof (projectionType),
    &projectionType
  );

  glBindBufferBase(GL_UNIFORM_BUFFER,
    GLSL::MatrixBlockBindingPoint, _gl->matrixBlock());
  glBindBufferBase(GL_UNIFORM_BUFFER,
    GLSL::ConfigBlockBindingPoint, _gl->configBlock());

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
  auto block = static_cast<GLSL::ConfigBlock*>(
    glMapNamedBuffer(_gl->configBlock(), GL_WRITE_ONLY)
  );

  block->material.Oa = material.ambient;
  block->material.Od = material.diffuse;
  block->material.Os = material.spot;
  block->material.shine = material.shine;
  block->material.roughness = material.roughness;
  block->material.metalness = material.metalness;
  // block->line.width = material.lineWidth;
  // block->line.color = material.lineColor;

  glUnmapNamedBuffer(_gl->configBlock());
}


inline mat4f
mvMatrix(const mat4f& t, const Camera* c)
{
  return c->worldToCameraMatrix() * t;
}

inline mat4f
mvpMatrix(const mat4f& mvm, const Camera* c)
{
  return c->projectionMatrix() * mvm;
}

inline auto
normalMatrix(const mat3f& n, const Camera* c)
{
  return mat3f{c->worldToCameraMatrix()} * n;
}

void
GLRenderer::defaultFragmentSubroutines()
{
  auto fs = fragmentShader();
  auto subIds = fs->subroutineUniforms();
  auto& mixColor = fs->mixColor();
  auto& matProps = fs->matProps();
  auto& shade = fs->shade();
  subIds[mixColor.location] = renderMode == RenderMode::HiddenLines
    ? mixColor.lineColorMixIdx
    : mixColor.noMixIdx;
  subIds[matProps.location] = flags.isSet(UseVertexColors)
    ? matProps.colorMapMaterialIdx
    : matProps.modelMaterialIdx;
  subIds[shade.location] = shade.pbrIdx;
  glUniformSubroutinesuiv(GL_FRAGMENT_SHADER, subIds.size(), subIds.data());
}

bool
GLRenderer::drawMesh(const Primitive& primitive)
{
  auto mesh = primitive.tesselate();

  if (!mesh)
    return false;

  auto& t = primitive.localToWorldMatrix();
  auto& n = primitive.normalMatrix();

  drawMesh(*mesh, *primitive.material(), t, n, mesh->data().triangleCount, 0);
  if (flags.isSet(DrawBounds))
  {
    setLineColor(boundsColor);
    drawBounds(primitive.bounds());
  }
  /*
  setVectorColor(Color::gray);
  drawNormals(*mesh, t, n, 0.5f);
  */
  return true;
}

void
GLRenderer::drawMesh(const TriangleMesh& mesh,
  const Material& material,
  const mat4f& t,
  const mat3f& n,
  int count,
  int offset)
{
  auto b = static_cast<GLSL::MatrixBlock*>(
    glMapNamedBuffer(_gl->matrixBlock(), GL_WRITE_ONLY)
  );

  auto mvm = mvMatrix(t, _camera);
  b->mvMatrix = mvm;
  b->mvpMatrix = mvpMatrix(mvm, _camera);
  b->normalMatrix = normalMatrix(n, _camera);

  glUnmapNamedBuffer(_gl->matrixBlock());

  // defaultFragmentSubroutines();
  _gl->beforeDrawing(*this);

  renderMaterial(material);

  auto m = glMesh(&mesh);

  m->bind();
  glDrawElements(GL_TRIANGLES,
    count * 3,
    GL_UNSIGNED_INT,
    (void*)(sizeof(TriangleMesh::Triangle) * offset));
}

bool
GLRenderer::drawSubMesh(const TriangleMesh& mesh,
  int count,
  int offset,
  const Material& material,
  const mat4f& t,
  const mat3f& n)
{
  {
    if (count <= 0 || offset < 0)
      return false;

    const auto nt = mesh.data().triangleCount;

    if (offset >= nt)
      return false;
    if (auto end = offset + count; end > nt)
      count = end - nt;
  }
  drawMesh(mesh, material, t, n, count, offset);
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

GLRenderer::Pipeline*
GLRenderer::pipeline(uint64_t id) const
{
  auto it = _pipelines.find(id);
  if (it == _pipelines.end())
    return nullptr;
  return it->second.get();
}

void
GLRenderer::setPipeline(uint64_t id, Pipeline* p)
{
  if (id == 0)
  {
    if (auto q = dynamic_cast<MeshPipeline*>(p))
      _pipelines[0] = _gl = q;
  }
  else
  {
    _pipelines[id] = p;
  }
}

} // end namespace cg
