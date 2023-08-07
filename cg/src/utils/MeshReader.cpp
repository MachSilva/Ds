//[]---------------------------------------------------------------[]
//|                                                                 |
//| Copyright (C) 2014, 2023 Paulo Pagliosa.                        |
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
// OVERVIEW: MeshReader.cpp
// ========
// Source file for OBJ mesh reader.
//
// Author: Paulo Pagliosa
// Modified by: Felipe Machado
// Last revision: 17/07/2023

#include "utils/MeshReader.h"
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#define OnFailClear() if (in.fail()) { in.clear(); return false; }

namespace cg
{ // begin namespace cg

bool
readInt2(std::istream& in, int skip, int &a, int &b)
{
  in >> a;
  OnFailClear();
  in.seekg(skip, std::ios::cur);
  in >> b;
  OnFailClear();

  return true;
}

bool
readInt3(std::istream& in, int skip, int &a, int &b, int &c)
{
  in >> a;
  OnFailClear();
  in.seekg(skip, std::ios::cur);
  in >> b;
  OnFailClear();
  in.seekg(skip, std::ios::cur);
  in >> c;
  OnFailClear();

  return true;
}

void
readMeshSize(std::fstream& file, TriangleMesh::Data& data)
{
  constexpr auto lineSize = 128;
  int nv{};
  int nt{};

  for (std::string line; file >> line;)
    switch (line[0])
    {
      case 'v':
        if (line.c_str()[1] == '\0')
            nv++;
        std::getline(file, line);
        break;

      case 'f':
      {
        int v;
        int n;
        int t;

        file >> line;
        std::stringstream ss (line);
        
        /* can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
        if (strstr(line.c_str(), "//"))
        {
          /* v//n */
          readInt2(ss, 2, v, n);
          readInt2(file, 2, v, n);
          readInt2(file, 2, v, n);
          nt++;
          while (readInt2(file, 2, v, n))
            nt++;
        }
        else if (readInt3(ss, 1, v, t, n))
        {
          /* v/t/n */
          readInt3(file, 1, v, t, n);
          readInt3(file, 1, v, t, n);
          nt++;
          while (readInt3(file, 1, v, t, n))
            nt++;
        }
        else if (readInt2(ss, 1, v, t))
        {
          /* v/t */
          readInt2(file, 1, v, t);
          readInt2(file, 1, v, t);
          nt++;
          while (readInt2(file, 1, v, t))
            nt++;
        }
        else
        {
          /* v */
          file >> v;
          file >> v;
          nt++;
          while (file >> v)
            nt++;
        }
        break;
      }

      default:
        std::getline(file, line);
    }
  data.vertexCount = nv;
  data.triangleCount = nt;

  if (file.fail()) file.clear();
}

void
readMeshData(std::fstream& file, TriangleMesh::Data& data)
{
  constexpr auto lineSize = 128;
  auto vertex = data.vertices;
  auto triangle = data.triangles;

  for (std::string line; file >> line;)
    switch (line[0])
    {
      case 'v':
      {
        float x;
        float y;
        float z;

        switch (line[1])
        {
          case '\0':
            file >> x >> y >> z;
            vertex->set(x, y, z);
            vertex++;
            break;

          default:
            std::getline(file, line);
        }
        break;
      }

      case 'f':
      {
        int v;
        int n;
        int t;

        file >> line;
        std::stringstream ss (line);

        /* Can be one of %d, %d//%d, %d/%d, %d/%d/%d %d//%d */
        if (strstr(line.c_str(), "//"))
        {
          /* v//n */
          readInt2(ss, 2, v, n);
          triangle->v[0] = v - 1;
          readInt2(file, 2, v, n);
          triangle->v[1] = v - 1;
          readInt2(file, 2, v, n);
          triangle->v[2] = v - 1;
          triangle++;
          while (readInt2(file, 2, v, n))
          {
            triangle->v[0] = triangle[-1].v[0];
            triangle->v[1] = triangle[-1].v[2];
            triangle->v[2] = v - 1;
            triangle++;
          }
        }
        else if (readInt3(ss, 1, v, t, n))
        {
          /* v/t/n */
          triangle->v[0] = v - 1;
          readInt3(file, 1, v, t, n);
          triangle->v[1] = v - 1;
          readInt3(file, 1, v, t, n);
          triangle->v[2] = v - 1;
          triangle++;
          while (readInt3(file, 1, v, t, n) > 0)
          {
            triangle->v[0] = triangle[-1].v[0];
            triangle->v[1] = triangle[-1].v[2];
            triangle->v[2] = v - 1;
            triangle++;
          }
        }
        else if (readInt2(ss, 1, v, t))
        {
          /* v/t */
          triangle->v[0] = v - 1;
          readInt2(file, 1, v, t);
          triangle->v[1] = v - 1;
          readInt2(file, 1, v, t);
          triangle->v[2] = v - 1;
          triangle++;
          while (readInt2(file, 1, v, t))
          {
            triangle->v[0] = triangle[-1].v[0];
            triangle->v[1] = triangle[-1].v[2];
            triangle->v[2] = v - 1;
            triangle++;
          }
        }
        else
        {
          /* v */
          ss >> v;
          triangle->v[0] = v - 1;
          file >> v;
          triangle->v[1] = v - 1;
          file >> v;
          triangle->v[2] = v - 1;
          triangle++;
          while (file >> v)
          {
            triangle->v[0] = triangle[-1].v[0];
            triangle->v[1] = triangle[-1].v[2];
            triangle->v[2] = v - 1;
            triangle++;
          }
        }
        break;
      }

      default:
        std::getline(file, line);
    }
  if (file.fail()) file.clear();
}

/////////////////////////////////////////////////////////////////////
//
// MeshReader implementation
// ==========
TriangleMesh*
MeshReader::readOBJ(const char* filename)
{
  std::fstream file (filename, std::ios::in);

  if (!file)
    return nullptr;

  TriangleMesh::Data data;

  readMeshSize(file, data);
  data.vertices = new vec3f[data.vertexCount];
  data.vertexNormals = nullptr;
  data.triangles = new TriangleMesh::Triangle[data.triangleCount];
  file.seekg(std::ios::beg);
  printf("Reading Wavefront OBJ file %s...\n", filename);
  readMeshData(file, data);

  auto mesh = new TriangleMesh{std::move(data)};

  mesh->computeNormals();
  return mesh;
}

} // end namespace cg
