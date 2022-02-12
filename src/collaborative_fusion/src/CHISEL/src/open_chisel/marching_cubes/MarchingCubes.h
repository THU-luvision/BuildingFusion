// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MARCHINGCUBES_H_
#define MARCHINGCUBES_H_

#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/mesh/Mesh.h>
#include <iostream>

#define PRECHECK 1
#define APPROXIMATE_NORMAL 1
#define APPROXIMATE_COLOR  0
namespace chisel
{

    typedef std::vector<Mat3x3, Eigen::aligned_allocator<Mat3x3> > TriangleVector;
    class MarchingCubes
    {
        public:
            static int triangleTable[256][16];
            static int edgeIndexPairs[12][2];

            MarchingCubes();
            virtual ~MarchingCubes();


            static void MeshCube(const Eigen::Matrix<float, 3, 8>& vertex_coordinates, const Eigen::Matrix<float, 8, 1>& vertexSDF, TriangleVector* triangles)
            {
                assert(triangles != nullptr);

                const int index = CalculateVertexConfiguration(vertexSDF);

                Eigen::Matrix<float, 3, 12> edgeCoords;
                InterpolateEdgeVertices(vertex_coordinates, vertexSDF, &edgeCoords);

                const int* table_row = triangleTable[index];

                int edgeIDX = 0;
                int tableCol = 0;
                while ((edgeIDX = table_row[tableCol]) != -1)
                {
                    Eigen::Matrix3f triangle;
                    triangle.col(0) = edgeCoords.col(edgeIDX);
                    edgeIDX = table_row[tableCol + 1];
                    triangle.col(1) = edgeCoords.col(edgeIDX);
                    edgeIDX = table_row[tableCol + 2];
                    triangle.col(2) = edgeCoords.col(edgeIDX);
                    triangles->push_back(triangle);
                    tableCol += 3;
                }
            }

            static void MeshCube(const Eigen::Matrix<float, 3, 8>& vertexCoords,
                                 const Eigen::Matrix<float, 8, 1>& vertexSDF,
                                 VertIndex* nextIDX,
                                 Mesh* mesh)
            {
                assert(nextIDX != nullptr);
                assert(mesh != nullptr);
                const int index = CalculateVertexConfiguration(vertexSDF);

                Eigen::Matrix<float, 3, 12> edge_vertex_coordinates;
                InterpolateEdgeVertices(vertexCoords, vertexSDF, &edge_vertex_coordinates);

                const int* table_row = triangleTable[index];

                int table_col = 0;
                while (table_row[table_col] != -1)
                {
                    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col + 2]));
                    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col + 1]));
                    mesh->vertices.emplace_back(edge_vertex_coordinates.col(table_row[table_col]));
                    mesh->indices.push_back(*nextIDX);
                    mesh->indices.push_back((*nextIDX) + 1);
                    mesh->indices.push_back((*nextIDX) + 2);
                    const Eigen::Vector3f& p0 = mesh->vertices[*nextIDX];
                    const Eigen::Vector3f& p1 = mesh->vertices[*nextIDX + 1];
                    const Eigen::Vector3f& p2 = mesh->vertices[*nextIDX + 2];
                    Eigen::Vector3f px = (p1 - p0);
                    Eigen::Vector3f py = (p2 - p0);
                    Eigen::Vector3f n = px.cross(py).normalized();
//                    printf("%.1f %.1f %.1f    %.1f %.1f %.1f    %.1f %.1f %.1f\r\n",
//                           p0(0) / 0.006,p0(1) / 0.006,p0(2) / 0.006,p1(0) / 0.006,p1(1) / 0.006,p1(2) / 0.006,p2(0) / 0.006,p2(1) / 0.006,p2(2) / 0.006);
//                    std::cout << p0.transpose() << "    " << p1.transpose() << "    " << p2.transpose() << std::endl;
                    mesh->normals.push_back(n);
                    mesh->normals.push_back(n);
                    mesh->normals.push_back(n);
#if APPROXIMATE_COLOR
                    Eigen::Vector3f color = Eigen::Vector3f(floor(colorData[0]) / 255.0f,
                            floor(colorData[1]) / 255.0f,
                            floor(colorData[2]) / 255.0f);

                    mesh->colors.push_back(color);
                    mesh->colors.push_back(color);
                    mesh->colors.push_back(color);
#endif
                    *nextIDX += 3;
                    table_col += 3;
                }
            }

            static int CalculateVertexConfiguration(const Eigen::Matrix<float, 8, 1>& vertexSDF)
            {
                return  (vertexSDF(0) < 0 ? (1<<0) : 0) |
                        (vertexSDF(1) < 0 ? (1<<1) : 0) |
                        (vertexSDF(2) < 0 ? (1<<2) : 0) |
                        (vertexSDF(3) < 0 ? (1<<3) : 0) |
                        (vertexSDF(4) < 0 ? (1<<4) : 0) |
                        (vertexSDF(5) < 0 ? (1<<5) : 0) |
                        (vertexSDF(6) < 0 ? (1<<6) : 0) |
                        (vertexSDF(7) < 0 ? (1<<7) : 0);
            }

            static void InterpolateEdgeVertices(const Eigen::Matrix<float, 3, 8>& vertexCoords, const Eigen::Matrix<float, 8, 1>& vertSDF, Eigen::Matrix<float, 3, 12>* edgeCoords)
            {
                assert(edgeCoords != nullptr);
                for (std::size_t i = 0; i < 12; ++i)
                {
                    const int* pairs = edgeIndexPairs[i];
                    const int edge0 = pairs[0];
                    const int edge1 = pairs[1];
                    // Only interpolate along edges where there is a zero crossing.
                    if ((vertSDF(edge0) < 0 && vertSDF(edge1) >= 0) || (vertSDF(edge0) >= 0 && vertSDF(edge1) < 0))
                        edgeCoords->col(i) = InterpolateVertex(vertexCoords.col(edge0), vertexCoords.col(edge1), vertSDF(edge0), vertSDF(edge1));
                }
            }

            // Performs linear interpolation on two cube corners to find the approximate
            // zero crossing (surface) value.
            static inline Vec3 InterpolateVertex(const Vec3& vertex1, const Vec3& vertex2, const float& sdf1, const float& sdf2)
            {
                const float minDiff = 1e-6;
                const float sdfDiff = sdf1 - sdf2;
                const float t = sdf1 / sdfDiff;
                return Vec3(vertex1 + t * (vertex2 - vertex1));
            }
    };

} // namespace chisel 

#endif // MARCHINGCUBES_H_ 
