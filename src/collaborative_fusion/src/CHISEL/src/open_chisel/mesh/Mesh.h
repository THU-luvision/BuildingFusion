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
// The above copyright noticec and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef MESH_H_
#define MESH_H_

#include <memory>
#include <vector>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/Chunk.h>
#include <open_chisel/threading/Threading.h>
#include <sophus/se3.hpp>
#include <algorithm>
namespace chisel
{


    typedef size_t VertIndex;
    typedef std::vector<VertIndex> VertIndexList;
    typedef std::vector<Sophus::SE3d , Eigen::aligned_allocator<Eigen::Vector3d> > PoseSE3dList;
    typedef Sophus::SE3d PoseSE3d;
    typedef std::unordered_map<Vec3, Vec3, matrix_hash<Vec3>> VoxelToVoxel;
    struct ChunkHasher
    {
            // Three large primes are used for spatial hashing.
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;
            static constexpr size_t p3 = 83492791;

            std::size_t operator()(const Point3& key) const
            {
                return ( key(0) * p1 ^ key(1) * p2 ^ key(2) * p3);
            }
    };
    class Mesh
    {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
            Mesh();
            virtual ~Mesh();
            inline bool empty() const {return vertices.empty() && normals.empty()&& indices.empty();}
            inline bool HasVertices() const { return !vertices.empty(); }
            inline bool HasNormals() const { return !normals.empty(); }
            inline bool HasColors() const { return !colors.empty(); }
            inline bool HasIndices() const { return !indices.empty(); }
            inline bool HasLabels() const {return !labels.empty();}

            inline void Clear()
            {
                //int vertexNum = vertices.size();
                vertices.clear();
                normals.clear();
                colors.clear();
                indices.clear();
                labels.clear();
                /*
                vertices.reserve(vertexNum + 3);
                normals.reserve(vertexNum + 3);
                colors.reserve(vertexNum + 3);
                indices.reserve(vertexNum + 3);
                */
            }
            size_t getOccupyMemory()
            {
               
               return vertices.capacity() * sizeof(Vec3) +normals.capacity()*sizeof(Vec3)+colors.capacity()*sizeof(Vec3)+
               sizeof(Vec3) +  sizeof(Point3) + sizeof(Vec3) * compact_vertices.capacity() + sizeof(Vec3)*compact_normals.capacity() + 
               sizeof(Vec3)*compact_colors.capacity() + (sizeof(int) + sizeof(Vec3))*compact_indices.size() + sizeof(Point3) * triangles.size() +
                sizeof(int)*indices.capacity() + sizeof(Vec3) + labels.size() * sizeof(long) + compact_labels.size() * sizeof(long);
            }
            float doubleArea(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2)
            {
                return ((v1 - v0).cross(v2 - v0)).norm();
            }
            void CompactWithoutBounding()
            {
                compact_vertices = vertices;
                compact_normals = normals;
                compact_colors = colors;
                if(labels.size() == vertices.size())
                compact_labels = labels;
                for(int i = 0; i < compact_vertices.size(); i+=3)
                {
                    triangles.push_back(Point3(i, i+1, i+2));
                }
            }
            void Compact()
            {
                compact_vertices.clear();
                compact_indices.clear();
                compact_normals.clear();
                compact_colors.clear();
                compact_labels.clear();
                triangles.clear();
                VoxelToVoxel compact_to_coor;
                std::cout<<vertices.size()<< std::endl;
                for(int i = 0;i+2<vertices.size(); i+=3)
                {
                    std::vector<Vec3> tmp_vertices(3);
                    for(int j = 0;j!=3;++j)
                    {
                    tmp_vertices[j] = vertices[i+j];
                    vertices[i+j] = toHalfVec3(vertices[i+j]);
                    //normals[i+j] = toHalfVec3(normals[i+j]);
                    }
                    float doubleA = doubleArea(vertices[i],vertices[i+1],vertices[i+2]);
                    if(doubleA >0 && doubleA < (std::numeric_limits<float>::max)())
                    {
                     for(int j = 0;j!=3;++j)
                    {
                    //vertices[i+j] = toHalfVec3(vertices[i+j]);
                    //normals[i+j] = toHalfVec3(normals[i+j]);
                    if(compact_indices.find(vertices[i+j]) == compact_indices.end() )
                       {
                        compact_vertices.push_back(tmp_vertices[j]);
                        compact_colors.push_back(colors[i+j]);
                        compact_normals.push_back(normals[i+j]);
                        if(labels.size() == vertices.size())
                        compact_labels.push_back(labels[i+j]);
                        compact_indices[vertices[i+j]] = compact_vertices.size()-1;
                       }
                       else
                       {
                           //int vid = compact_indices[vertices[i + j]];
                           //compact_vertices[vid] = (compact_vertices[vid] + tmp_vertices[j])/2;
                       }
                     /* else
                    {
                        //integrate the colors 
                        Vec3 &color1 = compact_colors[compact_indices[vertices[i+j]]];
                        Vec3 &color2 = colors[i+j];
                        color1 = (color1 +  color2)/2;
                    }*/
                    }
                    int v1 = compact_indices[vertices[i]];
                    int v2 = compact_indices[vertices[i+1]];
                    int v3 = compact_indices[vertices[i+2]];
                    if(v1 != v2 || v3 != v1 && v2 != v3)
                    triangles.push_back(Point3(v1,v2,v3));
                    }
                }
                //triangles = Point3List(sort_triangles.begin(),sort_triangles.end());
            }

            void Decompact()
            {
                vertices.clear();
                normals.clear();
                colors.clear();
                labels.clear();
                for(int i = 0; i != triangles.size(); ++i)
                {
                    Point3 & triangle = triangles[i];
                    normals.push_back(compact_normals[triangle(0)]);
                    normals.push_back(compact_normals[triangle(1)]);
                    normals.push_back(compact_normals[triangle(2)]);

                    vertices.push_back(compact_vertices[triangle(0)]);
                    vertices.push_back(compact_vertices[triangle(1)]);
                    vertices.push_back(compact_vertices[triangle(2)]);

                    colors.push_back(compact_colors[triangle(0)]);
                    colors.push_back(compact_colors[triangle(1)]);
                    colors.push_back(compact_colors[triangle(2)]);

                    labels.push_back(compact_labels[triangle(0)]);
                    labels.push_back(compact_labels[triangle(1)]);
                    labels.push_back(compact_labels[triangle(2)]);
                }

                clearCompact();
            }
            void clearCompact()
            {
                Vec3List r_compact_vertices;
                VoxelIndex r_compact_indices;
                Vec3List r_compact_normals;
                Vec3List r_compact_colors;
                Point3List r_triangles; 
                std::vector<long> r_compact_labels;
                
                compact_vertices.erase(compact_vertices.begin(),compact_vertices.end());
                compact_normals.erase(compact_normals.begin(),compact_normals.end());
                compact_colors.erase(compact_colors.begin(),compact_colors.end());
                compact_indices.erase(compact_indices.begin(),compact_indices.end());
                compact_labels.erase(compact_labels.begin(), compact_labels.end());
                triangles.erase(triangles.begin(),triangles.end());   

                compact_vertices.swap(r_compact_vertices);
                compact_colors.swap(r_compact_colors);
                compact_indices.swap(r_compact_indices);
                compact_normals.swap(r_compact_normals);
                compact_labels.swap(r_compact_labels);
                triangles.swap(r_triangles);    
                  
                /*compact_vertices.swap(Vec3List());
                compact_indices.clear();
                compact_indices.swap(VoxelIndex());
                compact_normals.clear();
                compact_normals.swap(Vec3List());
                compact_colors.clear();
                compact_colors.swap(Vec3List());
                triangles.clear();
                triangles.swap(Point3List());*/
            }
            void clearNotCompact()
            {
                Vec3List r_vertices;
                VertIndexList r_indices;
                Vec3List r_normals;
                Vec3List r_colors;
                std::vector<long> r_labels;
                vertices.erase(vertices.begin(),vertices.end());
                normals.erase(normals.begin(),normals.end());
                colors.erase(colors.begin(),colors.end());
                indices.erase(indices.begin(),indices.end());
                labels.erase(labels.begin(), labels.end());
                
                vertices.swap(r_vertices);
                normals.swap(r_normals);
                colors.swap(r_colors);
                indices.swap(r_indices);
                labels.swap(r_labels);
            }
            void transform(const PoseSE3d &relativeChange)
            {
                //if(relativeChange.log() == PoseSE3d().log()) return;
                Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
                Eigen::Vector3f t = relativeChange.translation().cast<float>();
                for(int i = 0;i!=vertices.size();++i)
                {
                    vertices[i] = R*vertices[i]+t;
                }
                for(int i = 0;i!=normals.size();++i)
                {
                    normals[i] = R* normals[i]; 
                }
                for(int i = 0;i!=compact_vertices.size();++i)
                {
                    compact_vertices[i] = R*compact_vertices[i]+t;
                }
                for(int i = 0;i!=compact_normals.size();++i)
                {
                    compact_normals[i] = R * compact_normals[i]; 
                }

            }
            void transform(const Eigen::Matrix4f &T)
            {
                for(int i = 0; i != compact_vertices.size(); ++i)
                {
                    auto &point = compact_vertices[i];
                    Eigen::Vector4f new_point =
                        T *Eigen::Vector4f(point(0), point(1), point(2), 1.0);
                    point = new_point.head<3>() / new_point(3);                
                
                    auto &normal = compact_normals[i];
                    Eigen::Vector4f new_normal =
                        T *Eigen::Vector4f(normal(0), normal(1), normal(2), 0.0);
                    normal = new_normal.head<3>() ;
                }
            }
            void parallel_transform(PoseSE3d &relativeChange)
            {

                if(relativeChange.log() == PoseSE3d().log()) return;
                Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
                Eigen::Vector3f t = relativeChange.translation().cast<float>();

                parallel_for(vertices.begin(),vertices.end(),[&](Vec3 &vert)
                {
                    vert = R*vert + t;
                });
                parallel_for(compact_vertices.begin(),compact_vertices.end(),[&](Vec3 &vert)
                {
                    vert = R*vert + t;
                });
            }
            bool WriteToPLY(const std::string &fileName) const
            {
                
            }
            Vec3List compact_vertices;
            VoxelIndex compact_indices;
            Vec3List compact_normals;
            Vec3List compact_colors;
            std::vector<long> compact_labels;
            Point3List triangles; 

            //std::vector<float> instance_labels;
            Vec3List vertices;
            VertIndexList indices;
            Vec3List normals;
            Vec3List colors;
            Vec3 averageNoraml;
            Point3 chunkID;
            std::vector<long> labels;

    };


    typedef std::shared_ptr<Mesh> MeshPtr;
    typedef std::shared_ptr<const Mesh> MeshConstPtr;

} // namespace chisel 

#endif // MESH_H_ 
