// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
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

#ifndef CHUNKMANAGER_H_
#define CHUNKMANAGER_H_

#include <memory>
#include <unordered_map>

#include <mutex>
#include <condition_variable>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/mesh/Mesh.h>
#include <open_chisel/ColorVoxel.h>
#include <open_chisel/DistVoxel.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <iostream>
#include <fstream>
//#include <atomic>
#include <sstream>
#include <memory>
#include <functional>   // std::unary_function
#include "Chunk.h"
#include "ForLabel.h"
#define TEST_GRADIENT 0
#define WALL 0 //the label of wall
#define FLOOR 1//the label of floor
#define WINDOW 8
#define DOOR 7
namespace chisel
{
    // Spatial hashing function from Matthias Teschner
    // Optimized Spatial Hashing for Collision Detection of Deformable Objects
    //static std::atomic_flag usingMesh = ATOMIC_FLAG_INIT;
    //static std::atomic_flag generatingMesh = ATOMIC_FLAG_INIT;
    //static std::condition_variable cv_generatingMesh;
    //static std::mutex generatingMesh;

    static std::mutex chunkManagerLockers[100];
    static std::mutex chunkManagerAggragateHasherLockers[100];

    struct vec8
    {
      __m256 xmm;

      vec8 (__m256 v) : xmm (v) {}

      vec8 (float v) { xmm = _mm256_set1_ps(v); }

      vec8 (float a, float b, float c, float d, float e, float f, float g, float h)
      { xmm = _mm256_set_ps(h,g,f,e,d,c,b,a); }

      vec8 floor()
      {
          return _mm256_floor_ps(xmm);
      }

      vec8 (const float *v) { xmm = _mm256_load_ps(v); }

      vec8 operator & (const vec8 &v) const
      { return vec8(_mm256_and_ps(xmm,v.xmm)); }


      vec8 operator > (const vec8 &v) const
      { return vec8(_mm256_cmp_ps(xmm,v.xmm,_CMP_GT_OS)); }
      vec8 operator < (const vec8 &v) const
      { return vec8(_mm256_cmp_ps(v.xmm,xmm,_CMP_GT_OS)); }

      vec8 operator* (const vec8 &v) const
      { return vec8(_mm256_mul_ps(xmm, v.xmm)); }

      vec8 operator+ (const vec8 &v) const
      { return vec8(_mm256_add_ps(xmm, v.xmm)); }

      vec8 operator- (const vec8 &v) const
      { return vec8(_mm256_sub_ps(xmm, v.xmm)); }

      vec8 operator/ (const vec8 &v) const
      { return vec8(_mm256_div_ps(xmm, v.xmm)); }

      void operator*= (const vec8 &v)
      { xmm = _mm256_mul_ps(xmm, v.xmm); }

      void operator+= (const vec8 &v)
      { xmm = _mm256_add_ps(xmm, v.xmm); }

      void operator-= (const vec8 &v)
      { xmm = _mm256_sub_ps(xmm, v.xmm); }

      void operator/= (const vec8 &v)
      { xmm = _mm256_div_ps(xmm, v.xmm); }

      void operator>> (float *v)
      { _mm256_store_ps(v, xmm); }

    };
    struct ChunkAggregatedHash
    {
        int simplified_vertex_count;
        Vec3 coord;//coord
        Vec3 feature;//color
        Vec3 global_coord;
        Vec3 normal;
        Vec3 global_normal;
        long label = -1;// instance value
        double residual = 999999999;// mesh residual
        bool is_new = false;
        bool is_room_border()
        {
            if(label == -1) return false;
            int semantic = get_semantic_label(label);
            return semantic == WALL || semantic == WINDOW || semantic == DOOR;
        }
        bool is_floor()
        {
            if(label == -1) return false;
            int semantic = get_semantic_label(label);
            return semantic == FLOOR;
        }
    };
    typedef std::shared_ptr<Chunk> ChunkPtr;
    typedef std::shared_ptr<const Chunk> ChunkConstPtr;
    typedef std::unordered_map<ChunkID, ChunkPtr, ChunkHasher> ChunkMap;
    typedef std::unordered_map<ChunkID, bool, ChunkHasher> ChunkSet;
    typedef std::unordered_map<ChunkID, MeshPtr, ChunkHasher> MeshMap;
    typedef std::unordered_map<ChunkID, std::vector<size_t>, ChunkHasher> ChunkPointMap;
    //This Hash is for unet
    typedef std::unordered_map<ChunkID, ChunkAggregatedHash, ChunkHasher> AggregatedHash; 


    class Frustum;
    class AABB;
    class ProjectionIntegrator;
    class ChunkManager
    {
        public: 
    
            typedef std::shared_ptr<ChunkManager> ChunkManagerPtr;
            typedef std::shared_ptr<const ChunkManager> ChunkManagerConstPtr;
            ChunkManager();
            ChunkManager(const Eigen::Vector3i& chunkSize, float voxelResolution, bool color);
            virtual ~ChunkManager();
            
            //inline ChunkMap& GetChunks() const { return chunks; }
            inline ChunkMap& GetMutableChunks() { return chunks; }
            void RecomputeMeshesWithoutCheck(const PinholeCamera& camera);
            void RecomputeMeshesWithoutCheck(const PinholeCamera& camera,MeshMap &_allMeshes);
            inline int round(float r)
            {
                return (r > 0.0) ? floor(r + 0.5) : ceil(r - 0.5);
            }
            void LockCurrentMesh()
            {
                //chunk_manager_mutex.lock();
                chunkManagerLockers[submap_id].lock();
            }

            void UnlockCurrentMesh()
            {
                chunkManagerLockers[submap_id].unlock();
            //    chunk_manager_mutex.unlock();
            }

            inline bool HasChunk(const ChunkID& chunk) const
            {
                return chunks.find(chunk) != chunks.end();
            }

            inline bool HasChunk(const ChunkID& chunk, ChunkMap & chunks) const
            {
                return chunks.find(chunk) != chunks.end();
            }
            inline ChunkPtr GetChunk(const ChunkID& chunk) const
            {
                return chunks.at(chunk);
            }
            inline ChunkPtr GetChunk(const ChunkID &chunk, ChunkMap & _chunks)const
            {
                return _chunks.at(chunk);
            }

            Vec3 GetChunkCenter(const ChunkID& cID)
            {
                return Vec3(chunkSize(0) * cID(0) * voxelResolutionMeters,
                            chunkSize(1) * cID(1) * voxelResolutionMeters,
                            chunkSize(2) * cID(2) * voxelResolutionMeters);
            }

            inline void AddChunk(const ChunkPtr& chunk)
            {
                chunks.insert(std::make_pair(chunk->GetID(), chunk));
            }

            inline bool RemoveChunk(const ChunkID& chunk)
            {
                if(HasChunk(chunk))
                {
                    chunks.erase(chunk);

                    if (HasMesh(chunk))
                    {
                        allMeshes.erase(chunk);
                    }
                    return true;
                }
                return false;
            }

            inline bool RemoveChunk(const ChunkPtr& chunk)
            {
                return RemoveChunk(chunk->GetID());
            }

            inline bool HasChunk(int x, int y, int z) const { return HasChunk(ChunkID(x, y, z)); }
            inline ChunkPtr GetChunk(int x, int y, int z) const { return GetChunk(ChunkID(x, y, z)); }

            inline ChunkPtr GetChunkAt(const Vec3& pos)
            {
                ChunkID id = GetIDAt(pos);

                if (HasChunk(id))
                {
                    return GetChunk(id);
                }

                return ChunkPtr();
            }

            inline ChunkPtr GetOrCreateChunkAt(const Vec3& pos, bool* wasNew)
            {
                ChunkID id = GetIDAt(pos);

                if (HasChunk(id))
                {
                    *wasNew = false;
                    return GetChunk(id);
                }
                else
                {
                    *wasNew = true;
                    CreateChunk(id);
                    return GetChunk(id);
                }
            }
        size_t GetOccupyMemory()
        {
            size_t _chunk = 0,voxelnindex = 0,mesh_m = 0,others=0,mesh_s=0;
            others += sizeof(Vec3) + sizeof(Vec3)*centroids.size()+sizeof(Eigen::Vector3i)+sizeof(Eigen::Matrix<int, 3, 8>)+sizeof(int)*(24)+sizeof(bool)+
            (neighborChunkIndexChunkWise.size()+neighborVoxelIndexChunkWise.size())*sizeof(int) + sizeof(Vec3)*interpolatedPoints.size()+sizeof(float);
            voxelnindex += voxelNeighborIndex.size()*sizeof(int);
            for(auto i = chunks.begin();i!=chunks.end();++i)
             {
            _chunk += sizeof(ChunkID)+sizeof(ChunkPtr)+sizeof(Chunk);
            if(i->second)
             _chunk+= i->second->getOccupyMemory();
             }
            for(auto i = allMeshes.begin();i!=allMeshes.end();++i)
            if(i->second)
            mesh_m += i->second->getOccupyMemory() + sizeof(ChunkID)+sizeof(MeshPtr);
            //mesh_m +=(allMeshes.capacity() - allMeshes.size())*(sizeof(ChunkID + sizeof(MeshPtr)));
            if(simplified_mesh)
            mesh_s = simplified_mesh->getOccupyMemory();
            mesh_s += sizeof(MeshPtr);
            std::cout<<"chunks: "<<_chunk/1024.0/1024.0<<std::endl;
         
            std::cout<<"AllMeshes: "<<mesh_m/1024.0/1024<<std::endl;

            std::cout<<"simplified meshes:" <<mesh_s/1024/1024<<std::endl;

            std::cout<<"others: "<<(others+voxelnindex)/1024.0/1024<<std::endl;

            return others+mesh_m+voxelnindex+_chunk+mesh_s;
        }
            inline ChunkID GetIDAt(const Vec3& pos) const
            {

                static const float roundingFactorX = 1.0f / (chunkSize(0) * voxelResolutionMeters);
                static const float roundingFactorY = 1.0f / (chunkSize(1) * voxelResolutionMeters);
                static const float roundingFactorZ = 1.0f / (chunkSize(2) * voxelResolutionMeters);
                return ChunkID(static_cast<int>(std::floor(pos(0) * roundingFactorX)),
                               static_cast<int>(std::floor(pos(1) * roundingFactorY)),
                               static_cast<int>(std::floor(pos(2) * roundingFactorZ)));
            }
            inline ChunkID GetPointIDAt (const Point3 &pos )
            {
                static const float roundingFactorX = 1.0f / chunkSize(0) ;
                static const float roundingFactorY = 1.0f / chunkSize(1) ;
                static const float roundingFactorZ = 1.0f / chunkSize(2);       
                return ChunkID(static_cast<int>(std::floor(pos(0) * roundingFactorX)),
                               static_cast<int>(std::floor(pos(1) * roundingFactorY)),
                               static_cast<int>(std::floor(pos(2) * roundingFactorZ)));

            }

            inline Vec3 GetCentroid(const Point3& globalVoxelID)
            {
                return globalVoxelID.cast<float>() * voxelResolutionMeters + halfVoxel;
            }
#if 0
            const DistVoxel* GetDistanceVoxel(const Vec3& pos);
            const ColorVoxel* GetColorVoxel(const Vec3& pos);
#endif

            template <class DataType> inline void GetChunkIDsIntersectCamera(ProjectionIntegrator& integrator,
                                                                             const Frustum &frustum, ChunkIDList * chunkList,
                                                                             const std::shared_ptr<const DepthImage<DataType> >& depthImage,
                                                                             const PinholeCamera& camera,
                                                                             const Transform& cameraPose)
            {
                assert(chunkList != nullptr);

                AABB frustumAABB;
                frustum.ComputeBoundingBox(&frustumAABB);

                ChunkID minID = GetIDAt(frustumAABB.min);
                ChunkID maxID = GetIDAt(frustumAABB.max) + Eigen::Vector3i(1, 1, 1);

                float resolutionDiagonal = chunkSize.norm() * voxelResolutionMeters;

                for (int x = minID(0) - 1; x <= maxID(0) + 1; x++)
                {
                    for (int y = minID(1) - 1; y <= maxID(1) + 1; y++)
                    {
                        for (int z = minID(2) - 1; z <= maxID(2) + 1; z++)
                        {
                            Vec3 min = Vec3(x * chunkSize(0), y * chunkSize(1), z * chunkSize(2)) * voxelResolutionMeters;
                            Vec3 max = min + chunkSize.cast<float>() * voxelResolutionMeters;


                            AABB chunkBox(min, max);

                            float corner[3][2];
                            for(int l = 0; l < 3; l++)
                            {
                                corner[l][0] = min(l);
                                corner[l][1] = max(l);
                            }

                            bool intersectFlag = 0;
                            int intersectCornersCnt = 0;
                            Vec3 corners[9];
                            for(int l = 0; l < 2; l++)
                            {
                                for(int m = 0; m < 2; m++)
                                {
                                    for(int n = 0; n < 2; n++)
                                    {
                                        corners[l*4+m*2+n] = Vec3(corner[0][l],corner[1][m],corner[2][n]);

                                    }
                                }
                            }
                            corners[8] = (min + max) / 2;
                            for(int i = 0; i < 9; i++)
                            {

                                Vec3 voxelCenter = corners[i];
                                Vec3 voxelCenterInCamera = cameraPose.linear().transpose() * (voxelCenter - cameraPose.translation());

                                Vec3 cameraPos = camera.ProjectPoint(voxelCenterInCamera);
                                if (!camera.IsPointOnImage(cameraPos) || voxelCenterInCamera.z() < 0)
                                    continue;
                                float voxelDist = voxelCenterInCamera.z();
                                float depth = depthImage->DepthAt((int)cameraPos(1), (int)cameraPos(0)); //depthImage->BilinearInterpolateDepth(cameraPos(0), cameraPos(1));

                                if(std::isnan(depth))
                                {
                                    continue;
                                }

                                float max_trancate_distance = integrator.GetTruncator()->GetTruncationDistance(depth);
                                float surfaceDist = depth - voxelDist;
                                if(voxelDist < 3 && fabs(surfaceDist) < max_trancate_distance + resolutionDiagonal)
                                {
                                    intersectCornersCnt += 1;
                                }
                            }


                                // transform current vertex to camera coordinate

                            if(intersectCornersCnt > 2)
                            {
                                chunkList->push_back(ChunkID(x, y, z));
                            }
                        }
                    }
                }

//                printf("%lu chunks intersect frustum\n", chunkList->size());
            }


            void findCubeCornerByMat(const float *filtered_depth,
                                     const PinholeCamera& depthCamera,
                                     const Transform& depthExtrinsic,
                                     Vec3 &maxCorner,
                                     Vec3 &minCorner)
            {
                int width = depthCamera.GetWidth();
                int height = depthCamera.GetHeight();

                const float fx = depthCamera.GetFx();
                const float fy = depthCamera.GetFy();
                const float cx = depthCamera.GetCx();
                const float cy = depthCamera.GetCy();
                Eigen::Matrix3f rotation = depthExtrinsic.linear();
                Eigen::MatrixXf translation = depthExtrinsic.translation();

                __m256 maxX = _mm256_set1_ps(-1e8);
                __m256 maxY = _mm256_set1_ps(-1e8);
                __m256 maxZ = _mm256_set1_ps(-1e8);
                __m256 minX = _mm256_set1_ps(1e8);
                __m256 minY = _mm256_set1_ps(1e8);
                __m256 minZ = _mm256_set1_ps(1e8);

                vec8 inc = vec8(0,1,2,3,4,5,6,7);
                for(int i = 0; i < height; i ++)
                {
                    for(int j = 0; j < width; j+=8)
                    {
                        int pos = i * width + j;
                        vec8 depth_c = _mm256_loadu_ps(&filtered_depth[pos]);
                        vec8 x = inc + vec8(j);
                        vec8 y = vec8(i);
                        depth_c = depth_c + vec8(0.2);
                        vec8 refLocalVertexX = (x - vec8(cx)) / vec8(fx) * depth_c;
                        vec8 refLocalVertexY = (y - vec8(cy)) / vec8(fy) * depth_c;
                        vec8 refVX = vec8(rotation(0,0)) * refLocalVertexX + vec8(rotation(0,1)) * refLocalVertexY + vec8(rotation(0,2)) * depth_c + vec8(translation(0));
                        vec8 refVY = vec8(rotation(1,0)) * refLocalVertexX + vec8(rotation(1,1)) * refLocalVertexY + vec8(rotation(1,2)) * depth_c + vec8(translation(1));
                        vec8 refVZ = vec8(rotation(2,0)) * refLocalVertexX + vec8(rotation(2,1)) * refLocalVertexY + vec8(rotation(2,2)) * depth_c + vec8(translation(2));
                        maxX = _mm256_max_ps(refVX.xmm,maxX);
                        maxY = _mm256_max_ps(refVY.xmm,maxY);
                        maxZ = _mm256_max_ps(refVZ.xmm,maxZ);

                        minX = _mm256_min_ps(refVX.xmm,minX);
                        minY = _mm256_min_ps(refVY.xmm,minY);
                        minZ = _mm256_min_ps(refVZ.xmm,minZ);
                    }
                }
                maxCorner = Eigen::Vector3f(-1e8,-1e8,-1e8);
                minCorner = Eigen::Vector3f(1e8,1e8,1e8);
                for(int i = 0; i < 8; i++ )
                {
                    maxCorner(0) = fmax(maxCorner(0),maxX[i]);
                    minCorner(0) = fmin(minCorner(0),minX[i]);

                    maxCorner(1) = fmax(maxCorner(1),maxY[i]);
                    minCorner(1) = fmin(minCorner(1),minY[i]);

                    maxCorner(2) = fmax(maxCorner(2),maxZ[i]);
                    minCorner(2) = fmin(minCorner(2),minZ[i]);
                }
            }

            void GetBoundaryChunkID(const float *depthImage,
                                    const PinholeCamera& depthCamera,
                                    const Transform& depthExtrinsic,
                                    ChunkID &maxChunkID,
                                    ChunkID &minChunkID)
            {
                float minimum, maximum;
                minimum = 10;
                maximum = -1;
                Vec3 maxValue, minValue;
                findCubeCornerByMat(depthImage,depthCamera,depthExtrinsic,maxValue, minValue);
                maxChunkID = GetIDAt(maxValue);
                minChunkID = GetIDAt(minValue);
            }

            inline void GetChunkIDsObservedByCamera(ProjectionIntegrator& integrator,
                                                    const Frustum &frustum, ChunkIDList * chunkList,
                                                    const float* depthImage,
                                                    const PinholeCamera& camera,
                                                    const Transform& cameraPose)
            {
                assert(chunkList != nullptr);

#if 0
                AABB frustumAABB;
                frustum.ComputeBoundingBox(&frustumAABB);

                ChunkID minID = GetIDAt(frustumAABB.min);
                ChunkID maxID = GetIDAt(frustumAABB.max) + Eigen::Vector3i(1, 1, 1);
#else
                ChunkID minID, maxID;
                GetBoundaryChunkID(depthImage,camera,cameraPose,maxID,minID);
#endif
                float resolutionDiagonal = chunkSize(0) * voxelResolutionMeters / 2;
                int stepSize = 4;
                float negativeTruncation = 0.03;
                if(voxelResolutionMeters > 0.01)
                {
                    resolutionDiagonal = chunkSize(0) * voxelResolutionMeters * sqrt(3) ;
                    stepSize = 1;
                    negativeTruncation = 0.05 * voxelResolutionMeters / 0.005;
                }
                float fx,fy,cx,cy,width,height;
                fx = camera.GetFx();
                fy = camera.GetFy();
                cx = camera.GetCx();
                cy = camera.GetCy();
                width = camera.GetWidth();
                height = camera.GetHeight();
                int cornerCheckCnt = 0;

                float cornerIndex[8] = {0, chunkSize(0), (chunkSize(1) - 1) * chunkSize(0) - 1, chunkSize(1) * chunkSize(0) - 1,
                                        (chunkSize(2) - 1 ) * chunkSize(1) * chunkSize(0),
                                        (chunkSize(2) - 1 ) * chunkSize(1) * chunkSize(0) + chunkSize(0),
                                        (chunkSize(2) - 1 ) * chunkSize(1) * chunkSize(0) + (chunkSize(1) - 1) * chunkSize(0) - 1 ,
                                        (chunkSize(2) - 1 ) * chunkSize(1) * chunkSize(0) + chunkSize(1) * chunkSize(0) - 1};
                // approximately 100, 000 candidate chunks,
                // while 4, 000 chunks are candidate intersects

                Eigen::MatrixXf rotation = cameraPose.linear().transpose();
                Eigen::VectorXf translation = rotation * cameraPose.translation();
                Vec3 r0 = Vec3(rotation(0,0),rotation(1,0),rotation(2,0)) * float(chunkSize(0)) * voxelResolutionMeters;
                Vec3 r1 = Vec3(rotation(0,1),rotation(1,1),rotation(2,1)) * float(chunkSize(1)) * voxelResolutionMeters;
                Vec3 r2 = Vec3(rotation(0,2),rotation(1,2),rotation(2,2)) * float(chunkSize(2)) * voxelResolutionMeters;


                Vec3 halfVoxel = Vec3(GetResolution(), GetResolution(), GetResolution()) * 0.5f;
                Vec3 originX, originY, originZ;

                int candidateChunks = 0;
                int negativeChunks = 0;
                Vec3List diffCentroidCoarse = Vec3List(8);
                Vec3List diffCentroidRefine = Vec3List(8);
                for(int x = 0; x < 2; x++)
                {
                    for(int y = 0; y < 2; y++)
                    {
                        for(int z = 0; z < 2; z++)
                        {
                            Vec3 cur = Vec3(x * chunkSize(0),y * chunkSize(1), z * chunkSize(2));
                            diffCentroidCoarse[x + y*2 + z*4] =
                                    rotation * cur * voxelResolutionMeters * stepSize + halfVoxel;
                            diffCentroidRefine[x + y*2 + z*4] =
                                    rotation * cur * voxelResolutionMeters * 1 + halfVoxel;
                        }
                    }
                }
                __m256 diffCentroidCoarseSIMD[3];
                __m256 diffCentroidRefineSIMD[3];
                for(int i = 0; i < 3; i++)
                {
                    diffCentroidCoarseSIMD[i] = _mm256_set_ps(diffCentroidCoarse[0](i),diffCentroidCoarse[1](i),diffCentroidCoarse[2](i),diffCentroidCoarse[3](i),
                            diffCentroidCoarse[4](i),diffCentroidCoarse[5](i),diffCentroidCoarse[6](i),diffCentroidCoarse[7](i));
                    diffCentroidRefineSIMD[i] = _mm256_set_ps(diffCentroidRefine[0](i),diffCentroidRefine[1](i),diffCentroidRefine[2](i),diffCentroidRefine[3](i),
                            diffCentroidRefine[4](i),diffCentroidRefine[5](i),diffCentroidRefine[6](i),diffCentroidRefine[7](i));
                }

                for (int x = minID(0) - 1; x <= maxID(0) + 1; x += stepSize)
                {
                    originX = r0 * x - translation;
                    for (int y = minID(1) - 1; y <= maxID(1) + 1; y += stepSize)
                    {
                        originY = originX + r1 * y;
                        for (int z = minID(2) - 1; z <= maxID(2) + 1; z += stepSize)
                        {
                            candidateChunks++;
                            bool intersectFlag = 0;
                            Vec3 originInCamera = originY + z * r2;
//                            Vec3 origin = Vec3(x * chunkSize(0), y * chunkSize(1), z * chunkSize(2)) * voxelResolutionMeters;
//                            Vec3 originInCamera = rotation * (origin - translation);


                            float truncation = -1;
                            truncation = integrator.GetTruncator()->GetTruncationDistance(originInCamera(2));
                            float dtp = truncation + resolutionDiagonal * stepSize;
                            float dtn = negativeTruncation + resolutionDiagonal * stepSize;
#if 0
                            intersectFlag = CheckCornerIntersecting(camera,
                                                                    originInCamera,
                                                                    depthImage,
                                                                    dtp,dtn,
                                                                    diffCentroidCoarse);
#else

                            intersectFlag = CheckCornerIntersectingSIMD(camera,
                                                                        originInCamera,
                                                                        depthImage,
                                                                        dtp,
                                                                        dtn,
                                                                        diffCentroidCoarseSIMD[0],
                                                                        diffCentroidCoarseSIMD[1],
                                                                        diffCentroidCoarseSIMD[2]);
#endif

                                // transform current vertex to camera coordinate
//                            if(intersectFlag)
                            {
                                for(int i = x; i < x + stepSize; i++)
                                {
                                    for(int j = y; j < y + stepSize; j++)
                                    {
                                        for(int k = z; k < z + stepSize; k++)
                                        {
                                            bool refinedIntersectFlag = 0;

//                                            if(HasChunk(ChunkID(i,j,k)) && refinedIntersectFlag)
//                                            {
//                                                refinedIntersectFlag = 1;
//                                            }
                                            if(intersectFlag)
                                            {
                                                Vec3 origin = Vec3(i * chunkSize(0), j * chunkSize(1), k * chunkSize(2)) * voxelResolutionMeters;
                                                originInCamera = rotation * origin - translation;
                                                truncation = integrator.GetTruncator()->GetTruncationDistance(originInCamera(2));
                                                float dtp = truncation + resolutionDiagonal;
                                                float dtn = negativeTruncation + resolutionDiagonal;
#if 0
                                                refinedIntersectFlag = CheckCornerIntersecting(camera,
                                                                                               originInCamera,
                                                                                               depthImage,
                                                                                               dtp,dtn,
                                                                                               diffCentroidRefine);
#else
                                                refinedIntersectFlag = CheckCornerIntersectingSIMD(camera,
                                                                                            originInCamera,
                                                                                            depthImage,
                                                                                            dtp,
                                                                                            dtn,
                                                                                            diffCentroidRefineSIMD[0],
                                                                                            diffCentroidRefineSIMD[1],
                                                                                            diffCentroidRefineSIMD[2]);
#endif

                                            }
                                            if(refinedIntersectFlag )
                                            {
                                                chunkList->push_back(ChunkID(i, j, k));
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

//                printf("select chunk statistics: %d %d\r\n", cornerCheckCnt,chunkList->size());

//                printf("%lu chunks intersect frustum\n", chunkList->size());
            }

            inline bool CheckCornerIntersectingSIMD(const PinholeCamera& camera,
                                                    const Vec3 &originInCamera,
                                                    const float *depthImage,
                                                    float distance_threshold_positive,
                                                    float distance_threshold_negative,
                                                    const __m256 &centroids_simd0,
                                                    const __m256 &centroids_simd1,
                                                    const __m256 &centroids_simd2
                                                    )
            {

                float fx,fy,cx,cy,width,height;
                fx = camera.GetFx();
                fy = camera.GetFy();
                cx = camera.GetCx();
                cy = camera.GetCy();
                width = camera.GetWidth();
                height = camera.GetHeight();
                bool intersectFlag = false;
                __m256 origin_simd0 = _mm256_set1_ps(originInCamera(0));
                __m256 origin_simd1 = _mm256_set1_ps(originInCamera(1));
                __m256 origin_simd2 = _mm256_set1_ps(originInCamera(2));

                __m256 voxelCenterInCamera_SIMD0 = _mm256_add_ps(origin_simd0, centroids_simd0);
                __m256 voxelCenterInCamera_SIMD1 = _mm256_add_ps(origin_simd1, centroids_simd1);
                __m256 voxelCenterInCamera_SIMD2 = _mm256_add_ps(origin_simd2, centroids_simd2);
                __m256 cameraPos_SIMD0 =
                        _mm256_add_ps(_mm256_mul_ps(_mm256_div_ps(voxelCenterInCamera_SIMD0,voxelCenterInCamera_SIMD2),_mm256_set1_ps(fx)),_mm256_set1_ps(cx));
                __m256 cameraPos_SIMD1 =
                        _mm256_add_ps(_mm256_mul_ps(_mm256_div_ps(voxelCenterInCamera_SIMD1,voxelCenterInCamera_SIMD2),_mm256_set1_ps(fy)),_mm256_set1_ps(cy));

                __m256i cameraX_SIMD = _mm256_cvtps_epi32(cameraPos_SIMD0);
                __m256i cameraY_SIMD = _mm256_cvtps_epi32(cameraPos_SIMD1);

                __m256 depthValid_SIMD = _mm256_and_ps(_mm256_cmp_ps(origin_simd2,_mm256_set1_ps(camera.GetNearPlane()),_CMP_GT_OS),
                                                       _mm256_cmp_ps(_mm256_set1_ps(camera.GetFarPlane()),origin_simd2,_CMP_GT_OS));
                __m256i valid = _mm256_and_si256(_mm256_cmpgt_epi32(cameraX_SIMD,_mm256_set1_epi32(1)),_mm256_cmpgt_epi32(_mm256_set1_epi32(width - 1),cameraX_SIMD));
                valid = _mm256_and_si256(valid,_mm256_cmpgt_epi32(cameraY_SIMD,_mm256_set1_epi32(1)));
                valid = _mm256_and_si256(valid,_mm256_cmpgt_epi32(_mm256_set1_epi32(height - 1),cameraY_SIMD));
                __m256 validf = _mm256_castsi256_ps(valid);
                if((_mm256_testz_ps(validf,validf)) )
                {
                    return false;
                }
                __m256i camera_plane_pos_SIMD = _mm256_add_epi32(_mm256_mullo_epi32(cameraY_SIMD,_mm256_set1_epi32(width)),cameraX_SIMD);
                __m256 depth_SIMD = _mm256_mask_i32gather_ps(_mm256_set1_ps(0.0),depthImage,camera_plane_pos_SIMD,validf,4);
                __m256 surfaceDist_SIMD = _mm256_sub_ps(depth_SIMD,voxelCenterInCamera_SIMD2);
                __m256 surfaceInside_SIMD =  _mm256_and_ps(_mm256_cmp_ps(surfaceDist_SIMD,_mm256_set1_ps(-distance_threshold_negative),_CMP_GT_OS),
                                                           _mm256_cmp_ps(_mm256_set1_ps(distance_threshold_positive),surfaceDist_SIMD,_CMP_GT_OS));

                __m256 integrateFlag = _mm256_and_ps(validf,surfaceInside_SIMD);
                integrateFlag = _mm256_and_ps(integrateFlag,depthValid_SIMD);
//                printf("%f %f %f %f %f %f %f %f\r\n",
//                       surfaceDist_SIMD[0],surfaceDist_SIMD[1],surfaceDist_SIMD[2],surfaceDist_SIMD[3],
//                       depth_SIMD[0],depth_SIMD[(1)],depth_SIMD[(2)],depth_SIMD[(3)]);
                if((!_mm256_testz_ps(integrateFlag,integrateFlag)) )
                {
                    intersectFlag = true;
                }
                return intersectFlag;

            }
            inline bool CheckCornerIntersecting(const PinholeCamera& camera,
                                                Vec3 originInCamera,
                                                const float *depthImage,
                                                float distance_threshold_positive,
                                                float distance_threshold_negative,
                                                const Vec3List &diff_centroids
                                                )
            {

                float fx,fy,cx,cy,width,height;
                fx = camera.GetFx();
                fy = camera.GetFy();
                cx = camera.GetCx();
                cy = camera.GetCy();
                width = camera.GetWidth();
                height = camera.GetHeight();
                bool intersectFlag = 0;

                for(int i = 0; i < 8; i++)
                {
                    Vec3 voxelCenterInCamera = originInCamera + diff_centroids[i];
                    Vec3 cameraPos = Vec3(voxelCenterInCamera(0)/voxelCenterInCamera(2)*fx+cx,
                                          voxelCenterInCamera(1)/voxelCenterInCamera(2)*fy+cy,
                                          voxelCenterInCamera(2));

                    if (cameraPos(0) < 1 || cameraPos(0) > width -1 ||
                        cameraPos(1) < 1 || cameraPos(1) > height - 1 ||
                        voxelCenterInCamera(2) < camera.GetNearPlane() || voxelCenterInCamera(2) > camera.GetFarPlane())
                    {
                        continue;
                    }

                    float voxelDist = voxelCenterInCamera.z();
                    int pixelPos = (int)cameraPos(1) * width + (int)cameraPos(0);
                    float depth = depthImage[pixelPos];
                    if(std::isnan(depth))
                    {
                        continue;
                    }
                    float surfaceDist = depth - voxelDist;


                    if(surfaceDist < distance_threshold_positive && surfaceDist > -distance_threshold_negative )
                    {
                        intersectFlag = 1;
                        return intersectFlag;
                    }
                }

                return intersectFlag;

            }

            void GetChunkIDsIntersecting(const AABB& box, ChunkIDList* chunkList);
            void GetChunkIDsIntersecting(const Frustum& frustum, ChunkIDList* chunkList);

            void CreateChunk(const ChunkID& id);

            void GenerateMeshEfficient(const ChunkPtr& chunk, Mesh* mesh,ChunkMap &chunks);
            void GenerateMesh(const ChunkPtr& chunk, Mesh* mesh);
            void ColorizeMesh(Mesh* mesh);
#if 0
            Vec3 InterpolateColor(const Vec3& colorPos);
#endif


            inline Vec3 InterpolateColorNearest(const Vec3 & colorPos)
            {
                const ChunkID& chunkID = GetIDAt(colorPos);
                if(!HasChunk(chunkID))
                {
                    return Vec3(0, 0, 0);
                }
                else
                {
                    const ChunkPtr& chunk = GetChunk(chunkID);
                    return chunk->GetColorAt(colorPos);
                }
            }
            void CacheCentroids();
            void ExtractBorderVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Eigen::Vector3f& coordinates, VertIndex* nextMeshIndex, Mesh* mesh,ChunkMap &chunks);
            void ExtractInsideVoxelMesh(const ChunkPtr& chunk, const Eigen::Vector3i& index, const Vec3& coords, VertIndex* nextMeshIndex, Mesh* mesh,ChunkMap &chunks);

            inline  MeshMap& GetAllMeshes() { return allMeshes; }
            inline MeshMap& GetAllMutableMeshes() { return allMeshes; }
            inline const MeshPtr& GetMesh(const ChunkID& chunkID) const { return allMeshes.at(chunkID); }
            inline MeshPtr& GetMutableMesh(const ChunkID& chunkID) { return allMeshes.at(chunkID); }
            inline bool HasMesh(const ChunkID& chunkID) const { return allMeshes.find(chunkID) != allMeshes.end(); }

            inline bool GetUseColor() { return useColor; }

            void RecomputeMeshes(const ChunkSet& chunks, const PinholeCamera &camera);
            void RecomputeMeshes(const ChunkSet& chunks, const PinholeCamera &camera, MeshMap &_allMeshes);
            //void ComputeNormalsFromGradients(Mesh* mesh,ChunkMap &chunks);

            void GetColorByProject(const PinholeCamera& camera, MeshPtr &mesh);
            inline const Eigen::Vector3i& GetChunkSize() const { return chunkSize; }
            inline float GetResolution() const { return voxelResolutionMeters; }

            inline const Vec3List& GetCentroids() const { return centroids; }

            void PrintMemoryStatistics();

            void Reset();

            bool GetSDFAndGradient(const Eigen::Vector3f& pos, Eigen::Vector3f &grad,ChunkMap &chunks);

            // cubicVoxelID: the id (from 0~7) of the corner
            // cubicVoxelIndex: the index (from 0~511) of the corner
            bool extractGradientFromCubic(float *cubicSDFPointer,
                                          Point3 &currentVoxelID,
                                          int cubicVoxelID,
                                          int cubicVoxelIndex,
                                          float   *neighborSDFPointer,
                                          ChunkID &neighborChunkID,
                                          Eigen::Vector3f &grad,ChunkMap &chunks);

            bool GetSDF(const Eigen::Vector3f& pos, double* dist);
            bool GetWeight(const Eigen::Vector3f & pos, double *weight);

            inline bool GetNeighborSDF(bool neighborFlag,
                                const Eigen::Vector3i &cid,
                                int voxelIndex,
                                const ChunkPtr &chunkCentral,
                                float &dd,ChunkMap& chunks)
            {
#if TEST_GRADIENT
                std::cout << "normal selection: " << neighborFlag << " " << cid.transpose() << " " << voxelIndex << "   ";
#endif
                if(neighborFlag == 0)
                {
                    dd = chunkCentral->voxels.sdf[voxelIndex];
#if TEST_GRADIENT
                    std::cout << "normal selection value: " << dd << std::endl;
#endif
                    if(dd < 1)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                    return true;
                }
                else
                {
                    ChunkMap::iterator got = chunks.find(cid);
                    if(got != chunks.end())
                    {
                        chisel::ChunkPtr &chunk = got->second;
                        dd = chunk->voxels.sdf[voxelIndex];
#if TEST_GRADIENT
                        std::cout << "normal selection value: " << dd << std::endl;
#endif
                        if(dd < 1)
                        {
                            return true;
                        }
                        else
                        {
                            return false;
                        }
                        return true;

                    }

                }
                return false;
            }
            inline bool GetNeighborSDF(bool neighborFlag,
                                const Eigen::Vector3i &cid,
                                int voxelIndex,
                                float *currentChunkSDFPointer,
                                float &dd,ChunkMap & chunks)
            {
#if TEST_GRADIENT
                std::cout << "smart selection: " << neighborFlag << " " << cid.transpose() << " " << voxelIndex << "   ";
#endif
                if(neighborFlag == 0)
                {
                    dd = currentChunkSDFPointer[voxelIndex];
#if TEST_GRADIENT
                    std::cout << "smart selection value: " << dd << std::endl;
#endif
                    if(dd < 1)
                    {
                        return true;
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    ChunkMap::iterator got = chunks.find(cid);
                    if(got != chunks.end())
                    {
                        chisel::ChunkPtr &chunk = got->second;
                        dd = chunk->voxels.sdf[voxelIndex];
#if TEST_GRADIENT
                        std::cout << "smart selection value: " << dd << std::endl;
#endif
                        if(dd < 1)
                        {
                            return true;
                        }
                        else
                        {
                            return false;
                        }
                    }

                }
                return false;
            }

            inline void deepCopyChunks(ChunkMap &c,
                                       const std::vector<int> &PoseChanged)
            {
                for (const std::pair<ChunkID, ChunkPtr>& chunk:chunks)
                {
//                    if(!PoseChanged[anchorFrameIndex])
//                    {
//                        continue;
//                    }
                    c.insert(std::make_pair(chunk.first,std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), chunk.first, chunkSize, voxelResolutionMeters, useColor)));
                    *c.at(chunk.first) = *chunk.second;


                }
            }

/*
This part was I write to transform a submap's TSDF
*/
            float readSDFFromInterpolated(const Vec3 &coord, ChunkMap &_chunks)
            {
                float v1,v2,res1,res2;
                Point3 pos = Point3(floor(coord(0)),floor(coord(1)), floor(coord(2)));
                Vec3 coeff = coord - Vec3(pos(0),pos(1),pos(2));
                v1 = readSDF(pos+Point3(0,0,0),_chunks);
                v2 = readSDF(pos+Point3(1,0,0),_chunks);
                res1 = (1.0-coeff.x()) * v1 + coeff.x() * v2;
                v1 = readSDF(pos+Point3(0,1,0),_chunks);
                v2 = readSDF(pos+Point3(1,1,0),_chunks);
                res1=(1.0-coeff.x())*res1 + coeff.y()*((1.0-coeff.x())*v1 + coeff.x()*v2);
                v1 = readSDF(pos+Point3(0,0,1),_chunks);
                v2 = readSDF(pos+Point3(1,0,1),_chunks);
                res2 = (1.0-coeff.x())*v1+coeff.x() * v2;
                v1 = readSDF(pos+Point3(0,1,1),_chunks);
                v2 = readSDF(pos+Point3(1,1,1),_chunks);
                res2=(1.0-coeff.x())*res2 + coeff.y()*((1.0-coeff.x())*v1 + coeff.x()*v2);      
                return (1.0-coeff.z())*res1 + coeff.z()*res2;          
                
            }
            float readSDF(const Point3 &pos, ChunkMap &_chunks)
            {
                ChunkID chunkID = GetPointIDAt(pos);
                if(_chunks.find(chunkID) ==_chunks.end()) 
                return 999;
                return _chunks[chunkID]->voxels.GetSDF(_chunks[chunkID]->GetLocalVoxelIDFromGlobal(pos));
            }
            void Merge(const ChunkMap &another_chunks)
            {
                for(auto iter = another_chunks.begin(); iter != another_chunks.end(); ++iter)
                {
                    if(HasChunk(iter->first))
                    {
                        chunks[iter->first]->integrateTwoChunksSIMD(iter->second);
                    }
                    else 
                    {
                        CreateChunk(iter->first);
                        chunks[iter->first]->integrateTwoChunksSIMD(iter->second);
                    }
                }
            }
            /*
            float readRGB(const Point3 &pos, ChunkMap &_chunks)
            {
                ChunkID chunkID = GetPointIDAt(pos);
                if(_chunks.find(chunkID) ==_chunks.end()) 
                return 0;
                return _chunks[chunkID]->colors.GetSDF(_chunks[chunkID]->GetLocalVoxelIDFromGlobal(pos));
            }*/
            ChunkMap ChunkTransformDense(const Transform &T)
            {
                Eigen::Matrix3f rotation =T.rotation();
                Eigen::VectorXf translation = T.translation();
                std::vector<VoxelMap > chunksCoords;
                    std::cout<<"GetVoxel world coord!"<<std::endl;
                for(auto i = chunks.begin();i!=chunks.end();++i)
                {
                    /*i: chunk*/
                      chunksCoords.push_back(i->second->GetworldCoords());
                }
        /*Now we got the chunks Coords*/
            VoxelMap afterTransform;
            std::cout<<"transform voxels: R:"<<rotation<<"\n"<<translation<<std::endl;
            for(int i = 0;i!= chunksCoords.size(); ++i)
            {
                for(auto j = chunksCoords[i].begin();j!=chunksCoords[i].end();++j)
                {
                Vec3 newCoord =  rotation * j->first + translation/voxelResolutionMeters;
                float _sdf = readSDFFromInterpolated(j->first,chunks);
                //newCoord = newCoord+Vec3(1,0,0);
                if(afterTransform.find(newCoord) == afterTransform.end())
                {
                    afterTransform[newCoord] = j->second;
                    afterTransform[newCoord][0] = _sdf;
                }
                else
                {
                    /*There is a possibility that multiple voxels are mapped to the same voxel. If that happens, use weighted sdf to compute the sdf*/
                    std::cout<<"The same position！"<<std::endl;
                    afterTransform[newCoord][0] = (afterTransform[newCoord][0] * afterTransform[newCoord][1] + j->second[0]*j->second[1])/(afterTransform[newCoord][1] + j->second[1]);
                    afterTransform[newCoord][1] += j->second[1];
                }

                //
                }
            }

        /*transform the voxels to Chunks*/
            ChunkMap result;
            std::cout<<"voxels: "<<afterTransform.size()<<std::endl;
            std::cout<<"translate voxels to chunk!"<<std::endl;
            for(auto i=afterTransform.begin();i!=afterTransform.end();++i)
            {

                Eigen::Vector3f pos = i->first;

                Eigen::Vector3i coord = Eigen::Vector3i(int(pos(0)),int(pos(1)),int(pos(2)));

                ChunkID chunkID = GetPointIDAt(coord);
            if(result.find(chunkID) == result.end())
            {
                result.insert(std::make_pair(chunkID,std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), chunkID, chunkSize, voxelResolutionMeters, useColor)));
                result[chunkID]->voxels.Reset();
                for(int vid = 0;vid!= 512;++vid)
                {
                result[chunkID]->voxels.SetSDF(readSDFFromInterpolated(result[chunkID]->GetVoxelCoordsFromVoxelID(vid),chunks),vid);
                result[chunkID]->voxels.SetWeight(i->second[1],vid);

                }
                int voxelID = result[chunkID]->GetLocalVoxelIDFromGlobal(coord);
                result[chunkID]->voxels.SetSDF(i->second[0],voxelID);
                result[chunkID]->voxels.SetWeight(i->second[1],voxelID); 

                           
            }
            else
            {
            //std::cout<<"b,voxelIDInt: "<<coord<<std::endl;   
            //std::cout<<"ChunkID: "<<chunkID<<std::endl;
            int voxelID = result[chunkID]->GetLocalVoxelIDFromGlobal(coord);
            result[chunkID]->voxels.SetSDF(i->second[0],voxelID);
            result[chunkID]->voxels.SetWeight(i->second[1],voxelID);    
            }
            }

            return result;

            }


            ChunkMap ChunkTransform(const Transform &T)
            {
                //return chunks;
                Eigen::Matrix3f rotation =T.rotation();                
                Eigen::VectorXf translation = T.translation();
                std::vector<VoxelMap > chunksCoords;
                    std::cout<<"GetVoxel world coord!"<<std::endl;
                for(auto i = chunks.begin();i!=chunks.end();++i)
                {
                      chunksCoords.push_back(i->second->GetworldCoords());
                }
            VoxelMap afterTransform;
            std::cout<<"transform voxels: R: "<<rotation<<"\nt: "<<translation<<std::endl;
            for(int i = 0;i!= chunksCoords.size(); ++i)
            {
                for(auto j = chunksCoords[i].begin();j!=chunksCoords[i].end();++j)
                {
                Vec3 newCoord =  rotation * j->first + translation/voxelResolutionMeters;
                //float _sdf = readSDFFromInterpolated(j->first,chunks);
                if(afterTransform.find(newCoord) == afterTransform.end())
                {
                    afterTransform[newCoord] = j->second;
                }
                else
                {
                    afterTransform[newCoord][0] = (afterTransform[newCoord][0] * afterTransform[newCoord][1] + j->second[0]*j->second[1])/(afterTransform[newCoord][1] + j->second[1]);
                    afterTransform[newCoord][1] += j->second[1];
                }
                }
            }

        /*transform the voxels to Chunks*/
            ChunkMap result;
            std::cout<<"voxels: "<<afterTransform.size()<<std::endl;
            std::cout<<"translate voxels to chunk!"<<std::endl;
            //int min_x=0x7fffffff,min_y=0x7fffffff,min_z=0x7fffffff;
            //int max_x=0x80000000,max_y=0x80000000,max_z=0x80000000;
            for(auto i=afterTransform.begin();i!=afterTransform.end();++i)
            {
        /*Note that this part we haven't consider the weighted sdf*/
                //Eigen::Vector3f pos = i->first;
                Eigen::Vector3i coord = i->first.cast<int>(); /*Eigen::Vector3i(std::floor(pos.x() / voxelResolutionMeters) ,
                std::floor(pos.y() / voxelResolutionMeters) ,
                std::floor(pos.z() / voxelResolutionMeters));*/
                ChunkID chunkID = GetPointIDAt(coord);
                //std::cout <<"coord: "<<i->first<< " chunkID: "<<chunkID<<std::endl;
            if(result.find(chunkID) == result.end())
            {

                result.insert(std::make_pair(chunkID,std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), chunkID, chunkSize, voxelResolutionMeters, useColor)));
                result[chunkID]->voxels.Reset();
                int voxelID = result[chunkID]->GetLocalVoxelIDFromGlobal(coord);
                result[chunkID]->voxels.SetSDF(i->second[0],voxelID);
                result[chunkID]->voxels.SetWeight(i->second[1],voxelID);
                result[chunkID]->colors.setRed(i->second[2],voxelID);
                result[chunkID]->colors.setGreen(i->second[3],voxelID);
                result[chunkID]->colors.setBlue(i->second[4],voxelID);   
                result[chunkID]->colors.setDivision(i->second[5],voxelID);                    
            }
            else
            {
            int voxelID = result[chunkID]->GetLocalVoxelIDFromGlobal(coord);
            result[chunkID]->voxels.SetSDF(i->second[0],voxelID);
            result[chunkID]->voxels.SetWeight(i->second[1],voxelID);    
            result[chunkID]->colors.setRed(i->second[2],voxelID);
            result[chunkID]->colors.setGreen(i->second[3],voxelID);
            result[chunkID]->colors.setBlue(i->second[4],voxelID);
            result[chunkID]->colors.setDivision(i->second[5],voxelID);  
            }
            }
            std::cout<<"transform down!"<<std::endl;
            return result;

            }

            int GetChunkCount()
            {
                return chunks.size();
            }
            /*
            Now the save submap just save the chunks(TSDF) and the pos, 
            about frames, I need a new scheme to handle the lCD problem and optimization.
            */
            void saveSubmap(const std::string & filename)
            {
            if(!isValid)
            return;
            std::ofstream ofs(filename);
            //std::cout<<"is valid: "<<isValid <<std::endl;
            isValid = false;   
            int size = chunks.size();
            ofs<<size<<"\n";
            //std::cout<<"chunks size: "<<size <<std::endl;
            for( auto begin = chunks.begin(); begin != chunks.end(); ++ begin)
            {
            //std::cout<<"---------------------"<<begin->first<<begin->first <<std::endl;
                ofs<<begin->second->toString();
            }
            ofs.close();
         
            }
            void saveSubmapB(const std::string & filename)
            {
            if(!isValid)
            return;
            std::ofstream ofs(filename,std::ios::binary);
            std::vector<float> buffer;
            //std::cout<<"is valid: "<<isValid <<std::endl;
            isValid = false;   
            buffer.push_back(0.0);
            int size = chunks.size();
            buffer.push_back(size);
            std::cout<<"chunks size: "<<buffer[1] <<std::endl;
            for( auto begin = chunks.begin(); begin != chunks.end(); ++ begin)
            {
            //std::cout<<"---------------------"<<begin->first<<begin->first <<std::endl;
                begin->second->toVector(buffer);
            }
            bufferSize = buffer.size();
            *((unsigned int *)(&buffer[0])) = buffer.size();
            ofs.write((char *) &buffer[0],sizeof(float) * buffer.size());
            //std::cout<<buffer[0]<<" "<<bufferSize<<std::endl;
            }       
            void loadSubmap(const std::string & filename)
            {
            if(isValid)
            return;
            std::ifstream ifs(filename);
            int size , i = 0;
            ifs>>size;
            ifs.get();
            std::cout<<"start load chunks!" <<std::endl;
            while(i!=size)
            {
                ChunkPtr chunk = std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), ChunkID(0,0,0), chunkSize, voxelResolutionMeters, useColor);
                chunk->fromStream(ifs);

                chunk->computeOrigin();
                chunks[chunk->GetID()] = chunk;
                ++i;
            }
            isValid = true;
            std::cout<<"load chunks done!" <<std::endl;
            }
            void loadSubmapB(const std::string & filename)
            {
                if(isValid)
                return;
                std::ifstream ifs(filename,std::ifstream::binary);
                std::vector<float> buffer;
                /*
                size_t buffer_size;
                ifs.read((char*)&buffer_size, sizeof(buffer_size));
                bufferSize = buffer_size;
                buffer.resize(bufferSize);
                //
                */
                ifs.seekg (0, ifs.end);
                size_t length = ifs.tellg();
                ifs.seekg (0, ifs.beg);
                buffer.resize(length/sizeof(float));
                ifs.read((char *)&buffer[0],length);
                bufferSize = buffer[0];
                size_t size = buffer[1];
                if (ifs)
                std::cout << "all characters read successfully.";
                else
                std::cout << "error: only " << ifs.gcount() << " could be read";
                ifs.close();                
                size_t ptr = 2;
                size_t i = 0;
                while(i<size)
                {
                    
                    ChunkPtr chunk = std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), ChunkID(0,0,0), chunkSize, voxelResolutionMeters, useColor);
                    chunk->fromVector(buffer,ptr);
                    chunk->computeOrigin();
                    chunks[chunk->GetID()] = chunk;
                    
                    ++i;
                }
                isValid = true;
                std::cout<<"Buffer Size: "<<bufferSize<<std::endl;
                std::cout<<"length: "<<length<<std::endl;
                std::cout<<"Chunk Size: "<<size<<std::endl;
                std::cout<<"load chunks done!(from Binary) "<<std::endl;
                
                
            }
            void clearChunks()
            {
                chunks.clear();
                ChunkMap r_chunks = ChunkMap();
                chunks.swap(r_chunks);
            }
            void clearMeshes()
            {
                //allMeshes.erase(allMeshes.begin(),allMeshes.end());
                allMeshes.clear();
                MeshMap r_allMeshes = MeshMap();
                allMeshes.swap(r_allMeshes);
                //allMeshes.clear();
                //allMeshes.swap(MeshMap());
            }

            void clearSemanticMeshes()
            {
                semantic_mesh->clearCompact();
                semantic_mesh->clearNotCompact();
            }

            int isSimplified()
            {
                return is_simplified;
/*                while(usingMesh.test_and_set()){}
                //std::cout<<"Is Simplified?????????????????????????"<<allMeshes.empty()<<(simplified_mesh == nullptr)<<std::endl;
                if(allMeshes.empty() && (!simplified_mesh ||simplified_mesh->empty()))//empty mesh, should never happen
                return 0;

                if(allMeshes.empty() && !simplified_mesh->empty())//simplified mesh
                return 1;
                if(!allMeshes.empty())//not simplified
                return 2;
*/
            }
//*/
            int isSemanticSimplified()
            {
                return is_simplified;
            }
            void prepareSimplifiedMesh()
            {
                if(simplified_mesh)
                return;
                else simplified_mesh = MeshPtr(new Mesh());
            }
            void prepareSemanticMesh()
            {
                if(semantic_mesh)
                return;
                else semantic_mesh = MeshPtr(new Mesh());
            }
            void setChunks(ChunkMap &_chunks)
            {
                for (const std::pair<ChunkID, ChunkPtr>& chunk:_chunks)
                {
                    chunks.insert(std::make_pair(chunk.first,std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), chunk.first, chunkSize, voxelResolutionMeters, useColor)));
                    *chunks.at(chunk.first) = *chunk.second;


                }
            }
            void clear_hash_aggregated_map()
            {
                std::vector<int>().swap(hash_aggregated_map); 
            }
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            bool is_simplified = false;
            bool isValid = true;
            bool is_active = true;
            //a indicator of whether the chunkmanager's chunks were propogated into the net. 
            bool propogated = false;
            int submap_id = -1;
            MeshMap allMeshes;
            MeshPtr simplified_mesh; 
            MeshPtr semantic_mesh;
            ChunkMap chunks;
            AggregatedHash aggregated_hash;
            //int lastChunkCount=0;
            std::vector<int> hash_aggregated_map;//each point to simplifed_id
            std::vector<int> voxelNeighborIndex;
            Eigen::Vector3i chunkSize;
            float voxelResolutionMeters;
            Vec3 halfVoxel;
            Vec3List centroids;
            protected:
            Eigen::Matrix<int, 3, 8> cubeIndexOffsets;
            std::vector<int,Eigen::aligned_allocator<Eigen::Vector4f> > neighborVoxelIndexChunkWise;
            std::vector<int,Eigen::aligned_allocator<Eigen::Vector4f> > neighborChunkIndexChunkWise;
            std::vector<Vec3, Eigen::aligned_allocator<Eigen::Vector4f> > interpolatedPoints;
            int edgeIndexPairs[12][2];

            size_t bufferSize;
            bool useColor; 
            Eigen::Matrix4f pos;//the submap's pos, need to be updated by global optimization.

    };
} // namespace chisel 

#endif // CHUNKMANAGER_H_ 
