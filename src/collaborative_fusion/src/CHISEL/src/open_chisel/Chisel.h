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

#ifndef CHISEL_H_
#define CHISEL_H_

#include <open_chisel/threading/Threading.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/geometry/Frustum.h>
#include "Stopwatch.h"
#include <chrono>
#include <thread>
#include <set>
#include <open_chisel/simplify/Simplify.h>
#include"half.hpp"
#include <open_chisel/mesh/PointCloud.h>
#include "ForLabel.h"
using namespace half_float;

//#define SUBMAP_MAX_VERTEX 10000000
#define REFERENCE_SUBMAP_RECOMPUTING 1
namespace chisel
{ 
    static void toFloat16(const float &a, void * f16)
    {   
//std::cout<<"to float:"<<a<<std::endl;
        unsigned char *_f16 = (unsigned char*)f16;
        
        half_float::half f=half_float::half_cast<half_float::half,std::round_to_nearest> (a);
        unsigned char * data = (unsigned char *)&f.data_;


        _f16[0] = data[0];
        _f16[1]= data[1];
    }
    class Chisel
    {
        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Chisel()=default;
            Chisel(const Eigen::Vector3i& chunkSize, float voxelResolution, bool useColor,const PinholeCamera &camera);
            virtual ~Chisel();

            inline const ChunkManager& GetChunkManager(int submapID) const { return chunkManagers[submapID]; }
            inline ChunkMap& GetChunkMap(int submapID) { return chunkManagers[submapID].chunks; }
            inline const std::vector<ChunkManager> & GetAllChunkManager() const{return chunkManagers;}
            inline ChunkManager& GetMutableChunkManager(int submapID) { return chunkManagers[submapID]; }
            //inline void SetChunkManager(int submapID,const ChunkManager& manager) { chunkManagers[submapID] = manager; }
            void GetAllMeshesSimplified(int submapID);
            void GetSubmapMeshSimplified();
            void GetSubmapMeshSimplified(int submapID);
            int GetFullMeshes(unsigned char *vertices,int submapID,int start=0);
            int GetFullMeshes(MeshPtr mesh,unsigned char *vertices,int v,PoseSE3d relativeChange = PoseSE3d(),PoseSE3d cameraPose=PoseSE3d());
            MeshPtr GetFullMeshes();
            int GetFullMeshes(unsigned char *vertices,MeshMap &all,size_t v = 0,PoseSE3d relativeChange = PoseSE3d(),PoseSE3d cameraPose=PoseSE3d(),Eigen::Vector3f turbulence=Eigen::Vector3f(0,0,0));
            //int GetFullMeshes(MeshPtr mesh,unsigned char *vertices);
            int GetFullMeshes(unsigned char * vertices, int mainSubmapID,int referenceSubmapID, PoseSE3dList &submapPosesRelativeChanges,PoseSE3d &cameraPose);
            //void static _GetFullMeshes(unsigned char *vertices, const MeshPtr & it,int start,int end);
            int GetFullMeshes(unsigned char *vertices);
            
            int GetChunkCount(int submapID);
            int GetSubmapSize();
            void bufferIntegratorSIMDCentroids(ProjectionIntegrator& integrator, const Transform &depthExtrinsic,int submapID);
            void bufferIntegratorSIMDCentroids(ProjectionIntegrator& integrator,const Transform& depthExtrinsic);

            void ClearGlobalChunkManager()
            {
                globalChunkManager.Reset();
            }
            //for semantic 
            int GetSegmentationInput( int submap_id, float *hash_aggregated_coords, 
                float *hash_aggregated_feature, int start, 
                const PoseSE3d & relativeChange, const PoseSE3d & cameraPose);
            int GetSegmentationInput(float *hash_aggregated_coords, 
                float *hash_aggregated_feature, const PoseSE3dList & relativeChanges, const PoseSE3d & cameraPose);
            int GetSegmentationInput(float *hash_aggregated_coords, float *hash_aggregated_feature, 
                const std::set<int> &unlabeled_submap, const PoseSE3dList & relativeChanges, const PoseSE3d & cameraPose);
            int GetSegmentationInputGlobal(float *hash_aggregated_coords, float *hash_aggregated_feature);

            int UpdateSemanticMeshWithLabel(int submap_id, long *semantic_label, long * instance_label);
            int UpdateSemanticMeshWithLabel(long *semantic_label, long * instance_label);
            int UpdateSemanticMeshWithLabelGlobal(long *semantic_label, long * instance_label);
            int UpdateSemanticMeshWithLabel(long *semantic_label, long * instance_label, const std::set<int> &unlabeled_submap);
            int GetPointCloud(int submap_id, Vec3List &points, Vec3List &normals, std::vector<double> & residuals);
            int GetPointCloud(Vec3List &points, Vec3List &normals, std::vector<double> &residuals);
            int GetPointCloud(Vec3List &points, Vec3List &normals, 
                std::vector<double> &residuals, const std::set<int> &unlabeled_submap);
            int GetFloorPCD(Vec3List &points);
            int GetFloorPCD(Vec3List &points, const std::set<int> &unlabeled_submap);
            int GetFloorPCD(int submap_id, Vec3List &points);
            Eigen::Vector3f GetFloorNormal(int submap_id);
            int GetNewPoints(int submap_id, Vec3List &points, Vec3List &normals);
            int GetNewPoints(Vec3List &points, Vec3List &normals);
            int GetSemanticMeshesNotSimplified(int submap_id, int start, unsigned char *vertices, 
                int * instance_centers_ptr, const PoseSE3d &relativeChange, const PoseSE3d &cameraPose);
            int GetSemanticMeshesSimplified(int submap_id, int start, unsigned char *vertices, 
                int * instance_centers_ptr, const PoseSE3d &relativeChange, const PoseSE3d &cameraPose);
            int GetAllSemanticMeshes(unsigned char *vertices, int * instance_centers_ptr ,
                PoseSE3dList & submapPoses, PoseSE3d & cameraPose);
            int GetRoomSemanticMeshes(unsigned char *vertices, int *instance_centers_ptr, const std::set<int> &room_submaps,
                PoseSE3dList &submapPoses, PoseSE3d &cameraPose);
            void SimplifySemanticSubmapMeshes(int submapID);
            void SaveAllSubmap(const std::string &filename);
            void GetSearchRegion(float *corners,
                                 const PinholeCamera& depthCamera,
                                 const Transform& depthExtrinsic,int submapID)
            {

                ChunkID minChunkID = minChunkIDs[submapID];
                ChunkID maxChunkID = maxChunkIDs[submapID];
                float minX = minChunkID(0) * float(chunkManagers[submapID].GetChunkSize()(0)) * chunkManagers[submapID].GetResolution();
                float minY = minChunkID(1) * float(chunkManagers[submapID].GetChunkSize()(1)) * chunkManagers[submapID].GetResolution();
                float minZ = minChunkID(2) * float(chunkManagers[submapID].GetChunkSize()(2)) * chunkManagers[submapID].GetResolution();

                float maxX = maxChunkID(0) * float(chunkManagers[submapID].GetChunkSize()(0)) * chunkManagers[submapID].GetResolution();
                float maxY = maxChunkID(1) * float(chunkManagers[submapID].GetChunkSize()(1)) * chunkManagers[submapID].GetResolution();
                float maxZ = maxChunkID(2) * float(chunkManagers[submapID].GetChunkSize()(2)) * chunkManagers[submapID].GetResolution();

                ChunkManager &chunkManager = chunkManagers[submapID];

                std::cout << "min: " << minX << " " << minY << " " << minZ << " max: ";
                std::cout << maxX << " " << maxY << " " << maxZ << std::endl;

                float vertexList[24] =
                {
                    minX, minY, minZ,
                    maxX, minY, minZ,
                    minX, maxY, minZ,
                    maxX, maxY, minZ,
                    minX, minY, maxZ,
                    maxX, minY, maxZ,
                    minX, maxY, maxZ,
                    maxX, maxY, maxZ
                };
                memcpy(corners, vertexList, 24 * sizeof(float));
            }



            size_t GetOccupyMemory()
            {
                size_t result = 0;
                for(int i = 0;i!= candidateCubes.size(); ++i)
                result += sizeof(float) * candidateCubes[i].capacity();
                result += sizeof(ChunkID) *2 *maxChunkIDs.size();
                result += sizeof(chunkSize);
                result += (sizeof(ChunkID) + sizeof(bool)) * meshesToUpdate.size();
                result += sizeof(float) + sizeof(bool) * 2+ sizeof(PinholeCamera) + sizeof(double) ;
                size_t temp = 0;
                for(int i = 0;i!=chunkManagers.size();++i)
                {
                    temp = chunkManagers[i].GetOccupyMemory();
                    std::cout<<"submap "<<i<<": "<<temp/1024.0/1024.0<<std::endl;
                    result+=temp;
                }
                temp = globalChunkManager.GetOccupyMemory();
                std::cout<<"Global map : "<<temp/1024.0/1024.0<<std::endl;
                result += temp;
                return result;
            }
            float EvaluateReferenceFrame(const ChunkID &cID, const PinholeCamera& camera,
                                         const Eigen::Matrix4f &pos,int submapID)
            {
                ChunkManager & chunkManager = chunkManagers[submapID];
                float threshold = 0.2;
                if(!chunkManager.HasMesh(cID))
                {
                    return 0.2;
                }
                const MeshPtr &mesh = chunkManager.GetMesh(cID);
                float fx = camera.GetFx();
                float fy = camera.GetFy();
                float avgLoc = 0;
                Vec3 chunkCenter = chunkManager.GetChunkCenter(cID);

                Vec3 vertex = chunkCenter;
                Vec4 localVertex = pos * Vec4(vertex(0),vertex(1),vertex(2),1);
                localVertex = localVertex / localVertex(2);
//                float x = localVertex(0) / fx;
//                float y = localVertex(1) / fy;
//                avgLoc = fmax(0.2,fmax(x,y));

                Vec3 viewAngle = Vec3(localVertex(0),localVertex(1),localVertex(2));
                viewAngle.normalize();
                float viewQuality = mesh->averageNoraml.dot(viewAngle);

                return viewQuality;
            }
            void UpdateReferenceFrame(ChunkIDList &chunksIntersecting,
                                   int keyframeIndex,
                                   const Mat4x4 pos,
                                   const float * normalPointer,
                                   const unsigned char * colorPointer,
                                   const float * depthPointer,
                                      const PinholeCamera &camera,
                                   int submapID)
            {
                ChunkManager &chunkManager = chunkManagers[submapID];
                for ( const ChunkID& chunkID : chunksIntersecting)
                {
                    if (chunkManager.HasChunk(chunkID))
                    {

                        ChunkPtr chunk = chunkManager.GetChunk(chunkID);
                        if(chunk->GetReferenceFrameIndex() < 0 || EvaluateReferenceFrame(chunkID, camera,pos,submapID) <= EvaluateReferenceFrame(chunkID,camera,chunk->GetReferenceFramePose(),submapID))
                        {
                            chunk->UpdateReferenceFrame(keyframeIndex, pos, normalPointer, colorPointer,depthPointer);
                        }
                    }
                }
            }

/*this part is used to prepare the chunks in the camera's frustum*/
            void PrepareIntersectChunks(ProjectionIntegrator& integrator,
                                                              float *depthImage,
                                                              const Transform& depthExtrinsic,
                                                              const PinholeCamera& depthCamera,
                                                              ChunkIDList &chunksIntersecting,
                                                              std::vector<bool> &needsUpdateFlag,
                                                              std::vector<bool> &newChunkFlag,
                                                              int submapID
                                                              )
            {
                ChunkManager &chunkManager = chunkManagers[submapID];
                //chunkManager.lastChunkCount = chunkManager.GetChunkCount();
                
                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::1::prepareChunks");
                //std::cout<<"start 1"<<std::endl;
                bufferIntegratorSIMDCentroids(integrator, depthExtrinsic,submapID);
                
                chunksIntersecting.clear();
                needsUpdateFlag.clear();
                newChunkFlag.clear();
                //std::cout<<"start 1.0"<<std::endl;
                //std::cout<<maxChunkIDs[submapID]<<" "<<minChunkIDs[submapID]<<std::endl;
                chunkManager.GetBoundaryChunkID(depthImage,depthCamera,depthExtrinsic,maxChunkIDs[submapID],minChunkIDs[submapID]);
                
                Frustum frustum;
                depthCamera.SetupFrustum(depthExtrinsic, &frustum);
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::1::prepareChunks");

                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::2::collectChunks");
                //std::cout<<"start 2"<<std::endl;
                chunkManager.GetChunkIDsObservedByCamera(integrator,frustum,&chunksIntersecting, depthImage,depthCamera,depthExtrinsic);
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::2::collectChunks");
                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");
                //std::cout<<"start 3"<<std::endl;
                //GetChunkCubes(candidateCubes[submapID],chunksIntersecting,submapID);
                if(submapID ==0)
                {
                for ( const ChunkID& chunkID : chunksIntersecting)
                {
                    bool chunkNew = false;
                    if (!chunkManager.HasChunk(chunkID) )
                    {
                       chunkNew = true;
                       chunkManager.CreateChunk(chunkID);
                    }
                    newChunkFlag.push_back(chunkNew);
                    needsUpdateFlag.push_back(false);
                }
                ////TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");
                }
                else
                {
                    ChunkManager &lastChunkManager = chunkManagers[submapID - 1];
                    for ( const ChunkID& chunkID : chunksIntersecting)
                    {
                    bool chunkNew = false;
                    if (!chunkManager.HasChunk(chunkID) && !lastChunkManager.HasChunk(chunkID))
                    {
                       chunkNew = true;
                       chunkManager.CreateChunk(chunkID);
                    }
                    newChunkFlag.push_back(chunkNew);
                    needsUpdateFlag.push_back(false);
                    }

                }
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");
            }
            void SimplifyAllSemantic()
            {
                for(int i = chunkManagers.size() - 1; i >= 0; --i)
                {
                    if(chunkManagers[i].isSemanticSimplified())
                    break;
                    SimplifySemanticSubmapMeshes(i);
                }
            }
            void PrepareIntersectChunksGlobal(ProjectionIntegrator& integrator,
                                                              float *depthImage,
                                                              const Transform& depthExtrinsic,
                                                              const PinholeCamera& depthCamera,
                                                              ChunkIDList &chunksIntersecting,
                                                              std::vector<bool> &needsUpdateFlag,
                                                              std::vector<bool> &newChunkFlag
                                                              )
            {
                ChunkManager &chunkManager = globalChunkManager;
                //chunkManager.lastChunkCount = chunkManager.GetChunkCount();
                
                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::1::prepareChunks");
                //std::cout<<"start 1"<<std::endl;
                bufferIntegratorSIMDCentroids(integrator, depthExtrinsic);
                
                chunksIntersecting.clear();
                needsUpdateFlag.clear();
                newChunkFlag.clear();
                //std::cout<<"start 1.0"<<std::endl;
                //std::cout<<maxChunkIDs[submapID]<<" "<<minChunkIDs[submapID]<<std::endl;
                chunkManager.GetBoundaryChunkID(depthImage,depthCamera,depthExtrinsic,globalMaxChunkID,globalMinChunkID);
                
                Frustum frustum;
                depthCamera.SetupFrustum(depthExtrinsic, &frustum);
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::1::prepareChunks");

                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::2::collectChunks");
                //std::cout<<"start 2"<<std::endl;
                chunkManager.GetChunkIDsObservedByCamera(integrator,frustum,&chunksIntersecting, depthImage,depthCamera,depthExtrinsic);
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::2::collectChunks");
                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");
                //std::cout<<"start 3"<<std::endl;
                GetChunkCubes(globalCandidateCube,chunksIntersecting);

                for ( const ChunkID& chunkID : chunksIntersecting)
                {
                    bool chunkNew = false;
                    if (!chunkManager.HasChunk(chunkID) )
                    {
                       chunkNew = true;
                       chunkManager.CreateChunk(chunkID);
                    }
                    newChunkFlag.push_back(chunkNew);
                    needsUpdateFlag.push_back(false);
                }
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");
            }
            void PrepareIntersectChunksGlobal(ProjectionIntegrator& integrator,
                                                              float *depthImage,
                                                              const Transform& depthExtrinsic,
                                                              const PinholeCamera& depthCamera,
                                                              ChunkIDList &chunksIntersecting,
                                                              std::vector<bool> &needsUpdateFlag
                                                              )
            {
                ChunkManager &chunkManager = globalChunkManager;
                //chunkManager.lastChunkCount = chunkManager.GetChunkCount();
                
                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::1::prepareChunks");
                bufferIntegratorSIMDCentroids(integrator, depthExtrinsic);
                
                chunksIntersecting.clear();
                needsUpdateFlag.clear();

                chunkManager.GetBoundaryChunkID(depthImage,depthCamera,depthExtrinsic,globalMaxChunkID,globalMinChunkID);
                //std::cout<<globalMaxChunkID<<" \n\n"<<globalMinChunkID<<std::endl;
                Frustum frustum;
                depthCamera.SetupFrustum(depthExtrinsic, &frustum);
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::1::prepareChunks");

                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::2::collectChunks");
                //std::cout<<"start 2"<<std::endl;
                chunkManager.GetChunkIDsObservedByCamera(integrator,frustum,&chunksIntersecting, depthImage,depthCamera,depthExtrinsic);
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::2::collectChunks");
                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");
                //std::cout<<"start 3"<<std::endl;
                GetChunkCubes(globalCandidateCube,chunksIntersecting);

                for(const ChunkID& chunkID : chunksIntersecting)
                {
                    if (!chunkManager.HasChunk(chunkID) )
                    {
                       chunkManager.CreateChunk(chunkID);
                    }
                    needsUpdateFlag.push_back(false);
                }
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");
            }

            
            void PrepareIntersectChunks(ProjectionIntegrator& integrator,
                                                              float *depthImage,
                                                              const Transform& depthExtrinsic,
                                                              const PinholeCamera& depthCamera,
                                                              std::vector<void *> &candidateChunks,
                                                              std::vector<bool> &needsUpdateFlag,
                                                              std::vector<bool> &newChunkFlag,int submapID)
            {
                
                ChunkManager &chunkManager = chunkManagers[submapID];
                //chunkManager.lastChunkCount = chunkManager.GetChunkCount();
                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::1::prepareChunks");
                bufferIntegratorSIMDCentroids(integrator, depthExtrinsic,submapID);
                ChunkIDList chunksIntersecting;
                chunksIntersecting.clear();
                needsUpdateFlag.clear();
                newChunkFlag.clear();

                chunkManager.GetBoundaryChunkID(depthImage,depthCamera,depthExtrinsic,maxChunkIDs[submapID],minChunkIDs[submapID]);

                Frustum frustum;
                depthCamera.SetupFrustum(depthExtrinsic, &frustum);
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::1::prepareChunks");

                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::2::collectChunks");
                chunkManager.GetChunkIDsObservedByCamera(integrator,frustum,&chunksIntersecting, depthImage,depthCamera,depthExtrinsic);
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::2::collectChunks");
                //TICK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");
                GetChunkCubes(candidateCubes[submapID],chunksIntersecting,submapID);
                if(submapID == 0)
                {
                    for ( const ChunkID& chunkID : chunksIntersecting)
                    {
                        bool chunkNew = false;
                        if (!chunkManager.HasChunk(chunkID))
                        {
                        chunkNew = true;
                        chunkManager.CreateChunk(chunkID);
                        }
                        candidateChunks.push_back((void *)chunkManager.GetChunk(chunkID).get());
                        newChunkFlag.push_back(chunkNew);
                        needsUpdateFlag.push_back(false);
                    }
                }
                else
                {
                    ChunkManager &lastChunkManager = chunkManagers[submapID-1];
                    for ( const ChunkID& chunkID : chunksIntersecting)
                    {
                        bool chunkNew = false;
                        if (!chunkManager.HasChunk(chunkID) && !lastChunkManager.HasChunk(chunkID))
                        {
                        chunkNew = true;
                        chunkManager.CreateChunk(chunkID);
                        }
                        candidateChunks.push_back((void *)chunkManager.GetChunk(chunkID).get());
                        newChunkFlag.push_back(chunkNew);
                        needsUpdateFlag.push_back(false);
                    }

                }
                //TOCK("CHISEL::Reintegration::1::prepareIntersectChunks::3::createChunks");

            }

            void FinalizeIntegrateChunksGlobal(ProjectionIntegrator& integrator,
                                         const Transform& globalPose,
                                         ChunkIDList &chunksIntersecting,
                                         std::vector<bool> &needsUpdateFlag,                                    
                                         int submap_id)
            {
                ChunkManager &chunkManager = globalChunkManager;
                ChunkManager &reference_chunkManager = chunkManagers[submap_id];
                bufferIntegratorSIMDCentroids(integrator, globalPose);
                //auto pose_inv = globalPose.inverse();
                std::vector<int> threadIndex;
                for(int i = 0; i < chunksIntersecting.size();i++)
                {
                    bool needsUpdate = needsUpdateFlag[i];
                    if(!needsUpdate)
                    {
                        threadIndex.push_back(i);
                    }
                }
                parallel_for(threadIndex.begin(),threadIndex.end(),[&](const int& i)
                {
                    ChunkID chunkID = chunksIntersecting[i];
                    if(chunkManager.GetUseColor())
                    {
                        const ChunkPtr &chunk = chunkManager.GetChunk(chunkID);
                        integrator.voxelUpdate(reference_chunkManager.chunks, globalPose, chunk.get());
                    }
                });
            }
            void FinalizeIntegrateChunksGlobal(ChunkIDList &chunksIntersecting,
                                         std::vector<bool> &needsUpdateFlag,
                                         std::vector<bool> &newChunkFlag,
                                         ChunkIDList &validChunks)
            {


                validChunks.clear();

                ChunkIDList garbageChunks;

                TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks::1::findMeshesToUpdate");
                //std::cout<<"get meshestoupdate... needsUpdateFlag size: "<<needsUpdateFlag.size()<<std::endl;
                for(int i = 0; i < chunksIntersecting.size();i++)
                {
                    const ChunkID& chunkID = chunksIntersecting[i];
                    bool chunkNew = newChunkFlag[i];
                    bool needsUpdate = needsUpdateFlag[i];
                    if (needsUpdate)
                    {
                          //
                        /*this term, how could we assure that all this chunk is in the chunk manager's chunks.*/
                        meshesToUpdate[chunkID] = true;
                        meshesToUpdate[chunkID + ChunkID(-1, 0, 0)] = true;
                        meshesToUpdate[chunkID + ChunkID( 1, 0, 0)] = true;
                        meshesToUpdate[chunkID + ChunkID( 0, -1, 0)] = true;
                        meshesToUpdate[chunkID + ChunkID(0, 1, 0)] = true;
                        meshesToUpdate[chunkID + ChunkID(0, 0, -1)] = true;
                        meshesToUpdate[chunkID + ChunkID(0, 0, 1)] = true;
                        
//                        for (int dx = -1; dx <= 1; dx++)
//                        {
//                            for (int dy = -1; dy <= 1; dy++)
//                            {
//                                for (int dz = -1; dz <= 1; dz++)
//                                {
//                                    meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
//                                }
//                            }
//                        }
                        validChunks.push_back(chunkID);
                    }
                    else if(chunkNew)
                    {
                        //std::cout<<"garbage"<<std::endl;
                        garbageChunks.push_back(chunkID);
                    }
                }
            
                TOCK("CHISEL::Reintegration::4::FinalizeIntegrateChunks::1::findMeshesToUpdate");


//              printf("chunk list size: %d, garbage chunks: %d\r\n",chunkNewList.size(),garbageChunks.size());
                TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks::2::GarbageCollect");
                GarbageCollect(garbageChunks);
                TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks::2::GarbageCollect");
                //meshesToUpdate[submapID].clear();
                //std::cout<<"update done！Need to update: "<<meshesToUpdate.size()<<std::endl;
                //std::cout<<"Meshes To Update: "<<meshesToUpdate.size()<<std::endl;
            }
            void FinalizeIntegrateChunks(ChunkIDList &chunksIntersecting,
                                         std::vector<bool> &needsUpdateFlag,
                                         std::vector<bool> &newChunkFlag,
                                         ChunkIDList &validChunks,int submapID)
            {


                validChunks.clear();

                ChunkIDList garbageChunks;

                TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks::1::findMeshesToUpdate");
                //std::cout<<"get meshestoupdate... needsUpdateFlag size: "<<needsUpdateFlag.size()<<std::endl;
                for(int i = 0; i < chunksIntersecting.size();i++)
                {
                    const ChunkID& chunkID = chunksIntersecting[i];
                    bool chunkNew = newChunkFlag[i];
                    bool needsUpdate = needsUpdateFlag[i];
                    if (needsUpdate)
                    {
                        
                        meshesToUpdate[chunkID] = true;
                        meshesToUpdate[chunkID + ChunkID(-1, 0, 0)] = true;
                        meshesToUpdate[chunkID + ChunkID( 1, 0, 0)] = true;
                        meshesToUpdate[chunkID + ChunkID( 0, -1, 0)] = true;
                        meshesToUpdate[chunkID + ChunkID(0, 1, 0)] = true;
                        meshesToUpdate[chunkID + ChunkID(0, 0, -1)] = true;
                        meshesToUpdate[chunkID + ChunkID(0, 0, 1)] = true;
                        
//                        for (int dx = -1; dx <= 1; dx++)
//                        {
//                            for (int dy = -1; dy <= 1; dy++)
//                            {
//                                for (int dz = -1; dz <= 1; dz++)
//                                {
//                                    meshesToUpdate[chunkID + ChunkID(dx, dy, dz)] = true;
//                                }
//                            }
//                        }
                        validChunks.push_back(chunkID);
                    }
                    else if(chunkNew)
                    {
                        //std::cout<<"garbage"<<std::endl;
                        garbageChunks.push_back(chunkID);
                    }
                }
            
                TOCK("CHISEL::Reintegration::4::FinalizeIntegrateChunks::1::findMeshesToUpdate");


//              printf("chunk list size: %d, garbage chunks: %d\r\n",chunkNewList.size(),garbageChunks.size());
                TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks::2::GarbageCollect");
                GarbageCollect(garbageChunks,submapID);
                if(submapID >0)
                GarbageCollect(garbageChunks,submapID-1);
                TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks::2::GarbageCollect");
                //meshesToUpdate[submapID].clear();
                //std::cout<<"update done！Need to update: "<<meshesToUpdate.size()<<std::endl;
                //std::cout<<"Meshes To Update: "<<meshesToUpdate.size()<<std::endl;
            }
            void IntegrateDepthScanColorGetSharedChunks(ProjectionIntegrator& integrator,
                                         const Transform& depthExtrinsic,
                                         ChunkIDList &chunksIntersecting,                                   
                                         int mainSubmapID,ChunkMap &shared_chunks )
            {
                ChunkManager &chunkManager = chunkManagers[mainSubmapID];
                bufferIntegratorSIMDCentroids(integrator, depthExtrinsic,mainSubmapID);

                std::cout<<"begin to get shared_chunks!"<<std::endl;
                if(chunksIntersecting.size() < 1)
                {
                    return;
                }

                for(int i = 0;i!=chunksIntersecting.size();++i)
                {
                    ChunkID chunkID = chunksIntersecting[i];
                    if(chunkManager.HasChunk(chunkID))
                    {
                    shared_chunks[chunkID] = chunkManager.chunks[chunkID]; 
                    chunkManager.chunks.erase(chunkID);
                    chunkManager.allMeshes.erase(chunkID);
                    }
                    if(chunkManager.HasChunk(chunkID + ChunkID(0,0,1)))
                    {
                    shared_chunks[chunkID+ ChunkID(0,0,1)] = chunkManager.chunks[chunkID+ChunkID(0,0,1)];                         
                    }    
                    if(chunkManager.HasChunk(chunkID + ChunkID(0,0,-1)))
                    {
                    shared_chunks[chunkID+ ChunkID(0,0,-1)] = chunkManager.chunks[chunkID+ChunkID(0,0,-1)];                         
                    }    
                    if(chunkManager.HasChunk(chunkID + ChunkID(0,1,0)))
                    {
                    shared_chunks[chunkID+ ChunkID(0,1,0)] = chunkManager.chunks[chunkID+ChunkID(0,1,0)];                         
                    }    
                    if(chunkManager.HasChunk(chunkID + ChunkID(0,-1,0)))
                    {
                    shared_chunks[chunkID+ ChunkID(0,-1,0)] = chunkManager.chunks[chunkID+ChunkID(0,-1,0)];                         
                    }    
                    if(chunkManager.HasChunk(chunkID + ChunkID(1,0,0)))
                    {
                    shared_chunks[chunkID+ ChunkID(1,0,0)] = chunkManager.chunks[chunkID+ChunkID(1,0,0)];                         
                    }    
                    if(chunkManager.HasChunk(chunkID + ChunkID(-1,0,0)))
                    {
                    shared_chunks[chunkID+ ChunkID(-1,0,0)] = chunkManager.chunks[chunkID+ChunkID(-1,0,0)];                         
                    }    
                }
                //std::cout<<"Integrate depth and color done!"<<std::endl;
            }
            void IntegrateDepthScanColorGlobal(ProjectionIntegrator& integrator,
                                         const Transform& globalPose,
                                         ChunkIDList &chunksIntersecting,
                                         std::vector<bool> &needsUpdateFlag,                                    
                                         int submap_id)
            {

                ChunkManager &chunkManager = globalChunkManager;
                ChunkManager &reference_chunkManager = chunkManagers[submap_id];

                //std::cout<<"begin to integrate depth and color!"<<std::endl;
                //std::cout<<"chunksIntersectingList: "<<chunksIntersecting.size()<<std::endl;
/*
            void IntegrateDepthScanColorGlobal(projectionIntegrator, depthImageData, nullptr, globalPose,
                cameraModel, localChunksIntersecting, localNeedsUpdateFlag, gcSLAM.submapPosesRelativeChanges[submap_id], submap_id)
            {
                auto pose_inv = globalPose.inverse();
                
            }
*/
                //auto pose_inv = globalPose.inverse();
                bufferIntegratorSIMDCentroids(integrator, globalPose);
                if(chunksIntersecting.size() < 1)
                {
                    std::cout<<"number of chunks is less than 1."<<std::endl;
                    return;
                }
#if 0
                for(int i = 0; i != chunksIntersecting.size(); ++i)
                {
                    ChunkID chunkID  = chunksIntersecting[i];
                    //std::cout<<i<<" "<<chunkID<<std::endl;
                    if(chunkManager.GetUseColor())
                    {
                        const ChunkPtr &chunk = chunkManager.GetChunk(chunkID);
                        bool needsUpdate = integrator.IntegrateColor(reference_chunkManager.chunks, globalPose, chunk.get());
                        needsUpdateFlag[i] = (needsUpdateFlag[i] || needsUpdate);
                    }
                }
#else
                std::vector<int> threadIndex;
                for(int i = 0; i < chunksIntersecting.size();i++)
                {
                    threadIndex.push_back(i);
                }

                parallel_for(threadIndex.begin(),threadIndex.end(),[&](const int& i)
                {
                    ChunkID chunkID = chunksIntersecting[i];
                    if(chunkManager.GetUseColor())
                    {
                        const ChunkPtr &chunk = chunkManager.GetChunk(chunkID);
                        bool needsUpdate = integrator.IntegrateColor(reference_chunkManager.chunks, globalPose, chunk.get());
                        needsUpdateFlag[i] = (needsUpdateFlag[i] || needsUpdate);
                    }
                });
#endif
                std::cout<<"Finish Integrating global depth and color!"<<std::endl;
            }

            void IntegrateDepthScanColorGlobal(ProjectionIntegrator& integrator,
                                         float * depthImage,
                                         unsigned char* colorImage,
                                         const Transform& depthExtrinsic,
                                         const PinholeCamera& depthCamera,
                                         ChunkIDList &chunksIntersecting,
                                         std::vector<bool> &needsUpdateFlag,
                                         int integrate_flag,                                         
                                         float *weight = NULL)
            {

                ChunkManager &chunkManager = globalChunkManager;
                bufferIntegratorSIMDCentroids(integrator, depthExtrinsic);

                //std::cout<<"begin to integrate depth and color!"<<std::endl;
                //std::cout<<"chunksIntersectingList: "<<chunksIntersecting.size()<<std::endl;

                if(chunksIntersecting.size() < 1)
                {
                    return;
                }
                std::vector<int> threadIndex;
                for(int i = 0; i < chunksIntersecting.size();i++)
                {
                    threadIndex.push_back(i);
                }

                parallel_for(threadIndex.begin(),threadIndex.end(),[&](const int& i)
                {
                    ChunkID chunkID = chunksIntersecting[i];
                    if(chunkManager.GetUseColor())
                    {
                        const ChunkPtr &chunk = chunkManager.GetChunk(chunkID);
                        bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, depthExtrinsic, colorImage, chunk.get(),integrate_flag,weight);
                        needsUpdateFlag[i] = (needsUpdateFlag[i] || needsUpdate);
                    }
                });

                //std::cout<<"Integrate depth and color done!"<<std::endl;
            }
            void IntegrateDepthScanColorGlobal(ProjectionIntegrator& integrator,
                                         float * depthImage,
                                         unsigned char* colorImage,
                                         const Transform& depthExtrinsic,
                                         const PinholeCamera& depthCamera)
            {

                ChunkIDList chunksIntersecting;
                std::vector<bool> needsUpdateFlag;
                std::vector<bool> newChunkFlag;
                ChunkIDList validChunks;

                PrepareIntersectChunksGlobal(integrator,depthImage,depthExtrinsic,depthCamera,chunksIntersecting,needsUpdateFlag,newChunkFlag);
                IntegrateDepthScanColorGlobal(integrator,depthImage,colorImage,depthExtrinsic,depthCamera,chunksIntersecting,needsUpdateFlag,1,nullptr);
                FinalizeIntegrateChunksGlobal(chunksIntersecting,needsUpdateFlag,newChunkFlag,validChunks);
            }
            void IntegrateDepthScanColor(ProjectionIntegrator& integrator,
                                         float * depthImage,
                                         unsigned char* colorImage,
                                         const Transform& depthExtrinsic,
                                         const PinholeCamera& depthCamera,
                                         ChunkIDList &chunksIntersecting,
                                         std::vector<bool> &needsUpdateFlag,
                                         int integrate_flag,                                         
                                         float *weight ,int mainSubmapID,int referenceSubmapID )
            {

                ChunkManager &chunkManager = chunkManagers[mainSubmapID];
                bufferIntegratorSIMDCentroids(integrator, depthExtrinsic,mainSubmapID);

                //std::cout<<"begin to integrate depth and color!"<<std::endl;
                //std::cout<<"chunksIntersectingList: "<<chunksIntersecting.size()<<std::endl;
                if(chunksIntersecting.size() < 1)
                {
                    return;
                }
                std::vector<int> threadIndex;
                for(int i = 0; i < chunksIntersecting.size();i++)
                {
                    threadIndex.push_back(i);
                }
#if REFERENCE_SUBMAP_RECOMPUTING
                if(mainSubmapID>0 && mainSubmapID - 1  == referenceSubmapID && chunkManagers[referenceSubmapID].isValid)
                {
                //std::cout<<"mainSubmapID: "<<mainSubmapID<<" referenceSubmapID: "<<referenceSubmapID<<std::endl;                    
                ChunkManager &lastChunkManager = chunkManagers[referenceSubmapID];
                
                parallel_for(threadIndex.begin(),threadIndex.end(),[&]( int& i)
                {
                    ChunkID chunkID = chunksIntersecting[i];
                    
                    if(lastChunkManager.HasChunk(chunkID)&&lastChunkManager.GetUseColor())
                    {
                        const ChunkPtr &chunk = lastChunkManager.GetChunk(chunkID);
                        bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, depthExtrinsic, colorImage, chunk.get(),integrate_flag,weight);
                        needsUpdateFlag[i] = (needsUpdateFlag[i] || needsUpdate);
                    }
                    
                });
                int common_chunks = 0;
                for(int i = 0;i!=chunksIntersecting.size();++i)
                {
                    ChunkID chunkID = chunksIntersecting[i];
                    if(lastChunkManager.HasChunk(chunkID))
                    {
                        common_chunks += 1;
                        chunkManager.AddChunk(lastChunkManager.chunks[chunkID]); 
                    }
                    //lastChunkManager.RemoveChunk(chunkID);
                }
                //std::cout<<"common_chunks: "<<common_chunks<<std::endl;
                need_to_recompute_for_reference = (common_chunks > 100);

                }
#else 
                //std::cout<<"mainSubmapID: "<<mainSubmapID<<" referenceSubmapID: "<<referenceSubmapID<<std::endl;                    
                int common_chunks = 0;
                for(int i = 0;i!=chunksIntersecting.size();++i)
                {
                    ChunkID chunkID = chunksIntersecting[i];
                    if(!chunkManager.HasChunk(chunkID))
                    chunkManager.CreateChunk(chunkID); 
                    //lastChunkManager.RemoveChunk(chunkID);
                }
                //std::cout<<"common_chunks: "<<common_chunks<<std::endl;
                need_to_recompute_for_reference = false;
#endif
                parallel_for(threadIndex.begin(),threadIndex.end(),[&]( int& i)
                {
                    ChunkID chunkID = chunksIntersecting[i];
                    if(chunkManager.HasChunk(chunkID) && chunkManager.GetUseColor())
                    {
                        const ChunkPtr &chunk = chunkManager.GetChunk(chunkID);
                        bool needsUpdate = integrator.IntegrateColor(depthImage, depthCamera, depthExtrinsic, colorImage, chunk.get(),integrate_flag,weight);
                        needsUpdateFlag[i] = (needsUpdateFlag[i] || needsUpdate);
                    }
                });  

                //std::cout<<"Integrate depth and color done!"<<std::endl;
            }

            float GetDistanceFromSurface(Eigen::Vector3f global_vertex, float &tsdfWeight,int submapID)
            {
                ChunkManager &chunkManager = chunkManagers[submapID];
                tsdfWeight = 0;

                global_vertex -= Eigen::Vector3f(1,1,1) * (chunkManager.GetResolution() / 2);
                const float roundingVoxelStep = 1.0f / chunkManager.GetResolution();
                Eigen::Vector3i chunkSize = chunkManager.GetChunkSize();
                Vec3 rasterizedPose = global_vertex*roundingVoxelStep;


                Eigen::Vector3i V[8];
                float spatialWeight[8];
                float distX = rasterizedPose(0) - floor(rasterizedPose(0));
                float distY = rasterizedPose(1) - floor(rasterizedPose(1));
                float distZ = rasterizedPose(2) - floor(rasterizedPose(2));
                spatialWeight[0] = (1 - distX) * (1 - distY) * (1 - distZ);
                spatialWeight[1] = (1 - distX) * (1 - distY) * (distZ);
                spatialWeight[2] = (1 - distX) * (distY) * (1 - distZ);
                spatialWeight[3] = (1 - distX) * (distY) * (distZ);
                spatialWeight[4] = (distX) * (1 - distY) * (1 - distZ);
                spatialWeight[5] = (distX) * (1 - distY) * (distZ);
                spatialWeight[6] = (distX) * (distY) * (1 - distZ);
                spatialWeight[7] = (distX) * (distY) * (distZ);
                V[0] = Eigen::Vector3i(floor(rasterizedPose(0)),floor(rasterizedPose(1)),floor(rasterizedPose(2)));
                V[1] = Eigen::Vector3i(floor(rasterizedPose(0)),floor(rasterizedPose(1)),ceil(rasterizedPose(2)));
                V[2] = Eigen::Vector3i(floor(rasterizedPose(0)),ceil(rasterizedPose(1)),floor(rasterizedPose(2)));
                V[3] = Eigen::Vector3i(floor(rasterizedPose(0)),ceil(rasterizedPose(1)),ceil(rasterizedPose(2)));
                V[4] = Eigen::Vector3i(ceil(rasterizedPose(0)),floor(rasterizedPose(1)),floor(rasterizedPose(2)));
                V[5] = Eigen::Vector3i(ceil(rasterizedPose(0)),floor(rasterizedPose(1)),ceil(rasterizedPose(2)));
                V[6] = Eigen::Vector3i(ceil(rasterizedPose(0)),ceil(rasterizedPose(1)),floor(rasterizedPose(2)));
                V[7] = Eigen::Vector3i(ceil(rasterizedPose(0)),ceil(rasterizedPose(1)),ceil(rasterizedPose(2)));

                float weight = 0;
                float distance = 0;
                for(int k = 0; k < 8; k++)
                {
                    ChunkID cid = Eigen::Vector3i(floor((float)V[k](0) / (float)chunkSize(0)),
                                                  floor((float)V[k](1) / (float)chunkSize(1)),
                                                  floor((float)V[k](2) / (float)chunkSize(2)));

                    if(chunkManager.HasChunk(cid))
                    {

                        ChunkPtr chunk = chunkManager.GetChunk(cid);
                        unsigned int vid = V[k](0) - cid(0) * chunkSize(0) +
                                (V[k](1) - cid(1) * chunkSize(1)) * chunkSize(0) +
                                (V[k](2) - cid(2) * chunkSize(2)) * chunkSize(0) * chunkSize(1);

                        const DistVoxel &voxels = chunk->GetVoxels();
                        weight += spatialWeight[k] * voxels.weight[vid];
                        distance += voxels.sdf[vid] * spatialWeight[k] * voxels.weight[vid];
                        tsdfWeight += voxels.weight[vid] * spatialWeight[k];
                    }
                }
                if(weight > 0)
                {
                    distance = distance / weight;
                    tsdfWeight = tsdfWeight / weight;
                }
#if 0
                if(distance > 0.1)
                {
                    for(int k = 0; k < 8; k++)
                    {
                        ChunkID cid = Eigen::Vector3i(floor((float)V[k](0) / (float)chunkSize(0)),
                                                      floor((float)V[k](1) / (float)chunkSize(1)),
                                                      floor((float)V[k](2) / (float)chunkSize(2)));

                        if(chunkManager.HasChunk(cid))
                        {

                            ChunkPtr chunk = chunkManager.GetChunk(cid);
                            unsigned int vid = V[k](0) - cid(0) * chunkSize(0) +
                                    (V[k](1) - cid(1) * chunkSize(1)) * chunkSize(0) +
                                    (V[k](2) - cid(2) * chunkSize(2)) * chunkSize(0) * chunkSize(1);

                            const DistVoxel &voxels = chunk->GetVoxels();

                            std::cout << "weight/distance/k: " << spatialWeight[k] << "/"
                                      << voxels.sdf[vid] << "/" << k << std::endl;
                        }
                    }
                }
#endif
                return distance;

            }


            void GetChunkCubes(std::vector<float> &cubes,
                               ChunkIDList &chunksIntersecting,int submapID)
            {
                cubes.clear();
                ChunkManager &chunkManager = chunkManagers[submapID];
                Vec3 offset[8];
                Vec3 resolution = chunkManager.GetResolution() * chunkManager.GetChunkSize().cast<float>();
                offset[0] = Vec3(0,0,0);
                offset[1] = Vec3(1,0,0);
                offset[2] = Vec3(0,1,0);
                offset[3] = Vec3(1,1,0);
                offset[4] = Vec3(0,0,1);
                offset[5] = Vec3(1,0,1);
                offset[6] = Vec3(0,1,1);
                offset[7] = Vec3(1,1,1);

                Eigen::Vector3i numVoxels = chunkManager.GetChunkSize();
                float voxelResolutionMeters = chunkManager.GetResolution();
                for(int i = 0; i < chunksIntersecting.size(); i++)
                {
                    ChunkID & chunkID = chunksIntersecting[i];
                    Vec3 origin = Vec3(numVoxels(0) * chunkID(0) * voxelResolutionMeters,
                                       numVoxels(1) * chunkID(1) * voxelResolutionMeters,
                                       numVoxels(2) * chunkID(2) * voxelResolutionMeters);
                    for(int j = 0; j < 8;j++)
                    {
                        Vec3 corner = origin + Vec3(offset[j](0)*resolution(0),offset[j](1)*resolution(1),offset[j](2)*resolution(2));
                        cubes.push_back(corner(0));
                        cubes.push_back(corner(1));
                        cubes.push_back(corner(2));
                    }
                }

            }
            void GetChunkCubes(std::vector<float> &cubes,
                               ChunkIDList &chunksIntersecting)
            {
                cubes.clear();
                ChunkManager &chunkManager = globalChunkManager;
                Vec3 offset[8];
                Vec3 resolution = chunkManager.GetResolution() * chunkManager.GetChunkSize().cast<float>();
                offset[0] = Vec3(0,0,0);
                offset[1] = Vec3(1,0,0);
                offset[2] = Vec3(0,1,0);
                offset[3] = Vec3(1,1,0);
                offset[4] = Vec3(0,0,1);
                offset[5] = Vec3(1,0,1);
                offset[6] = Vec3(0,1,1);
                offset[7] = Vec3(1,1,1);

                Eigen::Vector3i numVoxels = chunkManager.GetChunkSize();
                float voxelResolutionMeters = chunkManager.GetResolution();
                for(int i = 0; i < chunksIntersecting.size(); i++)
                {
                    ChunkID & chunkID = chunksIntersecting[i];
                    Vec3 origin = Vec3(numVoxels(0) * chunkID(0) * voxelResolutionMeters,
                                       numVoxels(1) * chunkID(1) * voxelResolutionMeters,
                                       numVoxels(2) * chunkID(2) * voxelResolutionMeters);
                    for(int j = 0; j < 8;j++)
                    {
                        Vec3 corner = origin + Vec3(offset[j](0)*resolution(0),offset[j](1)*resolution(1),offset[j](2)*resolution(2));
                        cubes.push_back(corner(0));
                        cubes.push_back(corner(1));
                        cubes.push_back(corner(2));
                    }
                }

            }
            void RefineFrameInVoxel(ProjectionIntegrator& integrator,
                                    float * depthImage,
                                    float * weight,
                                    const Transform& depthExtrinsic,
                                    const PinholeCamera& depthCamera,int submapID)
            {

                int height = depthCamera.GetHeight();
                int width = depthCamera.GetWidth();
                float cx = depthCamera.GetCx();
                float cy = depthCamera.GetCy();
                float fx = depthCamera.GetFx();
                float fy = depthCamera.GetFy();

                Eigen::Matrix3f rotationRef = depthExtrinsic.linear();
                Eigen::Vector3f translationRef = depthExtrinsic.translation();

                float tsdfWeight;
                for(int i = 0; i < height; i ++)
                {
                    for(int j = 0; j < width; j++)
                    {
                        float depth = depthImage[i*width+j];
                        if(depth < 0.05 || depth > 3)
                        {
                            continue;
                        }

                        Eigen::Vector3f dir = Eigen::Vector3f((j-cx)/fx,(i-cy)/fy,1);
                        Eigen::Vector3f global_vertex = rotationRef * dir * depth + translationRef;
                        float updated_distance = GetDistanceFromSurface(global_vertex,tsdfWeight,submapID);
                        float first_update = updated_distance;
                        float depth_init = depth;
                        depth += updated_distance;
                        global_vertex = rotationRef * dir * depth + translationRef;
                        updated_distance = GetDistanceFromSurface(global_vertex,tsdfWeight,submapID);

                        depth += updated_distance;
                        global_vertex = rotationRef * dir * depth + translationRef;
                        updated_distance = GetDistanceFromSurface(global_vertex,tsdfWeight,submapID);

                        depth += updated_distance;
                        global_vertex = rotationRef * dir * depth + translationRef;
                        updated_distance = GetDistanceFromSurface(global_vertex,tsdfWeight,submapID);

                        depth += updated_distance;
                        global_vertex = rotationRef * dir * depth + translationRef;
                        updated_distance = GetDistanceFromSurface(global_vertex,tsdfWeight,submapID);

                        depth += updated_distance;
                        global_vertex = rotationRef * dir * depth + translationRef;
                        updated_distance = GetDistanceFromSurface(global_vertex,tsdfWeight,submapID);

                        depth += updated_distance;
                        depthImage[i*width + j] = depth;
                        weight[i*width +j] = tsdfWeight;
                        if(fabs(updated_distance) > 5e-3 )
                        {
                            depthImage[i*width + j] = 0;
                            weight[i*width +j] = 0;
                        }

                        if(depthImage[i*width + j] > depthCamera.GetFarPlane() || depthImage[i*width+j] < depthCamera.GetNearPlane() )
                        {
                            depthImage[i*width + j] = 0;
                            weight[i*width +j] = 0;
                        }
                        if(fabs(depthImage[i * width + j] - depth_init) > 0.1 )
                        {
                            depthImage[i*width + j] = 0;
                            weight[i*width +j] = 0;
                        }
#if 0
                        if(fabs(updated_distance) > 1e-3)
                        {
                            std::cout << "warning! " << depth << " " << depth_init << " " << updated_distance << " " << first_update << std::endl;
                        }
#endif
                        // search along dir until iso surface is found

                    }
                }

            }

            bool SaveTSDFFiles(const std::string& fileName);


            void GarbageCollect(const ChunkIDList& chunks,int submapID = -1)
            {
                if(submapID < 0)
                {
                ChunkManager &chunkManager = globalChunkManager;
                       for (const ChunkID& chunkID : chunks)
                       {
                           chunkManager.RemoveChunk(chunkID);
                           meshesToUpdate.erase(chunkID);
                       }
                }
                else
                {
                ChunkManager &chunkManager = chunkManagers[submapID];
                       for (const ChunkID& chunkID : chunks)
                       {
                           chunkManager.RemoveChunk(chunkID);
                           meshesToUpdate.erase(chunkID);
                       }
                }
            }
            bool SaveMeshesToPLYBySubmapDense(int submapID, const PoseSE3d &submap_pose, const std::string &filename,  const std::string &map_filename = "./");
            //bool SaveAllMeshesToPLYBySubmapDense(const std::string &filename, const std::string &map_filename = "./");
            void clearMeshesToUpdate()
            {
                meshesToUpdate.clear();
                ChunkSet _meshesToUpdate = ChunkSet();
                meshesToUpdate.swap(_meshesToUpdate);
            }
            void UpdateMeshes(const PinholeCamera& camera,int submapID)
            {
                ChunkManager &chunkManager = chunkManagers[submapID];
                chunkManager.RecomputeMeshes(meshesToUpdate,camera);
                //printf("begin to clear update_meshes_flag\r\n");
                clearMeshesToUpdate();
 
                //printf("finish to clear update_meshes_flag\r\n");
            }
            void UpdateMeshes(const PinholeCamera& camera)
            {
                ChunkManager &chunkManager = globalChunkManager;
                chunkManager.RecomputeMeshesWithoutCheck(camera);
                printf("begin to clear update_meshes_flag\r\n");
                clearMeshesToUpdate();

                //printf("finish to clear update_meshes_flag\r\n");
            }


            /*
            Need to fix. Main submapID and Reference submapID may need to recompute mesh. 
            
            */
            void UpdateMeshes(const PinholeCamera& camera,int mainSubmapID, int referenceSubmapID)
            {

                std::cout<<"choose one scheme to recompute the mesh!"<<std::endl;
                 //chunkManagers[*submapIDs.begin()].RecomputeMeshesWithoutCheck(camera,globalChunkManager.allMeshes);
                
                 if(/*globalChunkManager*/chunkManagers[mainSubmapID].allMeshes.empty()|| meshesToUpdate.size() == 0)
                 {
                     std::cout<<"all meshes clear! Without checking to recompute Meshes!"<<std::endl;
                    chunkManagers[mainSubmapID].RecomputeMeshesWithoutCheck(camera/*,globalChunkManager.allMeshes*/);
                    //chunkManagers[*submapIDs.begin()].RecomputeMeshesWithoutCheck(camera);
                 }
                else 
                {
                     std::cout<<"recompute Meshes using meshesToUpdate!"<<std::endl;
                    chunkManagers[mainSubmapID].RecomputeMeshes(meshesToUpdate,camera/*,globalChunkManager.allMeshes*/);
                    //chunkManagers[*submapIDs.begin()].RecomputeMeshes(meshesToUpdate,camera);
                }
#if REFERENCE_SUBMAP_RECOMPUTING
                if(referenceSubmapID >= 0 && chunkManagers[referenceSubmapID].isValid  && need_to_recompute_for_reference)
                if( std::abs(referenceSubmapID-mainSubmapID) == 1 || chunkManagers[referenceSubmapID].allMeshes.empty())
                chunkManagers[referenceSubmapID].RecomputeMeshesWithoutCheck(camera);
#endif
                clearMeshesToUpdate();
            }
            void UpdateMeshes(const PinholeCamera& camera, const std::vector<int> &correspondence_submaps )
            {
                for(int i = 0; i != correspondence_submaps.size(); ++i)
                {
                    int submap_id = correspondence_submaps[i];

                    chunkManagers[submap_id].RecomputeMeshesWithoutCheck(camera);
                }
                clearMeshesToUpdate();
            }
            void UpdateMeshesDebug(const PinholeCamera& camera,int submapID/*,const Transform &T*/)
            {
                std::cout<<"update meshes submap "+submapID<<std::endl;
                ChunkManager &chunkManager = chunkManagers[submapID];
                //this->combineChunkMap(chunkManagers[submapID-1].chunks,chunkManager.chunks);
                //ChunkMap chunks = chunkManager.ChunkTransform(T);
                std::cout<<"recomputing meshes... "+submapID<<std::endl;
                chunkManager.RecomputeMeshesWithoutCheck(camera);
                //chunkManager.RecomputeMeshesWithoutCheck(camera,);
                //chunkManager.RecomputeMeshesWithoutCheck(camera,chunkManager.chunks);
            }

            bool SaveAllMeshesToPLY(const std::string& filename);
            bool SaveAllMeshesToPLY(const std::string &filename,std::set<int> activeSubmapIDs,PoseSE3dList &submapPoses, PoseSE3d &cameraPose);
            int SaveAllMeshesToPLY(MeshPtr mesh,size_t v,MeshPtr fullMesh,PoseSE3d &pose, PoseSE3d &cameraPose);
            int SaveAllMeshesToPLY(MeshMap &allMeshes,size_t v,MeshPtr fullMesh,PoseSE3d &pose, PoseSE3d &cameraPose);
            
            bool SaveAllMeshesToPLYSemantic(const std::string &filename,std::set<int> activeSubmapIDs,PoseSE3dList &submapPoses, PoseSE3d &cameraPose);
            bool SaveMeshesToPLYBySubmap(int submapID,const std::string &filename);
            bool SaveRoomToPLY(const std::vector<int> &room,const PoseSE3dList &submapPoses, const std::string &filename);
            PcdPtr ExtractRoomModel(const std::vector<int> &room, const PoseSE3dList &submapPoses);
            PcdPtr ExtractRoomModelSparse(const std::vector<int> &room);            
            bool SaveMeshesToPLYGlobal(const std::string &filename, const PoseSE3d &c_pose = PoseSE3d());
            MeshPtr GetMeshesGlobal();
            MeshPtr GetMeshesGlobalSemantic();
            bool SaveAllMeshesToPLYBySubmap(const std::string &filename);
            bool SavePointCloudToPLY(const std::string &filename, const Vec3List &points, 
                const Vec3List &normals =  Vec3List (), const Vec3List &colors = Vec3List ());
            bool SaveRoomMeshesToPLY(const std::string &filename,std::vector<int> room_submaps,PoseSE3dList &submapPoses, PoseSE3d &cameraPose);
            size_t GetFullPCD(std::vector<unsigned char> &buffer, const PoseSE3dList &submapPoses, const PoseSE3d & cameraPose);
            size_t GetFullPCD(PointCloud &pcd, std::vector<int> &rgb_colors, 
                std::vector<int> &semantic_colors, const PoseSE3dList &submapPoses);
            size_t GetFullPCD(std::vector<PointCloud> &pcds, std::vector<std::vector<int>> &rgb_colors, 
                std::vector<std::vector<int>> &semantic_colors);
            bool SaveLabels(const std::string &filename, std::vector<long> &labels);
            void Reset();
            int addNewSubmap()
            {

                //ChunkManager n_chunkManager = ChunkManager();
                //n_chunkManager.setChunks(chunkManagers.back().chunks);
                std::cout<<"chisel: add chunk manager!"<<std::endl;
                chunkManagers.emplace_back(chunkSize,voxelResolution,useColor);
                chunkManagers.back().submap_id = chunkManagers.size()-1;
                std::cout<<"chisel: add max and min ID!"<<std::endl;
                maxChunkIDs.push_back(ChunkID());
                minChunkIDs.push_back(ChunkID());
                std::cout<<"add candidate cubes!"<<std::endl;
                candidateCubes.push_back(std::vector<float>());
                std::cout<<"clear global allMeshes!"<<std::endl;
                //globalChunkManager.clearMeshes();//clearMeshes();
                //meshesToUpdate.push_back(ChunkSet());
                submapColor+=1;
                std::cout<<"chunkManagers' size: "<<chunkManagers.size()<<std::endl;
                return chunkManagers.size()-1;
            }
            void clearGlobalMeshes()
            {
                globalChunkManager.clearMeshes();
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

            void FinalIntegrationToGlobalChunkManager(ProjectionIntegrator& integrator, 
                PoseSE3dList &relativeChanges, const std::string &filename = "./");
            ChunkMap ChunkTransform(const Transform &T,ChunkMap &_chunks)
            {
                //return chunks;
                Eigen::Matrix3f rotation =T.rotation();                
                Eigen::VectorXf translation = T.translation();
                std::vector<VoxelMap > chunksCoords;
                    std::cout<<"GetVoxel world coord!"<<std::endl;
                for(auto i = _chunks.begin();i!=_chunks.end();++i)
                {
                      chunksCoords.push_back(i->second->GetworldCoords());
                }
                VoxelMap afterTransform;
                std::cout<<"transform voxels: R: "<<rotation<<"\nt: "<<translation<<std::endl;
                for(int i = 0;i!= chunksCoords.size(); ++i)
                {
                    for(auto j = chunksCoords[i].begin();j!=chunksCoords[i].end();++j)
                    {
                    Vec3 newCoord =  rotation * j->first + translation/ voxelResolution;
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
                Eigen::Vector3i coord = i->first.cast<int>();
                //Eigen::Vector3i coord =Eigen::Vector3i(std::floor(pos.x()) ,std::floor(pos.y()) ,std::floor(pos.z()) );
                ChunkID chunkID = GetPointIDAt(coord);
                //std::cout <<"coord: "<<i->first<< " chunkID: "<<chunkID<<std::endl;
            if(result.find(chunkID) == result.end())
            {

                result.insert(std::make_pair(chunkID,std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), chunkID, chunkSize, voxelResolution, useColor)));
                result[chunkID]->voxels.Reset();
            }
                int voxelID = result[chunkID]->GetLocalVoxelIDFromGlobal(coord);
                result[chunkID]->voxels.SetSDF(i->second[0],voxelID);
                result[chunkID]->voxels.SetWeight(i->second[1],voxelID);
                result[chunkID]->colors.setRed(i->second[2],voxelID);
                result[chunkID]->colors.setGreen(i->second[3],voxelID);
                result[chunkID]->colors.setBlue(i->second[4],voxelID);   
                result[chunkID]->colors.setDivision(i->second[5],voxelID);                    
            }
            std::cout<<"transform down!"<<std::endl;
            return result;

            }
            //const ChunkSet& GetMeshesToUpdate() const { return meshesToUpdate; }

            ChunkMap IntegrateSubmaps(ChunkMap &first,const Transform &T1,ChunkMap &second)
            {
                ChunkMap _first = ChunkTransform(T1,first);
                //_second.clear();
                std::cout<<"chunk size: "<<_first.size() << " "<<second.size()<<std::endl;
                return combineChunkMap(_first,second);
            }
            
            /*this step has a lot that could be improved. 2019.3.4*/
            ChunkMap &combineChunkMap( ChunkMap &first, ChunkMap &second)
            {
                //second.insert(first.begin(),first.end());
                for(auto i = first.begin();i!=first.end();++i)
                {
                    if(second.find(i->first) == second.end())
                    {
                    second.insert(std::make_pair(i->first,std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), i->first, 
                        chunkSize, voxelResolution, useColor)));
                    *(second.at(i->first)) = *(i->second);
                    //second[i->first] = i->second;
                    }
    
                    else
                    {
                    //std::cout<<"integrate two chunks"<<std::endl;
                    second[i->first]->integrateTwoChunksSIMD(i->second);
                    }
                }
                return second;
            }

            void saveSubmap(int submapID,const std::string &filepath="./", bool save_chunks = false)
            {

                //global_changed = true;
                //chunkManagers[submapID].isValid = false;
                std::string filename = filepath+"m"+std::to_string(submapID)+".map";
                std::cout<<filename<<std::endl;
                //maybe would be used later
                if(save_chunks)
                    chunkManagers[submapID].saveSubmapB(filename);           
                chunkManagers[submapID].clearChunks();
                //chunkManagers[submapID].loadSubmapB(filename);
                //chunkManagers[submapID].RecomputeMeshesWithoutCheck(_camera);
            }
            void simplifySubmap(int submapID)
            {
                if(chunkManagers[submapID].allMeshes.empty())
                {
                chunkManagers[submapID].RecomputeMeshesWithoutCheck(_camera);
                }
                else
                {
                    std::cout<<"No need to recompute..."<<std::endl;
                }
                GetSubmapMeshSimplified(submapID);
            }
            void simplifySubmapSemantic(int submapID)
            {
                /*
                if(chunkManagers[submapID].allMeshes.empty())
                {
                chunkManagers[submapID].RecomputeMeshesWithoutCheck(_camera);
                }
                else
                {
                    std::cout<<"No need to recompute..."<<std::endl;
                }
                */
                SimplifySemanticSubmapMeshes(submapID);
            }
            void deactivateSubmap(int submapID)
            {
                chunkManagers[submapID].LockCurrentMesh();
                chunkManagers[submapID].is_active = false;
                chunkManagers[submapID].clear_hash_aggregated_map();
                chunkManagers[submapID].clearMeshes();
                chunkManagers[submapID].UnlockCurrentMesh();
            }
            void loadSubmap(int submapID,const std::string &filepath="./")
            {
                //global_changed  = true;
                
                std::string filename = filepath+"m"+std::to_string(submapID)+".map";
                std::cout<<"load submap from "<<filename<<std::endl;
                chunkManagers[submapID].loadSubmapB(filename);
                std::cout<<"finish loading."<<std::endl;
                //chunkManagers[submapID].isValid = true;
            }
            void SetCompactRatio(double r)
            {
                compact_ratio = r;
                if(r > 1)
                setTargetNum((int)r);
            }
            void setTargetNum(int num)
            {
                target_num = num;
            }
            std::vector<ChunkManager> chunkManagers;
            ChunkManager globalChunkManager;
            ChunkID globalMaxChunkID;
            ChunkID globalMinChunkID;
            std::vector<float> globalCandidateCube;
            std::vector<ChunkID> maxChunkIDs;
            std::vector<ChunkID> minChunkIDs;
            
            std::vector<std::vector<float>> candidateCubes;
            ChunkSet meshesToUpdate;
            Eigen::Vector3i chunkSize;
            float voxelResolution;
            bool useColor;
            Simplifier simplifier;
            //bool global_changed = false;
            PinholeCamera _camera;
            double compact_ratio = 0.5;
            size_t target_num = 500000;
            MeshPtr global_mesh;
	        size_t max_vertices = 1800000;
            bool showSubmapStructure = false;
            int submapColor=0;
            bool need_to_recompute_for_reference = false;
            //int velocity_threshold = 500;
            //int total_threshold = 40000;
        protected:
    };
    typedef std::shared_ptr<Chisel> ChiselPtr;
    typedef std::shared_ptr<const Chisel> ChiselConstPtr;
    

} // namespace chisel 

#endif // CHISEL_H_ 
