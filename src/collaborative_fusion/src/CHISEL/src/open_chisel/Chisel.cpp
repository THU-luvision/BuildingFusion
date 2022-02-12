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

#include <open_chisel/Chisel.h>

#include <open_chisel/io/PLY.h>

#include <open_chisel/geometry/Raycast.h>

#include <iostream>
#include <fstream>
#include <open_chisel/simplify/MeshSimplification.h>
#include "cnpy.h"
#define MAX_SUBMAP_TO_OCCUSEG 12

namespace chisel
{
 

    Chisel::Chisel( const Eigen::Vector3i &_chunkSize, float _voxelResolution, bool _useColor, const PinholeCamera& camera):chunkSize(_chunkSize),voxelResolution(_voxelResolution),useColor(_useColor),_camera(camera)
    {

        chunkManagers = std::vector<ChunkManager>();        
        maxChunkIDs = std::vector<ChunkID>();
        minChunkIDs = std::vector<ChunkID>();
        candidateCubes = std::vector<std::vector<float>>();
        //chunkManagers.reserve(100);
        globalChunkManager = ChunkManager(chunkSize,voxelResolution,useColor);
        chunkManagers.emplace_back(chunkSize,voxelResolution,useColor);
        std::cout<<chunkManagers.size()<<std::endl;
        chunkManagers[0].submap_id = 0;
        maxChunkIDs.push_back(ChunkID());
        minChunkIDs.push_back(ChunkID());
        candidateCubes.push_back(std::vector<float>());

        //meshesToUpdate = std::vector<ChunkSet>();
        //meshesToUpdate.push_back(ChunkSet());
        //globalChunkManager= ChunkManager(chunkSize,voxelResolution,useColor);

    }

    Chisel::~Chisel()
    {
        // TODO Auto-generated destructor stub
    }

    void Chisel::Reset()
    {
        for(auto &chunkManager: chunkManagers)
        {
        chunkManager.Reset();
        }
        globalChunkManager.Reset();
        chunkManagers.clear();
        /*for(int i = 0;i!=meshesToUpdate.size();++i)
        meshesToUpdate[i].clear();*/
    }
    void Chisel::bufferIntegratorSIMDCentroids(ProjectionIntegrator& integrator,const Transform& depthExtrinsic)
    {
        ChunkManager &chunkManager = globalChunkManager;
        //set updated centroids
        Vec3 halfVoxel = Vec3(chunkManager.GetResolution(), chunkManager.GetResolution(), chunkManager.GetResolution()) * 0.5f;
        Vec3List diff_centroids;
        diff_centroids.resize(static_cast<size_t>(chunkManager.GetChunkSize()(0) * chunkManager.GetChunkSize()(1) * chunkManager.GetChunkSize()(2)));
        int i = 0;
        for (int z = 0; z < chunkManager.GetChunkSize()(2); z++)
        {
            for(int y = 0; y < chunkManager.GetChunkSize()(1); y++)
            {
                for(int x = 0; x < chunkManager.GetChunkSize()(0); x++)
                {
                    diff_centroids[i] = depthExtrinsic.linear().transpose() *Vec3(x, y, z) * chunkManager.GetResolution() + halfVoxel;
                    i++; 
                }
            }
        }

        integrator.diff_centroids = diff_centroids;

        int NumVoxels = chunkManager.GetChunkSize()(2) * chunkManager.GetChunkSize()(0) * chunkManager.GetChunkSize()(0);


        float data0[8],data1[8],data2[8];

        for (int z = 0; z <  chunkManager.GetChunkSize()(2); z++)
        {
            for(int y = 0; y <  chunkManager.GetChunkSize()(1); y++)
            {
                for(int x = 0; x <  chunkManager.GetChunkSize()(0); x+=8)
                {

                    int pos = z * chunkManager.GetChunkSize()(0) * chunkManager.GetChunkSize()(1)
                            + y * chunkManager.GetChunkSize()(0) + x;
                    // 8 * float3 std::vectors are converted to 3 * float8 std::vectors
                    //(f00 f01 f02 f10 f11 f12) to (f00 f10 f20 f30 ...)
                    for(int a = 0; a < 8; a++)
                    {
                        int local_pos = a;
                        data0[local_pos] = integrator.diff_centroids[pos + local_pos](0);
                        data1[local_pos] = integrator.diff_centroids[pos + local_pos](1);
                        data2[local_pos] = integrator.diff_centroids[pos + local_pos](2);
                    }

#if 1
                    int centroids_pos_simd = z * chunkManager.GetChunkSize()(0) * chunkManager.GetChunkSize()(1)
                            + y * chunkManager.GetChunkSize()(0) + x;
                    centroids_pos_simd /= 8;
                    integrator.centroids_simd0[centroids_pos_simd] = _mm256_loadu_ps(data0);
                    integrator.centroids_simd1[centroids_pos_simd] = _mm256_loadu_ps(data1);
                    integrator.centroids_simd2[centroids_pos_simd] = _mm256_loadu_ps(data2);
#endif

                }
            }
        }
    }
    void Chisel::bufferIntegratorSIMDCentroids(ProjectionIntegrator& integrator,const Transform& depthExtrinsic,int submapID)
    {
        ChunkManager &chunkManager = chunkManagers[submapID];
        //set updated centroids
        Vec3 halfVoxel = Vec3(chunkManager.GetResolution(), chunkManager.GetResolution(), chunkManager.GetResolution()) * 0.5f;
        Vec3List diff_centroids;
        diff_centroids.resize(static_cast<size_t>(chunkManager.GetChunkSize()(0) * chunkManager.GetChunkSize()(1) * chunkManager.GetChunkSize()(2)));
        int i = 0;
        for (int z = 0; z < chunkManager.GetChunkSize()(2); z++)
        {
            for(int y = 0; y < chunkManager.GetChunkSize()(1); y++)
            {
                for(int x = 0; x < chunkManager.GetChunkSize()(0); x++)
                {
                    diff_centroids[i] = depthExtrinsic.linear().transpose() *Vec3(x, y, z) * chunkManager.GetResolution() + halfVoxel;
                    i++; 
                }
            }
        }

        integrator.diff_centroids = diff_centroids;

        int NumVoxels = chunkManager.GetChunkSize()(2) * chunkManager.GetChunkSize()(0) * chunkManager.GetChunkSize()(0);


        float data0[8],data1[8],data2[8];

        for (int z = 0; z <  chunkManager.GetChunkSize()(2); z++)
        {
            for(int y = 0; y <  chunkManager.GetChunkSize()(1); y++)
            {
                for(int x = 0; x <  chunkManager.GetChunkSize()(0); x+=8)
                {

                    int pos = z * chunkManager.GetChunkSize()(0) * chunkManager.GetChunkSize()(1)
                            + y * chunkManager.GetChunkSize()(0) + x;
                    // 8 * float3 std::vectors are converted to 3 * float8 std::vectors
                    //(f00 f01 f02 f10 f11 f12) to (f00 f10 f20 f30 ...)
                    for(int a = 0; a < 8; a++)
                    {
                        int local_pos = a;
                        data0[local_pos] = integrator.diff_centroids[pos + local_pos](0);
                        data1[local_pos] = integrator.diff_centroids[pos + local_pos](1);
                        data2[local_pos] = integrator.diff_centroids[pos + local_pos](2);
                    }

#if 1
                    int centroids_pos_simd = z * chunkManager.GetChunkSize()(0) * chunkManager.GetChunkSize()(1)
                            + y * chunkManager.GetChunkSize()(0) + x;
                    centroids_pos_simd /= 8;
                    integrator.centroids_simd0[centroids_pos_simd] = _mm256_loadu_ps(data0);
                    integrator.centroids_simd1[centroids_pos_simd] = _mm256_loadu_ps(data1);
                    integrator.centroids_simd2[centroids_pos_simd] = _mm256_loadu_ps(data2);
#endif

                }
            }
        }
    }
    int Chisel::GetFullMeshes(unsigned char *vertices,int submapID,int start)
    {

        ChunkManager &chunkManager = chunkManagers[submapID];
        size_t v = start;
        unsigned char *cur_vert;
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            //int anchorFrameID = chunkManager.GetChunk(it.first)->GetReferenceFrameIndex();
            if(it.second->vertices.size() != it.second->colors.size() || it.second->vertices.size() != it.second->normals.size())
            {
                std::cout << "mesh vertex error!" <<std::endl;
                while(1)
                {
                } 
            }
                  //std::cout<<"number of vertices: "<<it.second->vertices.size()<<std::endl;
                 // getchar();
            for (int i =0; i < it.second->vertices.size();i+=3)
            {

                for(int j = 0; j < 3; j++)
                {
                    /* const Vec3& vert = it.second->vertices[i+j];
                    const Vec3& color = it.second->colors[i+j];
                    const Vec3& normal = it.second->normals[i+j];
		            cur_vert = &vertices[16*v];
                    toFloat16(vert(0),&cur_vert[0]);
                    toFloat16(vert(1),&cur_vert[2]);
                    toFloat16(vert(2),&cur_vert[4]);
                    
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[6])) = rgb_value;
                    toFloat16(normal(0),&cur_vert[10]);
                    toFloat16(normal(1),&cur_vert[12]);
                    toFloat16(normal(2),&cur_vert[14]);
                    v++;
                    */
                    const Vec3& vert = it.second->vertices[i+j];
                     Vec3 &color = it.second->colors[i+j];
                    const Vec3& normal = it.second->normals[i+j];
		            cur_vert = &vertices[22*v];
                    //toFloat16(vert(0),&cur_vert[0]);
                    //toFloat16(vert(1),&cur_vert[2]);
                    //toFloat16(vert(2),&cur_vert[4]);
                    *((float *)(&cur_vert[0])) = vert(0);
                    *((float *)(&cur_vert[4])) = vert(1);
                    *((float *)(&cur_vert[8])) = vert(2);
                    if(!showSubmapStructure)
                    {
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[12])) = rgb_value;
                    }
                    else
                    {
                    color(submapColor%3)=1;
                    //color = color * (submapColor/3);
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[12])) = rgb_value;                        
                    }
                    toFloat16(normal(0),&cur_vert[16]);
                    toFloat16(normal(1),&cur_vert[18]);
                    toFloat16(normal(2),&cur_vert[20]);
                    v++;
                }

            }
        }
        return v;
    }
    /*
    if you want to use this function, you need to check if the semantic mesh is deleted.
    */

    int Chisel::GetSegmentationInput( int submap_id, float *hash_aggregated_coords, 
        float *hash_aggregated_feature, int start, 
        const PoseSE3d & relativeChange, const PoseSE3d & cameraPose) 
    {
        TICK("SEMANTIC::GetSegmentationInput");
        std::cout<<"submap_id in GetSegmentationInput: "<<submap_id<<std::endl;
        chunkManagers[submap_id].propogated = true;
        float *cur_coords;
        float *cur_features;
        Vec3 trans;
        Mat3x3 rot;
        ChunkManager &chunkManager = chunkManagers[submap_id];
        Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
        Eigen::Vector3f t = relativeChange.translation().cast<float>();
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();
        
           // for scene 73
        if(0)
        {
           trans << 1.85862,1.45228,1.29919;
           rot << -0.601221,-0.386908,0.699167,
                   -0.797818,0.339854,-0.497982,
                   -0.0449414,-0.857206,-0.51301;
        }
////            for old pytorch ckpt
//            trans << 0,0,0;
//            rot << 0,1,0,
//                    1,0,0,
//                    0,0,1;
//            for newer one

             trans << 0,0,0;
             rot << 1,0,0,
                     0,0,1,
                     0,-1,0;
//            identiti
/*            trans << 0,0,0;
            rot << 1,0,0,
                    0,1,0,
                    0,0,1;*/

        //TICK("SEMANTIC::GetSegmentationInput::SaveMeshForSimplify");
        size_t full_vert_count = 0;
        size_t simplified_vertex_count=start;
        //int isSimplified = chunkManagers[submap_id].isSemanticSimplified();
        chunkManager.LockCurrentMesh();
        int is_active = chunkManagers[submap_id].is_active;
        //If a chunkManager is already deactivated, then we use the stored hashtable
        //for segmentation. 
        AggregatedHash & aggregated_hash = chunkManager.aggregated_hash;
        std::cout<<"is_active: "<<is_active<<std::endl;
        if(!is_active)
        {
            chunkManager.UnlockCurrentMesh();
            // all meshes is already deleted. We use old hash table for the segmentation
            // for each submap we maintain a vertex table.( color and coordinates)  
            
            for(auto iter = aggregated_hash.begin(); iter != aggregated_hash.end(); ++iter )
            {
                //Vec3 new_vert=rot * (cR * ( R * iter->second.coord + t ) + ct) + trans;
                Vec3 new_vert=rot *  ( R * iter->second.coord + t )  + trans;
                Vec3 new_normal = rot * R * iter->second.normal;

                iter->second.global_coord = new_vert;
                iter->second.global_normal = new_normal;
                cur_coords = &hash_aggregated_coords[3*simplified_vertex_count];
                cur_features = &hash_aggregated_feature[3*simplified_vertex_count];
                cur_coords[0] = new_vert(0);
                cur_coords[1] = new_vert(1);
                cur_coords[2] = new_vert(2);
                Vec3 mean_color = iter->second.feature;
                cur_features[0] = mean_color(0)*2-1.0f;
                cur_features[1] = mean_color(1)*2-1.0f;
                cur_features[2] = mean_color(2)*2-1.0f;
                iter->second.simplified_vertex_count = simplified_vertex_count;
                simplified_vertex_count ++ ;


            }          
            return simplified_vertex_count;
        }
        //if the chunkManager is active, we need to use the fresh mesh for semantation.

        chunkManager.prepareSemanticMesh();
        MeshPtr &fullMesh = chunkManager.semantic_mesh;
        fullMesh->Clear();


        //chunkManager.hash_aggregated_simplified_start = simplified_vertex_count;
        std::vector<int> & hash_aggregated_map = chunkManager.hash_aggregated_map;
        hash_aggregated_map.clear();//resize(SUBMAP_MAX_VERTEX);
        //chunkManager.hash_aggregated_map.resize(vertexs_count);
        std::cout<<"chunkManager.voxelResolutionMeters:"<<chunkManager.voxelResolutionMeters<<std::endl;
        std::cout<<"All meshes: "<<chunkManager.GetAllMeshes().size()<<std::endl;
        if(chunkManager.voxelResolutionMeters==0.00625f)
        {
            aggregated_hash.clear();
            for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
            {
                Vec3 mean_color,mean_coord,mean_normal;
                
                mean_color<<0,0,0;
                mean_coord<<0,0,0;
                mean_normal <<0,0,0;
                size_t vert_in_chunk_count=0;
                double residual = 0;
                //std::cout<<it.second->vertices.size()<<" "<<it.second->colors.size()<<" "<<it.second->normals.size()<<std::endl;
                for (int i = 0; i < it.second->vertices.size(); ++i) 
                {
                    /* we need to merge the scattered mesh into one mesh.*/
                    fullMesh->vertices.push_back(it.second->vertices[i]);
                    fullMesh->colors.push_back(it.second->colors[i]);
                    fullMesh->normals.push_back(it.second->normals[i]);
                    fullMesh->indices.push_back(full_vert_count);
                    
                    hash_aggregated_map.push_back(simplified_vertex_count);
//                ---------------------------------
                    mean_coord=mean_coord+it.second->vertices[i];
                    mean_color=mean_color+it.second->colors[i];
                    mean_normal = mean_normal + it.second->normals[i];
                    full_vert_count++;
                    vert_in_chunk_count++;
                }

                if (vert_in_chunk_count!=0)
                {
//                std::cout<<"before::"<<mean_coord<<"    "<<mean_color<<std::endl;
                    mean_coord=mean_coord/((float)(vert_in_chunk_count));
                    mean_color=mean_color/((float)(vert_in_chunk_count));
                    mean_normal = mean_normal/((float)(vert_in_chunk_count));
                    for (int i = 0; i < it.second->vertices.size(); ++i) 
                    {
                        /* we need to merge the scattered mesh into one mesh.*/
                        residual +=  (it.second->normals[i] - mean_normal).norm();
                    }                    
                    residual /= vert_in_chunk_count;

                    mean_normal.normalize();
//                  std::cout<<"after::"<<mean_coord<<"    "<<mean_color<<std::endl;
//                  std::cout<<"count::"<<(float)(vert_in_chunk_count)<<std::endl;
                    //Vec3 new_vert=rot * (cR * ( R * mean_coord + t ) + ct) + trans;
                    Vec3 new_vert=rot *  ( R * mean_coord + t )  + trans;
                    Vec3 new_normal=rot * cR * R * mean_normal;
                    cur_coords = &hash_aggregated_coords[3*simplified_vertex_count];
                    cur_features = &hash_aggregated_feature[3*simplified_vertex_count];

                    cur_coords[0] = new_vert(0);
                    cur_coords[1] = new_vert(1);
                    cur_coords[2] = new_vert(2);

                    cur_features[0] = mean_color(0)*2-1.0f;
                    cur_features[1] = mean_color(1)*2-1.0f;
                    cur_features[2] = mean_color(2)*2-1.0f;
                    
                    ChunkAggregatedHash cah;
                    cah.simplified_vertex_count = simplified_vertex_count;
                    cah.coord = mean_coord;
                    cah.global_coord = new_vert;
                    cah.normal = mean_normal;
                    cah.global_normal = new_normal;
                    cah.feature = mean_color;
                    cah.residual = residual;
                    if(aggregated_hash.find(it.first) == aggregated_hash.end())
                    cah.is_new = true;
                    aggregated_hash[it.first] = cah;  

                    simplified_vertex_count++;

                }

            }
            //std::cout<<"aggregated_hash: "<<aggregated_hash.size()<<std::endl;
            //std::cout<<"aggregated_hash: "<<chunkManagers[submap_id].aggregated_hash.size()<<std::endl;
        }
        else 
        {
            std::cerr << "voxel resolution must be 0.00625" << std::endl;
            std::abort();
        }
        chunkManager.UnlockCurrentMesh();
        //TOCK("SEMANTIC::GetSegmentationInput::SaveMeshForSimplify");
        //hash_aggregated_map.resize(full_vert_count);
        //TOCK("SEMANTIC::GetSegmentationInput");
        std::cout<<"Finish Getting the input of segmentation."<<std::endl;
        return simplified_vertex_count;
    }

    int Chisel::GetSegmentationInputGlobal( float *hash_aggregated_coords, 
        float *hash_aggregated_feature) 
    {
        TICK("SEMANTIC::GetSegmentationInput");
        float *cur_coords;
        float *cur_features;
        Vec3 trans;
        Mat3x3 rot;
        ChunkManager &chunkManager = globalChunkManager;
        
           // for scene 73
        if(0)
        {
           trans << 1.85862,1.45228,1.29919;
           rot << -0.601221,-0.386908,0.699167,
                   -0.797818,0.339854,-0.497982,
                   -0.0449414,-0.857206,-0.51301;
        }
////            for old pytorch ckpt
//            trans << 0,0,0;
//            rot << 0,1,0,
//                    1,0,0,
//                    0,0,1;
//            for newer one

             trans << 0,0,0;
             rot << 1,0,0,
                     0,0,1,
                     0,-1,0;
//            identiti
/*            trans << 0,0,0;
            rot << 1,0,0,
                    0,1,0,
                    0,0,1;*/

        //TICK("SEMANTIC::GetSegmentationInput::SaveMeshForSimplify");
        size_t full_vert_count = 0;
        size_t simplified_vertex_count=0;
        //If a chunkManager is already deactivated, then we use the stored hashtable
        //for segmentation. 
        AggregatedHash & aggregated_hash = chunkManager.aggregated_hash;

        //if the chunkManager is active, we need to use the fresh mesh for semantation.

        chunkManager.prepareSemanticMesh();
        MeshPtr &fullMesh = chunkManager.semantic_mesh;
        fullMesh->Clear();


        //chunkManager.hash_aggregated_simplified_start = simplified_vertex_count;
        std::vector<int> & hash_aggregated_map = chunkManager.hash_aggregated_map;
        hash_aggregated_map.clear();//resize(SUBMAP_MAX_VERTEX);
        //chunkManager.hash_aggregated_map.resize(vertexs_count);
        std::cout<<"chunkManager.voxelResolutionMeters:"<<chunkManager.voxelResolutionMeters<<std::endl;
        std::cout<<"All meshes: "<<chunkManager.GetAllMeshes().size()<<std::endl;
        if(chunkManager.voxelResolutionMeters==0.00625f)
        {
            aggregated_hash.clear();
            for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
            {
                Vec3 mean_color,mean_coord,mean_normal;
                
                mean_color<<0,0,0;
                mean_coord<<0,0,0;
                mean_normal <<0,0,0;
                size_t vert_in_chunk_count=0;
                double residual = 0;
                //std::cout<<it.second->vertices.size()<<" "<<it.second->colors.size()<<" "<<it.second->normals.size()<<std::endl;
                for (int i = 0; i < it.second->vertices.size(); ++i) 
                {
                    /* we need to merge the scattered mesh into one mesh.*/
                    fullMesh->vertices.push_back(it.second->vertices[i]);
                    fullMesh->colors.push_back(it.second->colors[i]);
                    fullMesh->normals.push_back(it.second->normals[i]);
                    fullMesh->indices.push_back(full_vert_count);
                    
                    hash_aggregated_map.push_back(simplified_vertex_count);
//                ---------------------------------
                    mean_coord=mean_coord+it.second->vertices[i];
                    mean_color=mean_color+it.second->colors[i];
                    mean_normal = mean_normal + it.second->normals[i];
                    full_vert_count++;
                    vert_in_chunk_count++;
                }

                if (vert_in_chunk_count!=0)
                {
//                std::cout<<"before::"<<mean_coord<<"    "<<mean_color<<std::endl;
                    mean_coord=mean_coord/((float)(vert_in_chunk_count));
                    mean_color=mean_color/((float)(vert_in_chunk_count));
                    mean_normal = mean_normal/((float)(vert_in_chunk_count));
                    for (int i = 0; i < it.second->vertices.size(); ++i) 
                    {
                        /* we need to merge the scattered mesh into one mesh.*/
                        residual +=  (it.second->normals[i] - mean_normal).norm();
                    }                    
                    residual /= vert_in_chunk_count;

                    mean_normal.normalize();
//                  std::cout<<"after::"<<mean_coord<<"    "<<mean_color<<std::endl;
//                  std::cout<<"count::"<<(float)(vert_in_chunk_count)<<std::endl;
                    //Vec3 new_vert=rot * (cR * ( R * mean_coord + t ) + ct) + trans;
                    Vec3 new_vert=rot *   mean_coord   + trans;
                    Vec3 new_normal=rot *  mean_normal;
                    cur_coords = &hash_aggregated_coords[3*simplified_vertex_count];
                    cur_features = &hash_aggregated_feature[3*simplified_vertex_count];

                    cur_coords[0] = new_vert(0);
                    cur_coords[1] = new_vert(1);
                    cur_coords[2] = new_vert(2);

                    cur_features[0] = mean_color(0)*2-1.0f;
                    cur_features[1] = mean_color(1)*2-1.0f;
                    cur_features[2] = mean_color(2)*2-1.0f;
                    
                    ChunkAggregatedHash cah;
                    cah.simplified_vertex_count = simplified_vertex_count;
                    cah.coord = mean_coord;
                    cah.global_coord = new_vert;
                    cah.normal = mean_normal;
                    cah.global_normal = new_normal;
                    cah.feature = mean_color;
                    cah.residual = residual;
                    if(aggregated_hash.find(it.first) == aggregated_hash.end())
                    cah.is_new = true;
                    aggregated_hash[it.first] = cah;  

                    simplified_vertex_count++;

                }

            }
            //std::cout<<"aggregated_hash: "<<aggregated_hash.size()<<std::endl;
            //std::cout<<"aggregated_hash: "<<chunkManagers[submap_id].aggregated_hash.size()<<std::endl;
        }
        else 
        {
            std::cerr << "voxel resolution must be 0.00625" << std::endl;
            std::abort();
        }
        //TOCK("SEMANTIC::GetSegmentationInput::SaveMeshForSimplify");
        //hash_aggregated_map.resize(full_vert_count);
        //TOCK("SEMANTIC::GetSegmentationInput");
        std::cout<<"Finish Getting the input of segmentation."<<std::endl;
        return simplified_vertex_count;
    }
    int Chisel::GetSegmentationInput(float *hash_aggregated_coords, 
    float *hash_aggregated_feature, const PoseSE3dList & relativeChanges, const PoseSE3d & cameraPose)
    {
        int start = 0;
        //TICK("CHISEL_MESHING::GetSemanticMesh:GetSegmentationInput");   
        for(int i = 0; i != chunkManagers.size(); ++i)
        {
            start = GetSegmentationInput(i, hash_aggregated_coords, hash_aggregated_feature, start ,
                relativeChanges[i], cameraPose);
        }
        //TOCK("CHISEL_MESHING::GetSemanticMesh:GetSegmentationInput"); 
        // return simplified_vertex_count
        std::cout<<"Number of vertices in OccuSeg: "<<start<<std::endl;
        return start;
    }
    int Chisel::GetSegmentationInput(float *hash_aggregated_coords, float *hash_aggregated_feature, 
        const std::set<int> &unlabeled_submap, const PoseSE3dList & relativeChanges, const PoseSE3d & cameraPose)
    {
        int start = 0, n = 0;
        
        if(unlabeled_submap.size() >= MAX_SUBMAP_TO_OCCUSEG)
        {
            std::cout<<YELLOW<<"[WARNING]::Too many mesh, may cause cuda crush."<<RESET<<std::endl;
        }
        //TICK("CHISEL_MESHING::GetSemanticMesh:GetSegmentationInput");   

        for(auto i = unlabeled_submap.rbegin(); i != unlabeled_submap.rend()&&n < MAX_SUBMAP_TO_OCCUSEG; ++i, ++n)
        {
            if(*i >= chunkManagers.size()) continue;
            start = GetSegmentationInput(*i, hash_aggregated_coords, hash_aggregated_feature, start ,
                relativeChanges[*i], cameraPose);
        }
        //TOCK("CHISEL_MESHING::GetSemanticMesh:GetSegmentationInput"); 
        // return simplified_vertex_count
        return start;
    }
    int Chisel::UpdateSemanticMeshWithLabel(int submap_id, long *semantic_label, long * instance_label)
    {
        std::cout<<"aggregated_hash size: "<<chunkManagers[submap_id].aggregated_hash.size()<<std::endl;
        ChunkManager &chunkManager = chunkManagers[submap_id];
        chunkManager.semantic_mesh->labels.resize(chunkManager.semantic_mesh->vertices.size());

        AggregatedHash & aggregated_hash = chunkManager.aggregated_hash;
        std::vector<int> &hash_aggregated_map = chunkManager.hash_aggregated_map;
        std::cout<<"aggregated_hash size: "<<aggregated_hash.size()<<std::endl;
        for(auto iter =  aggregated_hash.begin(); iter != aggregated_hash.end(); ++iter)
        {
            //assert(semantic_label[iter->second.simplified_vertex_count] < 20  );
            //assert(instance_label[iter->second.simplified_vertex_count] < 100 );
            //if(instance_label[iter->second.simplified_vertex_count] < 0)
            iter->second.label = semantic_label[iter->second.simplified_vertex_count] * 10000 + instance_label[iter->second.simplified_vertex_count]; 

            //std::cout<<"update aggregated hash labels."<<std::endl;
        }
        if(chunkManager.is_active)
        {
            chunkManager.LockCurrentMesh();
            std::cout<<"mesh is not simplified, need to update "<<hash_aggregated_map.size()<<" vertices."<<std::endl;
            for (int j = 0; j < hash_aggregated_map.size(); ++j)
            {
                chunkManager.semantic_mesh->labels[j]=semantic_label[hash_aggregated_map[j]] * 10000 + instance_label[hash_aggregated_map[j]];
            }
            return chunkManager.semantic_mesh->vertices.size();        
        }
        else if( !chunkManager.isSemanticSimplified())
        {
            chunkManager.LockCurrentMesh();
            // need a lock, and until  get full mesh.
            /*
            chunkManager.prepareSemanticMesh();
            MeshPtr &fullMesh = chunkManager.semantic_mesh;
            fullMesh->Clear();
            //chunkManager.hash_aggregated_map.resize(vertexs_count);
            std::cout<<"chunkManager.voxelResolutionMeters:"<<chunkManager.voxelResolutionMeters<<std::endl;
            if(chunkManager.voxelResolutionMeters==0.00625f)
            {
                for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
                {
                    Vec3 mean_color,mean_coord,mean_normal;
                    
                    mean_color<<0,0,0;
                    mean_coord<<0,0,0;
                    mean_normal <<0,0,0;
                    size_t vert_in_chunk_count=0;
                    double residual = 0;
                    for (int i = 0; i < it.second->vertices.size(); ++i) 
                    {
                        // we need to merge the scattered mesh into one mesh./
                        fullMesh->vertices.push_back(it.second->vertices[i]);
                        fullMesh->colors.push_back(it.second->colors[i]);
                        fullMesh->normals.push_back(it.second->normals[i]);
                        if(aggregated_hash.find(it.first) != aggregated_hash.end())
                        fullMesh->labels.push_back(aggregated_hash[it.first].label);
                        else fullMesh->labels.push_back(0);
                    }

                }
                //std::cout<<"aggregated_hash: "<<aggregated_hash.size()<<std::endl;
                //std::cout<<"aggregated_hash: "<<chunkManagers[submap_id].aggregated_hash.size()<<std::endl;
            }*/  
            return chunkManager.semantic_mesh->vertices.size();       
        }
        else 
        {
            MeshPtr & simplified_mesh = chunkManager.simplified_mesh;
            long last_label = 0; 
            //If semantic mesh is simplified, we use the chunk's label to replace the origin label
            for(int j = 0; j != simplified_mesh->labels.size(); ++j)
            {
                ChunkID chunk_id = chunkManager.GetIDAt( simplified_mesh->vertices[j]);
                if(aggregated_hash.find(chunk_id) != aggregated_hash.end())
                {
                    simplified_mesh->labels[j] = aggregated_hash[chunk_id].label;
                    last_label = simplified_mesh->labels[j];
                }
                else
                {
                    simplified_mesh->labels[j] = last_label;
                }
            }
            return chunkManager.simplified_mesh->vertices.size();
        }
        //memcpy(chunkManager.semantic_mesh->labels.data(),label,chunkManager.semantic_mesh->vertices.size()* sizeof(long));
        
    }
    int Chisel::UpdateSemanticMeshWithLabelGlobal(long *semantic_label, long * instance_label)
    {

        std::cout<<"aggregated_hash size: "<<globalChunkManager.aggregated_hash.size()<<std::endl;
        ChunkManager &chunkManager = globalChunkManager;
        chunkManager.semantic_mesh->labels.resize(chunkManager.semantic_mesh->vertices.size());

        AggregatedHash & aggregated_hash = chunkManager.aggregated_hash;
        std::vector<int> &hash_aggregated_map = chunkManager.hash_aggregated_map;
        std::cout<<"aggregated_hash size: "<<aggregated_hash.size()<<std::endl;
        for(auto iter =  aggregated_hash.begin(); iter != aggregated_hash.end(); ++iter)
        {
            //assert(semantic_label[iter->second.simplified_vertex_count] < 20  );
            //assert(instance_label[iter->second.simplified_vertex_count] < 100 );
            //if(instance_label[iter->second.simplified_vertex_count] < 0)
            iter->second.label = semantic_label[iter->second.simplified_vertex_count] * 10000 + instance_label[iter->second.simplified_vertex_count]; 

            //std::cout<<"update aggregated hash labels."<<std::endl;
        }

        for (int j = 0; j < hash_aggregated_map.size(); ++j)
        {
            chunkManager.semantic_mesh->labels[j]=semantic_label[hash_aggregated_map[j]] * 10000 + instance_label[hash_aggregated_map[j]];
        }
        return chunkManager.semantic_mesh->vertices.size();        
        
    }
    int Chisel::UpdateSemanticMeshWithLabel(long *semantic_label, long * instance_label, const std::set<int> &unlabeled_submap)
    {
        //TICK("CHISEL_MESHING::GetSemanticMesh::UpdateSemanticMeshWithLabel");        
        int all_count = 0;
        for(int i = 0 ; i != chunkManagers.size(); ++i)
        {
            if(unlabeled_submap.find(i) != unlabeled_submap.end() && chunkManagers[i].propogated == true)
            {
                int count_vertex;
                //std::cout<<"Update Semantic Mesh Label: submap "<<i<<"/"<<count_vertex<<std::endl;
                count_vertex = UpdateSemanticMeshWithLabel(i, semantic_label, instance_label);
                std::cout<<"Update Semantic Mesh Label: submap "<<i<<"/"<<count_vertex<<std::endl;
                all_count += count_vertex;
            }
            else
            {
                if(!chunkManagers[i].isSemanticSimplified())
                chunkManagers[i].LockCurrentMesh();
            }
            chunkManagers[i].propogated = false;
        }
        //TOCK("CHISEL_MESHING::GetSemanticMesh::UpdateSemanticMeshWithLabel");   
        return all_count;
    }
    int Chisel::GetFloorPCD(int submap_id, Vec3List &points)
    {
        ChunkManager &chunkManager = chunkManagers[submap_id];
        AggregatedHash & aggregated_hash = chunkManager.aggregated_hash;

        int count = 0;
        Eigen::Vector3f floor;
        floor<<0,0,1.0;
        for(auto iter =  aggregated_hash.begin(); iter != aggregated_hash.end(); ++iter)
        {
            if(iter->second.is_floor() && iter->second.global_normal.transpose() * floor > 0.5)
            {
                //Vec3 point = iter->coord;
                points.push_back(iter->second.coord);
                count ++;
            }
        }
        return count;        
    }
    int Chisel::GetFloorPCD(Vec3List &points, const std::set<int> &unlabeled_submap)
    {
        int count = 0;
        points.clear();
        for(auto iter = unlabeled_submap.begin(); iter != unlabeled_submap.end(); ++iter)
        {
            count += GetFloorPCD(*iter, points);
        }
        return count;
    }
    int Chisel::GetPointCloud(int submap_id, Vec3List &points, Vec3List &normals, std::vector<double> &residuals)
    {
        ChunkManager &chunkManager = chunkManagers[submap_id];
        AggregatedHash & aggregated_hash = chunkManager.aggregated_hash;
        /*
        Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
        Eigen::Vector3f t = relativeChange.translation().cast<float>()+turbulence;
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();
        Eigen::Vector3f trans;
        Eigen::Matrix3f rot;
        trans << 0,0,0;
        rot <<  1,0,0,
                0,0,1,
                0,-1,0;
        */
        int count = 0;
        Eigen::Vector3f floor;
        floor<<0,0,1;
        for(auto iter =  aggregated_hash.begin(); iter != aggregated_hash.end(); ++iter)
        {
            if(iter->second.is_room_border() && std::fabs(iter->second.global_normal.transpose() * floor)< 0.3)
            {
                //Vec3 point = iter->coord;
                points.push_back(iter->second.global_coord);
                normals.push_back(iter->second.global_normal);
                residuals.push_back(iter->second.residual);
                count ++;
            }
        }
        return count;
    }
    Eigen::Vector3f Chisel::GetFloorNormal(int submap_id)
    {
        if(submap_id >= chunkManagers.size())
        return Eigen::Vector3f(999.0,0,0);
        auto &chunkManager = chunkManagers[submap_id];
        auto &aggregated_hash = chunkManager.aggregated_hash;
        Eigen::Vector3f floor, result;
        floor<<0,0,1;
        result.setZero();
        int point_count = 0;
        for(auto iter = aggregated_hash.begin(); iter != aggregated_hash.end(); ++iter)
        {
            //std::cout<<std::endl;
            //std::cout<<"Error: "<<iter->second.global_normal.transpose() * floor<<" "<<std::endl;
            //std::cout<<iter->second.normal<<std::endl;
            if(get_semantic_label(iter->second.label)  == FLOOR && iter->second.global_normal.transpose() * floor >= 0.8)
            {
                result += iter->second.normal ;
                point_count += 1;
            }
        }
        if(point_count == 0)
        {
            result(0) = 999.0;
            return result;
        }
        result /= point_count;
        result.normalize();
        floor << 0, -1, 0;
        if(result.transpose() * floor >= 0.8 )
        return result;
        else 
        {
            result<<999.0,0,0;
            return result;
        }
    }
    bool Chisel::SavePointCloudToPLY(const std::string &filename, const Vec3List &points, 
        const Vec3List &normals, const Vec3List &colors )
    {
        bool success = SavePointCloudPLYASCII(filename, points, normals, colors);
        return success;
    }
    int Chisel::GetPointCloud(Vec3List &points, Vec3List &normals, std::vector<double> &residuals)
    {
        int count = 0;
        for(int i = 0; i != chunkManagers.size(); ++ i)
        {   
            count += GetPointCloud(i, points, normals,residuals);
        }
        return count;
    }
    int Chisel::GetPointCloud(Vec3List &points, Vec3List &normals, 
        std::vector<double> &residuals, const std::set<int> &unlabeled_submap)
    {
        int count = 0;
 
        for(auto iter = unlabeled_submap.rbegin(); iter != unlabeled_submap.rend() ; ++iter)
        {
            count += GetPointCloud(*iter, points, normals, residuals);
        }
        std::cout<<"Wall points: "<<count<<std::endl;
        return count;
    }
    
    int Chisel::GetNewPoints(int submap_id, Vec3List &points, Vec3List &normals)
    {
        ChunkManager &chunkManager = chunkManagers[submap_id];
        AggregatedHash & aggregated_hash = chunkManager.aggregated_hash;
        /*
        Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
        Eigen::Vector3f t = relativeChange.translation().cast<float>()+turbulence;
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();
        Eigen::Vector3f trans;
        Eigen::Matrix3f rot;
        trans << 0,0,0;
        rot <<  1,0,0,
                0,0,1,
                0,-1,0;
        */
        int count = 0;
        for(auto iter =  aggregated_hash.begin(); iter != aggregated_hash.end(); ++iter)
        {
            if(iter->second.is_room_border() && iter->second.is_new)
            {
                //Vec3 point = iter->coord;
                iter->second.is_new = false;
                points.push_back(iter->second.global_coord);
                normals.push_back(iter->second.global_normal);
                count ++;
            }
        }
        return count;
    }
    int Chisel::GetNewPoints(Vec3List &points, Vec3List &normals)
    {
        int count = 0;
        for(int i = 0; i != chunkManagers.size(); ++ i)
        {   
            count += GetNewPoints(i, points, normals);
        }
        return count;
    }
    int Chisel::GetSemanticMeshesNotSimplified(int submap_id, int start, unsigned char *vertices, 
        int * instance_centers_ptr, const PoseSE3d &relativeChange, const PoseSE3d &cameraPose)
    {
        unsigned char *cur_vert;
        ChunkManager &chunkManager = chunkManagers[submap_id];
        MeshPtr &semantic_mesh=chunkManager.semantic_mesh;
        int v = start;

        Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
        Eigen::Vector3f t = relativeChange.translation().cast<float>();
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();
        for (int i =0; i < semantic_mesh->vertices.size();i+=3)
        {
            for(int j = 0; j < 3; j++)
            {
                const Vec3& vert = cR * (R * semantic_mesh->vertices[i+j] + t) + ct;
                const Vec3& color = semantic_mesh->colors[i+j];
                const Vec3& normal = cR * R * semantic_mesh->normals[i+j];
                int label = get_semantic_label( semantic_mesh->labels[i+j]);
                int instance_id = get_instance_label(semantic_mesh->labels[i+j]);
                cur_vert = &vertices[(22 + 4 + 4)*v];// add semantic label and instance label

                *((float *)(&cur_vert[0])) = vert(0);
                *((float *)(&cur_vert[4])) = vert(1);
                *((float *)(&cur_vert[8])) = vert(2);


                int rgb_value = (color(0) * 255);
                rgb_value = (rgb_value << 8) + int(color(1) * 255);
                rgb_value = (rgb_value << 8) + int(color(2) * 255);
                //std::cout<<"Get not simplified rgb_value: "<<rgb_value << std::endl;
                int semantic_value=label_colors[3*label]*255;
                semantic_value = (semantic_value << 8) + int(label_colors[3*label+1]*255);
                semantic_value = (semantic_value << 8) + int(label_colors[3*label+2]*255);
                //if(label == 0)
                //semantic_value = 0;

                /*
                int instance_value = 0;
                if(instance_id < 1000)
                {
                    instance_id %= 20;
                    instance_value =label_colors[3*instance_id]*255;
                    instance_value = (instance_value << 8) + int(label_colors[3*instance_id+1]*255);
                    instance_value = (instance_value << 8) + int(label_colors[3*instance_id+2]*255);
                }
                */
                int instance_value = 0;
                if(instance_id < 2)
                {
                    if(instance_id >= 0)
                    {
                        instance_value =label_colors[3*instance_id]*255;
                        instance_value = (instance_value << 8) + int(label_colors[3*instance_id+1]*255);
                        instance_value = (instance_value << 8) + int(label_colors[3*instance_id+2]*255);
                    }
                }
                else if(instance_id != -1)
                {
                    int r = instance_centers_ptr[3*instance_id] ;
                    int g = instance_centers_ptr[3*instance_id + 1];
                    int b = instance_centers_ptr[3*instance_id + 2];
                    instance_value = r;
                    instance_value = (instance_value << 8) + int(g);
                    instance_value = (instance_value << 8) + int(b);
                }

                //std::cout<<"Get not simplified instance_value: "<<instance_value << std::endl;

                *((float *)(&cur_vert[12])) = rgb_value;   
                // instance_value
                *((float *)(&cur_vert[16])) = semantic_value;
                // semantic value
                *((float *)(&cur_vert[20])) = instance_value;

                toFloat16(normal(0),&cur_vert[24]);
                toFloat16(normal(1),&cur_vert[26]);
                toFloat16(normal(2),&cur_vert[28]);
                //std::cout<<"Get not simplified normal: "<<normal(0) << std::endl;               
                v++;
            }
        }
        return v;
    }
    int Chisel::GetSemanticMeshesSimplified(int submap_id, int start, unsigned char *vertices, 
        int * instance_centers_ptr, const PoseSE3d &relativeChange, const PoseSE3d &cameraPose)
    {
        unsigned char *cur_vert;
        ChunkManager &chunkManager = chunkManagers[submap_id];
        MeshPtr &semantic_mesh=chunkManager.simplified_mesh;
        int v = start;
        Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
        Eigen::Vector3f t = relativeChange.translation().cast<float>();
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();
        for (int i =0; i < semantic_mesh->vertices.size();i+=3)
        {
            for(int j = 0; j < 3; j++)
            {
                const Vec3& vert = cR * (R * semantic_mesh->vertices[i+j] + t) + ct;
                const Vec3& color = semantic_mesh->colors[i+j];
                const Vec3& normal =  cR * R *semantic_mesh->normals[i+j];
                int label = get_semantic_label(semantic_mesh->labels[i+j]);
                int instance_id = get_instance_label(semantic_mesh->labels[i+j]);
                cur_vert = &vertices[(22 + 4 + 4)*v];// add semantic label and instance label

                *((float *)(&cur_vert[0])) = vert(0);
                *((float *)(&cur_vert[4])) = vert(1);
                *((float *)(&cur_vert[8])) = vert(2);


                int rgb_value = (color(0) * 255);
                rgb_value = (rgb_value << 8) + int(color(1) * 255);
                rgb_value = (rgb_value << 8) + int(color(2) * 255);
                //std::cout<<"Get simplified rgb_value: "<<rgb_value << std::endl;
                int semantic_value=label_colors[3*label]*255;
                semantic_value = (semantic_value << 8) + int(label_colors[3*label+1]*255);
                semantic_value = (semantic_value << 8) + int(label_colors[3*label+2]*255);


                /*
                int instance_value = 0;
                if(instance_id < 1000)
                {
                    instance_id %= 20;
                    instance_value =label_colors[3*instance_id]*255;
                    instance_value = (instance_value << 8) + int(label_colors[3*instance_id+1]*255);
                    instance_value = (instance_value << 8) + int(label_colors[3*instance_id+2]*255);
                }
                */
                int instance_value = 0;
                if(instance_id < 2)
                {
                    if(instance_id >= 0)
                    {
                        instance_value =label_colors[3*instance_id]*255;
                        instance_value = (instance_value << 8) + int(label_colors[3*instance_id+1]*255);
                        instance_value = (instance_value << 8) + int(label_colors[3*instance_id+2]*255);
                    }
                }
                else if(instance_id != -1)
                {
                    int r = instance_centers_ptr[3*instance_id] ;
                    int g = instance_centers_ptr[3*instance_id + 1];
                    int b = instance_centers_ptr[3*instance_id + 2];
                    instance_value = r;
                    instance_value = (instance_value << 8) + int(g);
                    instance_value = (instance_value << 8) + int(b);
                }

                //std::cout<<"Get simplified instance_value: "<<instance_value << std::endl;

                *((float *)(&cur_vert[12])) = rgb_value;   
                // instance_value
                *((float *)(&cur_vert[16])) = semantic_value;
                // semantic value
                *((float *)(&cur_vert[20])) = instance_value;

                toFloat16(normal(0),&cur_vert[24]);
                toFloat16(normal(1),&cur_vert[26]);
                toFloat16(normal(2),&cur_vert[28]);
                //std::cout<<"Get simplified normal_value: "<<normal(0) << std::endl;
                v++;
            }
        }
        return v;
    }
    int Chisel::GetRoomSemanticMeshes(unsigned char *vertices, int *instance_centers_ptr, const std::set<int> &room_submaps,
        PoseSE3dList &submapPoses, PoseSE3d &cameraPose)
    {
        //TICK("CHISEL_MESHING::GetSemanticMesh::GetGlobalSemanticMesh");
        int v = 0;
        for(auto i = room_submaps.begin(); i != room_submaps.end(); ++i)
        {
            int submap_id = *i;
            if( chunkManagers[*i].isSemanticSimplified())
            v = GetSemanticMeshesSimplified(submap_id,v,vertices,instance_centers_ptr,submapPoses[submap_id], cameraPose);
            else
            {

                v = GetSemanticMeshesNotSimplified(submap_id,v,vertices,instance_centers_ptr, submapPoses[submap_id], cameraPose);

            }
        }

        //TOCK("CHISEL_MESHING::GetSemanticMesh::GetGlobalSemanticMesh");
        return v;        
    }
    size_t Chisel::GetFullPCD(std::vector<unsigned char> &buffer, const PoseSE3dList &submapPoses, const PoseSE3d & cameraPose)
    {
        int byte_per_point = 30;
        size_t ptr = 0;

        for(int id = 0;id != chunkManagers.size(); ++id)
        {
            buffer.resize(chunkManagers[id].aggregated_hash.size() * byte_per_point
                 + buffer.size());
            Eigen::Matrix3f R = submapPoses[id].rotationMatrix().cast<float>();
            Eigen::Vector3f t = submapPoses[id].translation().cast<float>();
            Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
            Eigen::Vector3f ct = cameraPose.translation().cast<float>();
            for(auto iter = chunkManagers[id].aggregated_hash.begin();
                 iter != chunkManagers[id].aggregated_hash.end(); ++iter)
            {
       
                auto  position = cR * (R *iter->second.coord+t) + ct;
                auto  normal = cR * R * iter->second.normal;
                auto & color = iter->second.feature;
                auto & label = iter->second.label;
                //long instance_id = label % 10000;
                int semantic_label = get_semantic_label(label);
                unsigned char *cur_vert = &buffer[ptr];// add semantic label and instance label

                *((float *)(&cur_vert[0])) = position(0);
                *((float *)(&cur_vert[4])) = position(1);
                *((float *)(&cur_vert[8])) = position(2);


                int rgb_value = (color(0) * 255);
                rgb_value = (rgb_value << 8) + int(color(1) * 255);
                rgb_value = (rgb_value << 8) + int(color(2) * 255);

                int semantic_value=label_colors[3*semantic_label]*255;
                semantic_value = (semantic_value << 8) + int(label_colors[3*semantic_label+1]*255);
                semantic_value = (semantic_value << 8) + int(label_colors[3*semantic_label+2]*255);


                *((float *)(&cur_vert[12])) = rgb_value;   
                // instance_value
                *((float *)(&cur_vert[16])) = semantic_value;
                // semantic value
                *((float *)(&cur_vert[20])) = 0;

                toFloat16(normal(0),&cur_vert[24]);
                toFloat16(normal(1),&cur_vert[26]);
                toFloat16(normal(2),&cur_vert[28]);
                ptr += byte_per_point;
            }

                
        }
        return ptr / byte_per_point;
    }
    size_t Chisel::GetFullPCD(PointCloud &pcd, std::vector<int> &rgb_colors, 
        std::vector<int> &semantic_colors, const PoseSE3dList &submapPoses)
    {
        size_t start = 0;        
        pcd.vertices.clear();
        pcd.normals.clear();
        rgb_colors.clear();
        semantic_colors.clear();

        for(int id = 0;id != chunkManagers.size(); ++id)
        {
            size_t add_size = chunkManagers[id].aggregated_hash.size();
            pcd.vertices.resize(pcd.vertices.size() +  add_size );
            pcd.normals.resize(pcd.normals.size() + add_size );
            semantic_colors.resize(semantic_colors.size() + add_size);
            rgb_colors.resize(rgb_colors.size() + add_size);

            Eigen::Matrix3f R = submapPoses[id].rotationMatrix().cast<float>();
            Eigen::Vector3f t = submapPoses[id].translation().cast<float>();
            for(auto iter = chunkManagers[id].aggregated_hash.begin();
                 iter != chunkManagers[id].aggregated_hash.end(); ++iter)
            {
       
                pcd.vertices[start] =  (R *iter->second.coord+t) ;
                pcd.normals[start] = R * iter->second.normal;
                auto & color = iter->second.feature;
                auto & label = iter->second.label;
                //long instance_id = label % 10000;
                int semantic_label = get_semantic_label(label);



                int rgb_value = (color(0) * 255);
                rgb_value = (rgb_value << 8) + int(color(1) * 255);
                rgb_value = (rgb_value << 8) + int(color(2) * 255);

                int semantic_value=label_colors[3*semantic_label]*255;
                semantic_value = (semantic_value << 8) + int(label_colors[3*semantic_label+1]*255);
                semantic_value = (semantic_value << 8) + int(label_colors[3*semantic_label+2]*255);

                rgb_colors[start] = rgb_value;
                semantic_colors[start] = semantic_value;
                start++;
            }
        }
        return pcd.vertices.size();
    }
    size_t Chisel::GetFullPCD(std::vector<PointCloud> &pcds, std::vector<std::vector<int>> &rgb_colors, 
        std::vector<std::vector<int>> &semantic_colors)
    {
        // meshes by submap
        pcds.resize(chunkManagers.size());
        rgb_colors.resize(chunkManagers.size());
        semantic_colors.resize(chunkManagers.size());
        size_t all_size = 0;
        for(int id = 0;id != chunkManagers.size(); ++id)
        {
            auto &pcd = pcds[id];
            auto &rgb_color = rgb_colors[id];
            auto &semantic_color = semantic_colors[id];
            size_t start = 0;
            pcd.normals.clear();
            pcd.vertices.clear();
            rgb_color.clear();
            semantic_color.clear();

            size_t add_size = chunkManagers[id].aggregated_hash.size();
            pcd.vertices.resize(add_size );
            pcd.normals.resize( add_size );
            semantic_color.resize(add_size);
            rgb_color.resize(add_size);


            for(auto iter = chunkManagers[id].aggregated_hash.begin();
                 iter != chunkManagers[id].aggregated_hash.end(); ++iter)
            {
       
                pcd.vertices[start] =  iter->second.coord ;
                pcd.normals[start] = iter->second.normal;
                auto & color = iter->second.feature;
                auto & label = iter->second.label;
                //long instance_id = label % 10000;
                int semantic_label = get_semantic_label(label);



                int rgb_value = (color(0) * 255);
                rgb_value = (rgb_value << 8) + int(color(1) * 255);
                rgb_value = (rgb_value << 8) + int(color(2) * 255);

                int semantic_value=label_colors[3*semantic_label]*255;
                semantic_value = (semantic_value << 8) + int(label_colors[3*semantic_label+1]*255);
                semantic_value = (semantic_value << 8) + int(label_colors[3*semantic_label+2]*255);

                rgb_color[start] = rgb_value;
                semantic_color[start] = semantic_value;
                start++;
            }
            all_size += start;
        }
        return all_size;
    }
    int Chisel::GetAllSemanticMeshes(unsigned char *vertices, int * instance_centers_ptr ,
    PoseSE3dList & submapPoses, PoseSE3d & cameraPose)
    {
        //TICK("CHISEL_MESHING::GetSemanticMesh::GetGlobalSemanticMesh");
        std::cout<<"begin to get all meshes."<<std::endl;
#if 1
        int max_threads = 10;
        int byte_per_point = 1;
        std::vector<std::thread> threads_get_meshes;
        int next_v = 0, current_v = 0;
        if(chunkManagers.size() <= max_threads)
        {
            threads_get_meshes.resize(chunkManagers.size());

            for(int i = 0; i != chunkManagers.size(); ++i)
            {
                if(chunkManagers[i].isSemanticSimplified())
                {

                    next_v = current_v + chunkManagers[i].simplified_mesh->vertices.size();
                    std::cout<<"submap "<<i<<" (simplified)current_v: "<<current_v<<" "<<next_v<<std::endl;
                    threads_get_meshes[i] = std::thread(&Chisel::GetSemanticMeshesSimplified,  this, i, current_v, std::ref(vertices), 
                        std::ref(instance_centers_ptr), std::ref(submapPoses[i]), std::ref(cameraPose));
                }
                else
                {
                    next_v = current_v + chunkManagers[i].semantic_mesh->vertices.size();
                    std::cout<<"(not simplified)current_v: "<<current_v<<" "<<next_v<<std::endl;

                    threads_get_meshes[i] = std::thread(&Chisel::GetSemanticMeshesNotSimplified, this, i, current_v, std::ref(vertices), 
                        std::ref(instance_centers_ptr), std::ref(submapPoses[i]), std::ref(cameraPose));
                }
                current_v = next_v;
            }

            for(int i = 0; i < chunkManagers.size(); ++i)
            {
                threads_get_meshes[i].join();
                if(!chunkManagers[i].isSemanticSimplified())
                    chunkManagers[i].UnlockCurrentMesh();
            }

        }
        else
        {
            int iteration_times = chunkManagers.size()/max_threads + 1;
            threads_get_meshes.resize(max_threads);
            for(int iter = 0; iter < iteration_times; ++iter)
            {
                for(int i = 0, id = iter * max_threads; i < max_threads && id < chunkManagers.size(); ++i, ++id)
                {
                    if(chunkManagers[id].isSemanticSimplified())
                    {
                        next_v = current_v + chunkManagers[id].simplified_mesh->vertices.size();
                        std::cout<<"submap "<<i<<" (simplified)current_v: "<<current_v<<" "<<next_v<<std::endl;
                        threads_get_meshes[i] = std::thread(&Chisel::GetSemanticMeshesSimplified,  this, id, current_v, std::ref(vertices), 
                        std::ref(instance_centers_ptr), std::ref(submapPoses[id]), std::ref(cameraPose));
                    }
                    else
                    {
                        next_v = current_v + chunkManagers[id].semantic_mesh->vertices.size();
                        std::cout<<"(not simplified)current_v: "<<current_v<<" "<<next_v<<std::endl;
                        threads_get_meshes[i] = std::thread(&Chisel::GetSemanticMeshesNotSimplified,  this, id, current_v, std::ref(vertices), 
                        std::ref(instance_centers_ptr), std::ref(submapPoses[id]), std::ref(cameraPose));
                    }
                    current_v = next_v;                    
                }
                for(int i = 0, id = iter * max_threads; i < max_threads && id < chunkManagers.size(); ++i, ++id)
                {
                    threads_get_meshes[i].join();
                    if(!chunkManagers[id].isSemanticSimplified())
                        chunkManagers[id].UnlockCurrentMesh();
                }
                
            }
        }
        //TOCK("CHISEL_MESHING::GetSemanticMesh::GetGlobalSemanticMesh");

        std::cout<<"Already Get all meshes."<<std::endl;
        return current_v;        
#else
        int v = 0;
        for(int i = 0; i != chunkManagers.size(); ++i)
        {
            if( chunkManagers[i].isSemanticSimplified())
            v = GetSemanticMeshesSimplified(i,v,vertices,instance_centers_ptr,submapPoses[i], cameraPose);
            else
            {
                
                v = GetSemanticMeshesNotSimplified(i,v,vertices,instance_centers_ptr, submapPoses[i], cameraPose);
                chunkManagers[i].UnlockCurrentMesh();

            }
        }
        TOCK("CHISEL_MESHING::GetSemanticMesh::GetGlobalSemanticMesh");
       return v;
#endif


    }
    void Chisel::GetAllMeshesSimplified(int submapID)
    {
        ChunkManager &chunkManager = chunkManagers[submapID];
        for (auto & it : chunkManager.GetAllMeshes())
        {
            it.second->Compact();
            //simplify_mesh_from_CHISEL(it.second,compact_ratio);
            simplifier.simplify_mesh_from_CHISEL_compact(it.second,compact_ratio);
            //it.second->clearCompact();
        }

    }

    void Chisel::SimplifySemanticSubmapMeshes(int submapID)
    {
        clock_t start = clock();
        std::cout<<"simplify submap mesh: "<<submapID <<std::endl;
        ChunkManager &chunkManager = chunkManagers[submapID];        
        //chunkManager.is_active = false;
        //chunkManager.clear_hash_aggregated_map();
        //chunkManager.clearMeshes();
        chunkManager.prepareSimplifiedMesh();
        MeshPtr &fullMesh = chunkManager.simplified_mesh;
        (*fullMesh) = (*chunkManager.semantic_mesh);
        int v = 0;
        fullMesh->Compact();
        clock_t compact_done = clock();
        std::cout<<"simplify mesh! Old vertices number: "<<fullMesh->vertices.size()<<" traingle number: "<<fullMesh->triangles.size()<<" "<<submapID<<std::endl;
        
        // compact_r = compact_ratio;
        int triangle_target_num = target_num/3;
        if(target_num > max_vertices) 
            triangle_target_num = max_vertices/3;
        std::cout<<"target_num: "<<triangle_target_num << std::endl;
        float compact_ratio = (triangle_target_num+0.0)/ fullMesh->triangles.size();
        simplifier.simplify_mesh_from_CHISEL_compact(fullMesh,compact_ratio);
        //QuadricSimplification(fullMesh, triangle_target_num);
        //fullMesh->Decompact();
        //ClusteringSimplification(fullMesh, 0.05);
        //fullMesh->Decompact();
        std::cout<<"simplify done! Now the vertices number: "<<fullMesh->vertices.size()*3<<" traingle number: "<<fullMesh->triangles.size()<<" "<<submapID<<std::endl;
        
        clock_t simplify_done = clock();
        /*
        std::cout<<"all: "<<(double)(simplify_done - start)/CLOCKS_PER_SEC<<"s"<<std::endl;
        std::cout<<"connection: "<<(double)(compact_done - start)/CLOCKS_PER_SEC<<"s"<<std::endl;
        std::cout<<"simplify: "<<(double)(simplify_done - compact_done)/CLOCKS_PER_SEC<<"s"<<std::endl;
        */
        chunkManager.LockCurrentMesh();
        std::cout<<"start to clear non-simplifed meshes."<<submapID<<std::endl;
        chunkManager.is_simplified = true;
        chunkManager.clearSemanticMeshes();
        chunkManager.clearMeshes();

        std::cout<<"Finish clearing non-simplified meshes."<<submapID<<std::endl;
        chunkManager.UnlockCurrentMesh();        
    }
    void Chisel::GetSubmapMeshSimplified(int submapID)
    {
        clock_t start = clock();
        ChunkManager &chunkManager = chunkManagers[submapID];        
        chunkManager.prepareSimplifiedMesh();
        MeshPtr &fullMesh = chunkManager.simplified_mesh;
        //chunkManager.LockCurrentMesh();
        (*fullMesh) = (*chunkManager.semantic_mesh);
        //chunkManager.UnlockCurrentMesh();
        /*
        int v = 0;

        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }
        }*/
        fullMesh->Compact();
        clock_t compact_done = clock();
        std::cout<<"simplify mesh! Old vertices number: "<<fullMesh->vertices.size()<<" traingle number: "<<fullMesh->triangles.size()<<" "<<submapID<<std::endl;
        
        float compact_r = compact_ratio;
        if(compact_r > 1)
            compact_r = (target_num/3.0 + 0.0)/fullMesh->triangles.size();
        else if(compact_r * fullMesh->triangles.size() > (max_vertices/3))
            compact_r = (max_vertices/3.0 + 0.0)/fullMesh->triangles.size();
        
        std::cout<<"final compact ratio: "<<compact_r <<std::endl;
        simplifier.simplify_mesh_from_CHISEL_compact(fullMesh,compact_r);
        std::cout<<"simplify done! Now the vertices number: "<<fullMesh->vertices.size()<<" traingle number: "<<fullMesh->triangles.size()<<" "<<submapID<<std::endl;
        clock_t simplify_done = clock();

        std::cout<<"all: "<<(double)(simplify_done - start)/CLOCKS_PER_SEC<<"s"<<std::endl;
        std::cout<<"connection: "<<(double)(compact_done - start)/CLOCKS_PER_SEC<<"s"<<std::endl;
        std::cout<<"simplify: "<<(double)(simplify_done - compact_done)/CLOCKS_PER_SEC<<"s"<<std::endl;

        chunkManager.LockCurrentMesh();
        chunkManager.clearMeshes();
        chunkManager.UnlockCurrentMesh();
    }
    void Chisel::GetSubmapMeshSimplified()
    {
            ChunkManager &chunkManager = globalChunkManager;        
            chunkManager.prepareSimplifiedMesh();
            MeshPtr &fullMesh = chunkManager.simplified_mesh;
            int v = 0;
            for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }
        }

        fullMesh->Compact();
        std::cout<<"Global simplify mesh! Old vertices number: "<<fullMesh->vertices.size()<<std::endl;
        simplifier.simplify_mesh_from_CHISEL_compact(fullMesh,0.5);
        std::cout<<"Global simplify done! Now the vertices number: "<<fullMesh->vertices.size()<<std::endl;
        chunkManager.clearMeshes();
    }
    int Chisel::GetFullMeshes(unsigned char *vertices)
    {

        ChunkManager &chunkManager = globalChunkManager;
        size_t v = 0;
        unsigned char *cur_vert;

        for (auto & it : chunkManager.GetAllMeshes())
        {
            //it.second->Compact();
                  //
                  //getchar();
            simplifier.simplify_mesh_from_CHISEL_compact(it.second,1.0);
            //simplify_mesh_from_CHISEL(it.second);
            if(it.second->vertices.size() != it.second->colors.size() || it.second->vertices.size() != it.second->normals.size())
            {
                std::cout << "mesh vertex error!" <<std::endl;
                while(1)
                {
                } 
            }
            for (int i =0; i < it.second->vertices.size();i+=3)
            {

                for(int j = 0; j < 3; j++)
                {
                    /* const Vec3& vert = it.second->vertices[i+j];
                    const Vec3& color = it.second->colors[i+j];
                    const Vec3& normal = it.second->normals[i+j];
		            cur_vert = &vertices[16*v];
                    toFloat16(vert(0),&cur_vert[0]);
                    toFloat16(vert(1),&cur_vert[2]);
                    toFloat16(vert(2),&cur_vert[4]);
                    
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[6])) = rgb_value;
                    toFloat16(normal(0),&cur_vert[10]);
                    toFloat16(normal(1),&cur_vert[12]);
                    toFloat16(normal(2),&cur_vert[14]);
                    v++;
                    */
                    const Vec3& vert = it.second->vertices[i+j];
                     Vec3 &color = it.second->colors[i+j];
                    const Vec3& normal = it.second->normals[i+j];
		            cur_vert = &vertices[22*v];
                    //toFloat16(vert(0),&cur_vert[0]);
                    //toFloat16(vert(1),&cur_vert[2]);
                    //toFloat16(vert(2),&cur_vert[4]);
                    *((float *)(&cur_vert[0])) = vert(0);
                    *((float *)(&cur_vert[4])) = vert(1);
                    *((float *)(&cur_vert[8])) = vert(2);
                    if(!showSubmapStructure)
                    {
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[12])) = rgb_value;
                    }
                    else
                    {
                    //color(submapColor%3)=1;
                    color = color * (submapColor/3);
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[12])) = rgb_value;                        
                    }
                    toFloat16(normal(0),&cur_vert[16]);
                    toFloat16(normal(1),&cur_vert[18]);
                    toFloat16(normal(2),&cur_vert[20]);
                    v++;
                }

            }
        }
        return v;
    }
int Chisel::GetFullMeshes(unsigned char *vertices, MeshMap &all,size_t v,PoseSE3d relativeChange,PoseSE3d cameraPose,Eigen::Vector3f turbulence)
    {

        //size_t v = 0;
        unsigned char *cur_vert;
        //std::cout<<"relativeChange: "<<relativeChange.log().transpose()<<std::endl;
        //std::cout<<"cameraPose: "<<cameraPose.log().transpose()<<std::endl;
        for (auto & it : all)
        {
            //it.second->Compact();
            //compact_to_no(it.second);
            //std::cout << "Transmit!"<<it.second->chunkID <<std::endl;
            //simplify_mesh_from_CHISEL_compact(it.second,1.0);
            //std::cout <<"mesh->vertices: "<<it.second->vertices.size()<<" mesh->colors: "<<it.second->colors.size() <<" mesh->normals: "<<it.second->normals.size()<<std::endl;
            if(it.second->compact_vertices.size() != it.second->compact_colors.size() || it.second->compact_vertices.size() != it.second->compact_normals.size())
            {
                
                std::cout << "mesh vertex error!" <<std::endl;
                std::cout <<"mesh->vertices: "<<it.second->vertices.size()<<" mesh->colors: "<<it.second->colors.size() <<" mesh->normals: "<<it.second->normals.size()<<std::endl;

                while(1)
                {
                } 
            }

        Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
        Eigen::Vector3f t = relativeChange.translation().cast<float>()+turbulence;
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();
        for (int i =0; i < it.second->vertices.size();i+=3)
            {

                for(int j = 0; j < 3; j++)
                {
                    const Vec3& vert = cR*(R*it.second->vertices[i+j]+t)+ct;
                     Vec3 &color = it.second->colors[i+j];
                    const Vec3& normal = cR*(R*it.second->normals[i+j]);
		            cur_vert = &vertices[22*v];
                    //toFloat16(vert(0),&cur_vert[0]);
                    //toFloat16(vert(1),&cur_vert[2]);
                    //toFloat16(vert(2),&cur_vert[4]);
                    *((float *)(&cur_vert[0])) = vert(0);
                    *((float *)(&cur_vert[4])) = vert(1);
                    //*((float *)(&cur_vert[0])) = -vert(1);
                    //*((float *)(&cur_vert[4])) = vert(0);
                    *((float *)(&cur_vert[8])) = vert(2);
                    if(!showSubmapStructure)
                    {
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[12])) = rgb_value;
                    }
                    else
                    {
                    color(submapColor%3)=1;
                    //color = color * (submapColor/3);
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[12])) = rgb_value;                        
                    }
                    toFloat16(normal(0),&cur_vert[16]);
                    toFloat16(normal(1),&cur_vert[18]);
                    toFloat16(normal(2),&cur_vert[20]);
                    v++;
                }

            }
        }

        return v;
    }

    int Chisel::GetFullMeshes(unsigned char *vertices, int mainSubmapID, int referenceSubmapID,PoseSE3dList & submapPoses, PoseSE3d & cameraPose)
    {
        int max_id = chunkManagers.size()-1;
        std::cout<<"mainSubmapID: "<<mainSubmapID<<"referenceSubmapID: "<<referenceSubmapID<<std::endl;
        TICK("CHISEL_MESHING::GetFullMeshes::2::GetGlobalMeshes");
        int v = 0;
#if 1
        v = GetFullMeshes(vertices,chunkManagers[mainSubmapID].allMeshes/*globalChunkManager.allMeshes*/,v,submapPoses[mainSubmapID],cameraPose);
        std::cout<<"mainSubmap vertices number: "<<v<<std::endl;
#endif
#if 1
        PoseSE3d pose;
        if(referenceSubmapID >= 0)
        v = GetFullMeshes(vertices,chunkManagers[referenceSubmapID].allMeshes,v,submapPoses[referenceSubmapID],cameraPose);
        std::cout<<"referenceSubmapID vertices number: "<<v<<std::endl;
        //if(referenceSubmapID >= 1)
        //v = GetFullMeshes(vertices,chunkManagers[referenceSubmapID-1].allMeshes,v,submapPoses[referenceSubmapID],cameraPose);

#endif
        TOCK("CHISEL_MESHING::GetFullMeshes::2::GetGlobalMeshes");

#if 1
        for(int i = 0; i<=max_id; ++i)
        {
            pose = submapPoses[i];
            
            if(i!=mainSubmapID && i!=referenceSubmapID)
            {
                std::cout<<"submap "<<i<<std::endl;
                int isSimplified = chunkManagers[i].isSimplified();
                if(isSimplified == 1)
                {
                std::cout<<"using simplified mesh! vertices number:"<<chunkManagers[i].simplified_mesh->vertices.size()<<std::endl;
                /*if(noChanges)
                v=GetFullMeshes(chunkManagers[i].simplified_mesh,vertices,v);
                else*/
                v=GetFullMeshes(chunkManagers[i].simplified_mesh,vertices,v,pose,cameraPose);
                //std::cout<<"vertices number: "<<v<<std::endl;
                }
                else if(isSimplified == 2)
                {
                std::cout<<"Mesh simplification is not over! using non-simplified mesh! "<<std::endl;
                TICK("CHISEL_MESHING::GetFullMeshes::3::GetNoSimplifiedMesh");
                v = GetFullMeshes(vertices,chunkManagers[i].allMeshes,v,pose,cameraPose);
                TOCK("CHISEL_MESHING::GetFullMeshes::3::GetNoSimplifiedMesh");         
                //std::cout<<"vertices number: "<<v<<std::endl;   
                
                }
                else 
                {
                //Just ignore it
               std::cout<<"Empty Mesh! Generating Mesh... "<<std::endl;
               /* std::unique_lock <std::mutex> lock(generatingMesh);
                cv_generatingMesh.wait(lock);
                //generatingMesh.clear();
                std::cout<<"Generating done! using non-simplified mesh! "<<std::endl;
                v = GetFullMeshes(vertices,chunkManagers[i].allMeshes,v);         
                std::cout<<"vertices number: "<<v<<std::endl;  
                */

                }
            }
        }
#endif
        return v;
    }
    bool Chisel::SaveLabels(const std::string &filename, std::vector<long> &labels)
    {
        std::vector<int> semantic_labels;
        std::vector<int> instance_labels;
        for(int i = 0; i != labels.size(); ++i)
        {   int semantic_label = get_semantic_label(labels[i]);
            semantic_labels.push_back(semantic_label);
            int instance_label = get_instance_label(labels[i]);
            instance_labels.push_back(instance_label);
        }
        //use cnpy to write the data to file.
        cnpy::npy_save(filename+"_semantic.npy", &semantic_labels[0],{semantic_labels.size()}, "w");
        cnpy::npy_save(filename+"_instance.npy", &instance_labels[0],{instance_labels.size()}, "w");
        return true;
    }
    bool Chisel::SaveMeshesToPLYBySubmap(int submapID,const std::string &filename)
    {
        
        chisel::MeshPtr fullMesh;
        ChunkManager &chunkManager = chunkManagers[submapID]; 
        if(!chunkManager.semantic_mesh) return true;
        size_t v = 0;
        /*
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }
        }
        */
        if(chunkManager.isSemanticSimplified())
         fullMesh = chunkManager.simplified_mesh;
        else
         fullMesh = chunkManager.semantic_mesh;
        fullMesh->Compact();
        //simplify_mesh_from_CHISEL_compact(fullMes);
        //fullMesh->Compact();
        std::cout<<"save mesh to "+filename <<std::endl;
        bool success = SaveCompactMeshPLYASCII(filename, fullMesh);
        bool label_success = SaveLabels(filename, fullMesh->compact_labels);

        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;
    }

    bool Chisel::SaveMeshesToPLYBySubmapDense(int submapID, const PoseSE3d &submap_pose,const std::string &filename, const std::string &map_filename)
    {
        
        chisel::MeshPtr fullMesh;
        ChunkManager &chunkManager = chunkManagers[submapID]; 
        if(!chunkManager.semantic_mesh) return true;
        fullMesh = chunkManager.semantic_mesh;
        fullMesh->Clear();
        size_t v = 0;
        /*
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }
        }
        */
        Eigen::Matrix3f R = submap_pose.rotationMatrix().cast<float>();
        Eigen::Vector3f t = submap_pose.translation().cast<float>();
        std::string map_path = map_filename+"m"+std::to_string(submapID)+".map";
        if(!chunkManager.isValid) 
        {

            std::cout<<"load submap from "<<map_path<<std::endl;
            chunkManager.loadSubmapB(map_path);
        }
        
        chunkManager.RecomputeMeshesWithoutCheck(_camera);
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (int i = 0; i < it.second->vertices.size(); ++i) 
            {
                /* we need to merge the scattered mesh into one mesh.*/
                fullMesh->vertices.push_back( R * it.second->vertices[i] + t);
                fullMesh->colors.push_back(it.second->colors[i]);
                fullMesh->normals.push_back(R * it.second->normals[i]);
            }
        }
        fullMesh->Compact();
        //ClusteringSimplification(fullMesh,0.00625);
        //simplify_mesh_from_CHISEL_compact(fullMes);
        //fullMesh->Compact();
        std::cout<<"save mesh to "+filename <<std::endl;
        bool success = SaveCompactMeshPLYASCII(filename, fullMesh);
        //bool label_success = SaveLabels(filename, fullMesh->compact_labels);
        chunkManager.saveSubmapB(map_path);
        chunkManager.clearChunks();
        chunkManager.clearMeshes();
        chunkManager.clearSemanticMeshes();
        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;
    }
    PcdPtr Chisel::ExtractRoomModel(const std::vector<int> &room, const PoseSE3dList &submapPoses)
    {
        chisel::PcdPtr fullPcd(new chisel::PointCloud());

        for(int i = 0; i != room.size(); ++i)
        {
            int index = room[i];
            std::cout<<"extracting submap "<<index<<" ..."<<std::endl;
            int isSimplified = chunkManagers[index].isSemanticSimplified();
            chisel::MeshPtr mesh_ptr;
            if(isSimplified)
            mesh_ptr = chunkManagers[index].simplified_mesh;
            else mesh_ptr = chunkManagers[index].semantic_mesh;
            auto &vertices = mesh_ptr->vertices;
            Eigen::Matrix3f R = submapPoses[index].rotationMatrix().cast<float>();
            Eigen::Vector3f t = submapPoses[index].translation().cast<float>();
            for(int j = 0; j != vertices.size(); ++j)
            {
                fullPcd->vertices.push_back(R * vertices[j]+ t);
            }
            //fullPcd->vertices.insert(mesh_ptr->vertices.begin(), mesh_ptr->vertices.end(), fullPcd->vertices.end());
            fullPcd->normals.insert(fullPcd->normals.end(), mesh_ptr->normals.begin(), mesh_ptr->normals.end());
            //fullPcd->colors.insert(fullPcd->colors.end(), mesh_ptr->colors.begin(), mesh_ptr->colors.end() );
            fullPcd->labels.insert(fullPcd->labels.end(), mesh_ptr->labels.begin(), mesh_ptr->labels.end());
        }
        std::cout<<"Finish Extracting Room Model!"<<std::endl;
        //SaveSemanticPointCloudPLYASCII("room_0.ply", fullPcd);
        return fullPcd;
    }
    PcdPtr Chisel::ExtractRoomModelSparse(const std::vector<int> &room)
    {
        chisel::PcdPtr fullPcd(new chisel::PointCloud());

        for(int i = 0; i != room.size(); ++i)
        {
            int index = room[i];
            std::cout<<"extracting submap "<<index<<" ..."<<std::endl;
            auto & aggregated_hash = chunkManagers[index].aggregated_hash;
            for(auto iter =  aggregated_hash.begin(); iter != aggregated_hash.end(); ++iter)
            {
                fullPcd->vertices.push_back(iter->second.global_coord);
                fullPcd->labels.push_back(iter->second.label);
                fullPcd->normals.push_back(iter->second.global_normal);
                fullPcd->colors.push_back(iter->second.feature);
            }
        }
        std::cout<<"Finish Extracting Room Model!"<<std::endl;
        //SaveSemanticPointCloudPLYASCII("sparse_room_0.ply", fullPcd);
        return fullPcd;        
    }
    bool Chisel::SaveRoomToPLY(const std::vector<int> &room, const PoseSE3dList &submapPoses, const std::string &filename)
    {
        auto pcdptr = ExtractRoomModel(room, submapPoses);
        bool success;
#if 0
        success = SavePointCloudPLYASCII(filename, pcdptr);
else
        success = SaveSemanticPointCloudPLYASCII(filename, pcdptr);
#endif

        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;
    }
    MeshPtr Chisel::GetMeshesGlobal()
    {
        chisel::MeshPtr fullMesh(new chisel::Mesh());
        ChunkManager &chunkManager = globalChunkManager; 
        size_t v = 0;
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }
        }
        //std::cout<<v<<std::endl;
        //if(v == 0) fullMesh = chunkManager.simplified_mesh;
        
        //simplify_mesh_from_CHISEL_compact(fullMes);
        fullMesh->CompactWithoutBounding();
        ClusteringSimplification(fullMesh, 0.00625);
        //std::cout<<"save mesh to "+filename <<std::endl;
        return fullMesh;

    }
    MeshPtr Chisel::GetMeshesGlobalSemantic()
    {
        
        ChunkManager &chunkManager = globalChunkManager; 
        chisel::MeshPtr fullMesh = chunkManager.semantic_mesh;
        
        fullMesh->CompactWithoutBounding();
        ClusteringSimplification(fullMesh, 0.00625);
        //std::cout<<"save mesh to "+filename <<std::endl;
        return fullMesh;

    }
    bool Chisel::SaveMeshesToPLYGlobal(const std::string &filename, const PoseSE3d &camera_pose)
    {
        chisel::MeshPtr fullMesh(new chisel::Mesh());
        ChunkManager &chunkManager = globalChunkManager; 
        size_t v = 0;
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }
        }

        std::cout<<"save mesh to "+filename <<std::endl;
        //std::cout<<v<<std::endl;
        //if(v == 0) fullMesh = chunkManager.simplified_mesh;
        fullMesh->Compact();
        fullMesh->transform(camera_pose);
        //simplify_mesh_from_CHISEL_compact(fullMes);
        //fullMesh->Compact();
        //std::cout<<"save mesh to "+filename <<std::endl;
        bool success = SaveCompactMeshPLYASCII(filename, fullMesh);

        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;
    }
    bool Chisel::SaveAllMeshesToPLYBySubmap(const std::string &filename)
    {
        bool success = true;
        for(int i =0;i!=chunkManagers.size();++i)
        {
            std::cout<<"save submap_"<<i<<"'s meshes..."<<std::endl;
            if(!SaveMeshesToPLYBySubmap(i,filename+"_"+std::to_string(i)+".ply"))
               success = false;
        }
        return success;
    }
    /*
    bool Chisel::SaveAllMeshesToPLYBySubmapDense(const std::string &filename, const std::string &map_path)
    {
        bool success = true;
        for(int i =0;i!=chunkManagers.size();++i)
        {
            std::cout<<"save submap_"<<i<<"'s meshes..."<<std::endl;
            if(!SaveMeshesToPLYBySubmapDense(i,filename+"_dense_"+std::to_string(i)+".ply", map_path))
               success = false;
        }
        return success;
    }*/
    bool Chisel::SaveAllMeshesToPLY(const std::string& filename)
    {
        printf("Saving all meshes to PLY file...\n");

        chisel::MeshPtr fullMesh(new chisel::Mesh());
        ChunkManager &chunkManager = globalChunkManager; 
        size_t v = 0;
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }

            for(const long&label: it.second->labels)
            {
                fullMesh->labels.push_back(label);
            }
        }

        printf("Full mesh has %lu verts\n", v);
        fullMesh->Compact();
        simplifier.simplify_mesh_from_CHISEL_compact(fullMesh,compact_ratio);
        fullMesh->Compact();
        bool success = SaveCompactMeshPLYASCII(filename, fullMesh);

        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;
    }
    int Chisel::SaveAllMeshesToPLY(MeshMap &allMeshes,size_t v,MeshPtr fullMesh,PoseSE3d &pose, PoseSE3d &cameraPose)
    {

        Eigen::Matrix3f R = pose.rotationMatrix().cast<float>();
        Eigen::Vector3f t = pose.translation().cast<float>();
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();
        for (const std::pair<ChunkID, MeshPtr>& it : allMeshes)
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(cR * (R*vert+t) + ct);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(cR * R*normal);
            }
            for(const long&label: it.second->labels)
            {
                fullMesh->labels.push_back(label);
            }
        }
        return v;
    }
    int Chisel::SaveAllMeshesToPLY(MeshPtr mesh,size_t v,MeshPtr fullMesh,PoseSE3d &pose, PoseSE3d & cameraPose)
    {
        Eigen::Matrix3f R = pose.rotationMatrix().cast<float>();
        Eigen::Vector3f t = pose.translation().cast<float>();
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();
        if(mesh->vertices.size() != mesh->colors.size() || mesh->vertices.size() != mesh->normals.size())
        {
            std::cout <<"mesh vertex error!" <<std::endl;
            while(1)
            {
            } 
        }
                //std::cout<<"number of vertices: "<<it.second->vertices.size()<<std::endl;
                // getchar();
        for(size_t i =0;i!=mesh->vertices.size();++i)
        {
            fullMesh->vertices.push_back(cR*(R*mesh->vertices[i] + t) + ct);
            fullMesh->colors.push_back(mesh->colors[i]);
            fullMesh->normals.push_back(cR * R*mesh->normals[i]);
            fullMesh->labels.push_back(mesh->labels[i]);
        }
        fullMesh->indices.insert(fullMesh->indices.end(),mesh->indices.begin(),mesh->indices.end());


        return fullMesh->vertices.size();     
    }
    bool Chisel::SaveRoomMeshesToPLY(const std::string &filename,std::vector<int> room_submaps,PoseSE3dList &submapPoses, PoseSE3d &cameraPose)
    {
       chisel::MeshPtr fullMesh(new chisel::Mesh());


        //int max_id = *(activeSubmapIDs.rbegin());
        //GetSubmapMeshSimplified();
        //int v=GetFullMeshes(globalChunkManager.simplified_mesh,vertices,0);
        int v = 0;//SaveAllMeshesToPLY(globalChunkManager.allMeshes,0,fullMesh,submapPoses[*activeSubmapIDs.rbegin()]);
        //std::cout<<"vertices number: "<<v<<std::endl;
        for(auto iter = room_submaps.begin(); iter != room_submaps.end(); ++iter)
        {
            int i = *iter;
            std::cout<<"submap "<<i<<" ";
            bool isSimplified = chunkManagers[i].isSemanticSimplified();
            if(isSimplified == 1)
            {
            std::cout<<"using simplified mesh! vertices number:"<<chunkManagers[i].simplified_mesh->vertices.size()<<std::endl;
            v=SaveAllMeshesToPLY(chunkManagers[i].simplified_mesh,v,fullMesh,submapPoses[i], cameraPose);
            std::cout<<"vertices number: "<<v<<std::endl;
            }
            else
            {
            std::cout<<"Mesh simplification is not over! using non-simplified mesh! "<<std::endl;
            TICK("CHISEL_MESHING::GetFullMeshes::3::GetNoSimplifiedMesh");
            v = SaveAllMeshesToPLY(chunkManagers[i].semantic_mesh,v,fullMesh,submapPoses[i], cameraPose);
            TOCK("CHISEL_MESHING::GetFullMeshes::3::GetNoSimplifiedMesh");         
            std::cout<<"vertices number: "<<v<<std::endl;   
            

            }
        }
        printf("Full mesh has %lu verts\n", v);
        fullMesh->Compact();
        //simplify_mesh_from_CHISEL_compact(fullMesh,1);
        //fullMesh->Compact();
        bool success = SaveCompactMeshPLYASCII(filename, fullMesh);

        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;        
    } 
    bool Chisel::SaveAllMeshesToPLY(const std::string &filename,std::set<int> activeSubmapIDs,PoseSE3dList &submapPoses, PoseSE3d &cameraPose)
    {
       chisel::MeshPtr fullMesh(new chisel::Mesh());


        int max_id = *(activeSubmapIDs.rbegin());
        //GetSubmapMeshSimplified();
        //int v=GetFullMeshes(globalChunkManager.simplified_mesh,vertices,0);
        int v = 0;//SaveAllMeshesToPLY(globalChunkManager.allMeshes,0,fullMesh,submapPoses[*activeSubmapIDs.rbegin()]);
        std::cout<<"vertices number: "<<v<<std::endl;
        for(int i = 0; i<=max_id; ++i)
        {
            std::cout<<"submap "<<i<<" ";

            int isSimplified = chunkManagers[i].isSemanticSimplified();
            if(isSimplified)
            {
            std::cout<<"using simplified mesh! vertices number:"<<chunkManagers[i].simplified_mesh->vertices.size()<<std::endl;
            v=SaveAllMeshesToPLY(chunkManagers[i].simplified_mesh,v,fullMesh,submapPoses[i], cameraPose);
            std::cout<<"vertices number: "<<v<<std::endl;
            }
            else 
            {
            std::cout<<"Mesh simplification is not over! using non-simplified mesh! "<<std::endl;
            TICK("CHISEL_MESHING::GetFullMeshes::3::GetNoSimplifiedMesh");
            chunkManagers[i].prepareSemanticMesh();
            v = SaveAllMeshesToPLY(chunkManagers[i].semantic_mesh,v,fullMesh,submapPoses[i],cameraPose);
            TOCK("CHISEL_MESHING::GetFullMeshes::3::GetNoSimplifiedMesh");         
            std::cout<<"vertices number: "<<v<<std::endl;   

            }
        }
        printf("Full mesh has %lu verts\n", v);
        fullMesh->Compact();
        //simplify_mesh_from_CHISEL_compact(fullMesh,1);
        //fullMesh->Compact();
        bool success = SaveCompactMeshPLYASCII(filename, fullMesh);
        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;
    }
    bool Chisel::SaveAllMeshesToPLYSemantic(const std::string &filename,std::set<int> activeSubmapIDs,PoseSE3dList &submapPoses, PoseSE3d &cameraPose)
    {
       chisel::MeshPtr fullMesh(new chisel::Mesh());


        int max_id = *(activeSubmapIDs.rbegin());
        //GetSubmapMeshSimplified();
        //int v=GetFullMeshes(globalChunkManager.simplified_mesh,vertices,0);
        int v = 0;//SaveAllMeshesToPLY(globalChunkManager.allMeshes,0,fullMesh,submapPoses[*activeSubmapIDs.rbegin()]);
        std::cout<<"vertices number: "<<v<<std::endl;
        for(int i = 0; i<=max_id; ++i)
        {
            std::cout<<"submap "<<i<<" ";

                int isSimplified = chunkManagers[i].isSemanticSimplified();
                if(isSimplified)
                {
                std::cout<<"using simplified mesh! vertices number:"<<chunkManagers[i].simplified_mesh->vertices.size()<<std::endl;
                v=SaveAllMeshesToPLY(chunkManagers[i].simplified_mesh,v,fullMesh,submapPoses[i], cameraPose);
                std::cout<<"vertices number: "<<v<<std::endl;
                }
                else
                {
                std::cout<<"Mesh simplification is not over! using non-simplified mesh! "<<std::endl;
                TICK("CHISEL_MESHING::GetFullMeshes::3::GetNoSimplifiedMesh");
                chunkManagers[i].prepareSemanticMesh();
                v = SaveAllMeshesToPLY(chunkManagers[i].semantic_mesh,v,fullMesh,submapPoses[i],cameraPose);
                TOCK("CHISEL_MESHING::GetFullMeshes::3::GetNoSimplifiedMesh");         
                std::cout<<"vertices number: "<<v<<std::endl;   

                }
        }
        printf("Full mesh has %lu verts\n", v);
        fullMesh->Compact();
        //simplify_mesh_from_CHISEL_compact(fullMesh,1);
        //fullMesh->Compact();
        bool success = SaveSemanticMeshPLYASCII(filename, fullMesh);

        if (!success)
        {
            printf("Saving failed!\n");
        }

        return success;
    }
    MeshPtr Chisel::GetFullMeshes()
    {

        chisel::MeshPtr fullMesh(new chisel::Mesh());
        ChunkManager &chunkManager = globalChunkManager; 
        size_t v = 0;
        
        for (const std::pair<ChunkID, MeshPtr>& it : chunkManager.GetAllMeshes())
        {
            for (const Vec3& vert : it.second->vertices)
            {
                fullMesh->vertices.push_back(vert);
                fullMesh->indices.push_back(v);
                v++;
            }

            for (const Vec3& color : it.second->colors)
            {
                fullMesh->colors.push_back(color);
            }

            for (const Vec3& normal : it.second->normals)
            {
                fullMesh->normals.push_back(normal);
            }
        }
        return fullMesh; 
    }
    int Chisel::GetFullMeshes(MeshPtr mesh,unsigned char *vertices,int v,PoseSE3d relativeChange,PoseSE3d cameraPose)
    {
            unsigned char *cur_vert;
            //mesh->Compact();
            
            //simplify_mesh_from_CHISEL_compact(mesh,compact_ratio);
            //std::cout<<"submap vertices number: "<<mesh->vertices.size()<<std::endl;
            //if(relativeChange.log() != PoseSE3d.log())
        TICK("CHISEL_MESHING::GetFullMeshes::1::TransformMeshes");
            //mesh->transform(relativeChange);
        TOCK("CHISEL_MESHING::GetFullMeshes::1::TransformMeshes");
            //mesh->parallel_transform(relativeChange);
            
            if(mesh->vertices.size() != mesh->colors.size() || mesh->vertices.size() != mesh->normals.size())
            {
                std::cout << "mesh vertex error!" <<std::endl;
                while(1)
                {
                std::cout <<"mesh->vertices: "<<mesh->vertices.size()<<" mesh->colors: "<<mesh->colors.size() <<" mesh->normals: "<<mesh->normals.size()<<std::endl;
                } 
            }
                  //std::cout<<"number of vertices: "<<it.second->vertices.size()<<std::endl;
                 // getchar();
        Eigen::Matrix3f R = relativeChange.rotationMatrix().cast<float>();
        Eigen::Vector3f t = relativeChange.translation().cast<float>();            
        Eigen::Matrix3f cR = cameraPose.rotationMatrix().cast<float>();
        Eigen::Vector3f ct = cameraPose.translation().cast<float>();                   
            for (int i =0; i < mesh->vertices.size();i+=3)
            {

                for(int j = 0; j < 3; j++)
                {
                    const Vec3& vert = cR*(R*mesh->vertices[i+j] +t)+ct;
                    const Vec3& color = mesh->colors[i+j];
                    const Vec3& normal = cR*(R*mesh->normals[i+j]);
		            cur_vert = &vertices[22*v];

                    *((float *)(&cur_vert[0])) = vert(0);
                    *((float *)(&cur_vert[4])) = vert(1);
                    //*((float *)(&cur_vert[0])) = -vert(1);
                    //*((float *)(&cur_vert[4])) = vert(0);
                    *((float *)(&cur_vert[8])) = vert(2);
                    
                    int rgb_value = (color(0) * 255);
                    rgb_value = (rgb_value << 8) + int(color(1) * 255);
                    rgb_value = (rgb_value << 8) + int(color(2) * 255);
                    *((float *)(&cur_vert[12])) = rgb_value;
                    
                    toFloat16(normal(0),&cur_vert[16]);
                    toFloat16(normal(1),&cur_vert[18]);
                    toFloat16(normal(2),&cur_vert[20]);
                    v++;
                }

            } 
        return v;      
    }
    int Chisel::GetChunkCount(int submapID)
    {
        return chunkManagers[submapID].GetChunkCount();
    }
    int Chisel::GetSubmapSize()
    {
        return chunkManagers.size();
    }
    void Chisel::SaveAllSubmap(const std::string &filename)
    {
        for(int i = 0; i != chunkManagers.size(); ++i)
        {
            saveSubmap(i,filename);
        }
    }
    void Chisel::FinalIntegrationToGlobalChunkManager(ProjectionIntegrator& integrator, 
        PoseSE3dList &relativeChanges, const std::string &filename)
    {
        for(int i = 0; i != chunkManagers.size(); ++i)
        {
            saveSubmap(i,filename);
        }
        globalChunkManager.Reset();
        PoseSE3d identity;
        for(int i = 0; i != chunkManagers.size(); ++i)
        {
            std::cout<<"Integrate "<<i<<"th submap..."<<std::endl;
            Transform relative_transform;
            //std::cout<<relativeChanges[i].inverse().matrix().cast<float>()<<std::endl;
            relative_transform = relativeChanges[i].inverse().matrix().cast<float>();
            bufferIntegratorSIMDCentroids(integrator, relative_transform);
            
            loadSubmap(i,filename);
            auto &processing_chunks = chunkManagers[i].chunks;
            ChunkMap after_transformed;
#if 1
            std::cout<<"Transform submap "<<i<<"..."<<std::endl;
            for(auto iter = processing_chunks.begin(); iter != processing_chunks.end(); ++iter)
            {
                integrator.AddTransformed8Neighbor(after_transformed, relative_transform, iter->second.get());
            }
            
            std::cout<<"Updating Voxels..., chunk: "<<after_transformed.size()<<std::endl;
            bufferIntegratorSIMDCentroids(integrator, relative_transform.inverse());
            for(auto iter = after_transformed.begin(); iter != after_transformed.end(); ++iter)
            {
                integrator.VoxelUpdate8Neighbor(iter->second.get(), relative_transform.inverse(), processing_chunks);
            }
            globalChunkManager.Merge(after_transformed);

#elif 1
            
            for(auto iter = processing_chunks.begin(); iter != processing_chunks.end(); ++iter)
            {
                integrator.voxelUpdateSIMDReverse(globalChunkManager.chunks, relative_transform, iter->second.get());
            }
#else 
                //Multi thread may cause segmentation fault.
                std::vector<int> threadIndex;
                std::vector<ChunkID> chunk_ids;
                int j = 0;
                for(auto iter = processing_chunks.begin(); iter != processing_chunks.end(); ++iter)
                {
                    threadIndex.push_back(j);
                    chunk_ids.push_back(iter->first);
                    j++;
                }

                parallel_for(threadIndex.begin(),threadIndex.end(),[&](const int& j)
                {
                    ChunkID chunkID =chunk_ids[j];
                    if(globalChunkManager.GetUseColor())
                    {
                        const ChunkPtr &chunk = chunkManagers[i].GetChunk(chunkID);
                    integrator.voxelUpdateSIMDReverse(globalChunkManager.chunks, relative_transform, chunk.get());
                    }
                });
#endif 
            //globalChunkManager.chunks = processing_chunks;
            std::cout<<"Wait to update: "<<globalChunkManager.chunks.size()<<std::endl;
            chunkManagers[i].clearChunks();
        }
        //Not we get global chunks. The next step is to extract the meshes.
    }

/*
    bool Chisel::SaveTSDFFiles(const std::string& fileName)
    {
        printf("Saving tsdf to PLY file...\n");
        chisel::MeshPtr fullMesh(new chisel::Mesh());
        Vec3List V;
        Point3List C;
        Vec3List N;
        size_t v = 0;

        int X = chunkManager.GetChunkSize()(0);
        int Y = chunkManager.GetChunkSize()(1);
        int Z = chunkManager.GetChunkSize()(2);
        const ChunkMap& chunks = chunkManager.GetChunks();

        Vec3 halfVoxel = Vec3(chunkManager.GetResolution(), chunkManager.GetResolution(), chunkManager.GetResolution()) * 0.5f;
        for(const std::pair<ChunkID, ChunkPtr>& chunk:chunks)
        {
            ChunkPtr cPtr = chunk.second;
            const DistVoxel &voxels = cPtr->voxels;
            Vec3 ori = cPtr->GetOrigin();
            for (int z = 0; z < chunkManager.GetChunkSize()(2); z++)
            {
                for(int y = 0; y < chunkManager.GetChunkSize()(1); y++)
                {
                    for(int x = 0; x < chunkManager.GetChunkSize()(0); x++)
                    {
                        Vec3 pos = ori  + Vec3(x, y, z) * chunkManager.GetResolution() + halfVoxel;
                        int voxelIndex = x  + y * X + z * X * Y;


                        Point3 color = Point3(cPtr->colors.GetBlue(voxelIndex) * 255,
                                              cPtr->colors.GetGreen(voxelIndex) * 255,
                                              cPtr->colors.GetRed(voxelIndex) * 255) ;

                        Vec3 normal = Vec3((voxels.sdf[voxelIndex] + 0.2) * 30, voxels.weight[voxelIndex],voxels.sdf[voxelIndex] * 100 );

                        V.push_back(pos);
                        C.push_back(color);
                        N.push_back(normal);
                    }
                }
            }
        }

        std::ofstream output_file(fileName.c_str(), std::ios::out | std::ios::trunc);
        int pointNum = fmin(V.size(), C.size());
        output_file << "ply" << std::endl;
        output_file << "format ascii 1.0           { ascii/binary, format version number }" << std::endl;
        output_file << "comment made by Greg Turk  { comments keyword specified, like all lines }" << std::endl;
        output_file << "comment this file is a cube" << std::endl;
        output_file << "element vertex " << pointNum << "           { define \"vertex\" element, 8 of them in file }" << std::endl;
        output_file << "property float x" << std::endl;
        output_file << "property float y" << std::endl;
        output_file << "property float z" << std::endl;
        output_file << "property float nx" << std::endl;
        output_file << "property float ny" << std::endl;
        output_file << "property float nz" << std::endl;
        output_file << "property float intensity" << std::endl;
        output_file << "property uchar red" << std::endl;
        output_file << "property uchar green" << std::endl;
        output_file << "property uchar blue" << std::endl;

        output_file << "end_header" << std::endl;
        for (int i = 0; i < V.size(); i++)
        {
          output_file << V[i](0) << " " << V[i](1) << " " << V[i](2) << " "
                      << N[i](0) << " " << N[i](1) << " " << N[i](2) << " " << 1 << " "
            << C[i](0) << " " << C[i](1)  << " " << C[i](2) << " " << std::endl;
        }
        output_file.close();
        printf("Full tsdf has %lu verts\n", V.size());

        while(1)
        {

        }
    }

*/

} // namespace chisel 
