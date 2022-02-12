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

#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/geometry/Raycast.h>
#include <iostream>

namespace chisel
{

    ProjectionIntegrator::ProjectionIntegrator() :
        carvingDist(0), enableVoxelCarving(false)
    {
        // TODO Auto-generated constructor stub
        centroids_simd0 = (__m256*)_mm_malloc(64 * sizeof(__m256), 32);
        centroids_simd1 = (__m256*)_mm_malloc(64 * sizeof(__m256), 32);
        centroids_simd2 = (__m256*)_mm_malloc(64 * sizeof(__m256), 32);

    }

    ProjectionIntegrator::ProjectionIntegrator(const TruncatorPtr& t, const WeighterPtr& w, float crvDist, bool enableCrv, const Vec3List& centers) :
            truncator(t), weighter(w), carvingDist(crvDist), enableVoxelCarving(enableCrv), centroids(centers)
    {
        centroids_simd0 = (__m256*)_mm_malloc(64 * sizeof(__m256), 32);
        centroids_simd1 = (__m256*)_mm_malloc(64 * sizeof(__m256), 32);
        centroids_simd2 = (__m256*)_mm_malloc(64 * sizeof(__m256), 32);
    }


    void printUCharFrom256(__m256i data)
    {
        unsigned char * ucharData = (unsigned char *)&data;
        for(int i = 0; i < 32;i++)
        {
            printf("%d ",ucharData[i]);
        }
        printf("\r\n");
    }
    void printShortFrom256(__m256i data)
    {
        short * ucharData = (short *)&data;
        for(int i = 0; i < 16;i++)
        {
            printf("%d ",ucharData[i]);
        }
        printf("\r\n");
    }

    ChunkID GetIDAt(const Vec3& pos, const Point3 & chunkSize, float voxelResolutionMeters)
    {
        static const float roundingFactorX = 1.0f / (chunkSize(0) * voxelResolutionMeters);
        static const float roundingFactorY = 1.0f / (chunkSize(1) * voxelResolutionMeters);
        static const float roundingFactorZ = 1.0f / (chunkSize(2) * voxelResolutionMeters);
        return ChunkID(static_cast<int>(std::floor(pos(0) * roundingFactorX)),
                        static_cast<int>(std::floor(pos(1) * roundingFactorY)),
                        static_cast<int>(std::floor(pos(2) * roundingFactorZ)));
    }
    Point3 GetPoint(const Vec3 &pos, float voxelResolutionMeters)
    {
        static const float roundingFactor = 1.0f / voxelResolutionMeters; 
        return Point3(static_cast<int>(std::floor(pos(0) * roundingFactor)),
                        static_cast<int>(std::floor(pos(1) * roundingFactor)),
                        static_cast<int>(std::floor(pos(2) * roundingFactor)));
    }

    ChunkID GetPointIDAt (const Point3 &pos, const Point3 &chunkSize)
    {
        static const float roundingFactorX = 1.0f / chunkSize(0) ;
        static const float roundingFactorY = 1.0f / chunkSize(1) ;
        static const float roundingFactorZ = 1.0f / chunkSize(2);       
        return ChunkID(static_cast<int>(std::floor(pos(0) * roundingFactorX)),
                        static_cast<int>(std::floor(pos(1) * roundingFactorY)),
                        static_cast<int>(std::floor(pos(2) * roundingFactorZ)));
    }
    //we change the chunks, not the chunk.
    bool ProjectionIntegrator::voxelUpdateSIMDReverse(ChunkMap &chunks, const Transform &pose, Chunk *chunk) const
    {
        bool updated = false;
        //if(chunks.size() == 0) return false;
        float resolution = chunk->GetVoxelResolutionMeters();
        Eigen::Vector3i chunksNum = chunk->GetNumVoxels();
        int pos = 0;

        Vec3 origin = chunk->GetOrigin();
        Vec3 originInRefSubmap = pose.linear().transpose() * (origin - pose.translation());
        //we can get the sdf in advance    

        DistVoxel & voxel = chunk->voxels;
        ColorVoxel & colorVoxel = chunk->colors;
        
        Chunk ref_chunk(chunk->ID, chunksNum, resolution, chunk->useColor);// will be released

        DistVoxel &ref_voxel  =ref_chunk.voxels;
        ColorVoxel &ref_colorVoxel = ref_chunk.colors; 


        float originX,originY,originZ;
        float roundingFactorX = 1.0f / chunksNum(0);
        float roundingFactorY = 1.0f / chunksNum(1);
        float roundingFactorZ = 1.0f / chunksNum(2);
        
        
        float chunkSize0 = (float)chunksNum(0);
        float chunkSize1 = (float)chunksNum(1);
        float chunkSize2 = (float)chunksNum(2);

        originX = originInRefSubmap(0);
        originY = originInRefSubmap(1);
        originZ = originInRefSubmap(2);
        
        std::vector<int> chunkIDX(8);
        std::vector<int> chunkIDY(8);
        std::vector<int> chunkIDZ(8);
        
        std::vector<int> voxelID(8);

        __m256 origin_simd0 =_mm256_broadcast_ss(&(originX));
        __m256 origin_simd1 =_mm256_broadcast_ss(&(originY));
        __m256 origin_simd2 =_mm256_broadcast_ss(&(originZ));
        __m256 resolution_simd = _mm256_broadcast_ss(&(resolution));
        __m256 roundingFactor0 =_mm256_broadcast_ss(&(roundingFactorX));
        __m256 roundingFactor1 =_mm256_broadcast_ss(&(roundingFactorY));
        __m256 roundingFactor2 =_mm256_broadcast_ss(&(roundingFactorZ));
        __m256 chunk_size_simd0 = _mm256_broadcast_ss(&(chunkSize0));
        __m256 chunk_size_simd1 = _mm256_broadcast_ss(&(chunkSize1));
        __m256 chunk_size_simd2 = _mm256_broadcast_ss(&(chunkSize2));
        __m256 chunk_size_xy = _mm256_mul_ps(chunk_size_simd0, chunk_size_simd1);

        __m256i chunk_size_simd0i = _mm256_cvtps_epi32(chunk_size_simd0);
        __m256i chunk_size_simd1i = _mm256_cvtps_epi32(chunk_size_simd1);
        __m256i chunk_size_simd2i = _mm256_cvtps_epi32(chunk_size_simd2);
        __m256i chunk_size_xyi = _mm256_cvtps_epi32(chunk_size_xy);
        
        float * voxelSDFPointer = (float * )(&voxel.sdf[0]);
        float * voxelWeightPointer = (float * )(&voxel.weight[0]);
        unsigned short *voxelColorPointer = (unsigned short *)(&colorVoxel.colorData[0]);


        float * refVoxelSDFPointer = (float * )(&ref_voxel.sdf[0]);
        float * refVoxelWeightPointer = (float * )(&ref_voxel.weight[0]);
        unsigned short *refVoxelColorPointer = (unsigned short *)(&ref_colorVoxel.colorData[0]);

        std::vector<float *> sdf_ptr(chunksNum(2) * chunksNum(1) *chunksNum(0) ,nullptr);
        std::vector<float *> weight_ptr(chunksNum(2) * chunksNum(1) *chunksNum(0) ,nullptr);     
        std::vector<unsigned short *> color_ptr(chunksNum(2) * chunksNum(1) *chunksNum(0) ,nullptr);   
        // Get the refchunk in advance.
        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x+=8)
                {

                    //std::cout<<x<<" "<<y<<" "<<z<<std::endl;   
                    __m256 voxelCenterInRefSubmap_SIMD0 = _mm256_add_ps(origin_simd0, centroids_simd0[pos]);
                    __m256 voxelCenterInRefSubmap_SIMD1 = _mm256_add_ps(origin_simd1, centroids_simd1[pos]);
                    __m256 voxelCenterInRefSubmap_SIMD2 = _mm256_add_ps(origin_simd2, centroids_simd2[pos]);

                    __m256 world_point_SIMD0 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD0, resolution_simd));
                    __m256 world_point_SIMD1 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD1, resolution_simd)); 
                    __m256 world_point_SIMD2 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD2, resolution_simd));

                    __m256i chunk_id_SIMD0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0, roundingFactor0)));
                    __m256i chunk_id_SIMD1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1, roundingFactor1)));
                    __m256i chunk_id_SIMD2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2, roundingFactor2)));
                    
                    __m256i local_point_SIMD0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0), chunk_size_simd0)));
                    __m256i local_point_SIMD1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1), chunk_size_simd1)));
                    __m256i local_point_SIMD2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2), chunk_size_simd2)));
                    
                    __m256i voxel_id_SIMD = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0, 
                         _mm256_mullo_epi32(local_point_SIMD1, chunk_size_simd0i)));
                                            
                    // this can be done by SIMD acceleration
                    //std::cout<<sizeof(__m256i)<<" "<<sizeof(int) * chunkIDX.size()<<std::endl;
                    _mm256_storeu_si256((__m256i *)&chunkIDX[0], chunk_id_SIMD0); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[0], chunk_id_SIMD1);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[0], chunk_id_SIMD2);

                    _mm256_storeu_si256((__m256i *)&voxelID[0], voxel_id_SIMD);

                    for(int i = 0; i != 8; ++i)
                    {
                        ChunkID ref_chunk_id = ChunkID(chunkIDX[i], chunkIDY[i], chunkIDZ[i]);
                        VoxelID ref_voxel_id = voxelID[i];
                        /*
                        if(ref_chunk_id(0) != chunk->GetID()(0) || ref_chunk_id(1) != chunk->GetID()(1) || ref_chunk_id(2) != chunk->GetID()(2))
                        {
                            std::cout<<ref_chunk_id<<std::endl;
                            std::cout<<chunk->GetID()<<std::endl;
                            std::cout<<"------------\n";
                        }
                        if(ref_voxel_id != pos*8+i)
                        {
                            std::vector<int> local_point_x(8), local_point_y(8), local_point_z(8);
                            _mm256_storeu_si256((__m256i *)&local_point_x[0], local_point_SIMD0);
                            _mm256_storeu_si256((__m256i *)&local_point_y[0], local_point_SIMD1);
                            _mm256_storeu_si256((__m256i *)&local_point_z[0], local_point_SIMD2);
                            std::cout<<ref_voxel_id<<" "<<pos*8+i<<std::endl;
                            std::cout<<local_point_x[i]<<" "<<local_point_y[i]<<" "<<local_point_z[i]<<std::endl;
                        }*/
                        //std::cout<<ref_voxel_id<<std::endl;
                        //if(chunks.find(ref_chunk_id) == chunks.end())
                        //continue;
                        updated = true;
                        if(chunks.find(ref_chunk_id ) == chunks.end())
                        {
                            chunks.insert(std::make_pair(ref_chunk_id, std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), 
                                ref_chunk_id, chunksNum, resolution, chunk->useColor)));
                        }
                        auto &tmp_chunk = chunks[ref_chunk_id];
                        //std::cout<<ref_voxel_id<<std::endl;
#if 0
                        //should never happen
                        if(ref_voxel_id >= tmp_chunk->voxels.weight.size() || ref_voxel_id < 0 )
                        {
                            std::cout<<ref_voxel_id<<" "<<(tmp_chunk->voxels.weight).size()<<std::endl;
                            std::cout<< chunkIDX[i]<<" "<<chunkIDY[i]<<" "<<chunkIDZ[i] <<std::endl;
                        }
#endif
                        refVoxelSDFPointer[pos*8 + i] = tmp_chunk->voxels.sdf[ref_voxel_id];
                        refVoxelWeightPointer[pos*8 + i] = tmp_chunk->voxels.weight[ref_voxel_id];
                        memcpy(&refVoxelColorPointer[(pos*8 + i)*4], &tmp_chunk->colors.colorData[ref_voxel_id*4], sizeof(unsigned short)*4); 
                        sdf_ptr[pos*8+i] = &(tmp_chunk->voxels.sdf[ref_voxel_id]);
                        weight_ptr[pos*8+i] = &(tmp_chunk->voxels.weight[ref_voxel_id]);
                        color_ptr[pos*8 + i] = &(tmp_chunk->colors.colorData[ref_voxel_id*4]);
                    }
                    //std::cout<<pos<<std::endl;
                    ++pos;
                }
            }
        }
        if(updated == false) 
        {
            std::cout<<"no chunk need to be updated."<<std::endl;
            return updated;
        }
        // integrate two chunk 
        pos = 0;
        __m256 sigma_simd = _mm256_set1_ps(1e-4);
        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x+=8)   //Each time we process 8 voxels
                {
                    /*integrate color*/
                    __m256i inputColorSIMD[2];
                    __m256i voxelColorShortSIMD[2];
                    voxelColorShortSIMD[0] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32]);
                    voxelColorShortSIMD[1] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32 + 16]);
                    inputColorSIMD[0] = _mm256_load_si256((__m256i *)&refVoxelColorPointer[pos*32]);
                    inputColorSIMD[1] = _mm256_load_si256((__m256i *)&refVoxelColorPointer[pos*32 + 16]);
                    __m256i inputColorShortSIMD;
                    __m256i updatedColorSIMD;
                    for(int i = 0; i < 2; i++)
                    {
                        inputColorShortSIMD = inputColorSIMD[i];
                        updatedColorSIMD = _mm256_add_epi16(voxelColorShortSIMD[i],inputColorShortSIMD);
                        // boundary check!
                        __m256i saturationFlag = _mm256_cmpgt_epi16(updatedColorSIMD,_mm256_set1_epi16(120));
                        saturationFlag = _mm256_shufflehi_epi16(saturationFlag,255);
                        saturationFlag = _mm256_shufflelo_epi16(saturationFlag,255);
                        updatedColorSIMD = _mm256_blendv_epi8(updatedColorSIMD,
                                                                _mm256_srli_epi16(updatedColorSIMD,2),
                                                                saturationFlag);
                        _mm256_store_si256((__m256i *)&refVoxelColorPointer[pos * 32 + 16*i],updatedColorSIMD);
                    }
                    /*integrate sdf and weight*/
                    __m256 weightSIMD = _mm256_loadu_ps(&voxelWeightPointer[pos * 8]);
                    __m256 sdfSIMD = _mm256_loadu_ps(&voxelSDFPointer[pos*8]);
                    __m256 newWeightSIMD = _mm256_loadu_ps(&refVoxelWeightPointer[pos*8]);//_mm256_blendv_ps(defaultDepth_simd,weight_SIMD,integrateFlag);
                    __m256 newSdfSIMD = _mm256_loadu_ps(&refVoxelSDFPointer[pos*8]);
                    __m256 updatedSdfSIMD = _mm256_div_ps(_mm256_add_ps(_mm256_mul_ps(sdfSIMD,weightSIMD),_mm256_mul_ps(newSdfSIMD,newWeightSIMD)),
                                            _mm256_add_ps(_mm256_add_ps(weightSIMD,newWeightSIMD),sigma_simd));
                    __m256 updatedWeightSIMD = _mm256_add_ps(weightSIMD,newWeightSIMD);
                    __m256 weightValidSIMD =  _mm256_cmp_ps(updatedWeightSIMD,_mm256_set1_ps(0.5),_CMP_GT_OS);
                    updatedSdfSIMD = _mm256_blendv_ps(_mm256_set1_ps(999),updatedSdfSIMD,weightValidSIMD);
                    updatedWeightSIMD = _mm256_blendv_ps(_mm256_set1_ps(0.0f),updatedWeightSIMD,weightValidSIMD);

                    _mm256_storeu_ps(&refVoxelWeightPointer[pos*8], updatedWeightSIMD);
                    _mm256_storeu_ps(&refVoxelSDFPointer[pos*8],updatedSdfSIMD);    
                    pos ++;

                }
            }
        }    
        pos = 0;
        
        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x++)
                {
                    //std::cout<<*(sdf_ptr[pos])<<std::endl;  
                    *(sdf_ptr[pos]) = refVoxelSDFPointer[pos];
            
                    *(weight_ptr[pos]) = refVoxelWeightPointer[pos];
                    
                    memcpy(color_ptr[pos], 
                        &refVoxelColorPointer[pos*4], sizeof(unsigned short) * 4); 
                    ++pos;
                }
            }
        }     
        return updated;

    }
    bool ProjectionIntegrator::AddTransformed8Neighbor(ChunkMap &chunks, const Transform &pose, Chunk *chunk) const
    {
        bool updated = false;
        //if(chunks.size() == 0) return false;
        float resolution = chunk->GetVoxelResolutionMeters();
        float half_resolution = resolution / 2;
        float one = 1.0;
        Eigen::Vector3i chunksNum = chunk->GetNumVoxels();
        int pos = 0;
        
        Vec3 origin = chunk->GetOrigin();
        Vec3 originInRefSubmap = pose.linear().transpose() * (origin - pose.translation());
        //we can get the sdf in advance    
        float originX,originY,originZ;
        float roundingFactorX = 1.0f / chunksNum(0);
        float roundingFactorY = 1.0f / chunksNum(1);
        float roundingFactorZ = 1.0f / chunksNum(2);
        
        
        float chunkSize0 = (float)chunksNum(0);
        float chunkSize1 = (float)chunksNum(1);
        float chunkSize2 = (float)chunksNum(2);

        originX = originInRefSubmap(0);
        originY = originInRefSubmap(1);
        originZ = originInRefSubmap(2);
        
        std::vector<std::vector<int>> chunkIDX(8, std::vector<int>(8));
        std::vector<std::vector<int>> chunkIDY(8, std::vector<int>(8));
        std::vector<std::vector<int>> chunkIDZ(8, std::vector<int>(8));
        
        std::vector<std::vector<int>> voxelID(8, std::vector<int>(8));

        __m256 origin_simd0 =_mm256_broadcast_ss(&(originX));
        __m256 origin_simd1 =_mm256_broadcast_ss(&(originY));
        __m256 origin_simd2 =_mm256_broadcast_ss(&(originZ));
        __m256 resolution_simd = _mm256_broadcast_ss(&(resolution));
        __m256 half_resolution_simd = _mm256_broadcast_ss(&(half_resolution));
        __m256 roundingFactor0 =_mm256_broadcast_ss(&(roundingFactorX));
        __m256 roundingFactor1 =_mm256_broadcast_ss(&(roundingFactorY));
        __m256 roundingFactor2 =_mm256_broadcast_ss(&(roundingFactorZ));
        __m256 chunk_size_simd0 = _mm256_broadcast_ss(&(chunkSize0));
        __m256 chunk_size_simd1 = _mm256_broadcast_ss(&(chunkSize1));
        __m256 chunk_size_simd2 = _mm256_broadcast_ss(&(chunkSize2));
        __m256 chunk_size_xy = _mm256_mul_ps(chunk_size_simd0, chunk_size_simd1);
        __m256 one_simd = _mm256_broadcast_ss(&(one));
        __m256i chunk_size_simd0i = _mm256_cvtps_epi32(chunk_size_simd0);
        __m256i chunk_size_simd1i = _mm256_cvtps_epi32(chunk_size_simd1);
        __m256i chunk_size_simd2i = _mm256_cvtps_epi32(chunk_size_simd2);
        __m256i chunk_size_xyi = _mm256_cvtps_epi32(chunk_size_xy);
        

        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x+=8)
                {

                    //std::cout<<x<<" "<<y<<" "<<z<<std::endl;   
                    __m256 voxelBottomInRefSubmap_SIMD0 = _mm256_sub_ps(_mm256_add_ps(origin_simd0, centroids_simd0[pos]), half_resolution_simd );
                    __m256 voxelBottomInRefSubmap_SIMD1 = _mm256_sub_ps(_mm256_add_ps(origin_simd1, centroids_simd1[pos]), half_resolution_simd );
                    __m256 voxelBottomInRefSubmap_SIMD2 = _mm256_sub_ps(_mm256_add_ps(origin_simd2, centroids_simd2[pos]), half_resolution_simd );
                    // 8 neighbor
                    __m256 world_point_SIMD0_0 =  _mm256_floor_ps(_mm256_div_ps(voxelBottomInRefSubmap_SIMD0, resolution_simd));
                    __m256 world_point_SIMD1_0 =  _mm256_floor_ps(_mm256_div_ps(voxelBottomInRefSubmap_SIMD1, resolution_simd)); 
                    __m256 world_point_SIMD2_0 =  _mm256_floor_ps(_mm256_div_ps(voxelBottomInRefSubmap_SIMD2, resolution_simd));

                    __m256 world_point_SIMD0_1 = _mm256_add_ps(world_point_SIMD0_0, one_simd); 
                    __m256 world_point_SIMD1_1 = world_point_SIMD1_0;
                    __m256 world_point_SIMD2_1 = world_point_SIMD2_0;

                    __m256 world_point_SIMD0_2 = world_point_SIMD0_0; 
                    __m256 world_point_SIMD1_2 = _mm256_add_ps(world_point_SIMD1_0, one_simd);
                    __m256 world_point_SIMD2_2 = world_point_SIMD2_0;              

                    __m256 world_point_SIMD0_3 = _mm256_add_ps(world_point_SIMD0_0, one_simd);
                    __m256 world_point_SIMD1_3 = _mm256_add_ps(world_point_SIMD1_0, one_simd);
                    __m256 world_point_SIMD2_3 = world_point_SIMD2_0;

                    __m256 world_point_SIMD0_4 = world_point_SIMD0_0; 
                    __m256 world_point_SIMD1_4 = world_point_SIMD1_0;
                    __m256 world_point_SIMD2_4 = _mm256_add_ps(world_point_SIMD2_0, one_simd); 

                    __m256 world_point_SIMD0_5 = _mm256_add_ps(world_point_SIMD0_0, one_simd); 
                    __m256 world_point_SIMD1_5 = world_point_SIMD1_0;
                    __m256 world_point_SIMD2_5 = _mm256_add_ps(world_point_SIMD2_0, one_simd);                                                            

                    __m256 world_point_SIMD0_6 = world_point_SIMD0_0; 
                    __m256 world_point_SIMD1_6 = _mm256_add_ps(world_point_SIMD1_0, one_simd);
                    __m256 world_point_SIMD2_6 = _mm256_add_ps(world_point_SIMD2_0, one_simd);        

                    __m256 world_point_SIMD0_7 = _mm256_add_ps(world_point_SIMD0_0, one_simd); 
                    __m256 world_point_SIMD1_7 = _mm256_add_ps(world_point_SIMD1_0, one_simd);
                    __m256 world_point_SIMD2_7 = _mm256_add_ps(world_point_SIMD2_0, one_simd); 

                    __m256i chunk_id_SIMD0_0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_0, roundingFactor0)));
                    __m256i chunk_id_SIMD1_0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_0, roundingFactor1)));
                    __m256i chunk_id_SIMD2_0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_0, roundingFactor2)));

                    __m256i chunk_id_SIMD0_1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_1, roundingFactor0)));
                    __m256i chunk_id_SIMD1_1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_1, roundingFactor1)));
                    __m256i chunk_id_SIMD2_1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_1, roundingFactor2)));

                    __m256i chunk_id_SIMD0_2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_2, roundingFactor0)));
                    __m256i chunk_id_SIMD1_2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_2, roundingFactor1)));
                    __m256i chunk_id_SIMD2_2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_2, roundingFactor2)));

                    __m256i chunk_id_SIMD0_3 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_3, roundingFactor0)));
                    __m256i chunk_id_SIMD1_3 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_3, roundingFactor1)));
                    __m256i chunk_id_SIMD2_3 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_3, roundingFactor2)));

                    __m256i chunk_id_SIMD0_4 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_4, roundingFactor0)));
                    __m256i chunk_id_SIMD1_4 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_4, roundingFactor1)));
                    __m256i chunk_id_SIMD2_4 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_4, roundingFactor2)));

                    __m256i chunk_id_SIMD0_5 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_5, roundingFactor0)));
                    __m256i chunk_id_SIMD1_5 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_5, roundingFactor1)));
                    __m256i chunk_id_SIMD2_5 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_5, roundingFactor2)));

                    __m256i chunk_id_SIMD0_6 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_6, roundingFactor0)));
                    __m256i chunk_id_SIMD1_6 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_6, roundingFactor1)));
                    __m256i chunk_id_SIMD2_6 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_6, roundingFactor2)));

                    __m256i chunk_id_SIMD0_7 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_7, roundingFactor0)));
                    __m256i chunk_id_SIMD1_7 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_7, roundingFactor1)));
                    __m256i chunk_id_SIMD2_7 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_7, roundingFactor2)));


                    __m256i local_point_SIMD0_0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_0), chunk_size_simd0)));
                    __m256i local_point_SIMD1_0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_0), chunk_size_simd1)));
                    __m256i local_point_SIMD2_0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_0), chunk_size_simd2)));

                    __m256i local_point_SIMD0_1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_1), chunk_size_simd0)));
                    __m256i local_point_SIMD1_1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_1), chunk_size_simd1)));
                    __m256i local_point_SIMD2_1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_1), chunk_size_simd2)));

                    __m256i local_point_SIMD0_2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_2), chunk_size_simd0)));
                    __m256i local_point_SIMD1_2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_2), chunk_size_simd1)));
                    __m256i local_point_SIMD2_2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_2), chunk_size_simd2)));

                    __m256i local_point_SIMD0_3 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_3, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_3), chunk_size_simd0)));
                    __m256i local_point_SIMD1_3 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_3, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_3), chunk_size_simd1)));
                    __m256i local_point_SIMD2_3 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_3, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_3), chunk_size_simd2)));

                    __m256i local_point_SIMD0_4 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_4, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_4), chunk_size_simd0)));
                    __m256i local_point_SIMD1_4 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_4, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_4), chunk_size_simd1)));
                    __m256i local_point_SIMD2_4 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_4, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_4), chunk_size_simd2)));

                    __m256i local_point_SIMD0_5 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_5, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_5), chunk_size_simd0)));
                    __m256i local_point_SIMD1_5 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_5, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_5), chunk_size_simd1)));
                    __m256i local_point_SIMD2_5 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_5, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_5), chunk_size_simd2)));

                    __m256i local_point_SIMD0_6 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_6, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_6), chunk_size_simd0)));
                    __m256i local_point_SIMD1_6 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_6, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_6), chunk_size_simd1)));
                    __m256i local_point_SIMD2_6 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_6, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_6), chunk_size_simd2)));

                    __m256i local_point_SIMD0_7 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_7, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_7), chunk_size_simd0)));
                    __m256i local_point_SIMD1_7 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_7, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_7), chunk_size_simd1)));
                    __m256i local_point_SIMD2_7 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_7, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_7), chunk_size_simd2)));

                    __m256i voxel_id_SIMD_0 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_0, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_0, 
                         _mm256_mullo_epi32(local_point_SIMD1_0, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_1 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_1, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_1, 
                         _mm256_mullo_epi32(local_point_SIMD1_1, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_2 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_2, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_2, 
                         _mm256_mullo_epi32(local_point_SIMD1_2, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_3 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_3, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_3, 
                         _mm256_mullo_epi32(local_point_SIMD1_3, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_4 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_4, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_4, 
                         _mm256_mullo_epi32(local_point_SIMD1_4, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_5 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_5, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_5, 
                         _mm256_mullo_epi32(local_point_SIMD1_5, chunk_size_simd0i)));           
                    __m256i voxel_id_SIMD_6 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_6, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_6, 
                         _mm256_mullo_epi32(local_point_SIMD1_6, chunk_size_simd0i)));            
                    __m256i voxel_id_SIMD_7 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_7, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_7, 
                         _mm256_mullo_epi32(local_point_SIMD1_7, chunk_size_simd0i)));                     
                    // this can be done by SIMD acceleration
                    //std::cout<<sizeof(__m256i)<<" "<<sizeof(int) * chunkIDX.size()<<std::endl;
                    _mm256_storeu_si256((__m256i *)&chunkIDX[0][0], chunk_id_SIMD0_0); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[0][0], chunk_id_SIMD1_0);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[0][0], chunk_id_SIMD2_0);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[1][0], chunk_id_SIMD0_1); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[1][0], chunk_id_SIMD1_1);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[1][0], chunk_id_SIMD2_1);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[2][0], chunk_id_SIMD0_2); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[2][0], chunk_id_SIMD1_2);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[2][0], chunk_id_SIMD2_2);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[3][0], chunk_id_SIMD0_3); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[3][0], chunk_id_SIMD1_3);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[3][0], chunk_id_SIMD2_3);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[4][0], chunk_id_SIMD0_4); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[4][0], chunk_id_SIMD1_4);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[4][0], chunk_id_SIMD2_4);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[5][0], chunk_id_SIMD0_5); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[5][0], chunk_id_SIMD1_5);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[5][0], chunk_id_SIMD2_5);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[6][0], chunk_id_SIMD0_6); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[6][0], chunk_id_SIMD1_6);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[6][0], chunk_id_SIMD2_6);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[7][0], chunk_id_SIMD0_7); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[7][0], chunk_id_SIMD1_7);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[7][0], chunk_id_SIMD2_7);
                    
                    _mm256_storeu_si256((__m256i *)&voxelID[0][0], voxel_id_SIMD_0);
                    _mm256_storeu_si256((__m256i *)&voxelID[1][0], voxel_id_SIMD_1);
                    _mm256_storeu_si256((__m256i *)&voxelID[2][0], voxel_id_SIMD_2);                    
                    _mm256_storeu_si256((__m256i *)&voxelID[3][0], voxel_id_SIMD_3);
                    _mm256_storeu_si256((__m256i *)&voxelID[4][0], voxel_id_SIMD_4);
                    _mm256_storeu_si256((__m256i *)&voxelID[5][0], voxel_id_SIMD_5);
                    _mm256_storeu_si256((__m256i *)&voxelID[6][0], voxel_id_SIMD_6);                    
                    _mm256_storeu_si256((__m256i *)&voxelID[7][0], voxel_id_SIMD_7);

                    for(int neighbor_id = 0; neighbor_id != 8; ++ neighbor_id)
                    {
                        for(int i = 0; i != 8; ++i)
                        {
                            ChunkID ref_chunk_id = ChunkID(chunkIDX[neighbor_id][i], chunkIDY[neighbor_id][i], chunkIDZ[neighbor_id][i]);
                            VoxelID ref_voxel_id = voxelID[neighbor_id][i];

                            updated = true;
                            if(chunks.find(ref_chunk_id ) == chunks.end())
                            {
                                chunks.insert(std::make_pair(ref_chunk_id, std::allocate_shared<Chunk>(Eigen::aligned_allocator<Chunk>(), 
                                    ref_chunk_id, chunksNum, resolution, chunk->useColor)));
                            }
                        }
                    }
                    ++pos;
                }
            }
        }
        return updated;

    }
    bool ProjectionIntegrator::VoxelUpdate8Neighbor(Chunk *chunk, const Transform &pose, ChunkMap &chunks) const
    {
        bool updated = false;
        //if(chunks.size() == 0) return false;
        float resolution = chunk->GetVoxelResolutionMeters();
        float half_resolution = resolution / 2;
        float one = 1.0;
        Eigen::Vector3i chunksNum = chunk->GetNumVoxels();
        int pos = 0;
        
        Vec3 origin = chunk->GetOrigin();
        Vec3 originInRefSubmap = pose.linear().transpose() * (origin - pose.translation());
        //we can get the sdf in advance    
        float originX,originY,originZ;
        float roundingFactorX = 1.0f / chunksNum(0);
        float roundingFactorY = 1.0f / chunksNum(1);
        float roundingFactorZ = 1.0f / chunksNum(2);
        
        
        float chunkSize0 = (float)chunksNum(0);
        float chunkSize1 = (float)chunksNum(1);
        float chunkSize2 = (float)chunksNum(2);

        originX = originInRefSubmap(0);
        originY = originInRefSubmap(1);
        originZ = originInRefSubmap(2);
        
        std::vector<std::vector<int>> chunkIDX(8, std::vector<int>(8));
        std::vector<std::vector<int>> chunkIDY(8, std::vector<int>(8));
        std::vector<std::vector<int>> chunkIDZ(8, std::vector<int>(8));
        
        std::vector<std::vector<int>> voxelID(8, std::vector<int>(8));

        __m256 origin_simd0 =_mm256_broadcast_ss(&(originX));
        __m256 origin_simd1 =_mm256_broadcast_ss(&(originY));
        __m256 origin_simd2 =_mm256_broadcast_ss(&(originZ));
        __m256 resolution_simd = _mm256_broadcast_ss(&(resolution));
        __m256 half_resolution_simd = _mm256_broadcast_ss(&(half_resolution));
        __m256 roundingFactor0 =_mm256_broadcast_ss(&(roundingFactorX));
        __m256 roundingFactor1 =_mm256_broadcast_ss(&(roundingFactorY));
        __m256 roundingFactor2 =_mm256_broadcast_ss(&(roundingFactorZ));
        __m256 chunk_size_simd0 = _mm256_broadcast_ss(&(chunkSize0));
        __m256 chunk_size_simd1 = _mm256_broadcast_ss(&(chunkSize1));
        __m256 chunk_size_simd2 = _mm256_broadcast_ss(&(chunkSize2));
        __m256 chunk_size_xy = _mm256_mul_ps(chunk_size_simd0, chunk_size_simd1);
        __m256 one_simd = _mm256_broadcast_ss(&(one));
        __m256i chunk_size_simd0i = _mm256_cvtps_epi32(chunk_size_simd0);
        __m256i chunk_size_simd1i = _mm256_cvtps_epi32(chunk_size_simd1);
        __m256i chunk_size_simd2i = _mm256_cvtps_epi32(chunk_size_simd2);
        __m256i chunk_size_xyi = _mm256_cvtps_epi32(chunk_size_xy);
        
        DistVoxel & voxel = chunk->voxels;
        ColorVoxel & colorVoxel = chunk->colors;

        float * voxelSDFPointer = (float * )(&voxel.sdf[0]);
        float * voxelWeightPointer = (float * )(&voxel.weight[0]);
        unsigned short *voxelColorPointer = (unsigned short *)(&colorVoxel.colorData[0]);
                
        std::vector<Chunk> ref_chunks;
        for(int i = 0; i != 8; ++i)
        ref_chunks.emplace_back(chunk->ID, chunksNum, resolution, chunk->useColor);// will be released
        


        std::vector<float *> refVoxelSDFPointers(8, nullptr);
        std::vector<float *> refVoxelWeightPointers(8, nullptr);
        std::vector<unsigned short *> refVoxelColorPointers(8, nullptr);

        std::vector<float *> bottom_left_x(8 * 8, nullptr);
        std::vector<float *> bottom_left_y(8 * 8, nullptr);
        std::vector<float *> bottom_left_z(8 * 8, nullptr);

        std::vector<float *> _8_neignbor_0_x(8 * 8, nullptr);
        std::vector<float *> _8_neignbor_0_y(8 * 8, nullptr);
        std::vector<float *> _8_neignbor_0_z(8 * 8, nullptr);

        std::vector<float *> x_weight(8 * 8, nullptr);
        std::vector<float *> y_weight(8 * 8, nullptr);
        std::vector<float *> z_weight(8 * 8, nullptr);

        for(int i = 0; i != 8; ++i)
        {
            refVoxelSDFPointers[i] = (float * )(&ref_chunks[i].voxels.sdf[0]);
            refVoxelWeightPointers[i] = (float * )(&ref_chunks[i].voxels.weight[0]);
            refVoxelColorPointers[i] = (unsigned short *)(&ref_chunks[i].colors.colorData[0]);
            
        }

        
        for(int i = 0; i != 8; ++i)
        {
            for(int j = 0; j != 8; ++j)
            {
                bottom_left_x[i * 8 + j] = new float [8];
                bottom_left_y[i * 8 + j] = new float [8];
                bottom_left_z[i * 8 + j] = new float [8];
                _8_neignbor_0_x[i * 8 + j] = new float [8];
                _8_neignbor_0_y[i * 8 + j] = new float [8];
                _8_neignbor_0_z[i * 8 + j] = new float [8];

                x_weight[i * 8 + j] = new float [8];
                y_weight[i * 8 + j] = new float [8];
                z_weight[i * 8 + j] = new float [8];
            }
        }

        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x+=8)
                {

                    //std::cout<<x<<" "<<y<<" "<<z<<std::endl;   
                    __m256 voxelBottomInRefSubmap_SIMD0 = _mm256_sub_ps(_mm256_add_ps(origin_simd0, centroids_simd0[pos]), half_resolution_simd );
                    __m256 voxelBottomInRefSubmap_SIMD1 = _mm256_sub_ps(_mm256_add_ps(origin_simd1, centroids_simd1[pos]), half_resolution_simd );
                    __m256 voxelBottomInRefSubmap_SIMD2 = _mm256_sub_ps(_mm256_add_ps(origin_simd2, centroids_simd2[pos]), half_resolution_simd );
                    // 8 neighbor
                    __m256 world_point_SIMD0_0 =  _mm256_floor_ps(_mm256_div_ps(voxelBottomInRefSubmap_SIMD0, resolution_simd));
                    __m256 world_point_SIMD1_0 =  _mm256_floor_ps(_mm256_div_ps(voxelBottomInRefSubmap_SIMD1, resolution_simd)); 
                    __m256 world_point_SIMD2_0 =  _mm256_floor_ps(_mm256_div_ps(voxelBottomInRefSubmap_SIMD2, resolution_simd));
                    
                    _mm256_storeu_ps(bottom_left_x[z * 8 + y], voxelBottomInRefSubmap_SIMD0);
                    _mm256_storeu_ps(bottom_left_y[z * 8 + y], voxelBottomInRefSubmap_SIMD1);
                    _mm256_storeu_ps(bottom_left_z[z * 8 + y], voxelBottomInRefSubmap_SIMD2);                    
                    
                    _mm256_storeu_ps(_8_neignbor_0_x[z * 8 + y], world_point_SIMD0_0);
                    _mm256_storeu_ps(_8_neignbor_0_y[z * 8 + y], world_point_SIMD1_0);
                    _mm256_storeu_ps(_8_neignbor_0_z[z * 8 + y], world_point_SIMD2_0);

                    /*
                            float x_weight = (new_position(0) - _8_neighbor[0](0) * c_para.VoxelResolution ) / c_para.VoxelResolution;
                            float y_weight = (new_position(1) - _8_neighbor[0](1) * c_para.VoxelResolution ) / c_para.VoxelResolution;
                            float z_weight = (new_position(2) - _8_neighbor[0](2) * c_para.VoxelResolution ) / c_para.VoxelResolution;
                    */
                    _mm256_storeu_ps(x_weight[z * 8 + y], _mm256_div_ps(_mm256_sub_ps(voxelBottomInRefSubmap_SIMD0, 
                        _mm256_mul_ps( world_point_SIMD0_0, resolution_simd)), resolution_simd));
                    _mm256_storeu_ps(y_weight[z * 8 + y], _mm256_div_ps(_mm256_sub_ps(voxelBottomInRefSubmap_SIMD1, 
                        _mm256_mul_ps( world_point_SIMD1_0, resolution_simd)), resolution_simd));
                    _mm256_storeu_ps(z_weight[z * 8 + y], _mm256_div_ps(_mm256_sub_ps(voxelBottomInRefSubmap_SIMD2, 
                        _mm256_mul_ps( world_point_SIMD2_0, resolution_simd)), resolution_simd));
                    
                    __m256 world_point_SIMD0_1 = _mm256_add_ps(world_point_SIMD0_0, one_simd); 
                    __m256 world_point_SIMD1_1 = world_point_SIMD1_0;
                    __m256 world_point_SIMD2_1 = world_point_SIMD2_0;

                    __m256 world_point_SIMD0_2 = world_point_SIMD0_0; 
                    __m256 world_point_SIMD1_2 = _mm256_add_ps(world_point_SIMD1_0, one_simd);
                    __m256 world_point_SIMD2_2 = world_point_SIMD2_0;              

                    __m256 world_point_SIMD0_3 = _mm256_add_ps(world_point_SIMD0_0, one_simd);
                    __m256 world_point_SIMD1_3 = _mm256_add_ps(world_point_SIMD1_0, one_simd);
                    __m256 world_point_SIMD2_3 = world_point_SIMD2_0;

                    __m256 world_point_SIMD0_4 = world_point_SIMD0_0; 
                    __m256 world_point_SIMD1_4 = world_point_SIMD1_0;
                    __m256 world_point_SIMD2_4 = _mm256_add_ps(world_point_SIMD2_0, one_simd); 

                    __m256 world_point_SIMD0_5 = _mm256_add_ps(world_point_SIMD0_0, one_simd); 
                    __m256 world_point_SIMD1_5 = world_point_SIMD1_0;
                    __m256 world_point_SIMD2_5 = _mm256_add_ps(world_point_SIMD2_0, one_simd);                                                            

                    __m256 world_point_SIMD0_6 = world_point_SIMD0_0; 
                    __m256 world_point_SIMD1_6 = _mm256_add_ps(world_point_SIMD1_0, one_simd);
                    __m256 world_point_SIMD2_6 = _mm256_add_ps(world_point_SIMD2_0, one_simd);        

                    __m256 world_point_SIMD0_7 = _mm256_add_ps(world_point_SIMD0_0, one_simd); 
                    __m256 world_point_SIMD1_7 = _mm256_add_ps(world_point_SIMD1_0, one_simd);
                    __m256 world_point_SIMD2_7 = _mm256_add_ps(world_point_SIMD2_0, one_simd); 

                    __m256i chunk_id_SIMD0_0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_0, roundingFactor0)));
                    __m256i chunk_id_SIMD1_0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_0, roundingFactor1)));
                    __m256i chunk_id_SIMD2_0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_0, roundingFactor2)));

                    __m256i chunk_id_SIMD0_1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_1, roundingFactor0)));
                    __m256i chunk_id_SIMD1_1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_1, roundingFactor1)));
                    __m256i chunk_id_SIMD2_1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_1, roundingFactor2)));

                    __m256i chunk_id_SIMD0_2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_2, roundingFactor0)));
                    __m256i chunk_id_SIMD1_2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_2, roundingFactor1)));
                    __m256i chunk_id_SIMD2_2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_2, roundingFactor2)));

                    __m256i chunk_id_SIMD0_3 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_3, roundingFactor0)));
                    __m256i chunk_id_SIMD1_3 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_3, roundingFactor1)));
                    __m256i chunk_id_SIMD2_3 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_3, roundingFactor2)));

                    __m256i chunk_id_SIMD0_4 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_4, roundingFactor0)));
                    __m256i chunk_id_SIMD1_4 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_4, roundingFactor1)));
                    __m256i chunk_id_SIMD2_4 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_4, roundingFactor2)));

                    __m256i chunk_id_SIMD0_5 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_5, roundingFactor0)));
                    __m256i chunk_id_SIMD1_5 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_5, roundingFactor1)));
                    __m256i chunk_id_SIMD2_5 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_5, roundingFactor2)));

                    __m256i chunk_id_SIMD0_6 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_6, roundingFactor0)));
                    __m256i chunk_id_SIMD1_6 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_6, roundingFactor1)));
                    __m256i chunk_id_SIMD2_6 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_6, roundingFactor2)));

                    __m256i chunk_id_SIMD0_7 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0_7, roundingFactor0)));
                    __m256i chunk_id_SIMD1_7 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1_7, roundingFactor1)));
                    __m256i chunk_id_SIMD2_7 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2_7, roundingFactor2)));


                    __m256i local_point_SIMD0_0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_0), chunk_size_simd0)));
                    __m256i local_point_SIMD1_0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_0), chunk_size_simd1)));
                    __m256i local_point_SIMD2_0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_0), chunk_size_simd2)));

                    __m256i local_point_SIMD0_1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_1), chunk_size_simd0)));
                    __m256i local_point_SIMD1_1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_1), chunk_size_simd1)));
                    __m256i local_point_SIMD2_1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_1), chunk_size_simd2)));

                    __m256i local_point_SIMD0_2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_2), chunk_size_simd0)));
                    __m256i local_point_SIMD1_2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_2), chunk_size_simd1)));
                    __m256i local_point_SIMD2_2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_2), chunk_size_simd2)));

                    __m256i local_point_SIMD0_3 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_3, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_3), chunk_size_simd0)));
                    __m256i local_point_SIMD1_3 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_3, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_3), chunk_size_simd1)));
                    __m256i local_point_SIMD2_3 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_3, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_3), chunk_size_simd2)));

                    __m256i local_point_SIMD0_4 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_4, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_4), chunk_size_simd0)));
                    __m256i local_point_SIMD1_4 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_4, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_4), chunk_size_simd1)));
                    __m256i local_point_SIMD2_4 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_4, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_4), chunk_size_simd2)));

                    __m256i local_point_SIMD0_5 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_5, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_5), chunk_size_simd0)));
                    __m256i local_point_SIMD1_5 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_5, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_5), chunk_size_simd1)));
                    __m256i local_point_SIMD2_5 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_5, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_5), chunk_size_simd2)));

                    __m256i local_point_SIMD0_6 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_6, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_6), chunk_size_simd0)));
                    __m256i local_point_SIMD1_6 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_6, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_6), chunk_size_simd1)));
                    __m256i local_point_SIMD2_6 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_6, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_6), chunk_size_simd2)));

                    __m256i local_point_SIMD0_7 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0_7, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0_7), chunk_size_simd0)));
                    __m256i local_point_SIMD1_7 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1_7, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1_7), chunk_size_simd1)));
                    __m256i local_point_SIMD2_7 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2_7, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2_7), chunk_size_simd2)));

                    __m256i voxel_id_SIMD_0 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_0, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_0, 
                         _mm256_mullo_epi32(local_point_SIMD1_0, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_1 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_1, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_1, 
                         _mm256_mullo_epi32(local_point_SIMD1_1, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_2 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_2, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_2, 
                         _mm256_mullo_epi32(local_point_SIMD1_2, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_3 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_3, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_3, 
                         _mm256_mullo_epi32(local_point_SIMD1_3, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_4 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_4, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_4, 
                         _mm256_mullo_epi32(local_point_SIMD1_4, chunk_size_simd0i)));
                    __m256i voxel_id_SIMD_5 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_5, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_5, 
                         _mm256_mullo_epi32(local_point_SIMD1_5, chunk_size_simd0i)));           
                    __m256i voxel_id_SIMD_6 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_6, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_6, 
                         _mm256_mullo_epi32(local_point_SIMD1_6, chunk_size_simd0i)));            
                    __m256i voxel_id_SIMD_7 = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2_7, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0_7, 
                         _mm256_mullo_epi32(local_point_SIMD1_7, chunk_size_simd0i)));                     
                    // this can be done by SIMD acceleration
                    //std::cout<<sizeof(__m256i)<<" "<<sizeof(int) * chunkIDX.size()<<std::endl;
                    _mm256_storeu_si256((__m256i *)&chunkIDX[0][0], chunk_id_SIMD0_0); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[0][0], chunk_id_SIMD1_0);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[0][0], chunk_id_SIMD2_0);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[1][0], chunk_id_SIMD0_1); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[1][0], chunk_id_SIMD1_1);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[1][0], chunk_id_SIMD2_1);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[2][0], chunk_id_SIMD0_2); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[2][0], chunk_id_SIMD1_2);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[2][0], chunk_id_SIMD2_2);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[3][0], chunk_id_SIMD0_3); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[3][0], chunk_id_SIMD1_3);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[3][0], chunk_id_SIMD2_3);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[4][0], chunk_id_SIMD0_4); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[4][0], chunk_id_SIMD1_4);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[4][0], chunk_id_SIMD2_4);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[5][0], chunk_id_SIMD0_5); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[5][0], chunk_id_SIMD1_5);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[5][0], chunk_id_SIMD2_5);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[6][0], chunk_id_SIMD0_6); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[6][0], chunk_id_SIMD1_6);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[6][0], chunk_id_SIMD2_6);

                    _mm256_storeu_si256((__m256i *)&chunkIDX[7][0], chunk_id_SIMD0_7); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[7][0], chunk_id_SIMD1_7);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[7][0], chunk_id_SIMD2_7);
                    
                    _mm256_storeu_si256((__m256i *)&voxelID[0][0], voxel_id_SIMD_0);
                    _mm256_storeu_si256((__m256i *)&voxelID[1][0], voxel_id_SIMD_1);
                    _mm256_storeu_si256((__m256i *)&voxelID[2][0], voxel_id_SIMD_2);                    
                    _mm256_storeu_si256((__m256i *)&voxelID[3][0], voxel_id_SIMD_3);
                    _mm256_storeu_si256((__m256i *)&voxelID[4][0], voxel_id_SIMD_4);
                    _mm256_storeu_si256((__m256i *)&voxelID[5][0], voxel_id_SIMD_5);
                    _mm256_storeu_si256((__m256i *)&voxelID[6][0], voxel_id_SIMD_6);                    
                    _mm256_storeu_si256((__m256i *)&voxelID[7][0], voxel_id_SIMD_7);


                    for(int neighbor_id = 0; neighbor_id != 8; ++ neighbor_id)
                    {
                        for(int i = 0; i != 8; ++i)
                        {
                            ChunkID ref_chunk_id = ChunkID(chunkIDX[neighbor_id][i], chunkIDY[neighbor_id][i], chunkIDZ[neighbor_id][i]);
                            VoxelID ref_voxel_id = voxelID[neighbor_id][i];
                            //std::cout<<ref_voxel_id<<std::endl;
                            if(chunks.find(ref_chunk_id) == chunks.end())
                            continue;
                            updated = true;
                            auto &tmp_chunk = chunks[ref_chunk_id];
                            refVoxelSDFPointers[neighbor_id][pos*8 + i] = tmp_chunk->voxels.sdf[ref_voxel_id];
                            refVoxelWeightPointers[neighbor_id][pos*8 + i] = tmp_chunk->voxels.weight[ref_voxel_id];
                            memcpy(&refVoxelColorPointers[neighbor_id][(pos*8 + i)*4], &tmp_chunk->colors.colorData[ref_voxel_id*4], sizeof(unsigned short)*4); 
                        }
                    }
                    ++pos;
                }
            }
        }

        //std::cout<<"Read all needed information..."<<std::endl;
        __m256 sigma_simd = _mm256_set1_ps(1e-4);
        //
#if 0
        for(int neighbor_id = 0; neighbor_id != 8; ++ neighbor_id)
        {
            pos = 0;
            for (int z = 0; z < chunksNum(2); z++)
            {
                for(int y = 0; y < chunksNum(1); y++)
                {
                    for(int x = 0; x < chunksNum(0); x+=8)   //Each time we process 8 voxels
                    {
                        //integrate color
                        __m256i inputColorSIMD[2];
                        __m256i voxelColorShortSIMD[2];
                        voxelColorShortSIMD[0] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32]);
                        voxelColorShortSIMD[1] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32 + 16]);
                        inputColorSIMD[0] = _mm256_load_si256((__m256i *)&refVoxelColorPointers[neighbor_id][pos*32]);
                        inputColorSIMD[1] = _mm256_load_si256((__m256i *)&refVoxelColorPointers[neighbor_id][pos*32 + 16]);
                        __m256i inputColorShortSIMD;
                        __m256i updatedColorSIMD;
                        for(int i = 0; i < 2; i++)
                        {
                            inputColorShortSIMD = inputColorSIMD[i];
                            updatedColorSIMD = _mm256_add_epi16(voxelColorShortSIMD[i],inputColorShortSIMD);
                            // boundary check!
                            __m256i saturationFlag = _mm256_cmpgt_epi16(updatedColorSIMD,_mm256_set1_epi16(120));
                            saturationFlag = _mm256_shufflehi_epi16(saturationFlag,255);
                            saturationFlag = _mm256_shufflelo_epi16(saturationFlag,255);
                            updatedColorSIMD = _mm256_blendv_epi8(updatedColorSIMD,
                                                                    _mm256_srli_epi16(updatedColorSIMD,2),
                                                                    saturationFlag);
                            _mm256_store_si256((__m256i *)&voxelColorPointer[pos * 32 + 16*i],updatedColorSIMD);
                        }
                        /*integrate sdf and weight*/
                        __m256 weightSIMD = _mm256_loadu_ps(&voxelWeightPointer[pos * 8]);
                        __m256 sdfSIMD = _mm256_loadu_ps(&voxelSDFPointer[pos*8]);
                        __m256 newWeightSIMD = _mm256_loadu_ps(&refVoxelWeightPointers[neighbor_id][pos*8]);//_mm256_blendv_ps(defaultDepth_simd,weight_SIMD,integrateFlag);
                        __m256 newSdfSIMD = _mm256_loadu_ps(&refVoxelSDFPointers[neighbor_id][pos*8]);
                        __m256 updatedSdfSIMD = _mm256_div_ps(_mm256_add_ps(_mm256_mul_ps(sdfSIMD,weightSIMD),_mm256_mul_ps(newSdfSIMD,newWeightSIMD)),
                                                _mm256_add_ps(_mm256_add_ps(weightSIMD,newWeightSIMD),sigma_simd));
                        __m256 updatedWeightSIMD = _mm256_add_ps(weightSIMD,newWeightSIMD);
                        __m256 weightValidSIMD =  _mm256_cmp_ps(updatedWeightSIMD,_mm256_set1_ps(0.5),_CMP_GT_OS);
                        updatedSdfSIMD = _mm256_blendv_ps(_mm256_set1_ps(999),updatedSdfSIMD,weightValidSIMD);
                        updatedWeightSIMD = _mm256_blendv_ps(_mm256_set1_ps(0.0f),updatedWeightSIMD,weightValidSIMD);

                        _mm256_storeu_ps(&voxelWeightPointer[pos*8], updatedWeightSIMD);
                        _mm256_storeu_ps(&voxelSDFPointer[pos*8],updatedSdfSIMD);    
                        pos ++;
                    }
                }
            }
        }
#else

/*
        const geometry::Point3f center = new_position; 
        float x_weight = (new_position(0) - _8_neighbor[0](0) * c_para.VoxelResolution ) / c_para.VoxelResolution;
        float y_weight = (new_position(1) - _8_neighbor[0](1) * c_para.VoxelResolution ) / c_para.VoxelResolution;
        float z_weight = (new_position(2) - _8_neighbor[0](2) * c_para.VoxelResolution ) / c_para.VoxelResolution;
        
        TSDFVoxel result1;
        if(_8_voxels[0].weight != 0 || _8_voxels[1].weight != 0)
        result1 = ((_8_voxels[0] * (1 - x_weight)).add( _8_voxels[1] * x_weight))
            / ((1 - x_weight) * (_8_voxels[0].weight != 0.0) + x_weight * (_8_voxels[1].weight != 0));
        TSDFVoxel result2;
        if(_8_voxels[2].weight != 0 || _8_voxels[3].weight != 0)
        result2 = ((_8_voxels[2] * (1 - x_weight)).add( _8_voxels[3] * x_weight))
            / ((1 - x_weight) * (_8_voxels[2].weight != 0.0) + x_weight * (_8_voxels[3].weight != 0));

        TSDFVoxel result_z_1;
        if(result1.weight != 0 || result2.weight != 0)
        result_z_1 = (result1 * (1 - y_weight)).add(result2 * y_weight)
            / ((1 - y_weight) * (result1.weight != 0) + y_weight * (result2.weight != 0));

            
        TSDFVoxel result3 = TSDFVoxel();
        if(_8_voxels[4].weight != 0 || _8_voxels[5].weight != 0)
        result3 = (_8_voxels[4] * (1 - x_weight)).add( _8_voxels[5] * x_weight)
            / ((1 - x_weight) * (_8_voxels[4].weight != 0.0) + x_weight * (_8_voxels[5].weight != 0));
        TSDFVoxel result4 = TSDFVoxel();
        if(_8_voxels[6].weight != 0 || _8_voxels[7].weight != 0)
        result4 = (_8_voxels[6] * (1 - x_weight)).add( _8_voxels[7] * x_weight)
            / ((1 - x_weight) * (_8_voxels[6].weight != 0.0) + x_weight * (_8_voxels[7].weight != 0));

        TSDFVoxel result_z_2;
        if(result3.weight != 0 || result4.weight != 0)
        result_z_2= (result3 * ( 1 - y_weight)).add( result4 * y_weight)
            / ((1 - y_weight) * (result3.weight != 0) + y_weight * (result4.weight != 0));

        TSDFVoxel final_voxel;
        if(result_z_1.weight != 0 || result_z_2.weight != 0)
        final_voxel = (result_z_1 * (1 - z_weight)).add( result_z_2 * z_weight)
            / ((1 - z_weight) * (result_z_1.weight != 0) + z_weight * (result_z_2.weight != 0));
        return final_voxel;
*/

        for(int neighbor_id = 0; neighbor_id != 8; ++ neighbor_id)
        {
            pos = 0;
            for (int z = 0; z < chunksNum(2); z++)
            {
                for(int y = 0; y < chunksNum(1); y++)
                {
                    for(int x = 0; x < chunksNum(0); x+=8)   //Each time we process 8 voxels
                    {
                        __m256 x_weight_tmp;
                        __m256 y_weight_tmp;
                        __m256 z_weight_tmp;
                        __m256 x_weight_simd = _mm256_loadu_ps(x_weight[8 * z + y]);
                        __m256 y_weight_simd = _mm256_loadu_ps(y_weight[8 *z + y]);
                        __m256 z_weight_simd = _mm256_loadu_ps(z_weight[8 *z + y]);
                        if(neighbor_id == 0)//000
                        {
                            x_weight_tmp = _mm256_sub_ps(one_simd, x_weight_simd);
                            y_weight_tmp = _mm256_sub_ps(one_simd, y_weight_simd);
                            z_weight_tmp = _mm256_sub_ps(one_simd, z_weight_simd);
                        }
                        else if(neighbor_id == 1)//100
                        {
                            x_weight_tmp = x_weight_simd;
                            y_weight_tmp = _mm256_sub_ps(one_simd, y_weight_simd);
                            z_weight_tmp = _mm256_sub_ps(one_simd, z_weight_simd);
                        }
                        else if(neighbor_id == 2)//010
                        {
                            x_weight_tmp = _mm256_sub_ps(one_simd, x_weight_simd);
                            y_weight_tmp = y_weight_simd;
                            z_weight_tmp = _mm256_sub_ps(one_simd, z_weight_simd);
                        }
                        else if(neighbor_id == 3)//110
                        {
                            x_weight_tmp = x_weight_simd;
                            y_weight_tmp = y_weight_simd;
                            z_weight_tmp = _mm256_sub_ps(one_simd, z_weight_simd);
                        }
                        else if(neighbor_id == 4)//001
                        {
                            x_weight_tmp = _mm256_sub_ps(one_simd, x_weight_simd);
                            y_weight_tmp = _mm256_sub_ps(one_simd, y_weight_simd);
                            z_weight_tmp = z_weight_simd;                            
                        }
                        else if(neighbor_id == 5)//101
                        {
                            x_weight_tmp = x_weight_simd;
                            y_weight_tmp = _mm256_sub_ps(one_simd, y_weight_simd);
                            z_weight_tmp = z_weight_simd;
                        }
                        else if(neighbor_id == 6)//011
                        {
                            x_weight_tmp = _mm256_sub_ps(one_simd, x_weight_simd);
                            y_weight_tmp = y_weight_simd;
                            z_weight_tmp = z_weight_simd;                            
                        }
                        else //111
                        {
                            x_weight_tmp = x_weight_simd;
                            y_weight_tmp = y_weight_simd;
                            z_weight_tmp = z_weight_simd;                                
                        }
                        __m256 multi_weight = _mm256_mul_ps(x_weight_tmp, _mm256_mul_ps(y_weight_tmp, z_weight_tmp));
                        //integrate color
                        __m256 world_point_SIMD0_0 =  _mm256_loadu_ps(bottom_left_x[8 * z + y]);
                        __m256 world_point_SIMD1_0 =  _mm256_loadu_ps(bottom_left_y[8 * z + y]);
                        __m256 world_point_SIMD2_0 =  _mm256_loadu_ps(bottom_left_x[8 * z + z]);

                        __m256i inputColorSIMD[2];
                        __m256i voxelColorShortSIMD[2];
                        voxelColorShortSIMD[0] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32]);
                        voxelColorShortSIMD[1] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32 + 16]);
                        inputColorSIMD[0] = _mm256_load_si256((__m256i *)&refVoxelColorPointers[neighbor_id][pos*32]);
                        inputColorSIMD[1] = _mm256_load_si256((__m256i *)&refVoxelColorPointers[neighbor_id][pos*32 + 16]);
                        __m256i inputColorShortSIMD;
                        __m256i updatedColorSIMD;
                        for(int i = 0; i < 2; i++)
                        {
                            inputColorShortSIMD = inputColorSIMD[i];
                            updatedColorSIMD = _mm256_add_epi16(voxelColorShortSIMD[i],inputColorShortSIMD);
                            // boundary check!
                            __m256i saturationFlag = _mm256_cmpgt_epi16(updatedColorSIMD,_mm256_set1_epi16(120));
                            saturationFlag = _mm256_shufflehi_epi16(saturationFlag,255);
                            saturationFlag = _mm256_shufflelo_epi16(saturationFlag,255);
                            updatedColorSIMD = _mm256_blendv_epi8(updatedColorSIMD,
                                                                    _mm256_srli_epi16(updatedColorSIMD,2),
                                                                    saturationFlag);
                            _mm256_store_si256((__m256i *)&voxelColorPointer[pos * 32 + 16*i],updatedColorSIMD);
                        }
                        /*integrate sdf and weight*/
                        __m256 weightSIMD = _mm256_loadu_ps(&voxelWeightPointer[pos * 8]);
                        __m256 sdfSIMD = _mm256_loadu_ps(&voxelSDFPointer[pos*8]);
                        __m256 newWeightSIMD = _mm256_loadu_ps(&refVoxelWeightPointers[neighbor_id][pos*8]);//_mm256_blendv_ps(defaultDepth_simd,weight_SIMD,integrateFlag);
                        newWeightSIMD = _mm256_mul_ps(multi_weight, newWeightSIMD);
                        __m256 newSdfSIMD = _mm256_loadu_ps(&refVoxelSDFPointers[neighbor_id][pos*8]);
                        __m256 updatedSdfSIMD = _mm256_div_ps(_mm256_add_ps(_mm256_mul_ps(sdfSIMD,weightSIMD),_mm256_mul_ps(newSdfSIMD,newWeightSIMD)),
                                                _mm256_add_ps(_mm256_add_ps(weightSIMD,newWeightSIMD),sigma_simd));
                        __m256 updatedWeightSIMD = _mm256_add_ps(weightSIMD,newWeightSIMD);
                        __m256 weightValidSIMD =  _mm256_cmp_ps(updatedWeightSIMD,_mm256_set1_ps(0.5),_CMP_GT_OS);
                        updatedSdfSIMD = _mm256_blendv_ps(_mm256_set1_ps(999),updatedSdfSIMD,weightValidSIMD);
                        updatedWeightSIMD = _mm256_blendv_ps(_mm256_set1_ps(0.0f),updatedWeightSIMD,weightValidSIMD);

                        _mm256_storeu_ps(&voxelWeightPointer[pos*8], updatedWeightSIMD);
                        _mm256_storeu_ps(&voxelSDFPointer[pos*8],updatedSdfSIMD);    
                        pos ++;    
                        
                        }
                        
                    }
                }
            }
#endif

        for(int i = 0; i != 8; ++i)
        {
            for(int j = 0; j != 8; ++j)
            {
                delete[] bottom_left_x[i * 8 + j] ;
                delete[] bottom_left_y[i * 8 + j];
                delete[] bottom_left_z[i * 8 + j];
                delete[] _8_neignbor_0_x[i * 8 + j];
                delete[] _8_neignbor_0_y[i * 8 + j];
                delete[] _8_neignbor_0_z[i * 8 + j];

                delete[] x_weight[i * 8 + j];
                delete[] y_weight[i * 8 + j];
                delete[] z_weight[i * 8 + j];
            }
        }

        return updated;
    }
    void ProjectionIntegrator::voxelUpdate(ChunkMap &chunks, const Transform &pose, Chunk *chunk) const
    {

        float resolution = chunk->GetVoxelResolutionMeters();
        Eigen::Vector3i chunksNum = chunk->GetNumVoxels();

        Vec3 origin = chunk->GetOrigin();
        Vec3 originInRefSubmap = pose.linear().transpose() * (origin - pose.translation());
        //we can get the sdf in advance

        DistVoxel & voxel = chunk->voxels;
        ColorVoxel & colorVoxel = chunk->colors;
        
        float originX,originY,originZ;
        float roundingFactorX = 1.0f / chunksNum(0);
        float roundingFactorY = 1.0f / chunksNum(1);
        float roundingFactorZ = 1.0f / chunksNum(2);

        float chunkSize0 = (float)chunksNum(0);
        float chunkSize1 = (float)chunksNum(1);
        float chunkSize2 = (float)chunksNum(2);
        float point5 = 0.5;
        originX = originInRefSubmap(0);
        originY = originInRefSubmap(1);
        originZ = originInRefSubmap(2);
        
        std::vector<int> chunkIDX(8);
        std::vector<int> chunkIDY(8);
        std::vector<int> chunkIDZ(8);

        std::vector<int> voxelID(8);
        __m256 point_5_simd =_mm256_broadcast_ss(&(point5));
        __m256 origin_simd0 =_mm256_broadcast_ss(&(originX));
        __m256 origin_simd1 =_mm256_broadcast_ss(&(originY));
        __m256 origin_simd2 =_mm256_broadcast_ss(&(originZ));
        __m256 resolution_simd = _mm256_broadcast_ss(&(resolution));
        __m256 roundingFactor0 =_mm256_broadcast_ss(&(roundingFactorX));
        __m256 roundingFactor1 =_mm256_broadcast_ss(&(roundingFactorY));
        __m256 roundingFactor2 =_mm256_broadcast_ss(&(roundingFactorZ));
        __m256 chunk_size_simd0 = _mm256_broadcast_ss(&(chunkSize0));
        __m256 chunk_size_simd1 = _mm256_broadcast_ss(&(chunkSize1));
        __m256 chunk_size_simd2 = _mm256_broadcast_ss(&(chunkSize2));
        __m256 chunk_size_xy = _mm256_mul_ps(chunk_size_simd0, chunk_size_simd1);
        __m256i chunk_size_simd0i = _mm256_cvtps_epi32(chunk_size_simd0);
        __m256i chunk_size_simd1i = _mm256_cvtps_epi32(chunk_size_simd1);
        __m256i chunk_size_simd2i = _mm256_cvtps_epi32(chunk_size_simd2);
        __m256i chunk_size_xyi = _mm256_cvtps_epi32(chunk_size_xy);

        float * voxelSDFPointer = (float * )(&voxel.sdf[0]);
        float * voxelWeightPointer = (float * )(&voxel.weight[0]);
        unsigned short *voxelColorPointer = (unsigned short *)(&colorVoxel.colorData[0]);

        int pos = 0;
        // Get the refchunk in advance.
        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x+=8)
                {
                    __m256 voxelCenterInRefSubmap_SIMD0 = _mm256_add_ps(origin_simd0, centroids_simd0[pos]);
                    __m256 voxelCenterInRefSubmap_SIMD1 = _mm256_add_ps(origin_simd1, centroids_simd1[pos]);
                    __m256 voxelCenterInRefSubmap_SIMD2 = _mm256_add_ps(origin_simd2, centroids_simd2[pos]);

                    __m256 world_point_SIMD0 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD0, resolution_simd));
                    __m256 world_point_SIMD1 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD1, resolution_simd)); 
                    __m256 world_point_SIMD2 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD2, resolution_simd));

                    __m256i chunk_id_SIMD0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0, roundingFactor0)));
                    __m256i chunk_id_SIMD1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1, roundingFactor1)));
                    __m256i chunk_id_SIMD2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2, roundingFactor2)));
                    
                    __m256i local_point_SIMD0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0), chunk_size_simd0)));
                    __m256i local_point_SIMD1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1), chunk_size_simd1)));
                    __m256i local_point_SIMD2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2), chunk_size_simd2)));
                    
                    __m256i voxel_id_SIMD = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0, 
                         _mm256_mullo_epi32(local_point_SIMD1, chunk_size_simd0i)));
                                            
                    // this can be done by SIMD acceleration
                    //std::cout<<sizeof(__m256i)<<" "<<sizeof(int) * chunkIDX.size()<<std::endl;
                    _mm256_storeu_si256((__m256i *)&chunkIDX[0], chunk_id_SIMD0); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[0], chunk_id_SIMD1);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[0], chunk_id_SIMD2);

                    _mm256_storeu_si256((__m256i *)&voxelID[0], voxel_id_SIMD);

                    for(int i = 0; i != 8; ++i)
                    {
                        if(voxelWeightPointer[pos+i] == 0) continue;

                        ChunkID ref_chunk_id = ChunkID(chunkIDX[i], chunkIDY[i], chunkIDZ[i]);
                        VoxelID ref_voxel_id = voxelID[i];
                        if(chunks.find(ref_chunk_id) == chunks.end())
                        continue;
                        auto &ref_chunk = chunks[ref_chunk_id];
                        ref_chunk->voxels.sdf[ref_voxel_id] = voxelSDFPointer[pos*8+i];
                        ref_chunk->voxels.weight[ref_voxel_id] = voxelWeightPointer[pos*8+i];
                        memcpy(&ref_chunk->colors.colorData[ref_voxel_id*4], &voxelColorPointer[(pos*8+i)*4], sizeof(unsigned short)*4  ); 
                    }
                    ++pos;
                }
            }
        }

    }
    bool ProjectionIntegrator::voxelUpdateSIMD(ChunkMap &chunks, const Transform &pose, Chunk *chunk) const
    {
        bool updated = false;
        if(chunks.size() == 0) return false;
        float resolution = chunk->GetVoxelResolutionMeters();
        Eigen::Vector3i chunksNum = chunk->GetNumVoxels();
        int pos = 0;

        Vec3 origin = chunk->GetOrigin();
        Vec3 originInRefSubmap = pose.linear().transpose() * (origin - pose.translation());
        //we can get the sdf in advance    

        DistVoxel & voxel = chunk->voxels;
        ColorVoxel & colorVoxel = chunk->colors;
        
        Chunk ref_chunk(chunk->ID, chunksNum, resolution, chunk->useColor);// will be released

        DistVoxel &ref_voxel  =ref_chunk.voxels;
        ColorVoxel &ref_colorVoxel = ref_chunk.colors; 


        float originX,originY,originZ;
        float point5 = 0.5;
        float roundingFactorX = 1.0f / chunksNum(0);
        float roundingFactorY = 1.0f / chunksNum(1);
        float roundingFactorZ = 1.0f / chunksNum(2);
        //float point_test_1 = 3.1f;
        //float point_test_2 = 3.5f;
        //float point_test_3 = 3.8f;

        float chunkSize0 = (float)chunksNum(0);
        float chunkSize1 = (float)chunksNum(1);
        float chunkSize2 = (float)chunksNum(2);

        originX = originInRefSubmap(0);
        originY = originInRefSubmap(1);
        originZ = originInRefSubmap(2);
        
        std::vector<int> chunkIDX(8);
        std::vector<int> chunkIDY(8);
        std::vector<int> chunkIDZ(8);

        std::vector<int> voxelID(8);
        /*
        __m256 point_test_1_simd = _mm256_broadcast_ss(&(point_test_1));
        __m256 point_test_3_simd = _mm256_broadcast_ss(&(point_test_3));
        __m256i point_test_1_i = _mm256_cvtps_epi32(point_test_1_simd);
        __m256i point_test_3_i = _mm256_cvtps_epi32(point_test_3_simd);
        _mm256_storeu_si256((__m256i *)&chunkIDX[0], point_test_1_i); 
        _mm256_storeu_si256((__m256i *)&chunkIDY[0], point_test_3_i);
        */
        //std::cout<<"test: "<<chunkIDX[0]<<" "<<chunkIDY[0]<<std::endl;
        //while(true);
        __m256 origin_simd0 =_mm256_broadcast_ss(&(originX));
        __m256 origin_simd1 =_mm256_broadcast_ss(&(originY));
        __m256 origin_simd2 =_mm256_broadcast_ss(&(originZ));
        __m256 point_5_simd =_mm256_broadcast_ss(&(point5));
        __m256 resolution_simd = _mm256_broadcast_ss(&(resolution));
        __m256 roundingFactor0 =_mm256_broadcast_ss(&(roundingFactorX));
        __m256 roundingFactor1 =_mm256_broadcast_ss(&(roundingFactorY));
        __m256 roundingFactor2 =_mm256_broadcast_ss(&(roundingFactorZ));
        __m256 chunk_size_simd0 = _mm256_broadcast_ss(&(chunkSize0));
        __m256 chunk_size_simd1 = _mm256_broadcast_ss(&(chunkSize1));
        __m256 chunk_size_simd2 = _mm256_broadcast_ss(&(chunkSize2));
        __m256 chunk_size_xy = _mm256_mul_ps(chunk_size_simd0, chunk_size_simd1);
        
        __m256i chunk_size_simd0i = _mm256_cvtps_epi32(chunk_size_simd0);
        __m256i chunk_size_simd1i = _mm256_cvtps_epi32(chunk_size_simd1);
        __m256i chunk_size_simd2i = _mm256_cvtps_epi32(chunk_size_simd2);
        __m256i chunk_size_xyi = _mm256_cvtps_epi32(chunk_size_xy);

        float * voxelSDFPointer = (float * )(&voxel.sdf[0]);
        float * voxelWeightPointer = (float * )(&voxel.weight[0]);
        unsigned short *voxelColorPointer = (unsigned short *)(&colorVoxel.colorData[0]);


        float * refVoxelSDFPointer = (float * )(&ref_voxel.sdf[0]);
        float * refVoxelWeightPointer = (float * )(&ref_voxel.weight[0]);
        unsigned short *refVoxelColorPointer = (unsigned short *)(&ref_colorVoxel.colorData[0]);
        // Get the refchunk in advance.
        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x+=8)
                {

                    __m256 voxelCenterInRefSubmap_SIMD0 = _mm256_add_ps(origin_simd0, centroids_simd0[pos]);
                    __m256 voxelCenterInRefSubmap_SIMD1 = _mm256_add_ps(origin_simd1, centroids_simd1[pos]);
                    __m256 voxelCenterInRefSubmap_SIMD2 = _mm256_add_ps(origin_simd2, centroids_simd2[pos]);

                    __m256 world_point_SIMD0 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD0, resolution_simd));
                    __m256 world_point_SIMD1 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD1, resolution_simd)); 
                    __m256 world_point_SIMD2 =  _mm256_floor_ps(_mm256_div_ps(voxelCenterInRefSubmap_SIMD2, resolution_simd));

                    __m256i chunk_id_SIMD0 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD0, roundingFactor0)));
                    __m256i chunk_id_SIMD1 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD1, roundingFactor1)));
                    __m256i chunk_id_SIMD2 = _mm256_cvtps_epi32(_mm256_floor_ps(_mm256_mul_ps(world_point_SIMD2, roundingFactor2)));
                    
                    __m256i local_point_SIMD0 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD0, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD0), chunk_size_simd0)));
                    __m256i local_point_SIMD1 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD1, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD1), chunk_size_simd1)));
                    __m256i local_point_SIMD2 = _mm256_cvtps_epi32(_mm256_sub_ps(world_point_SIMD2, _mm256_mul_ps(_mm256_cvtepi32_ps(chunk_id_SIMD2), chunk_size_simd2)));
                    
                    __m256i voxel_id_SIMD = _mm256_add_epi32(_mm256_mullo_epi32(local_point_SIMD2, chunk_size_xyi), _mm256_add_epi32(local_point_SIMD0, 
                         _mm256_mullo_epi32(local_point_SIMD1, chunk_size_simd0i)));
                                            
                    // this can be done by SIMD acceleration
                    //std::cout<<sizeof(__m256i)<<" "<<sizeof(int) * chunkIDX.size()<<std::endl;
                    _mm256_storeu_si256((__m256i *)&chunkIDX[0], chunk_id_SIMD0); 
                    _mm256_storeu_si256((__m256i *)&chunkIDY[0], chunk_id_SIMD1);
                    _mm256_storeu_si256((__m256i *)&chunkIDZ[0], chunk_id_SIMD2);

                    _mm256_storeu_si256((__m256i *)&voxelID[0], voxel_id_SIMD);

                    for(int i = 0; i != 8; ++i)
                    {
                        ChunkID ref_chunk_id = ChunkID(chunkIDX[i], chunkIDY[i], chunkIDZ[i]);
                        VoxelID ref_voxel_id = voxelID[i];
                        //std::cout<<ref_voxel_id<<std::endl;
                        if(chunks.find(ref_chunk_id) == chunks.end())
                        continue;
                        updated = true;
                        auto &tmp_chunk = chunks[ref_chunk_id];
                        refVoxelSDFPointer[pos*8 + i] = tmp_chunk->voxels.sdf[ref_voxel_id];
                        refVoxelWeightPointer[pos*8 + i] = tmp_chunk->voxels.weight[ref_voxel_id];
                        memcpy(&refVoxelColorPointer[(pos*8 + i)*4], &tmp_chunk->colors.colorData[ref_voxel_id*4], sizeof(unsigned short)*4); 
                    }
                    ++pos;
                }
            }
        }
        if(updated == false) 
        {
            //std::cout<<"no chunk need to be updated."<<std::endl;
            return updated;
        }
        // integrate two chunk 
        pos = 0;
        __m256 sigma_simd = _mm256_set1_ps(1e-4);
        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x+=8)   //Each time we process 8 voxels
                {
                    /*integrate color*/
                    __m256i inputColorSIMD[2];
                    __m256i voxelColorShortSIMD[2];
                    voxelColorShortSIMD[0] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32]);
                    voxelColorShortSIMD[1] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32 + 16]);
                    inputColorSIMD[0] = _mm256_load_si256((__m256i *)&refVoxelColorPointer[pos*32]);
                    inputColorSIMD[1] = _mm256_load_si256((__m256i *)&refVoxelColorPointer[pos*32 + 16]);
                    __m256i inputColorShortSIMD;
                    __m256i updatedColorSIMD;
                    for(int i = 0; i < 2; i++)
                    {
                        inputColorShortSIMD = inputColorSIMD[i];
                        updatedColorSIMD = _mm256_add_epi16(voxelColorShortSIMD[i],inputColorShortSIMD);
                        // boundary check!
                        __m256i saturationFlag = _mm256_cmpgt_epi16(updatedColorSIMD,_mm256_set1_epi16(120));
                        saturationFlag = _mm256_shufflehi_epi16(saturationFlag,255);
                        saturationFlag = _mm256_shufflelo_epi16(saturationFlag,255);
                        updatedColorSIMD = _mm256_blendv_epi8(updatedColorSIMD,
                                                                _mm256_srli_epi16(updatedColorSIMD,2),
                                                                saturationFlag);
                        _mm256_store_si256((__m256i *)&voxelColorPointer[pos * 32 + 16*i],updatedColorSIMD);
                    }
                    /*integrate sdf and weight*/
                    __m256 weightSIMD = _mm256_loadu_ps(&voxelWeightPointer[pos * 8]);
                    __m256 sdfSIMD = _mm256_loadu_ps(&voxelSDFPointer[pos*8]);
                    __m256 newWeightSIMD = _mm256_loadu_ps(&refVoxelWeightPointer[pos*8]);//_mm256_blendv_ps(defaultDepth_simd,weight_SIMD,integrateFlag);
                    __m256 newSdfSIMD = _mm256_loadu_ps(&refVoxelSDFPointer[pos*8]);
                    __m256 updatedSdfSIMD = _mm256_div_ps(_mm256_add_ps(_mm256_mul_ps(sdfSIMD,weightSIMD),_mm256_mul_ps(newSdfSIMD,newWeightSIMD)),
                                            _mm256_add_ps(_mm256_add_ps(weightSIMD,newWeightSIMD),sigma_simd));
                    __m256 updatedWeightSIMD = _mm256_add_ps(weightSIMD,newWeightSIMD);
                    __m256 weightValidSIMD =  _mm256_cmp_ps(updatedWeightSIMD,_mm256_set1_ps(0.5),_CMP_GT_OS);
                    updatedSdfSIMD = _mm256_blendv_ps(_mm256_set1_ps(999),updatedSdfSIMD,weightValidSIMD);
                    updatedWeightSIMD = _mm256_blendv_ps(_mm256_set1_ps(0.0f),updatedWeightSIMD,weightValidSIMD);

                    _mm256_storeu_ps(&voxelWeightPointer[pos*8], updatedWeightSIMD);
                    _mm256_storeu_ps(&voxelSDFPointer[pos*8],updatedSdfSIMD);    
                    pos ++;
                }
            }
        }    
        return updated;

    }
    bool ProjectionIntegrator::voxelUpdateSIMD(const float *depth_pointer,
                                               unsigned char * colorImage,
                                               const PinholeCamera& depthCamera,
                                               const Transform& depthCameraPose,
                                               int integrateFlag,
                                               Chunk * chunk,
                                               float *weight_pointer) const
    {

        bool updated = false;
        float resolution = chunk->GetVoxelResolutionMeters();
        Eigen::Vector3i chunksNum = chunk->GetNumVoxels();
        Vec3 origin = chunk->GetOrigin();
        float resolutionDiagonal = sqrt(3.0f) * resolution;
        float fx,fy,cx,cy,width,height;
        fx = depthCamera.GetFx();
        fy = depthCamera.GetFy();
        cx = depthCamera.GetCx();
        cy = depthCamera.GetCy();
        width = depthCamera.GetWidth();
        height = depthCamera.GetHeight();

        float depthFar = depthCamera.GetFarPlane();
        float depthNear = depthCamera.GetNearPlane();
        Vec3 originInCamera = depthCameraPose.linear().transpose() * (origin - depthCameraPose.translation());
        float truncation = 0.15;
        //truncation = truncator->GetTruncationDistance(originInCamera(2));
        //std::cout<<"truncation: " << truncation << std::endl;
        
        float depth_weight = weighter->GetWeight(1.0f, truncation);
        float color_weight = 1;
        __m256 weightSignSIMD = _mm256_set1_ps(1);
        if(!integrateFlag)
        {
            weightSignSIMD = _mm256_set1_ps(-1);
            depth_weight *= -1.0f;
            color_weight *= -1;
        }
        float threshold_dist = truncation + resolutionDiagonal;
        float threshold_color = resolutionDiagonal + 0.01;
        int NumOfVoxels = centroids.size();


        float originX,originY,originZ;
        originX = originInCamera(0);
        originY = originInCamera(1);
        originZ = originInCamera(2);
        __m256 origin_simd0 =_mm256_broadcast_ss(&(originX));
        __m256 origin_simd1 =_mm256_broadcast_ss(&(originY));
        __m256 origin_simd2 =_mm256_broadcast_ss(&(originZ));

        __m256 fx_simd =_mm256_set1_ps((fx));
        __m256 fy_simd =_mm256_set1_ps((fy));
        __m256 cx_simd =_mm256_set1_ps((cx + 0.5));
        __m256 cy_simd =_mm256_set1_ps((cy + 0.5));

        __m256i leftThreshold_simd =_mm256_set1_epi32(1);
        __m256i rightThreshold_simd =_mm256_set1_epi32(width - 1);
        __m256i topThreshold_simd =_mm256_set1_epi32(1);
        __m256i bottomThreshold_simd =_mm256_set1_epi32(height - 1);

        __m256i width_simd = _mm256_set_epi32(width,width,width,width,width,width,width,width);


        __m256 defaultDepth_simd = _mm256_set1_ps(0.0);
        __m256 sigma_simd = _mm256_set1_ps(1e-4);

        __m256 minDepth_simd = _mm256_set1_ps(depthNear);
        __m256 maxDepth_simd = _mm256_set1_ps(depthFar);
        __m256 inputWeight_simd = _mm256_set1_ps(depth_weight);

        bool updated_flag = false;

        DistVoxel& voxel = chunk->voxels;
        ColorVoxel &colorVoxel = chunk->colors;
        float * voxelSDFPointer = (float * )(&voxel.sdf[0]);
        float * voxelWeightPointer = (float * )(&voxel.weight[0]);
        unsigned short *voxelColorPointer = (unsigned short *)(&colorVoxel.colorData[0]);


        // use _mm256_mmask_i32gather_ps to load depth information
        // use _mm256_cmp_ps_mask to store the information
        // use floor to save data
        int pos = 0;
        for (int z = 0; z < chunksNum(2); z++)
        {
            for(int y = 0; y < chunksNum(1); y++)
            {
                for(int x = 0; x < chunksNum(0); x+=8)
                {
                    __m256 voxelCenterInCamera_SIMD0 = _mm256_add_ps(origin_simd0, centroids_simd0[pos]);
                    __m256 voxelCenterInCamera_SIMD1 = _mm256_add_ps(origin_simd1, centroids_simd1[pos]);
                    __m256 voxelCenterInCamera_SIMD2 = _mm256_add_ps(origin_simd2, centroids_simd2[pos]);

                    __m256 cameraPos_SIMD0 =
                            _mm256_add_ps(_mm256_mul_ps(_mm256_div_ps(voxelCenterInCamera_SIMD0,voxelCenterInCamera_SIMD2),fx_simd),cx_simd);
                    __m256 cameraPos_SIMD1 =
                            _mm256_add_ps(_mm256_mul_ps(_mm256_div_ps(voxelCenterInCamera_SIMD1,voxelCenterInCamera_SIMD2),fy_simd),cy_simd);
                    __m256i cameraX_SIMD = _mm256_cvtps_epi32(cameraPos_SIMD0);
                    __m256i cameraY_SIMD = _mm256_cvtps_epi32(cameraPos_SIMD1);
                    __m256i valid = _mm256_and_si256(_mm256_cmpgt_epi32(cameraX_SIMD,leftThreshold_simd),_mm256_cmpgt_epi32(rightThreshold_simd,cameraX_SIMD));
                    valid = _mm256_and_si256(valid,_mm256_cmpgt_epi32(cameraY_SIMD,topThreshold_simd));
                    valid = _mm256_and_si256(valid,_mm256_cmpgt_epi32(bottomThreshold_simd,cameraY_SIMD));
                    __m256 validf = _mm256_castsi256_ps(valid);

                    
                    if(_mm256_testz_si256(valid, valid))
                    {
                        continue;
                    }
                    // load depth data and store them
                    __m256i camera_plane_pos_SIMD = _mm256_add_epi32(_mm256_mullo_epi32(cameraY_SIMD,width_simd),cameraX_SIMD);
                    __m256 depth_SIMD = _mm256_mask_i32gather_ps(defaultDepth_simd,depth_pointer,camera_plane_pos_SIMD,validf,4);



                    __m256 weight_SIMD = _mm256_set1_ps(depth_weight);
//                    if(weight_pointer != NULL)
//                    {
//                        weight_SIMD = _mm256_mask_i32gather_ps(defaultDepth_simd,weight_pointer,camera_plane_pos_SIMD,validf,4);
//                    }
                    __m256 surfaceDist_SIMD = _mm256_sub_ps(depth_SIMD,voxelCenterInCamera_SIMD2);
//                    _mm256_store_ps(loadSurfaceDistFromSIMD,surfaceDist_SIMD);
//                    _mm256_store_ps(loadDepthFromSIMD,depth_SIMD);


#if 1

#if 1
                    if(colorImage != NULL)
                    {
                        __m256i updateColor_SIMD =  _mm256_and_si256(valid,
                                                                    _mm256_cvtps_epi32(_mm256_and_ps(_mm256_cmp_ps(surfaceDist_SIMD,_mm256_set1_ps(-threshold_color),_CMP_GT_OS),
                                                                   _mm256_cmp_ps(_mm256_set1_ps(threshold_color),surfaceDist_SIMD,_CMP_GT_OS))));

                        if((!_mm256_testz_si256(updateColor_SIMD,updateColor_SIMD)) )
                        {
                            __m256i inputColorSIMD = _mm256_mask_i32gather_epi32(_mm256_set1_epi16(0),(const int*)colorImage,camera_plane_pos_SIMD,updateColor_SIMD,4);
                            __m256i voxelColorShortSIMD[2];
                            voxelColorShortSIMD[0] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32]);
                            voxelColorShortSIMD[1] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32 + 16]);
                            __m256i inputColorShortSIMD;
                            __m256i updatedColorSIMD;
                            if(integrateFlag)
                            {

                                for(int i = 0; i < 2; i++)
                                {
                                    //_mm256_cvtepu8_epi16 conversion 8 -> 16
                                    inputColorShortSIMD = _mm256_cvtepu8_epi16(_mm256_extracti128_si256(inputColorSIMD,i));
                                    updatedColorSIMD = _mm256_add_epi16(voxelColorShortSIMD[i],inputColorShortSIMD);
                                    // boundary check!
                                   __m256i saturationFlag = _mm256_cmpgt_epi16(updatedColorSIMD,_mm256_set1_epi16(120));
                                   saturationFlag = _mm256_shufflehi_epi16(saturationFlag,255);
                                   saturationFlag = _mm256_shufflelo_epi16(saturationFlag,255);
                                   updatedColorSIMD = _mm256_blendv_epi8(updatedColorSIMD,
                                                                         _mm256_srli_epi16(updatedColorSIMD,2),
                                                                         saturationFlag);

                                    _mm256_store_si256((__m256i *)&voxelColorPointer[pos * 32 + 16*i],updatedColorSIMD);
                                }
                            }
                            else
                            {

                                for(int i = 0; i < 2; i++)
                                {
                                    inputColorShortSIMD = _mm256_cvtepu8_epi16(_mm256_extracti128_si256(inputColorSIMD,i));
                                    updatedColorSIMD = _mm256_sub_epi16(voxelColorShortSIMD[i],inputColorShortSIMD);
                                    // boundary check! yet should always larger than 0 by default.
                                    _mm256_store_si256((__m256i *)&voxelColorPointer[pos * 32 + 16*i],updatedColorSIMD);


                                }
                            }
                        }
                    }

#endif

                    validf = _mm256_and_ps(_mm256_cmp_ps(depth_SIMD,minDepth_simd,_CMP_GT_OS),_mm256_cmp_ps(maxDepth_simd,depth_SIMD,_CMP_GT_OS));
                    __m256 surfaceInside_SIMD =  _mm256_and_ps(_mm256_cmp_ps(surfaceDist_SIMD,_mm256_set1_ps(-0.03),_CMP_GT_OS),
                                                               _mm256_cmp_ps(_mm256_set1_ps(truncation + resolutionDiagonal),surfaceDist_SIMD,_CMP_GT_OS));
                    __m256 integrateFlag = _mm256_and_ps(validf,surfaceInside_SIMD);

                    if((!_mm256_testz_ps(integrateFlag,integrateFlag)) )
                    {
                        updated = true;
                        __m256 weightSIMD = _mm256_loadu_ps(&voxelWeightPointer[pos * 8]);
                        __m256 sdfSIMD = _mm256_loadu_ps(&voxelSDFPointer[pos*8]);
                        __m256 newWeightSIMD = _mm256_blendv_ps(defaultDepth_simd,weight_SIMD,integrateFlag);

                        __m256 updatedSdfSIMD = _mm256_div_ps(_mm256_add_ps(_mm256_mul_ps(sdfSIMD,weightSIMD),_mm256_mul_ps(surfaceDist_SIMD,newWeightSIMD)),
                                                _mm256_add_ps(_mm256_add_ps(weightSIMD,newWeightSIMD),sigma_simd));
                        __m256 updatedWeightSIMD = _mm256_add_ps(weightSIMD,newWeightSIMD);
                        __m256 weightValidSIMD =  _mm256_cmp_ps(updatedWeightSIMD,_mm256_set1_ps(0.5),_CMP_GT_OS);
                        updatedSdfSIMD = _mm256_blendv_ps(_mm256_set1_ps(999),updatedSdfSIMD,weightValidSIMD);
                        updatedWeightSIMD = _mm256_blendv_ps(_mm256_set1_ps(0.0f),updatedWeightSIMD,weightValidSIMD);

                        _mm256_storeu_ps(&voxelWeightPointer[pos*8], updatedWeightSIMD);
                        _mm256_storeu_ps(&voxelSDFPointer[pos*8],updatedSdfSIMD);

                    }
#if 0
                    for(int i = 0; i < 8; i++)
                    {
                        if(std::isnan(updatedSdfSIMD[i]) || ((updatedWeightSIMD[i]) < 1.5 && fabs(updatedSdfSIMD[i]) < 1e-3))
                        {
                            printf("weight: %f %f\r\n", weightSIMD[i], newWeightSIMD[i]);
                            printf("sdf: %f %f\r\n", sdfSIMD[i], surfaceDist_SIMD[i]);
                            printf("newWeight: %f %f\r\n", updatedSdfSIMD[i],updatedWeightSIMD[i]);
                            printf("pos: %d %d %d %d\r\n",
                                   chunk->GetID()(0),
                                   chunk->GetID()(1),
                                   chunk->GetID()(2),
                                   pos * 8 + i);

                            while(1)
                            {

                            }
                        }
                    }
#endif

#else
                    for(int i = 0; i < 8; i++)
                    {
                        int voxel_index = pos * 8 + i;
                        float surfaceDist = surfaceDist_SIMD[i];
                        float weight = weight_SIMD[i];
//                        std::cout << "depth_SIMD: " << depth_SIMD[i] << std::endl;




//                        if(chunk->GetID()(0) == 23 && chunk->GetID()(1) == 19 && chunk->GetID()(2) == 53 && voxel_index == 121)
//                        {
//                            printf("%f %f %d %f %f\r\n", surfaceDist,weight,integrateFlag,voxel.GetSDF(voxel_index), voxel.GetWeight(voxel_index));
//                        }

                        if ((surfaceDist < truncation + resolutionDiagonal && surfaceDist > -0.05)
                                && depth_SIMD[i] < depthFar
                                && depth_SIMD[i] > depthNear)
                        {
        #if 1
                            ColorVoxel& colorVoxel = chunk->colors;
                            int *cameraIndex = (int *)&camera_plane_pos_SIMD;
                            int cameraPose = cameraIndex[i];
                            if (cameraPose > 0 && cameraPose < width * height && std::abs(surfaceDist) < 0.005 + resolutionDiagonal)
                            {
                                colorVoxel.Integrate(colorImage[cameraPose * 4 + 2], colorImage[cameraPose * 4 + 1], colorImage[cameraPose * 4], 1, voxel_index);
                            }
        #endif

                            // decrease the weight of unknown part
                            if(surfaceDist < -0.02)
                            {
                                weight *= 0.1;
                            }

                            voxel.Integrate(surfaceDist, weight, voxel_index);

                            if(voxel.GetWeight(voxel_index) < 0.1)
                            {
                                voxel.SetSDF(999,voxel_index);
                                voxel.SetWeight(0,voxel_index);
                            }



                            if(voxel.GetWeight(voxel_index) > 0.01 && voxel.GetSDF(voxel_index) > 10)
                            {
                                voxel.SetSDF(999,voxel_index);
                                voxel.SetWeight(0,voxel_index);
//                                std::cout << "wrong voxel! " << voxel_index << " " << surfaceDist << " " << weight << std::endl;
                            }
                            updated = true;
                        }

//                        if(surfaceDist >= 0.2)
//                        {
//                            voxel.SetWeight(voxel.GetWeight(voxel_index)-weight, voxel_index);
//                            if(voxel.GetWeight(voxel_index) < 0.1)
//                            {
//                                voxel.SetSDF(999,voxel_index);
//                                voxel.SetWeight(0,voxel_index);
//                            }
//                            updated = true;
//                        }
                    }

#endif
                    // integrate voxel or color

                    // calculate surface distance

                    // do voxel integration and curve here.

                    //
                    pos ++;
                }
            }
        }
        return updated;
    }


    ProjectionIntegrator::~ProjectionIntegrator()
    {
        // TODO Auto-generated destructor stub
        _mm_free(centroids_simd0);
        _mm_free(centroids_simd1);
        _mm_free(centroids_simd2);
    }

} // namespace chisel 
