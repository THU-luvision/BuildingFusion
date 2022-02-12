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

#ifndef CHUNK_H_
#define CHUNK_H_

#include <memory>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Core>
#include<sstream>
#include <open_chisel/geometry/AABB.h>
#include "DistVoxel.h"
#include "ColorVoxel.h"
#include "half.hpp"
#include <xmmintrin.h>
#include <smmintrin.h>
#include <immintrin.h>

using namespace half_float;


namespace chisel
{

    typedef Eigen::Vector3i ChunkID;
    typedef int VoxelID;
    typedef std::vector<ChunkID, Eigen::aligned_allocator<ChunkID> > ChunkIDList;
    
    struct ChunkStatistics
    {
            size_t numKnownInside;
            size_t numKnownOutside;
            size_t numUnknown;
            float totalWeight;
    };
    template<typename T>
    struct matrix_hash : std::unary_function<T, size_t> {
    std::size_t operator()(T const& matrix) const {
    // Note that it is oblivious to the storage order of Eigen matrix (column- or
    // row-major). It will give you the same hash value for two different matrices if they
    // are the transpose of each other in different storage order.
        size_t seed = 0;
        for (size_t i = 0; i < matrix.size(); ++i) {
         auto elem = *(matrix.data() + i);
        seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
        }
    };
    float reduceTo16(const float &a);
    Vec3 toHalfVec3(const Vec3 & v);


    typedef std::unordered_map<Vec3, std::vector<float>, matrix_hash<Vec3>> VoxelMap;
    typedef std::unordered_map<Vec3,int,matrix_hash<Vec3>> VoxelIndex;
    typedef std::unordered_set<Point3,matrix_hash<Point3>> PointSet;

    class Chunk
    {
        public:
            Chunk();
            Chunk(const ChunkID id, const Eigen::Vector3i& numVoxels, float resolution, bool useColor);
            virtual ~Chunk();
            Chunk(const Chunk &c)
            {
                
                voxels = c.voxels;
                colors = c.colors;
                origin = c.origin;
             p_normal=c.p_normal;
             p_color = c.p_color;
             p_depth=c.p_depth;
             pos = c.pos;
             frameIndex = c.frameIndex;
            }
            Chunk &operator= (const Chunk &c)
            {
                
                voxels = c.voxels;
                colors = c.colors;
                origin = c.origin;
             p_normal=c.p_normal;
             p_color = c.p_color;
             p_depth=c.p_depth;
             pos = c.pos;
             frameIndex = c.frameIndex;
                return *this;
            }
            void resetChunk()
            {
                AllocateColorVoxels();
                AllocateDistVoxels();
            }

            void AllocateDistVoxels();
            void AllocateColorVoxels();

            inline const ChunkID& GetID() const { return ID; }
            inline ChunkID& GetIDMutable() { return ID; }
            inline void SetID(const ChunkID& id) { ID = id; }

            inline bool HasColors() const { return !(colors.colorData == NULL); }
            inline bool HasVoxels() const { return !voxels.sdf.empty(); }
            inline const DistVoxel & GetVoxels() const { return voxels; }

            inline const Eigen::Vector3i& GetNumVoxels() const { return numVoxels; }
            inline float GetVoxelResolutionMeters() const { return voxelResolutionMeters; }

//            inline const DistVoxel& GetDistVoxel(const VoxelID& voxelID) const { return voxels.at(voxelID); }
//            inline DistVoxel& GetDistVoxelMutable(const VoxelID& voxelID) { return voxels.at(voxelID); }
//            inline const ColorVoxel& GetColorVoxel(const VoxelID& voxelID) const { return colors.at(voxelID); }
//            inline ColorVoxel& GetColorVoxelMutable(const VoxelID& voxelID) { return colors.at(voxelID); }

            Point3 GetVoxelCoords(const Vec3& worldCoords) const;

            inline VoxelID GetVoxelID(const Point3& coords) const
            {
                return GetVoxelID(coords.x(), coords.y(), coords.z());
            }

            inline VoxelID GetVoxelID(int x, int y, int z) const
            {
                return (z * numVoxels(1) + y) * numVoxels(0) + x;
            }

//            inline const DistVoxel& GetDistVoxel(int x, int y, int z) const
//            {
//                return GetDistVoxel(GetVoxelID(x, y, z));
//            }

//            inline DistVoxel& GetDistVoxelMutable(int x, int y, int z)
//            {
//                return GetDistVoxelMutable(GetVoxelID(x, y, z));
//            }

//            inline const ColorVoxel& GetColorVoxel(int x, int y, int z) const
//            {
//                return GetColorVoxel(GetVoxelID(x, y, z));
//            }

//            inline ColorVoxel& GetColorVoxelMutable(int x, int y, int z)
//            {
//                return GetColorVoxelMutable(GetVoxelID(x, y, z));
//            }

            inline bool IsCoordValid(VoxelID idx) const
            {
                return idx >= 0 && idx < voxels.sdf.size();
            }
            size_t getOccupyMemory()
            {
                size_t result =  voxels.sdf.size()*sizeof(float) + voxels.weight.size()*sizeof(float);
                result += 4096;
            result += sizeof(ChunkID) + sizeof(Eigen::Vector3i) + sizeof(float)+sizeof(bool) + sizeof(int )+sizeof(pos) +sizeof(float *)*2 + sizeof(unsigned char *);
                //result += GetTotalNumVoxels()*(sizeof(float)*2 + sizeof(unsigned char));
                return result;
            }
            inline bool IsCoordValid(int x, int y, int z) const
            {
                return (x >= 0 && x < numVoxels(0) && y >= 0 && y < numVoxels(1) && z >= 0 && z < numVoxels(2));
            }


            inline size_t GetTotalNumVoxels() const
            {
                return numVoxels(0) * numVoxels(1) * numVoxels(2);
            }


            void ComputeStatistics(ChunkStatistics* stats);

            AABB ComputeBoundingBox();

            inline const Vec3& GetOrigin() { return origin; }
            void computeOrigin(){  origin = Vec3(numVoxels(0) * ID(0) * voxelResolutionMeters, numVoxels(1) * ID(1) * voxelResolutionMeters, numVoxels(2) * ID(2) * voxelResolutionMeters);}
            Vec3 GetColorAt(const Vec3& pos);

            VoxelID GetVoxelID(const Vec3& relativePos) const;
            VoxelID GetLocalVoxelIDFromGlobal(const Point3& worldPoint) const;
            Point3 GetLocalCoordsFromGlobal(const Point3& worldPoint) const;
            Vec3 GetVoxelCoordsFromVoxelID(int voxelID) const;
            VoxelMap GetworldCoords();
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW




            void UpdateReferenceFrame(const int keyframeIndex,
                                   const Mat4x4 inputPos,
                                   const float *normalPointer,
                                   const unsigned char * colorPointer,
                                   const float *depthPointer)
            {
                frameIndex = keyframeIndex;
                pos = inputPos;
                p_normal = normalPointer;
                p_color = colorPointer;
                p_depth = depthPointer;
            }


            Mat4x4 GetReferenceFramePose()
            {
                return pos;
            }

            void GetReferenceFrame(int &keyframeIndex,
                                   Mat4x4 &inputPos,
                                   const float * &normalPointer,
                                   const unsigned char * &colorPointer,
                                   const float * &depthPointer)
            {
                keyframeIndex = frameIndex;
                inputPos = pos;
                normalPointer = p_normal;
                colorPointer = p_color;
                depthPointer = p_depth;
            }

            int GetReferenceFrameIndex() {return frameIndex;}

           
            void integrateTwoChunksSIMD(std::shared_ptr<Chunk> cp)
            {
            int pos = 0;
            unsigned short *voxelColorPointer = (unsigned short *)(&colors.colorData[0]);
            unsigned short *inputVoxelColorPointer = (unsigned short *)(&cp->colors.colorData[0]);
            float * voxelSDFPointer = (float * )(&voxels.sdf[0]);
            float * voxelWeightPointer = (float * )(&voxels.weight[0]);
            float * inputVoxelSDFPointer = (float * )(&cp->voxels.sdf[0]);
            float * inputVoxelWeightPointer = (float * )(&cp->voxels.weight[0]);
            __m256 sigma_simd = _mm256_set1_ps(1e-4);
            for (int z = 0; z < numVoxels(2); z++)
            {
                for(int y = 0; y < numVoxels(1); y++)
                {
                for(int x = 0; x < numVoxels(0); x+=8)   //Each time we process 8 voxels
                    {
                    /*integrate color*/
                    __m256i inputColorSIMD[2];
                    __m256i voxelColorShortSIMD[2];
                    voxelColorShortSIMD[0] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32]);
                    voxelColorShortSIMD[1] = _mm256_load_si256((__m256i *)&voxelColorPointer[pos * 32 + 16]);
                    inputColorSIMD[0] = _mm256_load_si256((__m256i *)&inputVoxelColorPointer[pos*32]);
                    inputColorSIMD[1] = _mm256_load_si256((__m256i *)&inputVoxelColorPointer[pos*32 + 16]);
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
                        __m256 newWeightSIMD = _mm256_loadu_ps(&inputVoxelWeightPointer[pos*8]);//_mm256_blendv_ps(defaultDepth_simd,weight_SIMD,integrateFlag);
                        __m256 newSdfSIMD = _mm256_loadu_ps(&inputVoxelSDFPointer[pos*8]);
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

#if 0
            void _integrateTwoChunks(ChunkPtr cp)
            {
                int totalNum = GetTotalNumVoxels();
                unsigned short *voxelColorPointer = (unsigned short *)(&colors.colorData[0]);
                for(int i = 0;i!=totalNum;++i)
                {


                    float sdf1 = voxels.GetSDF(i);
                    float sdf2 = cp->voxels.GetSDF(i);
                    float w1 = voxels.GetWeight(i);
                    float w2 = cp->voxels.GetWeight(i);
                    unsigned short r1 = colors.getRed(i);
                    unsigned short g1 = colors.getGreen(i);
                    unsigned short b1 = colors.getBlue(i);
                    unsigned short d1 = colors.getDivision(i);
                    unsigned short r2 = cp->colors.getRed(i);
                    unsigned short g2 = cp->colors.getGreen(i);
                    unsigned short b2 = cp->colors.getBlue(i);
                    unsigned short d2 = cp->colors.getDivision(i);                    
                    if(w1 == 0|| sdf1 >= 999)
                    {
                    voxels.SetSDF(sdf2,i);
                    voxels.SetWeight(w2,i);
                    colors.setRed(r2,i);
                    colors.setBlue(b2,i);
                    colors.setGreen(g2,i);
                    colors.setDivision(d2,i);
                    }
                    else if(w2 == 0 || sdf2 >= 999)
                    {
                    voxels.SetSDF(sdf1,i);
                    voxels.SetWeight(w1,i);
                    colors.setRed(r1,i);
                    colors.setBlue(b1,i);
                    colors.setGreen(g1,i);
                    colors.setDivision(d1,i);                        
                    }
                    else
                    {
                    voxels.SetSDF((sdf1*w1 + sdf2*w2)/(w1+w2),i);
                    voxels.SetWeight(w1+w2,i);
                      
                    }

                }
            }
#endif

            DistVoxel voxels;
            ColorVoxel colors;

            std::string toString()
            {
                std::ostringstream os;
                std::vector<int> index;
                os<<"ID:\n"<<ID(0)<<" "<<ID(1)<<" "<<ID(2)<<"\n";
                /* this is no  need to store those shit. */
                //os<<"resolution:\n"<<voxelResolutionMeters<<"\n";
                //os<<"numVoxels:\n"<<numVoxels(0)<<" "<<numVoxels(1)<<" "<<numVoxels(2)<<"\n";
                //os<<"useColor:\n"<<useColor<<"\n";
                os<<"sdf:\n";
                os<<voxels.toString(index)<<"\n";
                os<<"color:\n";
                os<<colors.toString(index)<<"\n";
                //os<<"pos:\n";
                //os<<pos;
            /*
            Output ID, resolution, useColor, sdf, color,pos, not sure if the p_color and p_depth is needed to be stored.
            The frame store-scheme is really a big problem.
            */
            return os.str();
            }

            
            void toVector(std::vector<float> &buffer)
            {
                std::vector<int> index;
                buffer.push_back(ID(0));
                buffer.push_back(ID(1));
                buffer.push_back(ID(2));
                voxels.toVector(index,buffer);
                colors.toVector(index,buffer);
            }
            void fromStream(std::istream & is)
            {
                //std::istringstream is(s);
                std::string attri;
                std::string content;
                int i = 0;
                while(i<3 && getline(is,attri))
                {
                    /*this is about load the chunk from stream*/
                    ++i;
                    getline(is,content);
                    std::istringstream isc(content);
                    if(attri == "ID:")
                    {
                     int x,y,z;
                     isc >> x>>y>>z;
                    ID(0) = x;
                    ID(1) = y;
                    ID(2) = z;
                    //std::cout<<ID<< std::endl;
                    }
                    if(attri == "sdf:")
                    {
                        voxels.fromStream(isc);
                    //std::cout<<"voxel: "<<voxels.toString(index)<< std::endl;
                    }
                    if(attri == "color:")
                    {
                        colors.fromStream(isc);
                    //std::cout<<"color: "<<colors.toString(index)<< std::endl;
                    }
                }
            }
            void fromVector(std::vector<float> &buffer,size_t &ptr)
            {
                ID(0) = buffer[ptr++];
                ID(1) = buffer[ptr++];
                ID(2) = buffer[ptr++];
                voxels.fromVector(buffer,ptr);
                colors.fromVector(buffer,ptr);
            }

            ChunkID ID;
            Eigen::Vector3i numVoxels;
            float voxelResolutionMeters;
            Vec3 origin;
            bool useColor;
        protected:
            int frameIndex;
            Eigen::Matrix4f pos;
            const float * p_normal;
            const unsigned char * p_color;
            const float * p_depth;

    };
                                                                                                                                                                                                                                                                                                                                                                                                                                                                    


} // namespace chisel 

#endif // CHUNK_H_ 
