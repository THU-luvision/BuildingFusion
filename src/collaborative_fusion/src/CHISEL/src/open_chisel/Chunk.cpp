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

#include <open_chisel/Chunk.h>
#include <unordered_map>
namespace chisel
{
 float reduceTo16(const float &a)
        {     
            half f=half_cast<half,std::round_to_nearest> (a);
           return float(f);
            }
 Vec3 toHalfVec3(const Vec3 & v)
{
    return  Vec3(reduceTo16(v(0)),reduceTo16(v(1)),reduceTo16(v(2)));
}

    Chunk::Chunk() :
            voxelResolutionMeters(0)
    {
        // TODO Auto-generated constructor stub

        frameIndex = -1;
        p_normal == NULL;
        p_color == NULL;

    }

    Chunk::Chunk(const ChunkID id, const Eigen::Vector3i& nv, float r, bool _useColor) :
            ID(id), numVoxels(nv), voxelResolutionMeters(r),useColor(_useColor)
    {
        AllocateDistVoxels();

        if(useColor)
        {
            AllocateColorVoxels();
        }

        frameIndex = -1;
        p_normal == NULL;
        p_color == NULL;

        origin = Vec3(numVoxels(0) * ID(0) * voxelResolutionMeters, numVoxels(1) * ID(1) * voxelResolutionMeters, numVoxels(2) * ID(2) * voxelResolutionMeters);
    }

    Chunk::~Chunk()
    {
       // std::cout<<"release chunk! "<<std::endl;
    }

    void Chunk::AllocateDistVoxels()
    {
        int totalNum = GetTotalNumVoxels();
        voxels.clear();
        voxels.sdf.assign(totalNum,999.0f);
        voxels.weight.assign(totalNum,0.0f);

//        voxels.resize(totalNum, DistVoxel());
    }

    void Chunk::AllocateColorVoxels()
    {
        int totalNum = GetTotalNumVoxels();

        memset(colors.colorData,0,totalNum * 4);
//        colors.colorData.assign(totalNum * 4, 0);
   //     colors.resize(totalNum, ColorVoxel());
    }

    AABB Chunk::ComputeBoundingBox()
    {
        Vec3 pos = origin;
        Vec3 size = numVoxels.cast<float>() * voxelResolutionMeters;
        return AABB(pos, pos + size);
    }

    Point3 Chunk::GetVoxelCoords(const Vec3& worldCoords) const
    {
        const float roundingFactorX = 1.0f / (voxelResolutionMeters);
        const float roundingFactorY = 1.0f / (voxelResolutionMeters);
        const float roundingFactorZ = 1.0f / (voxelResolutionMeters);

        return Point3( static_cast<int>(std::floor(worldCoords(0) * roundingFactorX)),
                       static_cast<int>(std::floor(worldCoords(1) * roundingFactorY)),
                       static_cast<int>(std::floor(worldCoords(2) * roundingFactorZ)));
    }
    VoxelMap Chunk::GetworldCoords() 
    {
        int totalNum = GetTotalNumVoxels();
        VoxelMap worldCoords;
        for(int i = 0;i!=totalNum;++i)
        {
            float sdf = this->voxels.GetSDF(i);
            //for efficiency, i just put the voxels, which have the valid sdf, into transformation.
            //Note that this truncation should be modified
            //std::cout<<"sdf: "<<sdf<<std::endl;
            //delete the voxel which sdf = 999, because of the dynamic truncator
            
            if(fabs(sdf) < 999)
            {
            //std::cout<<"voxel coord: "<<GetVoxelCoordsFromVoxelID(i)<<std::endl;
            worldCoords[GetVoxelCoordsFromVoxelID(i)] = std::vector<float>({sdf,this->voxels.GetWeight(i),this->colors.getRed(i),this->colors.getGreen(i),this->colors.getBlue(i),this->colors.getDivision(i)});
            }
        }
        return worldCoords;
    }
    Vec3 Chunk::GetVoxelCoordsFromVoxelID(int voxelID) const
    {
        int x = voxelID & 7;
        int y = (voxelID>>3)&7;
        int z = (voxelID>>6)&7; 
        return (Vec3(x,y,z)+Vec3(ID.x() * numVoxels.x(), ID.y() * numVoxels.y(), ID.z() * numVoxels.z()));//*voxelResolutionMeters;
    }
    VoxelID Chunk::GetVoxelID(const Vec3& relativePos) const
    {
        return GetVoxelID(GetVoxelCoords(relativePos));
    }

    VoxelID Chunk::GetLocalVoxelIDFromGlobal(const Point3& worldPoint) const
    {

        return GetVoxelID(GetLocalCoordsFromGlobal(worldPoint));
    }

    Point3 Chunk::GetLocalCoordsFromGlobal(const Point3& worldPoint) const
    {
        return (worldPoint - Point3(ID.x() * numVoxels.x(), ID.y() * numVoxels.y(), ID.z() * numVoxels.z()));
    }


    void Chunk::ComputeStatistics(ChunkStatistics* stats)
    {
        assert(stats != nullptr);
        int voxelsNum = voxels.sdf.size();
        for(int i = 0; i < voxelsNum; i++)
        {
            float weight = voxels.GetWeight(i);
            if (weight > 0)
            {
                float sdf = voxels.GetSDF(i);
                if (sdf < 0)
                {
                    stats->numKnownInside++;
                }
                else
                {
                    stats->numKnownOutside++;
                }
            }
            else
            {
                stats->numUnknown++;
            }

            stats->totalWeight += weight;

        }
    }

    Vec3 Chunk::GetColorAt(const Vec3& pos)
    {
        if(ComputeBoundingBox().Contains(pos))
        {
            Vec3 chunkPos = (pos - origin) / voxelResolutionMeters;
            int chunkX = static_cast<int>(chunkPos(0));
            int chunkY = static_cast<int>(chunkPos(1));
            int chunkZ = static_cast<int>(chunkPos(2));

            if(IsCoordValid(chunkX, chunkY, chunkZ))
            {
                VoxelID voxelIndex = GetVoxelID(chunkX,chunkY,chunkZ);
                return Vec3((colors.GetRed(voxelIndex)),
                            (colors.GetGreen(voxelIndex)),
                            (colors.GetBlue(voxelIndex)));
            }
        }

        return Vec3::Zero();
    }


} // namespace chisel 
