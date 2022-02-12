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

#ifndef DISTVOXEL_H_
#define DISTVOXEL_H_

#include <limits>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "AllignedAllocator.h"
#include<iostream>
#include<sstream>
namespace chisel
{

    class DistVoxel
    {
        public:
            DistVoxel();
            virtual ~DistVoxel();
            DistVoxel(const DistVoxel &d)
            {
                sdf.resize(d.sdf.size());
                weight.resize(d.weight.size());
                for(int i = 0;i!=sdf.size();++i)
                {
                    sdf[i] = d.sdf[i];
                    weight[i] = d.weight[i];
                }
            }
            DistVoxel &operator=(const DistVoxel &d)
            {
                sdf.resize(d.sdf.size());
                weight.resize(d.weight.size());
                for(int i = 0;i!=sdf.size();++i)
                {
                    //std::cout<<"???"<<std::endl;
                    sdf[i] = d.sdf[i];
                    weight[i] = d.weight[i];
                }
                return *this;
            }

            inline float GetSDF(int i) const
            {
                return sdf[i];
            }

            inline void SetSDF(const float& distance, int i)
            {
                sdf[i] = distance;
            }
            inline void clear()
            {
                sdf.clear();
                weight.clear();
            }

            inline float GetWeight(int i) const { return weight[i]; }
            inline void SetWeight(const float& w, int i) { weight[i] = w; }

            inline void Deintegrate(const float& distUpdate, const float& weightUpdate, int i)
            {
#if 0
                if(weightUpdate < 1e-5)
                {
                    return;
                }
                if(oldWeight <= weightUpdate)
                {
                    Reset();
                }
                float oldSDF = GetSDF();
                float oldWeight = GetWeight();
                float newDist = (oldWeight * oldSDF - weightUpdate * distUpdate) / ( oldWeight - weightUpdate);
                SetSDF(newDist);
                SetWeight( oldWeight - weightUpdate);
#else

                Reset(i);
                return;
#endif
            }
            inline void Integrate(const float& distUpdate, const float& weightUpdate, int i)
            {
                float oldSDF = GetSDF(i);
                float oldWeight = GetWeight(i);
                float newDist = (oldWeight * oldSDF + weightUpdate * distUpdate) / (weightUpdate + oldWeight);
                SetSDF(newDist,i);
                SetWeight(oldWeight + weightUpdate,i);

            }

            inline void Carve(int i)
            {
#if 1
                float oldSDF = GetSDF(i);
                float oldWeight = GetWeight(i);
                SetWeight(oldWeight/2,i);
                if(oldWeight < 2)
                {
                    SetSDF(999,i);
                    SetWeight(0,i);
                    return;
                }
#else
                Integrate(0.0, 1.5,i);
#endif
            }

            inline void Reset(int i)
            {
                sdf[i] = 99999;
                weight[i] = 0;
            }
            inline void Reset()
            {
                for(int i = 0;i!=sdf.size();++i)
                sdf[i]=999,weight[i]=0;
            }
            std::string toString(std::vector<int> &index)
            {
                std::ostringstream os;
                os<<sdf.size();
                index.clear();
                for( int i = 0;i!= sdf.size();++i)
                if(sdf[i]<1 || weight[i]!= 0)
                {
                os<<" "<<i<<" "<<sdf[i]<<" "<<weight[i];
                index.push_back(i);
                }
                sdf.clear();
                weight.clear();
                os<<" "<<-1;
                return os.str();
            }
            void toVector(std::vector<int> &index,std::vector<float> &buffer)
            {
                buffer.push_back(sdf.size());
                index.clear();
                for( int i = 0;i!= sdf.size();++i)
                if(sdf[i]<1 || weight[i]!= 0)
                {
                buffer.push_back(i);
                buffer.push_back(sdf[i]);
                buffer.push_back(weight[i]);
                
                index.push_back(i);
                }
                buffer.push_back(-2.0);                
            }
            void fromVector(std::vector<float> &buffer,size_t &ptr)
            {
                int size,i;
                size = buffer[ptr++];
                sdf.resize(size,999);
                weight.resize(size,0);

                while(buffer[ptr]!=-2.0)
                {
                    i = buffer[ptr++];
                    sdf[i] = buffer[ptr++];
                    weight[i] = buffer[ptr++];
                }
                ptr++;
            }
            void fromString(const std::string &s)
            {
                int size,i = 0;
                std::istringstream is(s);
                is>> size;
                sdf.resize(size,999);
                weight.resize(size,0);
                is>>i;
                while(is && i!=-1)
                {
                is>>sdf[i]>>weight[i];
                is>>i;
                }
            }
            void fromStream(std::istream &is)
            {
                int size,i = 0;
                is>> size;
                sdf.resize(size,999);
                weight.resize(size,0);
                is>>i;
                while(is && i!=-1)
                {
                is>>sdf[i]>>weight[i];
                is>>i;
                }
            }
            std::vector<float,Eigen::aligned_allocator<Eigen::Vector4f> > sdf;
            std::vector<float,Eigen::aligned_allocator<Eigen::Vector4f> > weight;
        protected:
    };

} // namespace chisel 

#endif // DISTVOXEL_H_ 
