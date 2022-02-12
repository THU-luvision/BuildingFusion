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

#ifndef COLORVOXEL_H_
#define COLORVOXEL_H_

#include <algorithm>
#include <limits>
#include <stdint.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


#define VOXEL_NUM 512 // 8 * 8 * 8
namespace chisel
{

    class ColorVoxel
    {
        public:
            ColorVoxel();
            ~ColorVoxel();

            ColorVoxel(const ColorVoxel &c)
            {
                for(int i= 0;i!=512;++i)
                {
                    colorData[i*4+0] = c.colorData[i*4+0];
                     colorData[i*4+1] = c.colorData[i*4+1];
                    colorData[i*4+2] = c.colorData[i*4+2];
                    colorData[i*4+3] = c.colorData[i*4+3];                   
                }
            }
            ColorVoxel &operator= (const ColorVoxel &c)
            {
                for(int i= 0;i!=512;++i)
                {
                    colorData[i*4+0] = c.colorData[i*4+0];
                    colorData[i*4+1] = c.colorData[i*4+1];
                    colorData[i*4+2] = c.colorData[i*4+2];
                    colorData[i*4+3] = c.colorData[i*4+3];                   
                }
                return *this;
            }
            float GetRed(int voxel_index)
            {
                return colorData[voxel_index*4 + 0] / ((float)colorData[voxel_index*4 + 3]);
            }
            float GetGreen(int voxel_index)
            {
                return colorData[voxel_index*4 + 1] / ((float)colorData[voxel_index*4 + 3]);
            }
            float GetBlue(int voxel_index)
            {
                return colorData[voxel_index*4 + 2] / ((float)colorData[voxel_index*4 + 3]);
            }
            unsigned short getRed(int voxel_index) 
            {
                return colorData[voxel_index*4 + 0] ;/// ((float)colorData[voxel_index*4 + 3]);
            }
            unsigned short getGreen(int voxel_index) 
            {
                return colorData[voxel_index*4 + 1] ;/// ((float)colorData[voxel_index*4 + 3]);
            }
            unsigned short getBlue(int voxel_index) 
            {
                return colorData[voxel_index*4 + 2] ;/// ((float)colorData[voxel_index*4 + 3]);
            }
            unsigned short getDivision(int voxel_index)
            {
                return colorData[voxel_index*4 +3];
            }
            void setRed(unsigned short r,int voxel_index)
            {
                colorData[voxel_index*4 +0] = r;
            }
            void setGreen(unsigned short g,int voxel_index)
            {
                colorData[voxel_index*4+1] = g;
            }
            void setBlue(unsigned short b,int voxel_index)
            {
                colorData[voxel_index*4+2] = b;
            }
            void setDivision(unsigned short d,int voxel_index)
            {
                colorData[voxel_index*4 +3] = d;
            }
            inline void Reset(int voxel_index)
            {
                colorData[voxel_index * 4 + 0] = 0;
                colorData[voxel_index * 4 + 1] = 0;
                colorData[voxel_index * 4 + 2] = 0;
                colorData[voxel_index * 4 + 3] = 0;
            }
            std::string toString(const std::vector<int>& index)
            {

                std::ostringstream os;
                os<<index.size();
                for(int i = 0;i!= index.size();++i)
                os<<" "<<index[i]<<" "<<colorData[index[i]*4+0]<<" "<<colorData[index[i]*4+1]<<" "<<colorData[index[i]*4+2]<<" "<<colorData[index[i]*4+3];
                //free(colorData);
                os<<" "<<-1;
                return os.str();
            }
            void toVector(const std::vector<int> &index,std::vector<float > &buffer)
            {
                buffer.push_back(index.size());
                for(int i = 0;i!= index.size();++i)
                {
                    buffer.push_back(index[i]);
                    buffer.push_back(colorData[index[i]*4+0]);
                    buffer.push_back(colorData[index[i]*4+1]);
                    buffer.push_back(colorData[index[i]*4+2]);
                    buffer.push_back(colorData[index[i]*4+3]);
                }    
            }
            void fromVector(std::vector<float> &buffer,size_t &ptr)
            {
                size_t size = buffer[ptr++],index;
                size_t all_size = 512;
                size_t count = 0;
                colorData = (unsigned short *)_mm_malloc(all_size * 4 * 2, 32);
            for(int i = 0; i < 128; i++)
            {
                _mm256_store_si256((__m256i *)&colorData[i * 16],_mm256_set1_epi16(0));
                
            }
                while(count<size)
                {
                    count++;
                    index = buffer[ptr++];
                    colorData[index *4+0] = buffer[ptr++];
                    colorData[index *4+1] = buffer[ptr++];
                    colorData[index *4+2] = buffer[ptr++];
                    colorData[index *4+3] = buffer[ptr++];
                }
            }
            void fromString(const std::string &s)
            {
                int index,size;
                std::istringstream is(s);
            colorData = (unsigned short *)_mm_malloc(512 * 4 * 2, 32);
            for(int i = 0; i < 128; i++)
            {
                _mm256_store_si256((__m256i *)&colorData[i * 16],_mm256_set1_epi16(0));
                
            }
                is>>size;
                is>>index;
                while(is&& index!=-1 )
                {
                is>>colorData[index*4+0]>>colorData[index*4+1]>>colorData[index*4+2]>>colorData[index*4+3];
                is>>index;
                }
            }
            void fromStream(std::istream &is)
            {
                int index,size;
            colorData = (unsigned short *)_mm_malloc(512 * 4 * 2, 32);
            for(int i = 0; i < 128; i++)
            {
                _mm256_store_si256((__m256i *)&colorData[i * 16],_mm256_set1_epi16(0));
                
            }   is>>size;
                is>>index;
                while(is&& index!=-1 )
                {
                is>>colorData[index*4+0]>>colorData[index*4+1]>>colorData[index*4+2]>>colorData[index*4+3];
                is>>index;
                }

            }
            unsigned short *colorData;

        protected:

    };

} // namespace chisel 

#endif // COLORVOXEL_H_ 
