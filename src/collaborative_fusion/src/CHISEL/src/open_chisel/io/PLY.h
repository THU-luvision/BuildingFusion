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


#ifndef PLY_H_
#define PLY_H_

#include <string>
#include <open_chisel/geometry/Geometry.h>
#include <open_chisel/mesh/Mesh.h>
#include <open_chisel/mesh/PointCloud.h>
#include "LabelColor.h"
#include "../ForLabel.h"

namespace chisel
{
    bool SaveMeshPLYASCII(const std::string& fileName, const chisel::MeshConstPtr& mesh, PoseSE3d pose = PoseSE3d());
    bool SaveCompactMeshPLYASCII(const std::string& fileName, const chisel::MeshConstPtr& mesh, PoseSE3d pose = PoseSE3d());
    bool SaveSemanticMeshPLYASCII(const std::string& fileName, const chisel::MeshConstPtr& mesh, PoseSE3d pose = PoseSE3d());
    bool SavePointCloudPLYASCII(const std::string& fileName, const Vec3List &points, 
       const Vec3List & normals = Vec3List(), const Vec3List &colors = Vec3List());
    bool SavePointCloudPLYASCII(const std::string&filename, const chisel::PcdConstPtr &pcd);
    bool SaveSemanticPointCloudPLYASCII(const std::string&filename, const chisel::PcdConstPtr &pcd);
    bool SaveInstancePointCloudPLYASCII(const std::string&filename, const chisel::PcdConstPtr &pcd);

    bool SaveFPCDASCII(const std::string &filename, const chisel::PcdConstPtr &pcd);
    bool ReadFPCDASCII(const std::string &fileName,  chisel::PcdPtr &pcd);

    bool SavePointCloudWithLabel(const std::string &filename, const chisel::PcdConstPtr &pcd);
}

#endif // PLY_H_ 
