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

#include <open_chisel/io/PLY.h>

#include <iostream>
#include <fstream>
#include <sstream>
namespace chisel
{
    bool SaveMeshPLYASCII(const std::string& fileName, const chisel::MeshConstPtr& mesh,PoseSE3d pose)
    {
        std::ofstream stream(fileName.c_str());

        if (!stream)
        {
            return false;
        }
        Eigen::Matrix3d R = pose.rotationMatrix();
        size_t numPoints = mesh->vertices.size();
        stream << "ply" << std::endl;
        stream << "format ascii 1.0" << std::endl;
        stream << "element vertex " << numPoints << std::endl;
        stream << "property float x" << std::endl;
        stream << "property float y" << std::endl;
        stream << "property float z" << std::endl;
        if (mesh->HasColors())
        {
            stream << "property uchar red" << std::endl;
            stream << "property uchar green" << std::endl;
            stream << "property uchar blue" << std::endl;
        }
        stream << "element face " << numPoints / 3 <<  std::endl;
        stream << "property list uchar int vertex_index" << std::endl;
        stream << "end_header" << std::endl;

        size_t vert_idx = 0;
        for (const Vec3& vert : mesh->vertices)
        {
            stream << vert(0) << " " << vert(1) << " " << vert(2);

            if (mesh->HasColors())
            {
                const Vec3& color = mesh->colors[vert_idx];
                int r = static_cast<int>(color(0) * 255.0f);
                int g = static_cast<int>(color(1) * 255.0f);
                int b = static_cast<int>(color(2) * 255.0f);
                if(r >= 255) r = 255;
                if(g >= 255) g = 255;
                if(b >= 255) b = 255;
                stream << " " << r << " " << g << " " << b;
            }

            stream << std::endl;
            vert_idx++;
        }

        for (size_t i = 0; i < mesh->indices.size(); i+=3)
        {
            stream << "3 ";

            for(int j = 0; j < 3; j++)
            {
                stream << mesh->indices.at(i + j) << " ";
            }

            stream << std::endl;
        }


    }
    bool SavePointCloudPLYASCII(const std::string&filename, const chisel::PcdConstPtr &pcd)
    {
        return SavePointCloudPLYASCII(filename, pcd->vertices, pcd->normals, pcd->colors);
    }
    bool SaveSemanticPointCloudPLYASCII(const std::string&filename, const chisel::PcdConstPtr &pcd)
    {
        Vec3List colors ( pcd->labels.size());

        for(int i = 0; i != colors.size(); ++i)
        {
            long label =get_semantic_label( pcd->labels[i]);
            colors[i] = Vec3(label_colors[3*label], label_colors[3*label+1], label_colors[3*label+2]);
        }
        return SavePointCloudPLYASCII(filename, pcd->vertices, pcd->normals, colors);
    }
    bool SaveInstancePointCloudPLYASCII(const std::string&filename, const chisel::PcdConstPtr &pcd)
    {
        Vec3List colors ( pcd->labels.size());

        for(int i = 0; i != colors.size(); ++i)
        {
            long label = (pcd->labels[i] % 10000)%20;
            colors[i] = Vec3(label_colors[3*label], label_colors[3*label+1], label_colors[3*label+2]);
        }
        return SavePointCloudPLYASCII(filename, pcd->vertices, pcd->normals, colors);
    }
    bool SaveFPCDASCII(const std::string &fileName, const chisel::PcdConstPtr &pcd)
    {
        std::ofstream stream(fileName.c_str());

        if (!stream)
        {
            std::cout <<"Error occurs when open file "<<fileName<<std::endl;
            return false;
        }
        size_t size = pcd->vertices.size();
        stream <<size<<std::endl;
        for(int i = 0; i !=size; ++i)
        {
            auto & v = pcd->vertices[i];
            auto & n = pcd->normals[i];
            stream << v(0) << " " << v(1) << " " << v(2)<<" "
                <<n(0)<<" "<<n(1)<<" "<<n(2)
                <<" "<<pcd->labels[i] << std::endl;
        }
        stream.close();
        return true;
    }

    bool ReadFPCDASCII(const std::string &fileName,  chisel::PcdPtr &pcd)
    {
        std::ifstream stream(fileName.c_str());       
        if (!stream)
        {
            std::cout <<"Error occurs when open file "<<fileName<<std::endl;
            return false;
        }
        std::string content;
        size_t size;

        std::getline(stream, content);
        std::istringstream iss(content);
        iss>> size;
        
        pcd->vertices.clear();
        pcd->labels.clear();
        pcd->normals.clear();
        while(std::getline(stream, content))
        {
            std::istringstream iss(content);
            Vec3 v, n;
            long label;
            iss>>v(0)>>v(1)>>v(2)
                >>n(0)>>n(1)>>n(2)>>label;
            pcd->vertices.push_back(v);
            pcd->labels.push_back(label);
            pcd->normals.push_back(n);  
        }         
        return pcd->vertices.size() == size;
    }
    bool SavePointCloudPLYASCII(const std::string& fileName, const Vec3List &points, 
       const Vec3List & normals, const Vec3List &colors)
    {
        std::ofstream stream(fileName.c_str());

        if (!stream)
        {
            return false;
        }
        size_t numPoints = points.size();
        bool has_normal = normals.size() != 0;
        bool has_color = colors.size() != 0; 
        stream << "ply" << std::endl;
        stream << "format ascii 1.0" << std::endl;
        stream << "element vertex " << numPoints << std::endl;
        stream << "property float x" << std::endl;
        stream << "property float y" << std::endl;
        stream << "property float z" << std::endl;
        if(has_normal)
        {
            stream << "property float nx"<<std::endl;
            stream << "property float ny"<<std::endl;
            stream << "property float nz"<<std::endl;
        }
        if (has_color)
        {
            stream << "property uchar red" << std::endl;
            stream << "property uchar green" << std::endl;
            stream << "property uchar blue" << std::endl;
        }
        stream << "end_header" << std::endl;

        for (int i = 0; i!= points.size(); ++i)
        {
            stream << points[i](0) << " " << points[i](1) << " " << points[i](2);
            if (has_normal)
            {
                stream << " " << normals[i](0) << " " << normals[i](1) << " " << normals[i](2);
            }            
            if (has_color)
            {
                const Vec3& color = colors[i];
                int r = static_cast<int>(color(0) * 255.0f);
                int g = static_cast<int>(color(1) * 255.0f);
                int b = static_cast<int>(color(2) * 255.0f);

                stream << " " << r << " " << g << " " << b;
            }

            stream << std::endl;
        }
        return true;
    }

 bool SaveCompactMeshPLYASCII(const std::string& fileName,  const chisel::MeshConstPtr& mesh,PoseSE3d pose)
    {
        std::ofstream stream(fileName.c_str());
        if (!stream)
        {
            return false;
        }

        size_t numPoints = mesh->compact_vertices.size();
        stream << "ply" << std::endl;
        stream << "format ascii 1.0" << std::endl;
        stream << "element vertex " << numPoints << std::endl;
        stream << "property float x" << std::endl;
        stream << "property float y" << std::endl;
        stream << "property float z" << std::endl;
        if(mesh->HasNormals())
        {
            stream << "property float nx"<<std::endl;
            stream << "property float ny"<<std::endl;
            stream << "property float nz"<<std::endl;
        }
        if (mesh->HasColors())
        {
            stream << "property uchar red" << std::endl;
            stream << "property uchar green" << std::endl;
            stream << "property uchar blue" << std::endl;
        }
        stream << "element face " << mesh->triangles.size() <<  std::endl;
        stream << "property list uchar int vertex_index" << std::endl;
        stream << "end_header" << std::endl;

        size_t vert_idx = 0;
        for (const Vec3& vert : mesh->compact_vertices)
        {
            //Eigen::Vector3d vertd = R * vert.cast<double>();
            stream << vert(0) << " " << vert(1) << " " << vert(2);
            if (mesh->HasNormals())
            {
                const Vec3& normal = mesh->compact_normals[vert_idx];

                stream << " " << normal(0) << " " << normal(1) << " " << normal(2);
            }
            if (mesh->HasColors())
            {
                const Vec3& color = mesh->compact_colors[vert_idx];
                int r = static_cast<int>(color(0) * 255.0f);
                int g = static_cast<int>(color(1) * 255.0f);
                int b = static_cast<int>(color(2) * 255.0f);
                if(r >= 255) r = 255;
                if(g >= 255) g = 255;
                if(b >= 255) b = 255;
                stream << " " << r << " " << g << " " << b;
            }

            stream << std::endl;
            vert_idx++;
        }

        for (size_t i = 0; i < mesh->triangles.size(); i+=1)
        {
            stream << "3 ";

            for(int j = 0; j < 3; j++)
            {
                stream << mesh->triangles[i](j) << " ";
            }

            stream << std::endl;
        }

    return true;
    }
    bool SaveSemanticMeshPLYASCII(const std::string& fileName,  const chisel::MeshConstPtr& mesh,PoseSE3d pose)
    {
        std::ofstream stream(fileName.c_str());
        if (!stream)
        {
            return false;
        }

        size_t numPoints = mesh->compact_vertices.size();
        stream << "ply" << std::endl;
        stream << "format ascii 1.0" << std::endl;
        stream << "element vertex " << numPoints << std::endl;
        stream << "property float x" << std::endl;
        stream << "property float y" << std::endl;
        stream << "property float z" << std::endl;
        if(mesh->HasNormals())
        {
            stream << "property float nx"<<std::endl;
            stream << "property float ny"<<std::endl;
            stream << "property float nz"<<std::endl;
        }
        if (mesh->HasColors())
        {
            stream << "property uchar red" << std::endl;
            stream << "property uchar green" << std::endl;
            stream << "property uchar blue" << std::endl;
        }
        stream << "element face " << mesh->triangles.size() <<  std::endl;
        stream << "property list uchar int vertex_index" << std::endl;
        stream << "end_header" << std::endl;

        size_t vert_idx = 0;
        for (const Vec3& vert : mesh->compact_vertices)
        {
            //Eigen::Vector3d vertd = R * vert.cast<double>();
            stream << vert(0) << " " << vert(1) << " " << vert(2);
            if (mesh->HasNormals())
            {
                const Vec3& normal = mesh->compact_normals[vert_idx];

                stream << " " << normal(0) << " " << normal(1) << " " << normal(2);
            }
            /*
            if (mesh->HasColors())
            {
                const Vec3& color = mesh->compact_colors[vert_idx];
                int r = static_cast<int>(color(0) * 255.0f);
                int g = static_cast<int>(color(1) * 255.0f);
                int b = static_cast<int>(color(2) * 255.0f);

                stream << " " << r << " " << g << " " << b;
            }*/
            const long& label = mesh->compact_labels[vert_idx] / 10000;
            int r=label_colors[3*label]*255;
            int g = int(label_colors[3*label+1]*255);
            int b = int(label_colors[3*label+2]*255);
            stream << " " << r << " " << g << " " << b;
            stream << std::endl;
            vert_idx++;
        }

        for (size_t i = 0; i < mesh->triangles.size(); i+=1)
        {
            stream << "3 ";

            for(int j = 0; j < 3; j++)
            {
                stream << mesh->triangles[i](j) << " ";
            }

            stream << std::endl;
        }

    return true;
    }

}
