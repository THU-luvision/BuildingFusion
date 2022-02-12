#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include "Mesh.h"
namespace chisel
{
    class PointCloud
    {
        public:
        //Remember the mesh should be compacted
        PointCloud()=default;
        PointCloud(const Mesh &mesh)
        {
            vertices = mesh.vertices;
            normals = mesh.normals;
            colors = mesh.colors;
            labels = mesh.labels;
        }
        void Rotate(const Eigen::Matrix3f &rotate)
        {
            for(int i = 0; i < vertices.size(); ++i)
            {
                vertices[i] = rotate * vertices[i];
                normals[i] = rotate * normals[i];
            }
        }
        void Transform(const Eigen::Matrix4f &transform)
        {
            Eigen::Matrix3f R = transform.block<3,3>(0, 0);
            Eigen::Vector3f t = transform.block<3,1>(0, 3);

            for(int i = 0; i < vertices.size(); ++i)
            {
                vertices[i] = R * vertices[i] + t;
                normals[i] = R * normals[i];                
            }
        }
        Vec3List vertices;
        Vec3List normals;
        Vec3List colors;
        std::vector<long> labels;
    };
    typedef std::shared_ptr<PointCloud> PcdPtr;
    typedef std::shared_ptr<const PointCloud> PcdConstPtr;
}
#endif