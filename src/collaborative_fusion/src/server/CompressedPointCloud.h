#ifndef COMPRESSED_POINT_CLOUD_H
#define COMPRESSED_POINT_CLOUD_H
#include <open_chisel/Chisel.h>
#include "../Geometry/Geometry.h"
namespace Server 
{

class ServerPointCloud
{
    public:


    Point3fList vertices;
    Point3fList normals;
    std::vector<int> colors;// compressed_colors
    std::vector<int> semantic;// compressed_semantic colors;
    std::vector<int> instance;//compressed_instance_colors;
    size_t transform_to_buffer(std::vector<unsigned char > &buffer, const TransformationMatrix &camera_start_pose)
    {
        Eigen::Matrix3f R = camera_start_pose.block<3,3>(0,0);
        Eigen::Vector3f t = camera_start_pose.block<3,1>(0,3);
        
        
        int byte_per_point = 30;
        size_t ptr = buffer.size();

        buffer.resize(buffer.size() + vertices.size() * byte_per_point);
        for(int i =0 ; i < vertices.size() ;++i)
        {
            auto  position = R * vertices[i] + t;
            auto  normal = R * normals[i];
            unsigned char *cur_vert = &buffer[ptr];// add semantic label and instance label

            *((float *)(&cur_vert[0])) = position(0);
            *((float *)(&cur_vert[4])) = position(1);
            *((float *)(&cur_vert[8])) = position(2);


            int rgb_value = colors[i];


            int semantic_value=semantic[i];

            int instance_value = instance[i];
            *((float *)(&cur_vert[12])) = rgb_value;   
            // instance_value
            *((float *)(&cur_vert[16])) = semantic_value;
            // semantic value
            *((float *)(&cur_vert[20])) = instance_value;

            chisel::toFloat16(normal(0),&cur_vert[24]);
            chisel::toFloat16(normal(1),&cur_vert[26]);
            chisel::toFloat16(normal(2),&cur_vert[28]);          
            ptr += byte_per_point;
        }
        //std::cout<<"push to meshes!"<<std::endl;
        return vertices.size();
        
    }

    void clear()
    {
        vertices.clear();
        normals.clear();
        colors.clear();
        semantic.clear();
        instance.clear();
    }
};
};

#endif