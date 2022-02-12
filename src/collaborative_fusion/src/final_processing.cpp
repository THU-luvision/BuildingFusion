#include "cnpy.h"
#include "./Geometry/Geometry.h"
#include "./IO/RPLYReader.h"
#include <open_chisel/io/PLY.h>
void LoadLabel(const std::string &filename, std::vector<int> &semantic_label, std::vector<int> &instance_label)
{
    cnpy::NpyArray semantic_array = cnpy::npy_load(filename+"_semantic.npy");
    cnpy::NpyArray instance_array = cnpy::npy_load(filename+"_instance.npy");
    int *loaded_semantic = semantic_array.data<int>();
    int *loaded_instance = instance_array.data<int>();
    
    semantic_label.resize(semantic_array.shape[0]);
    instance_label.resize(instance_array.shape[0]);
    std::cout<<semantic_label.size()<<" "<<instance_label.size()<<std::endl;
    assert(semantic_label.size() == instance_label.size());
    for(int i = 0; i != semantic_array.shape[0]; ++i)
    {
        //std::cout<<semantic_label[i]<<" "<<instance_label[i]<<std::endl;
        semantic_label[i] = loaded_semantic[i] * 10000;
        instance_label[i] = loaded_instance[i];
        //std::cout<<semantic_label[i]<<" "<<instance_label[i]<<std::endl;
    }
}
void GetAgentSubmapPoses(const std::string filename, Mat4fList & relative_poses)
{
    size_t submap_count;
    std::ifstream ifs(filename);
    ifs>>submap_count;
    relative_poses.resize(submap_count);
    Eigen::Matrix4f tmp_pos = Eigen::Matrix4f::Zero();
    for(int i = 0; i != relative_poses.size(); ++i)
    {
        int submap_id;
        ifs>>submap_id;
        assert(submap_id == i);
        ifs>>tmp_pos(0,0)>>tmp_pos(0,1)>>tmp_pos(0,2)>>
            tmp_pos(1,0)>>tmp_pos(1,1)>>tmp_pos(1,2)>>
            tmp_pos(2,0)>>tmp_pos(2,1)>>tmp_pos(2,2)>>
            tmp_pos(0,3)>>tmp_pos(1,3)>>tmp_pos(2,3);
        tmp_pos(3,3) = 1.0;
        relative_poses[i] = tmp_pos;
    }
}

void ProcessOneCamera(int camera_id)
{
    std::string submap_model_path = "./model/by_submap/"+std::to_string(camera_id);
    std::string submap_pose_path = "./global_submap_pose/"+std::to_string(camera_id)+".sbp";
    Mat4fList submap_poses;
    
    GetAgentSubmapPoses(submap_pose_path, submap_poses);
    int submap_count = submap_poses.size();
    for(int i = 0; i != submap_count; ++i)
    {
        chisel::MeshPtr mesh_ptr(new chisel::Mesh());
        std::string tmp_model_path = submap_model_path + "/"+std::to_string(i)+".ply";
        /*
        cnpy::NpyArray array = cnpy::npy_load(tmp_model_path+"_semantic.npy");
        int *loaded_data = array.data<int>();
        for(int i = 0; i != array.shape[0]; ++i)
        std::cout<<loaded_data[i]<<std::endl;
        */
        ReadPLYToChiselMesh(tmp_model_path, mesh_ptr);
        mesh_ptr->transform(submap_poses[i]);
        WritePLYFromChiselMesh(tmp_model_path+"_refined.ply",mesh_ptr);
        // Transform the data by submap poses
        // WritePLY
    }
}
void MergeIntoOnePCD(int camera_id, chisel::PcdPtr &final_pcd)
{
    std::string submap_model_path = "./model/by_submap/"+std::to_string(camera_id);
    std::string submap_pose_path = "./global_submap_pose/"+std::to_string(camera_id)+".sbp";
    Mat4fList submap_poses;
    
    GetAgentSubmapPoses(submap_pose_path, submap_poses);
    int submap_count = submap_poses.size();
    for(int i = 0; i != submap_count; ++i)
    {
        chisel::MeshPtr mesh_ptr(new chisel::Mesh());
        std::string tmp_model_path = submap_model_path + "/"+std::to_string(i)+".ply";
        std::cout<<camera_id<<" "<<i<<std::endl;
        ReadPLYToChiselMesh(tmp_model_path+"_refined.ply", mesh_ptr);
        std::vector<int> semantic_labels, instance_labels;
        if(mesh_ptr->compact_vertices.size() > 0)
        {
            LoadLabel(tmp_model_path, semantic_labels, instance_labels);
            if(mesh_ptr->vertices.size() < 500000)
            {
                final_pcd->vertices.insert(final_pcd->vertices.end(), mesh_ptr->compact_vertices.begin(), mesh_ptr->compact_vertices.end());
                final_pcd->normals.insert(final_pcd->normals.end(), mesh_ptr->compact_normals.begin(), mesh_ptr->compact_normals.end());
                final_pcd->colors.insert(final_pcd->colors.end(), mesh_ptr->compact_colors.begin(), mesh_ptr->compact_colors.end());
                for(int j = 0; j != semantic_labels.size(); ++j)
                {
                    //std::cout<<semantic_labels[j]<<std::endl;
                    final_pcd->labels.push_back(semantic_labels[j]);
                }
            }
        }
        // Transform the data by submap poses
        // WritePLY
    }
    std::cout<<"Read "<<final_pcd->vertices.size()<<" points."<<std::endl;
}
int main(int argc, char **argv)
{
    int camera_count = 0;
    if(argc == 2)
    {
        camera_count = std::atoi(argv[1]);
    }
    for(int i = 0 ; i < camera_count; ++i)
    ProcessOneCamera(i);
    chisel::PcdPtr final_pcd(new chisel::PointCloud());
    for(int i = 0; i < camera_count; ++i)
    MergeIntoOnePCD(i, final_pcd);
    WritePLYFromChiselPcd("./final_pcd.ply", final_pcd);
    chisel::SaveSemanticPointCloudPLYASCII("./semantic_final_pcd.ply", final_pcd);
    return 0;
}