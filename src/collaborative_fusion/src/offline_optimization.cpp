// Created by Guoqing 2020.07.14
// this program is used for offline-optimization
/*
we need: associate.txt for each room and submap.
poses, for each submap.
maybe connections between submap and room, if we want to optimize the final results.
*/

#include "cnpy.h"
#include "./Geometry/Geometry.h"
#include "./IO/RPLYReader.h"
#include <open_chisel/io/PLY.h>
#include "offline_reconstruction.h"

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
void GetAgentBasePaths(const std::string filename, std::vector<std::string> & base_paths)
{
    size_t agent_count;
    std::ifstream ifs(filename);
    ifs>>agent_count;
    if(agent_count != base_paths.size())
    {
        std::cout<<YELLOW<<"agent_count is not equal to camera_count."<<RESET<<std::endl;
    }
    for(int i = 0; i != base_paths.size(); ++i)
    {
        int agent_id;
        ifs>>agent_id;
        assert(agent_id == i);
        ifs>>base_paths[i];
        std::cout<<agent_id<<": "<<base_paths[i]<<std::endl;
    }
    ifs.close();
    
}
void GetRoomToSubmap(const std::string & room_info_path, std::vector<std::vector<int>> &room_to_submap)
{
    std::ifstream ifs(room_info_path);
    int room_count;
    ifs >> room_count;
    if(room_count <= 0) return;
    room_to_submap.resize(room_count);
    for(int i = 0; i != room_count; ++i)
    {
        int room_id, submap_count ;
        ifs >> room_id;
        assert(room_id == i);
        ifs >> submap_count;
        for(int j = 0; j != submap_count; ++j)
        {
            int submap_id ;
            ifs >> submap_id;
            room_to_submap[room_id].push_back(submap_id);
        }
        //std::sort(room_to_submap[room_id].begin(), room_to_submap[room_id].end());
    }
    ifs.close();
}
void ProcessOneCamera(int camera_id, const std::string & base_path)
{
    //std::string submap_model_path = "./model/by_submap/"+std::to_string(camera_id);
    std::string submap_pose_path = "./global_submap_pose/final_"+std::to_string(camera_id)+".sbp";
    std::string room_info_path = "./rooms/"+std::to_string(camera_id)+".room";
    std::vector<std::vector<int> > room_to_submap;
    GetRoomToSubmap(room_info_path, room_to_submap);
    Mat4fList submap_poses;
    std::set<int> unlabeled_submap;
    std::cout<<"processing on camera "<<camera_id<<std::endl;
    GetAgentSubmapPoses(submap_pose_path, submap_poses);
    int submap_count = submap_poses.size();
    for(int i = 0; i != submap_count; ++i)
    unlabeled_submap.insert(i);
    //this is the naive version, which means we just update the room model
    for(int room_id = 0; room_id != room_to_submap.size(); ++room_id)
    {
        for(auto iter = room_to_submap[room_id].begin(); iter != room_to_submap[room_id].end(); ++iter)
        unlabeled_submap.erase(*iter);        
    }
    for(int room_id = 0; room_id != room_to_submap.size(); ++room_id)
    {
        if(room_to_submap[room_id].size() <= 0) 
        continue;
        
        std::string associate = "./room_associate/"+std::to_string(camera_id)+"/room_"+std::to_string(room_id)+"_associate.txt";
        
        int first_submap_id = room_to_submap[room_id][0];
        int end_submap_id = room_to_submap[room_id].back();



        std::string final_model_file = "offline_generated_c_"+std::to_string(camera_id)+"_r_"+std::to_string(room_id)+".ply";
        auto start_pose = submap_poses[first_submap_id];
        //OfflineReconstruction(base_path, associate, start_pose, final_model_file);
        std::string last_submap_associate = "", next_submap_associate = "";
        if(first_submap_id > 0 && unlabeled_submap.find(first_submap_id - 1) != unlabeled_submap.end()) 
        {
            last_submap_associate = "./submap_associate/"+std::to_string(camera_id)+
            "/submap_" +std::to_string(first_submap_id - 1) +"_associate.txt";
            start_pose = submap_poses[first_submap_id - 1];
            unlabeled_submap.erase(first_submap_id - 1);
        }
        if(end_submap_id < submap_poses.size() - 1 && unlabeled_submap.find(end_submap_id + 1) != unlabeled_submap.end())
        {
            next_submap_associate = "./submap_associate/"+std::to_string(camera_id)+
            "/submap_" +std::to_string(end_submap_id + 1) +"_associate.txt";
            unlabeled_submap.erase(end_submap_id + 1);
        }

        OfflineReconstruction(base_path, last_submap_associate, associate, next_submap_associate, start_pose, final_model_file);
        //use start pose as the first keyframe pose, and reconstruct the room
        //reconstruction process 
        //save global room model.
        std::cout<<"Finish Reconstruct room "<<room_id<<"."<<std::endl;
    }
    std::ofstream ofs("./valid_submap_"+std::to_string(camera_id)+".txt");
    for(auto iter = unlabeled_submap.begin(); iter != unlabeled_submap.end(); ++iter)
    {
        ofs<<*iter<<std::endl;
    /*
        int submap_id = *iter;
        std::string associate = "./submap_associate/"+std::to_string(camera_id)+"/submap_"+std::to_string(submap_id)+"_associate.txt";
        std::string final_model_file = "offline_generated_c_"+std::to_string(camera_id)+"_s_"+std::to_string(submap_id)+".ply";
        auto start_pose = submap_poses[submap_id];
        //OfflineReconstruction(base_path, associate, start_pose, final_model_file);
        std::string last_submap_associate = "", next_submap_associate = "";
        if(submap_id > 0) 
        {
            last_submap_associate = "./submap_associate/"+std::to_string(camera_id)+
            "/submap_" +std::to_string(submap_id - 1) +"_associate.txt";
            start_pose = submap_poses[submap_id - 1];
        }
        if(submap_id < submap_poses.size() - 1)
        {
            next_submap_associate = "./submap_associate/"+std::to_string(camera_id)+
            "/submap_" +std::to_string(submap_id + 1) +"_associate.txt";
        }

        //OfflineReconstruction(base_path, last_submap_associate, associate, next_submap_associate, start_pose, final_model_file);
        OfflineReconstruction(base_path, associate, start_pose, final_model_file);

        //use start pose as the first keyframe pose, and reconstruct the room
        //reconstruction process 
        //save global room model.
        std::cout<<"Finish Reconstruct submap "<<submap_id<<"."<<std::endl;        
    */
    }
    //what if we want to integrate the connection between submap
}
void ProcessOneCameraMatchToRoom(int camera_id, int room_id,const std::string & base_path)
{
    //std::string submap_model_path = "./model/by_submap/"+std::to_string(camera_id);
    std::string submap_pose_path_final = "./global_submap_pose/final_"+std::to_string(camera_id)+".sbp";
    std::string submap_pose_path_relatve = "./global_submap_pose/"+std::to_string(camera_id)+".sbp";
    std::string room_info_path = "./rooms/"+std::to_string(camera_id)+".room";
    std::vector<std::vector<int> > room_to_submap;
    GetRoomToSubmap(room_info_path, room_to_submap);
    Mat4fList submap_poses;
    Mat4fList submap_poses_final;
    Mat4fList submap_poses_relative;
    std::set<int> unlabeled_submap;
    std::cout<<"processing on camera "<<camera_id<<std::endl;
    GetAgentSubmapPoses(submap_pose_path_final, submap_poses_final);
    GetAgentSubmapPoses(submap_pose_path_relatve, submap_poses_relative);
    for(int i = 0; i != submap_poses_relative.size(); ++i)
    {
        submap_poses.push_back(submap_poses_relative[0].inverse() *  submap_poses_final[i]);
    }
    int submap_count = submap_poses.size();
    for(int i = 0; i != submap_count; ++i)
    unlabeled_submap.insert(i);
    //this is the naive version, which means we just update the room model
    for(int rid = 0; rid != room_to_submap.size(); ++rid)
    {
        for(auto iter = room_to_submap[rid].begin(); iter != room_to_submap[rid].end(); ++iter)
        unlabeled_submap.erase(*iter);        
    }
    if(room_to_submap[room_id].size() <= 0) 
    return;
    
    std::string associate = "./room_associate/"+std::to_string(camera_id)+"/room_"+std::to_string(room_id)+"_associate.txt";
    
    int first_submap_id = room_to_submap[room_id][0];
    int end_submap_id = room_to_submap[room_id].back();



    std::string final_model_file = "offline_generated_c_"+std::to_string(camera_id)+"_r_"+std::to_string(room_id)+".ply";
    auto start_pose = submap_poses[first_submap_id];
    //OfflineReconstruction(base_path, associate, start_pose, final_model_file);
    std::string last_submap_associate = "", next_submap_associate = "";
    /*
    if(first_submap_id > 0 && unlabeled_submap.find(first_submap_id - 1) != unlabeled_submap.end()) 
    {
        last_submap_associate = "./submap_associate/"+std::to_string(camera_id)+
        "/submap_" +std::to_string(first_submap_id - 1) +"_associate.txt";
        start_pose = submap_poses[first_submap_id - 1];
        unlabeled_submap.erase(first_submap_id - 1);
    }
    if(end_submap_id < submap_poses.size() - 1 && unlabeled_submap.find(end_submap_id + 1) != unlabeled_submap.end())
    {
        next_submap_associate = "./submap_associate/"+std::to_string(camera_id)+
        "/submap_" +std::to_string(end_submap_id + 1) +"_associate.txt";
        unlabeled_submap.erase(end_submap_id + 1);
    }
    */
    OfflineReconstruction(base_path, associate, start_pose, final_model_file);
    //use start pose as the first keyframe pose, and reconstruct the room
    //reconstruction process 
    //save global room model.
    std::cout<<"Finish Reconstruct room "<<room_id<<"."<<std::endl;
}
void ProcessOneCamera(int camera_id, int room_id,const std::string & base_path)
{
    //std::string submap_model_path = "./model/by_submap/"+std::to_string(camera_id);
    std::string submap_pose_path = "./global_submap_pose/final_"+std::to_string(camera_id)+".sbp";
    std::string room_info_path = "./rooms/"+std::to_string(camera_id)+".room";
    std::vector<std::vector<int> > room_to_submap;
    GetRoomToSubmap(room_info_path, room_to_submap);
    Mat4fList submap_poses;
    std::set<int> unlabeled_submap;
    std::cout<<"processing on camera "<<camera_id<<std::endl;
    GetAgentSubmapPoses(submap_pose_path, submap_poses);
    int submap_count = submap_poses.size();
    for(int i = 0; i != submap_count; ++i)
    unlabeled_submap.insert(i);
    //this is the naive version, which means we just update the room model
    for(int rid = 0; rid != room_to_submap.size(); ++rid)
    {
        for(auto iter = room_to_submap[rid].begin(); iter != room_to_submap[rid].end(); ++iter)
        unlabeled_submap.erase(*iter);        
    }
    if(room_to_submap[room_id].size() <= 0) 
    return;
    
    std::string associate = "./room_associate/"+std::to_string(camera_id)+"/room_"+std::to_string(room_id)+"_associate.txt";
    
    int first_submap_id = room_to_submap[room_id][0];
    int end_submap_id = room_to_submap[room_id].back();



    std::string final_model_file = "offline_generated_c_"+std::to_string(camera_id)+"_r_"+std::to_string(room_id)+".ply";
    auto start_pose = submap_poses[first_submap_id];
    //OfflineReconstruction(base_path, associate, start_pose, final_model_file);
    std::string last_submap_associate = "", next_submap_associate = "";
    /*
    if(first_submap_id > 0 && unlabeled_submap.find(first_submap_id - 1) != unlabeled_submap.end()) 
    {
        last_submap_associate = "./submap_associate/"+std::to_string(camera_id)+
        "/submap_" +std::to_string(first_submap_id - 1) +"_associate.txt";
        start_pose = submap_poses[first_submap_id - 1];
        unlabeled_submap.erase(first_submap_id - 1);
    }
    if(end_submap_id < submap_poses.size() - 1 && unlabeled_submap.find(end_submap_id + 1) != unlabeled_submap.end())
    {
        next_submap_associate = "./submap_associate/"+std::to_string(camera_id)+
        "/submap_" +std::to_string(end_submap_id + 1) +"_associate.txt";
        unlabeled_submap.erase(end_submap_id + 1);
    }
    */
    OfflineReconstruction(base_path, associate, start_pose, final_model_file);
    //use start pose as the first keyframe pose, and reconstruct the room
    //reconstruction process 
    //save global room model.
    std::cout<<"Finish Reconstruct room "<<room_id<<"."<<std::endl;
}
/*
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
}*/
int main(int argc, char **argv)
{
    int camera_count = 0;
    if(argc == 2)
    {
        camera_count = std::atoi(argv[1]);
    }
    std::vector<std::string> base_paths(camera_count);
    std::string base_path_file = "./base_paths.txt";
    GetAgentBasePaths(base_path_file, base_paths);
    // ProcessOneCamera(1, 0, base_paths[1]);
    // ProcessOneCamera(2, 2, base_paths[2]);
    
    for(int i = 0; i != camera_count; ++i)
    {
        ProcessOneCamera(i,base_paths[i]);
    }
    return 0;
}