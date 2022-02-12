#ifndef SERVER_SLAM_H
#define SERVER_SLAM_H
#include <vector>
#include <map>
#include "ServerFrame.h"
#include "ServerMild.h"
#include "../Geometry/Geometry.h"
#include <collaborative_fusion/UpdateFrame.h>
#include <collaborative_fusion/WithDraw.h>
#include "../GCSLAM/MultiViewGeometry.h"
#include "../CommunicationAPI.h"
#include "Correspondence.h"
#include "RoomHandler.h"
#include "CameraNode.h"
#include "../Tools/TickTock.h"
#include "../RoomLCD/utils.h"
#include "CompressedPointCloud.h"
#include "NormalCorrespondence.h"
#include "RoomColorTable.h"

/*
#define RESET   "\033[0m"
#define BLACK   "\033[30m"     
#define RED     "\033[31m"      
#define GREEN   "\033[32m"    
#define YELLOW  "\033[33m" 
*/
#define FOR_REBUTTAL 0
#if FOR_REBUTTAL
extern std::map<int, float> t_mild_lcd;
extern std::map<int, float> t_global_optimization;
extern std::map<int, float> t_room_lcd;
#endif
namespace Server 
{
    class ServerSLAM 
    {
        public:
        ServerSLAM()
        {
            //server_mild.init();
            camera_num = 0;
            R<<1,0,0,0,0,1,0,-1,0;
            T<< 1,0,0,0, 
                0,0,1,0,
                0,-1,0,0, 
                0,0,0,1;
            Gravity<<0,-1,0;
        }
        /*
        ServerSLAM(int camera_num)
        {
            keyframes.resize(camera_num);
            submapPoses.resize(camera_num);
            submapPosesFinal.resize(camera_num);
            submapPosesRelativeChanges.resize(camera_num);
            cameraStartPoses.resize(camera_num);
            server_mild.init();
            isInitialized.resize(camera_num,false);

            for(int i = 0;i!=submapPoses.size();++i)
            {
                submapPoses[i].push_back(PoseSE3d());
                submapPosesFinal[i].push_back(PoseSE3d());
                submapPosesRelativeChanges[i].push_back(PoseSE3d());
            }
            if(camera_num == 1)
            allInitialized = true;
        }
        void setCameraNum(int camera_num)
        {
            keyframes.resize(camera_num);
            submapPoses.resize(camera_num);
            submapPosesFinal.resize(camera_num);
            submapPosesRelativeChanges.resize(camera_num);
            cameraStartPoses.resize(camera_num);
            isInitialized.resize(camera_num,false);

            for(int i = 0;i!=submapPoses.size();++i)
            {
                submapPoses[i].push_back(PoseSE3d());
                submapPosesFinal[i].push_back(PoseSE3d());
                submapPosesRelativeChanges[i].push_back(PoseSE3d());
            }

            if(camera_num == 1)
            allInitialized = true;
        }        
        
        */
        void update_room_normal(int camera_id)
        {
            int camera_node_id = node_id[camera_id];
            for(auto iter = camera_nodes[camera_node_id].camera.begin() ; iter != camera_nodes[camera_node_id].camera.end(); ++iter)
            {
                int camera_id = *iter;
                for(int j = 0; j != rooms_for_each_camera[camera_id].size(); ++j)
                {
                    rooms_for_each_camera[camera_id][j].ComputeFloorNormal(submap_floor_normals, submapPosesRelativeChanges);
                }
            }
        }
        void addCamera(const std::string &base_path)
        {
            keyframes.push_back(std::map<int, ServerFrame>());
            submapPoses.push_back(PoseSE3dList(1,PoseSE3d()));
            submapPosesFinal.push_back(PoseSE3dList(1,PoseSE3d()));
            submapPosesRelativeChanges.push_back(PoseSE3dList(1,PoseSE3d()));
            cameraStartPoses.push_back(PoseSE3d());            
            camera_nodes.push_back(CameraNode(camera_num));
            node_id.push_back(camera_num);
            rooms_for_each_camera.push_back(std::vector<RoomInfo>());
            room_poses.push_back(PoseSE3dList());
            submap_to_room_for_each_camera.push_back(std::vector<int>(1,-1));
            server_milds.push_back(ServerMild());
            server_milds.back().init();
            server_pcds.push_back(std::vector<ServerPointCloud>());
            submap_floor_normals.push_back(Point3fList());
            last_updated_time.push_back(std::chrono::steady_clock::time_point());
            camera_num++;
            base_paths.push_back(base_path);
            is_terminated.push_back(false);
        }
        std::vector<float> GetCalibPara()
        {
            auto &camera = MultiViewGeometry::g_camera;
            std::vector<float> result(13);
            result[0] = camera.width;
            result[1] = camera.height;
            result[2] = camera.c_fx;
            result[3] = camera.c_fy;
            result[4] = camera.c_cx;
            result[5] = camera.c_cy;
            result[6] = camera.d[0];
            result[7] = camera.d[1];
            result[8] = camera.d[2];
            result[9] = camera.d[3];
            result[10] = camera.d[4];
            result[11] = camera.depth_scale;
            result[12] = camera.maximum_depth;
            return result;         
        }
        void save_all_submap_info()
        {
            
            for(int i = 0; i != submapPosesRelativeChanges.size(); ++i)
            {
                std::string filename = "global_submap_pose/"+ std::to_string(i)+".sbp";
                std::string filename_final = "global_submap_pose/final_" +std::to_string(i)+".sbp";
                std::ofstream ofs(filename);
                ofs<<submapPosesRelativeChanges[i].size()<<std::endl;
                for(int j = 0; j != submapPosesRelativeChanges[i].size(); ++j)
                {
                    Eigen::Matrix3d rotation = (cameraStartPoses[i] * submapPosesRelativeChanges[i][j]).rotationMatrix();
                    Eigen::Vector3d translation = (cameraStartPoses[i] * submapPosesRelativeChanges[i][j]).translation();
                    int room_id = submap_to_room_for_each_camera[i][j];
                    //submap_id, rot, trans
                    ofs<<j<<" "<<rotation.block<1,3>(0,0)<<" "<<
                        rotation.block<1,3>(1,0)<<" "<<rotation.block<1,3>(2,0)<<" "<<translation.transpose()<<"\n";
                }
                ofs.close();
                std::ofstream ofs_final(filename_final);
                ofs_final<<submapPosesFinal[i].size()<<std::endl;
                for(int j = 0; j != submapPosesFinal[i].size(); ++j)
                {
                    Eigen::Matrix3d rotation = (cameraStartPoses[i] * submapPosesFinal[i][j]).rotationMatrix();
                    Eigen::Vector3d translation = (cameraStartPoses[i] * submapPosesFinal[i][j]).translation();
                    int room_id = submap_to_room_for_each_camera[i][j];
                    //submap_id, rot, trans
                    ofs_final<<j<<" "<<rotation.block<1,3>(0,0)<<" "<<
                        rotation.block<1,3>(1,0)<<" "<<rotation.block<1,3>(2,0)<<" "<<translation.transpose()<<"\n";
                }
                ofs_final.close();
            }
        }

        void reset()
        {
            //RESET the server
        }
        void save_base_paths()
        {
            std::string filename = "./base_paths.txt";
            std::ofstream ofs(filename);
            ofs<<base_paths.size();
            for(int i = 0; i != base_paths.size(); ++i)
            {
                ofs<<"\n"<<i<<" "<<base_paths[i];
            }
            ofs.close();
        }
        void change_submap_color(int camera_id, int submap_id, int room_pos)
        {

            auto &pcd = server_pcds[camera_id];
            if(submap_id >= pcd.size()) return;
            if(room_pos * 3 >= 120) room_pos = -1;
            int color_index = (room_pos + 1) * 3;
            int int_color = room_color_table[color_index];
            int_color = (int_color << 8) + room_color_table[color_index + 1];
            int_color = (int_color << 8) + room_color_table[color_index + 2];   
            pcd[submap_id].instance.clear();
            pcd[submap_id].instance.resize(pcd[submap_id].vertices.size(), int_color);
            /*
            for(int i = 0; i != pcd.vertices.size(); ++i)
            {
                pcd.instance[i] = int_color;
            }
            */
        }

        void change_submap_color(int camera_id)
        {
            auto &submaps = submap_to_room_for_each_camera[camera_id];
            auto &rooms = rooms_for_each_camera[camera_id];
            for(int i = 0; i !=submaps.size() ; ++i)
            {
                //std::cout<<camera_id<<" "<<i<<" "<<-1<<std::endl;
                if(submaps[i] == -1)
                {
                    change_submap_color(camera_id, i, -1);
                }
                else
                {
                    int room_pos = rooms[submaps[i]].position_in_database;
                    change_submap_color(camera_id, i, room_pos);
                }
            }
            // for(int room_id = 0; room_id != rooms.size(); ++room_id)
            // {
            //     auto &submaps = rooms[room_id].contained_submaps;
            //     int room_pos = rooms[room_id].position_in_database;
            //     for(int i = 0; i != submaps.size(); ++i)
            //     {
            //         change_submap_color(camera_id, submaps[i], room_pos);
            //     }
            // }
        }
        void save_all_room_info()
        {
            for(int i = 0; i != rooms_for_each_camera.size(); ++i)
            {
                std::string filename = "rooms/"+std::to_string(i)+".room";
                std::ofstream ofs(filename);
                ofs<<rooms_for_each_camera[i].size()<<std::endl;
                for(int j = 0; j != rooms_for_each_camera[i].size(); ++j)
                {
                    auto &room_info = rooms_for_each_camera[i][j];
                    //room_id, contained_submap
                    ofs<<room_info.room_id<<" "<<room_info.contained_submaps.size();
                    std::sort(room_info.contained_submaps.begin(), room_info.contained_submaps.end());
                    for(int k = 0; k != room_info.contained_submaps.size(); ++k)
                    {
                        ofs<<" "<<room_info.contained_submaps[k];
                    }
                    ofs<<"\n";
                }
                ofs.close();
            }
        }
//merge two nodes, node handle
        void merge(int camera_id1, int camera_id2, PoseSE3d &relative_pose_1_to_2)
        {

            int node_id1 = node_id[camera_id1];
            int node_id2 = node_id[camera_id2];

            std::cout<<"merge "<<node_id1<<" "<<node_id2<<std::endl;
            if(node_id1 == node_id2) return;
            for(auto i = camera_nodes[node_id2].camera.begin(); i!=camera_nodes[node_id2].camera.end(); ++i)
            {
                node_id[*i] = node_id1;
                if(*i != camera_id2)
                {
                    cameraStartPoses[*i] = cameraStartPoses[camera_id1] * relative_pose_1_to_2 * cameraStartPoses[camera_id2].inverse() * cameraStartPoses[*i];
                    //move_bodily(*i, relative_pose_1_to_2);
                }
            }
            
            cameraStartPoses[camera_id2] = cameraStartPoses[camera_id1] * relative_pose_1_to_2;
            std::cout<<"anchor camera pose: \n"<<cameraStartPoses[camera_id1].matrix()<<std::endl;
            std::cout<<"camera pose after merged: \n"<<cameraStartPoses[camera_id2].matrix()<<std::endl;
            //move_bodily(camera_id2, relative_pose_1_to_2);
            camera_nodes[node_id1].merge(camera_nodes[node_id2]);
        }

        void update_keyframe_poses(int camera_id,std::vector<collaborative_fusion::UpdateFrame> &updatedposes);
        //void update_keyframe(ServerFrame &f,std::vector<int > &candidate_index, std::vector<int > &candidate_submap, std::vector<int > & candidate_camera, bool &need_optimization);                                        
        void update_keyframe(ServerFrame &f,bool is_newsubmap,std::vector<int > &candidate_index, std::vector<int > &candidate_submap,
         std::vector<int > & candidate_camera, bool &need_optimization,bool &_submap_tracking_success,bool &_submap_lcd,int &closureID, int &closureIndex, geometry_msgs::Transform & t,geometry_msgs::Transform &camera_start_pose);     
        void preintegrate(int cameraID)
        {
            std::vector<Correspondence> &globalCorrespondences = camera_nodes[node_id[cameraID]].correspondences;            
            for(int i = 0;i!=globalCorrespondences.size();i++)
            {
                if(!globalCorrespondences[i].is_valid)
                continue;
                if(globalCorrespondences[i].use_icp)
                globalCorrespondences[i].preIntegrateICPInterSubmap(MultiViewGeometry::g_para.icp_weight);
                else
                globalCorrespondences[i].preIntegrateInterSubmap();
            }
        }
        void add_newsubmap(int cameraID, int last_submap_id, PoseSE3d &relative_pose)
        {
            submapPoses[cameraID].push_back(submapPoses[cameraID][last_submap_id] *relative_pose);
            submapPosesFinal[cameraID].push_back(submapPosesFinal[cameraID][last_submap_id] *relative_pose);
            submapPosesRelativeChanges[cameraID].push_back( submapPosesFinal[cameraID].back()* submapPoses[cameraID].back().inverse());
            submap_to_room_for_each_camera[cameraID].push_back(-1);
            keyframes[cameraID].rbegin()->second.pose_sophus[3] = submapPosesFinal[cameraID].back();
            //server_pcds[cameraID].push_back(Server::ServerPointCloud());
        }
        
        void useICPToAddPair(std::vector<RoomCorrespondence> &tmp_room_correspondences, Mat4dList &transformations);
        void update_node_keyframe(int camera_id)
        {
            int nid = node_id[camera_id];
            for(auto i = camera_nodes[nid].camera.begin(); i != camera_nodes[nid].camera.end(); ++i)
            {
                for(auto iter = keyframes[*i].begin(); iter != keyframes[*i].end(); ++iter)
                {
                    if(iter->second.submapID < submapPosesFinal[*i].size())
                    iter->second.pose_sophus[3] =submapPosesFinal[*i][iter->second.submapID] * iter->second.pose_sophus[2];
                }
            } 
        }
        bool hasRoom()
        {
            return scene_database.get_size() > 0;
        }
        void clear_useless_frame(int camera_id)
        {
            //TODO: clear the redundant part, remember to preserve the local_points.
            auto &frames = keyframes[camera_id];
            for(auto iter = frames.begin(); iter != frames.end(); ++iter)
            iter->second.clear_frame();
            std::cout<<"Clear all the useless parts of camera "<<camera_id<<"."<<std::endl;
        }
        bool check_if_stuck(int camera_id)
        {
            std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
            std::chrono::duration<float> diff = start_time - last_updated_time[camera_id];
            float elapsed_time = diff.count()*1000;
            if(elapsed_time > 100000) return true;
            return false;
        }
        void clear_stuck_camera()
        {
            for(int i = 0; i != submapPoses.size(); ++i)
            {
                if(is_terminated[i]) continue;
                if(keyframes[i].size() == 0) continue;
                if(check_if_stuck(i)) 
                {
                    std::cout<<"Clean agent "<<i<<std::endl;
                    is_terminated[i] = true;
                    clear_useless_frame(i);
                }
            }
        }
        void update_time(int camera_id)
        {
            last_updated_time[camera_id] =  std::chrono::steady_clock::now();
        }
        bool registerRoom(RoomInfo &room_info);
        std::vector<ServerMild> server_milds;
        int latest_keyframe_index;
        int latest_keyframe_camera;
        int latest_room_index;
        std::vector<std::map<int,ServerFrame>> keyframes; 
        std::vector<CameraNode > camera_nodes;
        std::vector<PoseSE3dList> submapPoses;                   
        std::vector<PoseSE3dList> submapPosesRelativeChanges; 
        std::vector<PoseSE3dList> submapPosesFinal;        
        std::vector<PoseSE3dList> room_poses;
        std::vector<Point3fList> submap_floor_normals;
        std::vector<std::vector<RoomInfo>> rooms_for_each_camera;
        std::vector<std::vector<int>> submap_to_room_for_each_camera;
        std::vector<std::string> base_paths;
        std::vector<bool> is_terminated;
        std::vector<std::chrono::steady_clock::time_point> last_updated_time; 
        Eigen::Matrix3f R;
        Eigen::Matrix4d T;
        SceneDatabase scene_database;
        //std::vector<bool> submapInitialized; 
        std::vector<int > node_id;
        PoseSE3dList cameraStartPoses;
        int camera_num;
        std::vector<std::vector<ServerPointCloud>> server_pcds;//for each agent and each submap in agent
        tool::Timer timer;

    };
    static ServerSLAM server_slam;
};

#endif