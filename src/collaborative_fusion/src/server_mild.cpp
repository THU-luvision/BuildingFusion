#include <ros/ros.h>
#include "server/ServerGui.h"
#include "server/ServerSLAM.h"
#include "server/ServerOptimizer.h"
#include "BasicAPI.h"
#include <collaborative_fusion/LoopClosureDetection.h>
#include <collaborative_fusion/GlobalOptimization.h>
#include <collaborative_fusion/MeshVisualization.h>
#include <collaborative_fusion/Registration.h>
#include <collaborative_fusion/RoomRegistration.h>
#include <collaborative_fusion/Termination.h>
#include <thread>


//std::vector<float> time_consuming;
static int new_camera_id = 0;


void thread_gui()
{
    while(true)
    {
    Server::server_gui.show(Server::server_slam);
    usleep(30000);
    }
}
void update_meshes(int camera_id)
{
    Server::server_gui.updateMeshes(camera_id,Server::server_slam.server_pcds[camera_id], 
        Server::server_slam.submapPosesRelativeChanges[camera_id], Server::server_slam.cameraStartPoses[camera_id]);   
}
void update_all_meshes()
{
    for(int i = 0; i != Server::server_slam.cameraStartPoses.size(); ++i)
        update_meshes(i);
}
bool handleFunction_registration(collaborative_fusion::Registration::Request &req, collaborative_fusion::Registration::Response & res)
{
    if(req.ready)
    {
        res.camera_id = new_camera_id;
        res.calib_parameters = Server::server_slam.GetCalibPara();
    }
    new_camera_id++;
    Server::server_slam.addCamera(req.base_path);
    ROS_INFO("register camera: %d" ,res.camera_id);
    return 1;
}
bool handleFunction_termination(collaborative_fusion::Termination::Request &req, collaborative_fusion::Termination::Response &res)
{
    if(req.terminate)
    {
        Server::server_slam.save_all_submap_info();
        Server::server_slam.save_all_room_info();
        Server::server_slam.save_base_paths();
        Server::server_slam.reset();

        ROS_INFO("Receive a termination request, server reset all the date and preserve the submap poses.");
        return 1;
    }
}
bool handleFunction_withdraw(collaborative_fusion::WithDraw::Request &req, collaborative_fusion::WithDraw::Response & res)
{
    if(req.withdraw)
    {
        Server::server_slam.server_pcds[req.camera_id].clear();
        ROS_INFO("withdraw meshes: %d" ,req.camera_id);
    }
#if FOR_REBUTTAL

    {
        std::ofstream file_submap_corr("./submap_corrs.txt", std::ios::binary | std::ios::out);
        std::vector<double> buffer;
        buffer.push_back(Server::server_slam.camera_nodes[req.camera_id].correspondences.size());
        for(int i = 0; i != Server::server_slam.camera_nodes[req.camera_id].correspondences.size(); ++i)
        {
            // save the ref_submap, the new submap, save the sum_value
            auto &corr = Server::server_slam.camera_nodes[req.camera_id].correspondences[i];
            if(!corr.is_valid) continue;
            if(corr.use_icp)
            corr.preIntegrateICPInterSubmap(MultiViewGeometry::g_para.icp_weight);
            else corr.preIntegrateInterSubmap();
            buffer.push_back(corr.frame_ref.submapID);
            buffer.push_back(corr.frame_new.submapID);

            BasicAPI::EigenMatrixToStdVector(corr.sum_p_ref_s, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_new_s, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_ref_new_s, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_ref_ref_s, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_new_new_s, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_new_ref_s, buffer);
            buffer.push_back(corr.sum_weight_s);
        }
        file_submap_corr.write((char *) &buffer[0],sizeof(double) * buffer.size());
        file_submap_corr.close();
    }
    {
        std::ofstream file_keyframe_corr("./keyframe_corrs_server.txt", std::ios::binary | std::ios::out);
        std::vector<double> buffer;
        buffer.push_back(Server::server_slam.camera_nodes[req.camera_id].correspondences.size());

        for(int i = 0; i != Server::server_slam.camera_nodes[req.camera_id].correspondences.size(); ++i)
        {
            // save the ref_submap, the new submap, save the sum_value for keyframe 
            auto &corr = Server::server_slam.camera_nodes[req.camera_id].correspondences[i];
            if(!corr.is_valid) continue;
            if(corr.use_icp)
            corr.preIntegrateICP(MultiViewGeometry::g_para.icp_weight);
            else corr.preIntegrate();

            buffer.push_back(corr.frame_ref.frame_index);
            buffer.push_back(corr.frame_new.frame_index);

            BasicAPI::EigenMatrixToStdVector(corr.sum_p_ref, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_new, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_ref_new, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_ref_ref, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_new_new, buffer);
            BasicAPI::EigenMatrixToStdVector(corr.sum_p_new_ref, buffer);
            buffer.push_back(corr.sum_weight);
        }
        file_keyframe_corr.write((char *) &buffer[0],sizeof(double) * buffer.size());
        file_keyframe_corr.close();
        //save submap information
    }
    {
        std::ofstream file_submap("./submaps_info.txt", std::ios::binary | std::ios::out);
        std::vector<double> buffer;
        buffer.push_back(Server::server_slam.submapPosesFinal[req.camera_id].size());

        for(int i = 0; i != Server::server_slam.submapPosesFinal[req.camera_id].size(); ++i)
        {
            auto &spf = Server::server_slam.submapPosesFinal[req.camera_id][i];
            buffer.push_back(i);

            Eigen::Matrix3d R = spf.rotationMatrix();
            Eigen::Vector3d t = spf.translation();
            BasicAPI::EigenMatrixToStdVector(R, buffer);
            BasicAPI::EigenMatrixToStdVector(t, buffer);
        }
        file_submap.write((char *) &buffer[0],sizeof(double) * buffer.size());
        file_submap.close();
    }
    std::ofstream file_room_lcd("./running_time/room_lcd.txt");
    for(auto iter = t_room_lcd.begin(); iter != t_room_lcd.end(); ++iter)
    {
        file_room_lcd << iter->first<<" "<<iter->second<<std::endl;
    }
    file_room_lcd.close();

    std::ofstream file_mild_lcd("./running_time/mild_lcd.txt");
    for(auto iter = t_mild_lcd.begin(); iter != t_mild_lcd.end(); ++iter)
    {
        file_mild_lcd << iter->first<<" "<<iter->second<<std::endl;
    }    
    file_mild_lcd.close();

    std::ofstream file_global_optimization("./running_time/global_optimization.txt");
    for(auto iter = t_global_optimization.begin(); iter != t_global_optimization.end(); ++iter)
    {
        file_global_optimization << iter->first<<" "<<iter->second<<std::endl;
    } 
    file_global_optimization.close();
#endif

    Server::server_slam.clear_useless_frame(req.camera_id);
    Server::server_slam.is_terminated[req.camera_id] = true;
    res.success = true;
    return 1;
}
bool handleFunction_detection(collaborative_fusion::LoopClosureDetection::Request &req, collaborative_fusion::LoopClosureDetection::Response & res)
{
    ROS_INFO("detection request from client %d, frame index: %d, submap index: %d", req.new_frame.camera_id, req.new_frame.frame_index, req.new_frame.submap_id);
    Server::server_slam.timer.Tick("loop closure detection");
    std::vector<int > index, submap, camera;
    bool need_optimization;
    int closureID, closureIndex;
    collaborative_fusion::FramePose poses;
    Server::ServerFrame newFrame;
    Server::server_slam.timer.Tick("format switching");
    Server::server_slam.update_time(req.new_frame.camera_id);
    Server::server_slam.clear_stuck_camera();
    CommunicationAPI::toServerFrame(req.new_frame,newFrame);
    /*
    std::string filename_depth = std::to_string(newFrame.cameraID) + "_"+std::to_string(newFrame.frame_index)+"_d.png";
    std::string filename_rgb = std::to_string(newFrame.cameraID) + "_"+std::to_string(newFrame.frame_index)+"_r.png";
    cv::imwrite(filename_depth, newFrame.depth);
    cv::imwrite(filename_rgb, newFrame.rgb);
    */
    newFrame.tracking_success = req.tracking_success;
    //std::cout<<newFrame.tracking_success<<std::endl;
    Server::server_slam.timer.Tock("format switching");
    geometry_msgs::Transform relative_pose;
    bool submap_tracking_success, submap_lcd;
    geometry_msgs::Transform camera_start_pose;
    Server::server_slam.timer.Tick("Update Keyframe");
    Server::server_slam.update_keyframe(newFrame,req.is_newsubmap,index,submap,camera,
        need_optimization,submap_tracking_success,submap_lcd,closureID,closureIndex,relative_pose,camera_start_pose);
    Server::server_slam.timer.Tock("Update Keyframe");
    std::cout<<Server::server_slam.keyframes[req.new_frame.camera_id][req.new_frame.frame_index].pose_sophus[3].log().transpose()<<std::endl;
    res.frame_candidates_index = index;
    res.frame_candidates_submap =submap;
    res.frame_candidates_camera = camera;
    res.need_optimization = need_optimization;
    res.submap_tracking_success = submap_tracking_success;
    res.submap_lcd = submap_lcd;
    res.closure_submapID = closureID;
    res.closure_index = closureIndex;
    res.relative_pose = relative_pose;
    res.camera_start_pose = camera_start_pose;
    Server::server_slam.timer.Tick("show");
    //if(res.new_frame.camera_id  == 0 && res.new_frame.camera_id )
    Server::server_gui.show(Server::server_slam);
    Server::server_slam.timer.Tock("show");
    Server::server_slam.timer.Tock("loop closure detection");
    Server::server_slam.timer.LogAll();
    return 1;
}

bool handleFunction_optimization(collaborative_fusion::GlobalOptimization::Request &req, collaborative_fusion::GlobalOptimization::Response & res)
{
    ROS_INFO("optimization request from client %d: ", req.camera_id);
    std::vector<collaborative_fusion::UpdateFrame> update_frames = req.updated_frames;
    Server::server_slam.update_keyframe_poses(req.camera_id,update_frames);
    if(req.is_newsubmap)
    {
    std::cout<<"Add new submap poses! "<<std::endl;
    PoseSE3d submap_relative_pose;
    CommunicationAPI::toCvTransform(req.submap_relative_pose,submap_relative_pose);
    Server::server_slam.add_newsubmap(req.camera_id,req.last_submap_id,submap_relative_pose);
    }

    CommunicationAPI::toEigenVec3f(req.floor_normals, Server::server_slam.submap_floor_normals[req.camera_id]);
    
    std::cout<<"room: "<<Server::server_slam.rooms_for_each_camera[req.camera_id].size()<<std::endl;
    /*
    for(int i = 0;i!=Server::server_slam.globalCorrespondences.size();++i)
    {
        Server::server_slam.globalCorrespondences[i].printSum();
    }*/
    Server::server_slam.update_node_keyframe(req.camera_id);
    Server::server_slam.timer.Tick("combining optimization");

        // optimization with normal
    // std::cout<<"TRIGGER NORMAL OPTIMIZATION!"<<std::endl;
    if(g_para.normal_optimization)
    Server::combineOptimization(Server::server_slam,req.camera_id, true, true);
    else 
    Server::combineOptimization(Server::server_slam,req.camera_id, true);        


    Server::server_slam.timer.Tock("combining optimization");
#if FOR_REBUTTAL
    t_global_optimization[Server::server_slam.latest_keyframe_index] = Server::server_slam.timer.Elapsed("combining optimization");
#endif
    std::vector<geometry_msgs::Transform> submap_poses;
    CommunicationAPI::toRosPoseList(Server::server_slam.submapPosesRelativeChanges[req.camera_id],submap_poses);
    res.submap_poses = submap_poses;
    //Server::server_gui.show(Server::server_slam);
    return 1;
}
bool handleFunction_visualization(collaborative_fusion::MeshVisualization::Request &req, collaborative_fusion::MeshVisualization::Response & res)
{
    ROS_INFO("visualization request from client %d: ", req.camera_id);
    //Server::server_slam.timer.Tick("visualization");
    std::cout<<"transmit to server pointcloud, submap size: "<<req.pcds.size()<<std::endl;

    CommunicationAPI::toServerPointCloud(req.pcds, Server::server_slam.server_pcds[req.camera_id]);
    std::cout<<"change submap color..."<<std::endl;
    Server::server_slam.change_submap_color(req.camera_id);
    //Server::server_gui.updateMeshes(req.camera_id,Server::server_slam.server_pcds[req.camera_id], Server::server_slam.cameraStartPoses[req.camera_id]);
    //update_meshes(req.camera_id);
    std::cout<<"update all meshes..."<<std::endl;
    update_all_meshes();
    Server::server_gui.putMeshInBuffer();
    Server::server_gui.global_vertex_data_updated=true;
    Server::server_gui.has_data = true;
    //thread_gui(Server::server_gui, Server::server_slam);
    Server::server_gui.show(Server::server_slam);
    //Server::server_slam.timer.Tock("visualization");
    //Server::server_slam.timer.LogAll();
    return 1;
}

bool handleFunction_room_registration(collaborative_fusion::RoomRegistration::Request& req, collaborative_fusion::RoomRegistration::Response & res)
{
    int camera_id = req.room_info.camera_id;
    ROS_INFO("room registration request from client %d", camera_id);
    Server::RoomInfo room_info;
    ROS_INFO("room area: %f", req.room_info.room_area);
    CommunicationAPI::toServerRoomInfo(req.room_info, room_info);
    room_info.ComputeFloorNormal();
    Server::server_slam.registerRoom(room_info);
    Server::server_slam.server_milds[camera_id].reset();
    Server::server_slam.update_node_keyframe(camera_id);    
    if(g_para.normal_optimization)
    Server::combineOptimization(Server::server_slam, camera_id, false, true);
    else 
    Server::combineOptimization(Server::server_slam, camera_id, false);

    std::vector<geometry_msgs::Transform> submap_poses;
    CommunicationAPI::toRosPoseList(Server::server_slam.submapPosesRelativeChanges[camera_id],submap_poses);
    //for(int i = 0; i != Server::server_slam.submapPosesRelativeChanges[camera_id].size(); ++i)
    //std::cout<<Server::server_slam.submapPosesRelativeChanges[camera_id][i].matrix()<<std::endl<<std::endl;
    res.submap_poses = submap_poses;
    return 1;
}
int main(int argc, char *argv[])
{
    std::string filename="./calib.txt";

    ros::init(argc, argv, "collaborative_fusion_server"); 
    if(argc > 1)
    {
        filename = argv[1];
        std::cout<<filename<<std::endl;
    }
    ros::NodeHandle n;
    // Eigen::Matrix4d mvp;
    // mvp<<0.836796,  0.00559278,    -1.01114,     1.39663,
    // 1.34794,     0.02841,     1.11568,     1.75194,
    // -0.0152264,     1.00006, -0.00706958,     16.1094,
    // -0.0152234,    0.999859, -0.00706816,     16.3062;
    // Server::server_gui.setModelViewMatrix(mvp);
    BasicAPI::loadGlobalParameters(MultiViewGeometry::g_para,"./settings.yaml");

    BasicAPI::initCamera(MultiViewGeometry::g_camera,filename);
    //Server::server_slam.setCameraNum(MultiViewGeometry::g_para.camera_num);
    Server::server_gui.setCameraNum(MultiViewGeometry::g_para.camera_num);
    float threshold = 0.05;
    BasicAPI::makeDir("./global_submap_pose");
    BasicAPI::makeDir("./rooms"); 
    Server::Estimator.Initialize(threshold, 100);
    Server::server_slam.scene_database.initialize_estimator();
    update_all_meshes();
    Server::server_gui.putMeshInBuffer();
    Server::server_gui.global_vertex_data_updated=true;
    Server::server_gui.has_data = true;
    //thread_gui(Server::server_gui, Server::server_slam);
    Server::server_gui.show(Server::server_slam);
    ros::ServiceServer server_registration = n.advertiseService("registration",handleFunction_registration);
    ros::ServiceServer server_detection = n.advertiseService("loop_closure_detection",handleFunction_detection);
    ros::ServiceServer server_optimization = n.advertiseService("global_optimization",handleFunction_optimization);
    ros::ServiceServer server_visualization = n.advertiseService("mesh_visualization",handleFunction_visualization);
    ros::ServiceServer server_room_registration = n.advertiseService("room_registration",handleFunction_room_registration);
    ros::ServiceServer server_withdraw = n.advertiseService("withdraw_mesh",handleFunction_withdraw);
    ros::ServiceServer server_termination = n.advertiseService("server_termination", handleFunction_termination);
    ros::spin();
    return 0;
}
