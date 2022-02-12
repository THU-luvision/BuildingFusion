#include "ServerSLAM.h"
#include "ServerOptimizer.h"
#include "../GCSLAM/ICPRegistration.h"

#if FOR_REBUTTAL
std::map<int, float> t_mild_lcd;
std::map<int, float> t_global_optimization;
std::map<int, float> t_room_lcd;
#endif

namespace Server 
{

void ServerSLAM::update_keyframe(ServerFrame &f,bool is_newsubmap,std::vector<int > &candidate_index, std::vector<int > &candidate_submap, std::vector<int > & candidate_camera,
 bool &need_optimization,bool &_submap_tracking_success, bool &_submap_lcd, int &closureID, int &closureIndex,  geometry_msgs::Transform &relative_pose, geometry_msgs::Transform &camera_start_pose)
{
  float scale_change_ratio;
  latest_keyframe_index = f.frame_index;
  latest_keyframe_camera = f.cameraID;
  bool update_keyframe_from_dense_matching =0;
  int submap_tracking_success = 0; 
  int camera_tracking_success = 0;
  int last_tracking_success = 0;
  int submap_lcd = 0;
  //keyframeIDCorr will only have the correspondency whose ref frame and new frame come from the same submap.
  //std::cout<<f.tracking_success<<std::endl;
  timer.Tick("mild");
  MILD::SparseMatcher sparseMatcher(FEATURE_TYPE_ORB, 32, 0, 50);
  sparseMatcher.train(f.getDescriptor());

  // loop closure detection

  /* close the loop closure detection if we use ICP*/
  std::cout<<"keypoints number: "<<f.keyPointsNum<<std::endl;

 
  if(f.keyPointsNum > MultiViewGeometry::g_para.closure_key_points)//
  {
    server_milds[f.cameraID].select_closure_candidates(f,candidate_index,candidate_submap,candidate_camera);
  }  

  int last_keyframe_index = -1; 
  int last_keyframe_submap = -1;
  if(keyframes[f.cameraID].size())
  {
  last_keyframe_index = keyframes[f.cameraID].rbegin()->second.frame_index;
  last_keyframe_submap = keyframes[f.cameraID].rbegin()->second.submapID;
  candidate_index .push_back(last_keyframe_index);
  candidate_submap.push_back(last_keyframe_submap);
  candidate_camera.push_back(f.cameraID);

  }


  //std::cout<<keyframes.size()<<" "<<f.cameraID<<" "<<f.frame_index<<std::endl;
  keyframes[f.cameraID][f.frame_index] = f;
  
  ServerFrame &frame_input = keyframes[f.cameraID][f.frame_index];
  
  //*********************** select candidates
  std::vector<Correspondence> fCorrCandidate;
  bool has_last_keyframe = false;
  for (size_t k = 0; k < candidate_index.size(); k++)
  {
    int candidate_frame_index = candidate_index[k];
    int candidate_cameraID = candidate_camera[k];
    int candidate_submapID = candidate_submap[k];
    if(candidate_cameraID != frame_input.cameraID) continue;
    if(candidate_cameraID == frame_input.cameraID  && std::fabs(candidate_submapID - frame_input.submapID) > 2)
    {
      if(submap_to_room_for_each_camera[candidate_cameraID][candidate_submapID] !=-1)
      continue;
    }
    if(is_newsubmap &&std::fabs( frame_input.submapID - candidate_submapID  > 1 ))
    continue;
    if(candidate_submapID != frame_input.submapID || candidate_cameraID != frame_input.cameraID ||candidate_frame_index == last_keyframe_index)
    {

      Correspondence global_frame_corr(keyframes[candidate_cameraID][candidate_frame_index], frame_input);
      if(candidate_frame_index == last_keyframe_index && is_newsubmap)
      global_frame_corr.setConnection(true);
      if(candidate_frame_index == last_keyframe_index)
      {
        global_frame_corr.allowance_icp = true;
        has_last_keyframe = true;
      }
      fCorrCandidate.push_back(global_frame_corr);
    }
    //if is new submap, only insert the connection parts
    
  }

  if(!has_last_keyframe && last_keyframe_index != -1)
  {
    std::cout<<YELLOW<<"Candidates don't contain last keyframe. "<<RESET<<std::endl;
    Correspondence global_frame_corr(keyframes[frame_input.cameraID][last_keyframe_index], frame_input);
    global_frame_corr.setConnection(true);
    global_frame_corr.allowance_icp = true;
    fCorrCandidate.push_back(global_frame_corr);
  }

  std::vector<float> average_disparity_list(fCorrCandidate.size());
  std::vector<int> registration_success_list(fCorrCandidate.size());
  PoseSE3dList relative_pose_from_ref_to_new_list(fCorrCandidate.size());
  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {

    registration_success_list[k] = false;
    average_disparity_list[k] = 1e8;
    registration_success_list[k] = Server::FrameMatchingTwoViewRGB(fCorrCandidate[k],
                                                                              MultiViewGeometry::g_camera,
                                                                              sparseMatcher,
                                                                              relative_pose_from_ref_to_new_list[k],
                                                                              average_disparity_list[k],
                                                                              scale_change_ratio,
                                                                              update_keyframe_from_dense_matching,0);
                                                
    relative_pose_from_ref_to_new_list[k] = relative_pose_from_ref_to_new_list[k].inverse();
  }
  timer.Tock("mild");
#if FOR_REBUTTAL
  t_mild_lcd[f.frame_index] = timer.Elapsed("mild");
#endif
  //update camera pose based on previous results
  float min_average_disparity_submap = 0.5;
  float min_average_disparity_camera = 0.5;
  float average_disparity_last_index = 0.5;
  int min_index_camera = -1;  
  int min_index_submap = -1;
  int min_last_index = -1;

//  std::cout << "average disparity / reprojection error: ";

  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
    std::cout << fCorrCandidate[k].frame_ref.frame_index << "	/"<< fCorrCandidate[k].frame_ref.cameraID << "	/"
      << average_disparity_list[k] << "	/"
      << registration_success_list[k] << std::endl;


    if(fCorrCandidate[k].frame_ref.frame_index == last_keyframe_index && fCorrCandidate[k].sameCamera == true )
    {
      min_last_index = k;
      if(average_disparity_last_index > average_disparity_list[k] && registration_success_list[k])
      {
      average_disparity_last_index = average_disparity_list[k];
      last_tracking_success = 1;
      }
    } 
    if(fCorrCandidate[k].sameCamera == true&& fCorrCandidate[k].sameSubmap == false && min_average_disparity_submap > average_disparity_list[k] && registration_success_list[k] )
    {
      min_average_disparity_submap = average_disparity_list[k];
      min_index_submap = k;
      submap_tracking_success = 1;
      if(std::fabs(fCorrCandidate[k].frame_ref.submapID - fCorrCandidate[k].frame_new.submapID) >2)
      submap_lcd = 1;
      continue;
    }
    else if(fCorrCandidate[k].sameCamera == false  &&  min_average_disparity_camera > average_disparity_list[k] && registration_success_list[k])
    {
        min_average_disparity_camera = average_disparity_list[k];
        min_index_camera = k;
        camera_tracking_success = 1;
        continue;
    }
  }
//  std::cout << "average disparity / reprojection error: ";

//  std::cout << std::endl;

  int currentSubmapID = frame_input.submapID;
  if(submap_tracking_success ) 
  {
    CommunicationAPI::toRosTransform(relative_pose_from_ref_to_new_list[min_index_submap],relative_pose);
  }

  if(submap_tracking_success || camera_tracking_success)
  need_optimization = true;
  else 
  {
    need_optimization = false;
    //closureCameraID = -1;
    closureID = -1;
    closureIndex = -1;  
  }
  _submap_tracking_success = submap_tracking_success;
  _submap_lcd = submap_lcd;
  //std::cout<<"submap_tracking_success/submapInitialized: "<<submap_tracking_success<<"/"<<submapInitialized[currentSubmapID]<<std::endl;
  if(last_tracking_success && !is_newsubmap)
  {
    frame_input.pose_sophus[2] =  keyframes[frame_input.cameraID][last_keyframe_index].pose_sophus[2] * relative_pose_from_ref_to_new_list[min_last_index];
    frame_input.pose_sophus[3] = submapPosesFinal[frame_input.cameraID][frame_input.submapID] * frame_input.pose_sophus[2];
  }
  else if(submap_tracking_success && !is_newsubmap)
  {
    frame_input.pose_sophus[2] = submapPosesFinal[frame_input.cameraID][currentSubmapID].inverse() *
      submapPosesFinal[frame_input.cameraID][fCorrCandidate[min_index_submap].frame_ref.submapID] * 
      fCorrCandidate[min_index_submap].frame_ref.pose_sophus[2] * relative_pose_from_ref_to_new_list[min_index_submap].inverse();
    frame_input.pose_sophus[3] = submapPosesFinal[frame_input.cameraID][frame_input.submapID] * frame_input.pose_sophus[2]; 
  }
  
  if(!last_tracking_success && last_keyframe_index != -1 && !is_newsubmap)
  {
    ServerFrame &last_frame = keyframes[frame_input.cameraID][last_keyframe_index];
    if(frame_input.tracking_success == true)
    {
      //PoseSE3d relative_transform_from_1_to_2 = frame_input.pose_sophus[2].inverse() * last_frame.pose_sophus[2];
      //std::cout<<"frame_input: "<<frame_input.tracking_success<<std::endl;
      //std::cout<<frame_input.pose_sophus[2].matrix()<<std::endl;
      frame_input.pose_sophus[3] = submapPosesFinal[frame_input.cameraID][currentSubmapID] * frame_input.pose_sophus[2];
      //ConstructCorrespondence(last_frame, frame_input, relative_transform_from_1_to_2, fCorrCandidate[min_last_index]);
      last_tracking_success = true;
      std::cout<<"use agent tracking result to locate."<<std::endl;
    }

  }

  if(is_newsubmap && !last_tracking_success)
  {
    std::cout<<RED<<"ERROR::Tracking Failed!!!!!!"<<RESET<<std::endl;
  }
  if(submap_tracking_success) 
  {

    int oldsubmap = keyframes[frame_input.cameraID][fCorrCandidate[min_index_submap].frame_ref.frame_index].submapID;
    closureID = oldsubmap;
    closureIndex = fCorrCandidate[min_index_submap].frame_ref.frame_index;
    //detectLoopClosure = true;
    //submapCorrLists.push_back(fCorrCandidate[min_index]);
  } 
#if 0
  if(camera_tracking_success && !is_newsubmap)
  {
    std::cout<<"detect overlap, start to calibrate!!!!!!!!!!!!!!!!!!!!"<<std::endl;
    int ref_cameraID = fCorrCandidate[min_index_camera].frame_ref.cameraID;
    int new_cameraID = fCorrCandidate[min_index_camera].frame_new.cameraID;
    int ref_submapID = fCorrCandidate[min_index_camera].frame_ref.submapID;
    int oldKeyFrameIndex = fCorrCandidate[min_index_camera].frame_ref.frame_index;
    PoseSE3d relative_pose_from_1_to_2;
    std::cout<<"tracking last keyframe? "<<last_tracking_success<<std::endl;

    if(node_id[ref_cameraID] != node_id[new_cameraID])
    {
      if(last_tracking_success || submap_tracking_success)
      {
        /*relative_pose_from_1_to_2 =  submapPosesFinal[ref_cameraID][ref_submapID] * keyframes[ref_cameraID][oldKeyFrameIndex].pose_sophus[2] *
      relative_pose_from_ref_to_new_list[min_index_camera] * (submapPosesFinal[new_cameraID][last_keyframe_submap]
      *keyframes[new_cameraID][last_keyframe_index].pose_sophus[2] * relative_pose_from_ref_to_new_list[min_last_index]).inverse() ;*/
      relative_pose_from_1_to_2 = submapPosesFinal[ref_cameraID][ref_submapID] * keyframes[ref_cameraID][oldKeyFrameIndex].pose_sophus[2] * 
        relative_pose_from_ref_to_new_list[min_index_camera].inverse() * 
          (submapPosesFinal[new_cameraID][currentSubmapID] *frame_input.pose_sophus[2] ).inverse();
      merge(ref_cameraID,new_cameraID,relative_pose_from_1_to_2);
    
      }
    /*
    else if(last_keyframe_index == -1)
    {
    relative_pose_from_1_to_2 =  submapPosesFinal[ref_cameraID][ref_submapID] * keyframes[ref_cameraID][oldKeyFrameIndex].pose_sophus[2] *
    relative_pose_from_ref_to_new_list[min_index_camera] ;      
    merge(ref_cameraID,new_cameraID,relative_pose_from_1_to_2);
    }*/
    }
    else
    {
      std::cout<<"Nodes are already merged!"<<std::endl;
    }
  }
#endif

  CommunicationAPI::toRosTransform(cameraStartPoses[f.cameraID],camera_start_pose);
  std::vector<Correspondence> &globalCorrespondences = camera_nodes[node_id[frame_input.cameraID]].correspondences;
  //if(last_tracking_success)
  if(last_tracking_success)
  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
    if (registration_success_list[k] &&(!fCorrCandidate[k].sameCamera || !fCorrCandidate[k].sameSubmap) )
    {
        //discard the keyframe which is far away from the ref frame
        Eigen::Vector3f camera_pos_ref = fCorrCandidate[k].frame_ref.pose_sophus[3].translation().cast<float>();
        Eigen::Vector3f camera_pos_new = fCorrCandidate[k].frame_new.pose_sophus[3].translation().cast<float>();
        float distance = (camera_pos_new - camera_pos_ref).norm();
        //because of the Rotation of model, the y of the camera pose is acturally the z axis.
        float z_dist = std::fabs(camera_pos_new(1) - camera_pos_ref(1));
        if(distance < 5 && z_dist < 3 || is_newsubmap == true)
        {
          globalCorrespondences.push_back(fCorrCandidate[k]);
        }
        else
        {
          std::cout<<"ignore correspondence using distance check."<<std::endl;
        }
    }
  }
  
    std::cout<<"globalCorrespondences: "<<globalCorrespondences.size()<<std::endl;
}
/*
float computeScoreByInstance(const chisel::PcdPtr & ref_pcd, const chisel::PcdPtr &new_pcd, 
    const std::vector<Eigen::Vector2i> &correspondence_set)
{
    std::map<long, std::vector<long>> ref_label2new_label; 
    std::map<long, int> ref_label_count, new_label_count;
    float score = 0;
    //count the points for each instance 
    for(int i = 0; i != ref_pcd->vertices.size(); ++i)
    {
        if(ref_label_count.find(ref_pcd->labels[i]) == ref_label_count.end())
            ref_label_count[ref_pcd->labels[i]] = 0;
        ref_label_count[ref_pcd->labels[i]] += 1;
    }
    for(int i = 0; i != new_pcd->vertices.size(); ++i)
    {

        if(new_label_count.find(new_pcd->labels[i]) == new_label_count.end())
            new_label_count[new_pcd->labels[i]] = 0;
        new_label_count[new_pcd->labels[i]] += 1;
    }    
    //find the corresponding points for the instance in reference pcd
    for(int i = 0; i != correspondence_set.size(); ++i)
    {
        ref_label2new_label[ref_pcd->labels[correspondence_set[i](0)]].push_back(new_pcd->labels[correspondence_set[i](1)]);
    }
    //find the corresponding instance, and compute the ratio.

    for(auto iter = ref_label2new_label.begin(); iter != ref_label2new_label.end(); ++iter)
    {
        float tmp_score = (iter->second.size() + 0.0) /  ref_label_count[iter->first];
        if(tmp_score > 1)
        tmp_score = 1;
        score += tmp_score;
        
    }
    //compute final score.
    score /= std::min(ref_label_count.size(), new_label_count.size());
    return score;
}*/
float computeScoreByInstance(const chisel::PcdPtr & ref_pcd, const chisel::PcdPtr &new_pcd, 
    const std::vector<Eigen::Vector2i> &correspondence_set, int min_instance_count = 200)
{
    std::map<long, int> ref_label_inlier; 
    std::map<long, int> new_label_inlier;
    std::map<long, int> ref_label_count, new_label_count;
    float new_score = 0, ref_score = 0;

    //count the points for each instance 
    for(int i = 0; i != ref_pcd->vertices.size(); ++i)
    {
        long label = ref_pcd->labels[i];
        int instance_label = get_instance_label(label);
        int semantic_label = get_semantic_label(label);
        if(semantic_label == CHAIR && instance_label != -1)
        {
            if(ref_label_count.find(ref_pcd->labels[i]) == ref_label_count.end())
                ref_label_count[ref_pcd->labels[i]] = 0;
            ref_label_count[ref_pcd->labels[i]] += 1;
        }
    }

    
    for(int i = 0; i != new_pcd->vertices.size(); ++i)
    {
        long label = new_pcd->labels[i];
        int instance_label = get_instance_label(label);
        int semantic_label = get_semantic_label(label);
        if(semantic_label == CHAIR && instance_label != -1)
        {
            if(new_label_count.find(new_pcd->labels[i]) == new_label_count.end())
                new_label_count[new_pcd->labels[i]] = 0;
            new_label_count[new_pcd->labels[i]] += 1;
        }
    }    
    //find the corresponding points for the instance in reference pcd
    for(int i = 0; i != correspondence_set.size(); ++i)
    {
        long ref_label = ref_pcd->labels[correspondence_set[i](0)];
        long new_label = new_pcd->labels[correspondence_set[i](1)];
        if(ref_label_count[ref_label] > min_instance_count)
        {
            if(ref_label_inlier.find(ref_label) == ref_label_inlier.end())
            ref_label_inlier[ref_label] = 0;
            ref_label_inlier[ref_label]+=1;
        }
        if(new_label_count[new_label]> min_instance_count)
        {
            if(new_label_inlier.find(new_label) == new_label_inlier.end())
            new_label_inlier[new_label] = 0;
            new_label_inlier[new_label]+=1;            
        }


    }
    int ref_valid_label = 0;
    int new_valid_label = 0;
    for(auto iter = ref_label_count.begin(); iter != ref_label_count.end(); ++iter)
    {
        if(iter->second > min_instance_count)
        ref_valid_label += 1;
    }
    for(auto iter = new_label_count.begin(); iter != new_label_count.end(); ++iter)
    {
        if(iter->second > min_instance_count)
        new_valid_label += 1;
    }
    //find the corresponding instance, and compute the ratio.
    for(auto iter = ref_label_inlier.begin(); iter != ref_label_inlier.end(); ++iter)
    {
        float tmp_score = (iter->second + 0.0) /  ref_label_count[iter->first];
        std::cout<<iter->first<<" "<<tmp_score<<std::endl;
        if(tmp_score > 1)
        tmp_score = 1;
        else if(tmp_score < 0.1)
        tmp_score = -0.1;
        ref_score += tmp_score;
    }
    for(auto iter = new_label_inlier.begin(); iter != new_label_inlier.end(); ++iter)
    {
        float tmp_score = (iter->second + 0.0) /  new_label_count[iter->first];
        std::cout<<iter->first<<" "<<tmp_score<<std::endl;
        if(tmp_score > 1)
        tmp_score = 1;
        else if(tmp_score < 0.1)
        tmp_score = -0.5;
        new_score += tmp_score;
    }
    
    //compute final score.
    //score /= std::min(ref_label_count.size(), new_label_count.size());
    ref_score /= ref_valid_label;
    new_score /= new_valid_label;
    std::cout<<"score: "<<ref_score<<" "<<new_score<<std::endl;
    return std::min(ref_score, new_score);
}
chisel::PcdPtr labelMask(const chisel::PcdPtr &origin_pcd)
{
    chisel::PcdPtr result_pcd(new chisel::PointCloud());
    std::map<long, std::vector<int>> label_points;
    for(int i = 0; i != origin_pcd->vertices.size(); ++i)
    {
        int semantic_label = get_semantic_label(origin_pcd->labels[i]);
        int instance_label = get_instance_label(origin_pcd->labels[i]);
        if(semantic_label == CHAIR && instance_label != -1)
        {
            label_points[origin_pcd->labels[i]].push_back(i);
        }
    }

    for(auto iter = label_points.begin(); iter != label_points.end(); ++iter)
    {
        auto &points = iter->second;
        if(iter->second.size() >= 500)
        {

            for(int i = 0; i != points.size(); ++i)
            {
                result_pcd->vertices.push_back(origin_pcd->vertices[points[i]]);
                result_pcd->normals.push_back(origin_pcd->normals[points[i]]);
                result_pcd->labels.push_back(origin_pcd->labels[points[i]]);
            }
        } 
    }
    return result_pcd;
}

void ServerSLAM::useICPToAddPair(std::vector<RoomCorrespondence> &tmp_room_correspondences, Mat4dList &transformations)
{
  transformations.resize(tmp_room_correspondences.size(), Eigen::Matrix4d::Identity());
  std::cout<<"use Icp to add correspondences.."<<std::endl;
  for(int i = 0; i != tmp_room_correspondences.size(); ++i)
  {
    transformations[i] = ICPTransformation(tmp_room_correspondences[i].instance_center_correspondences).cast<double>();
    std::cout<<"initialized_T: "<<transformations[i]<<std::endl;
    int source_camera_id = tmp_room_correspondences[i].source_camera_id;
    int target_camera_id = tmp_room_correspondences[i].target_camera_id;
    std::vector<pair<Eigen::Vector3f, Eigen::Vector3f>> point_pairs_3d;
    int source_room_id = tmp_room_correspondences[i].source_id;
    int target_room_id = tmp_room_correspondences[i].target_id;
    std::cout<<source_room_id <<" "<<source_camera_id<<" "<<target_room_id<<" "<<target_camera_id<<std::endl;
    int inlier_count = 0;
    auto &ref_pcd =rooms_for_each_camera[source_camera_id][source_room_id].model;// labelMask(rooms_for_each_camera[source_camera_id][source_room_id].model);
    auto &new_pcd =rooms_for_each_camera[target_camera_id][target_room_id].model; //labelMask(rooms_for_each_camera[target_camera_id][target_room_id].model);
    std::vector<Eigen::Vector2i> correspondence_set;
#if 1
    inlier_count = icp_registration::open3d_icp_registration(ref_pcd, new_pcd, transformations[i], point_pairs_3d, correspondence_set, 0.06);
#else
    inlier_count = icp_registration::open3d_icp_registration(new_pcd,
      ref_pcd, transformations[i], point_pairs_3d);
#endif
    tmp_room_correspondences[i].instance_center_correspondences =std::move(point_pairs_3d);
    tmp_room_correspondences[i].icp_inlier_ratio = (inlier_count + 0.0)/std::min(ref_pcd->vertices.size(),
      new_pcd->vertices.size());
    tmp_room_correspondences[i].score_by_instance = computeScoreByInstance(ref_pcd, new_pcd, correspondence_set);
    if(rooms_for_each_camera[source_camera_id][source_room_id].room_area > 45.0 && 
      rooms_for_each_camera[target_camera_id][target_room_id].room_area > 45.0)
      tmp_room_correspondences[i].score_by_instance *= 2;
    std::cout<<"inlier ratio: "<<tmp_room_correspondences[i].icp_inlier_ratio<<std::endl;
    std::cout<<"score: "<<tmp_room_correspondences[i].score_by_instance<<std::endl;
    std::cout<<"extract "<<tmp_room_correspondences[i].instance_center_correspondences.size()<<" pairs of points."<<std::endl;
  }
}
bool ServerSLAM::registerRoom(RoomInfo &room_info)
{
    std::vector<RoomCorrespondence> tmp_room_correspondences;
    assert(rooms_for_each_camera[room_info.camera_id].size() == room_info.room_id);
    room_info.position_in_database = scene_database.get_size();
    rooms_for_each_camera[room_info.camera_id].push_back(room_info);
    room_poses[room_info.camera_id].push_back(PoseSE3d());
    latest_room_index = room_info.room_id;
    auto obj_graph = get_graph(*(room_info.model), room_info.room_id, room_info.camera_id);
    timer.Tick("room lcd");
    scene_database.query(obj_graph, tmp_room_correspondences);
    timer.Tock("room lcd");
#if FOR_REBUTTAL
    t_room_lcd[latest_keyframe_index] = timer.Elapsed("room lcd");
#endif
    Mat4dList transformation_source_to_target;
    bool need_optimization = false;
    if(tmp_room_correspondences.size()>0)
      useICPToAddPair(tmp_room_correspondences, transformation_source_to_target);
    for(int i = 0; i < tmp_room_correspondences.size(); ++i)
    {
      tmp_room_correspondences[i].Rotate(R.inverse());
      transformation_source_to_target[i] = T.inverse() * transformation_source_to_target[i] * T;
    }  
    if(tmp_room_correspondences.size() > 0)
    {
      for(int i = 0; i != tmp_room_correspondences.size();++i)
      {
        if(tmp_room_correspondences[i].icp_inlier_ratio < MultiViewGeometry::g_para.room_lcd_min_icp_ratio||
          tmp_room_correspondences[i].score_by_instance < MultiViewGeometry::g_para.room_lcd_min_score)
        continue;
        
        int source_camera_id = tmp_room_correspondences[i].source_camera_id;
        int target_camera_id = tmp_room_correspondences[i].target_camera_id;
        int source_room_id = tmp_room_correspondences[i].source_id;
        int target_room_id = tmp_room_correspondences[i].target_id;
        if(node_id[source_camera_id] != node_id[target_camera_id])
        {
          PoseSE3d relative_pose_from_1_to_2(transformation_source_to_target[i].inverse());
          merge(source_camera_id, target_camera_id, relative_pose_from_1_to_2);
        }
        else
        {
          //height check, don't know if is necessary.
          /*
          Eigen::Vector3d ref_position = (cameraStartPoses[source_camera_id] * room_poses[source_camera_id][source_room_id]).translation();
          Eigen::Vector3d new_position = (cameraStartPoses[target_camera_id] * room_poses[target_camera_id][target_room_id]).translation();
          if(std::fabs(ref_position(1) - new_position(1)) > 3.5)
          {
            tmp_room_correspondences[i].icp_inlier_ratio = 0;
            std::cout<<ref_position.transpose()<<" "<<new_<<std::endl;
            std::cout<<"Reject room correspondence using height check."<<std::endl;
          }*/
        }
        //tmp_room_correspondences[i].preIntegrate(room_poses);
      }

      std::vector<RoomCorrespondence> &room_correspondences = camera_nodes[node_id[room_info.camera_id]].room_correspondences;   
      for(int i = 0; i != tmp_room_correspondences.size(); ++i)
      {
        if(tmp_room_correspondences[i].icp_inlier_ratio < MultiViewGeometry::g_para.room_lcd_min_icp_ratio||
            tmp_room_correspondences[i].score_by_instance < MultiViewGeometry::g_para.room_lcd_min_score)
            continue;
          room_correspondences.push_back(tmp_room_correspondences[i]);
      }
      need_optimization = true;
    } 

    scene_database.insert(obj_graph);
    
    for(int i = 0; i != room_info.contained_submaps.size(); ++i)
    {
      //std::cout<<room_info.contained_submaps[i]<<std::endl;
      submap_to_room_for_each_camera[room_info.camera_id][room_info.contained_submaps[i]] = room_info.room_id;
    }

    return need_optimization;
}
void ServerSLAM::update_keyframe_poses(int cameraID,std::vector<collaborative_fusion::UpdateFrame> &updatedposes)
{
    std::map<int, ServerFrame> &wait_to_update = keyframes[cameraID];
    for(int i = 0;i!= updatedposes.size(); ++i)
    {
        CommunicationAPI::updateServerFrame(updatedposes[i],wait_to_update[updatedposes[i].frame_index]);
    }
}
};