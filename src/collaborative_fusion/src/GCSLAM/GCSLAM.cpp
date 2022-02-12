#include "GCSLAM.h"
#include "../CHISEL/src/open_chisel/Stopwatch.h"






bool GCSLAM::allInitialized = false;
std::mutex GCSLAM::update_kflist_mutex;
#if FOR_REBUTTAL
//running time of each components
std::map<int, float> t_tracking;
std::map<int, float> t_local_optimization;
std::vector<MultiViewGeometry::FrameCorrespondence> CorrIdList;
PoseSE3dList relative_pose_list;
#endif
void GCSLAM::select_closure_candidates(Frame &f, std::vector<int> &candidate_frame_index,bool inside_submap)
{

  std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist = KeyframeDataList;
  MILD::LoopClosureDetector & lcd = mild;
  //std::vector<size_t> &kfplist = keyframeDataPtrList;
  MILD::BayesianFilter spatial_filter;
  std::vector<float > similarity_score;
  lcd.query_database(f.getDescriptor(), similarity_score);
  // select candidates
  candidate_frame_index.clear();
  std::vector<float> salient_score;
  std::vector<MILD::LCDCandidate> candidates;
  spatial_filter.calculateSalientScore(similarity_score, salient_score);

  TICK("GCSLAM::GlobalRegistration");

  //only select top 5, disgard the last frame
  for (int k = 0; k < kflist.size() - 1; k++)
  {

    if (salient_score[k] > salientScoreThreshold)
    {

    MILD::LCDCandidate candidate(salient_score[k],k,0,0);
    candidates.push_back(candidate);
    }
  }
  std::sort(candidates.begin(), candidates.end(),greater<MILD::LCDCandidate>());
  for (int k = 0; k < fmin(candidates.size(), maxCandidateNum); k++)
  {
//    std::cout << kflist[candidates[k].index].keyFrameIndex << " " << candidates[k].salient_score << std::endl;
  candidate_frame_index.push_back(candidates[k].index);
  }
  
  std::string candidates_str = "candidates: ";
  //std::string candidates_score;
  for (int k = 0; k < candidate_frame_index.size(); k++)
  {
    candidates_str += std::to_string(kflist[candidate_frame_index[k]].keyFrameIndex) + " ";
  }
  std::cout << "running frame : " << f.frame_index << " " << candidates_str << std::endl;
}
#if 1
void GCSLAM::update_keyframe(int newKeyFrameIndex,
                         MultiViewGeometry::FrameCorrespondence &key_frame_corr,
                         float average_disparity,
                         PoseSE3d relative_pose_from_key_to_new,
                         int registration_success)
{
  float scale_change_ratio;

  bool update_keyframe_from_dense_matching =0;
  int global_tracking_success = 0;

  std::set<int> &keyCorrIDs = keyFrameIDCorrs[0]; 
  std::vector<MultiViewGeometry::FrameCorrespondence> &fCorrList_keyframes = keyFrameCorrLists[0];
  std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist = KeyframeDataList;
  MILD::LoopClosureDetector & lcd = mild;
  std::vector<Frame> & frame_list = globalFrameList;

  MILD::SparseMatcher sparseMatcher(FEATURE_TYPE_ORB, 32, 0, 50);
  sparseMatcher.train(frame_list[newKeyFrameIndex].descriptor);
  // loop closure detection
  std::vector<int> candidate_keyframes;

  TICK("GCSLAM::MILD");
  select_closure_candidates(frame_list[newKeyFrameIndex],candidate_keyframes);

  TOCK("GCSLAM::MILD");

  TICK("GCSLAM::GlobalRegistration");
  //*********************** add current keyframe to keyframe list
  MultiViewGeometry::KeyFrameDatabase kfd(newKeyFrameIndex);
  kflist.push_back(kfd);

  //*********************** select candidates

  std::vector<MultiViewGeometry::FrameCorrespondence> fCorrCandidate;

  for (size_t k = 0; k < candidate_keyframes.size(); k++)
  {
    int candidate_frame_index = kflist[candidate_keyframes[k]].keyFrameIndex;
    MultiViewGeometry::FrameCorrespondence global_frame_corr(frame_list[candidate_frame_index], frame_list[newKeyFrameIndex]);
    fCorrCandidate.push_back(global_frame_corr);
//    std::cout << "candidate key frame: " << kflist[candidate_keyframes[k]].keyFrameIndex << std::endl;
  }

  std::vector<float> average_disparity_list(candidate_keyframes.size());
  std::vector<int> registration_success_list(candidate_keyframes.size());
  PoseSE3dList relative_pose_from_ref_to_new_list(candidate_keyframes.size());
  for (size_t k = 0; k < candidate_keyframes.size(); k++)
  {
    int candidate_frame_index = kflist[candidate_keyframes[k]].keyFrameIndex;

    registration_success_list[k] = 1e8;
    average_disparity_list[k] = 1e8;
    registration_success_list[k] = MultiViewGeometry::FrameMatchingTwoViewRGB(fCorrCandidate[k],
                                                                              camera_para,
                                                                              sparseMatcher,
                                                                              relative_pose_from_ref_to_new_list[k],
                                                                              average_disparity_list[k],
                                                                              scale_change_ratio,
                                                                              update_keyframe_from_dense_matching, local_tracking_cnt);
    relative_pose_from_ref_to_new_list[k] = relative_pose_from_ref_to_new_list[k].inverse();

  }
  relative_pose_from_ref_to_new_list.push_back(relative_pose_from_key_to_new);
  registration_success_list.push_back(registration_success);
  average_disparity_list.push_back(average_disparity);
  fCorrCandidate.push_back(key_frame_corr);

  //update camera pose based on previous results
  float min_average_disparity = 1e9;
  int min_index = 0;
//  std::cout << "average disparity / reprojection error: ";
  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
//    std::cout << fCorrCandidate[k].frame_ref.frame_index << "	/"
//      << average_disparity_list[k] << "	/"
//      << registration_success_list[k] << std::endl;
    if (min_average_disparity > average_disparity_list[k] && registration_success_list[k])
    {
      min_average_disparity = average_disparity_list[k];
      min_index = k;
      global_tracking_success = 1;
    }
  }
//  std::cout << std::endl;

  int current_map_origin = 0;
  if (global_tracking_success == 1)
  {
    frame_list[newKeyFrameIndex].tracking_success = 1;

    frame_list[newKeyFrameIndex].pose_sophus[2] = frame_list[fCorrCandidate[min_index].frame_ref.frame_index].pose_sophus[2]
            * relative_pose_from_ref_to_new_list[min_index] ;
    frame_list[newKeyFrameIndex].pose_sophus[0] = submapPoses[0]*frame_list[newKeyFrameIndex].pose_sophus[2];
    frame_list[newKeyFrameIndex].pose_sophus[3] = submapPosesFinal[0] * frame_list[newKeyFrameIndex].pose_sophus[2];

    current_map_origin = frame_list[fCorrCandidate[min_index].frame_ref.frame_index].origin_index;
  }


  if(!global_tracking_success)
  {
    current_map_origin = newKeyFrameIndex;
//      std::cout << "update anchor keyframe index! " << std::endl;
  }
  else
  {

    std::vector<int> matched_frames;
    for (size_t k = 0; k < fCorrCandidate.size(); k++)
    {
      if (registration_success_list[k] )
      {
        matched_frames.push_back(fCorrCandidate[k].frame_ref.origin_index);
      }
    }
    current_map_origin = *max_element(matched_frames.begin(), matched_frames.end());
  }
//  std::cout << "add new keyframe!" << std::endl;
  frame_list[newKeyFrameIndex].is_keyframe = 1;
  frame_list[newKeyFrameIndex].origin_index = current_map_origin;
  int reg_success_cnt = 0;
  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
    if (registration_success_list[k] )
    {
        reg_success_cnt ++;
    }
  }
  if(reg_success_cnt < 4)
  {
      lcd.construct_database(frame_list[newKeyFrameIndex].descriptor, newKeyFrameIndex, 0, 0);
  }
  else
  {
      cv::Mat descriptor;
      descriptor.release();
      lcd.construct_database(descriptor, newKeyFrameIndex, 0, 0);
  }
  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
    if (registration_success_list[k] )
    {
      fCorrList_keyframes.push_back(fCorrCandidate[k]);
      keyCorrIDs.insert(fCorrCandidate[k].frame_ref.frame_index);
      keyCorrIDs.insert(fCorrCandidate[k].frame_new.frame_index);
    }
  }
  updateMapOrigin(fCorrCandidate, keyCorrIDs,registration_success_list,newKeyFrameIndex);
  TOCK("GCSLAM::GlobalRegistration");
}
#endif

void GCSLAM::update_keyframe(ros::ServiceClient &client,int newKeyFrameIndex,
                          MultiViewGeometry::FrameCorrespondence &key_frame_corr,
                          float average_disparity,
                          PoseSE3d relative_pose_from_key_to_new,
                          int registration_success,int currentSubmapID,int &closureID, int &closureIndex, bool &detectLoopClosure,int &needCollibration,bool is_newSubmap)
{
  float scale_change_ratio;

  bool update_keyframe_from_dense_matching =0;
  int keyframe_tracking_success = 0;
  int submap_tracking_success = 0; 
  int camera_tracking_success = 0;
  PoseSE3d relative_pose_submap;
  std::vector<MultiViewGeometry::FrameCorrespondence> &fCorrList_keyframes = keyFrameCorrLists[currentSubmapID];
  std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist = KeyframeDataList;
  //std::vector<size_t> &kfplist = keyframeDataPtrList;
  //keyframeIDCorr will only have the correspondency whose ref frame and new frame come from the same submap.
  std::set<int> &keyCorrIDs = keyFrameIDCorrs[currentSubmapID]; 
  MILD::LoopClosureDetector & lcd = mild;
  std::vector<Frame> & frame_list = globalFrameList;
  std::vector<int> candidate_keyframes_index;
  std::vector<int> candidate_keyframes_submap;
  std::vector<int> candidate_keyframes;
  std::vector<int> candidate_keyframes_camera;
  int last_keyframe_index = key_frame_corr.frame_ref.frame_index;
  MILD::SparseMatcher sparseMatcher(FEATURE_TYPE_ORB, 32, 0, 50);
  sparseMatcher.train(frame_list[newKeyFrameIndex].getDescriptor());
#if 0
  // loop closure detection
  TICK("GCSLAM::MILD");
  /* close the loop closure detection if we use ICP*/
  if(key_frame_corr.use_icp)
  {
  std::cout<<"ICP key frame, no need to select closure candidates."<<std::endl;
  }
  else if(!allInitialized||frame_list[newKeyFrameIndex].keyPointsNum > MultiViewGeometry::g_para.closure_key_points)//else if(frame_list[newKeyFrameIndex].local_points.size() > 1900)//(newKeyFrameIndex >= 1819 &&newKeyFrameIndex <= 2037  || newKeyFrameIndex >= 14680 ) //
  select_closure_candidates(frame_list[newKeyFrameIndex],candidate_keyframes);
  else
  select_closure_candidates(frame_list[newKeyFrameIndex],candidate_keyframes,true);
 //select_closure_candidates(frame_list[newKeyFrameIndex],candidate_keyframes);
  //if(newKeyFrameIndex >=9280)
  TOCK("GCSLAM::MILD");
      //std::cout<<"point1"<<std::endl;
  TICK("GCSLAM::GlobalRegistration");
  //*********************** add current keyframe to keyframe list
  MultiViewGeometry::KeyFrameDatabase kfd(newKeyFrameIndex);
  kfd.cameraID = frame_list[newKeyFrameIndex].cameraID;
  {
  //std::unique_lock<std::mutex> lock(update_kflist_mutex);  
  kflist.push_back(kfd);
  }
#else
  collaborative_fusion::Frame cf;
  CommunicationAPI::toRosFrame(frame_list[newKeyFrameIndex],cf);
  
  collaborative_fusion::LoopClosureDetection srv;
  srv.request.new_frame = cf;
  srv.request.is_newsubmap = is_newSubmap;
  srv.request.tracking_success = frame_list[newKeyFrameIndex].tracking_success;

  if(client.call(srv))
  {
 
    candidate_keyframes_index = srv.response.frame_candidates_index;
    candidate_keyframes_submap = srv.response.frame_candidates_submap;
    candidate_keyframes_camera = srv.response.frame_candidates_camera;
    needCollibration = srv.response.need_optimization;
    CommunicationAPI::toCvTransform(srv.response.relative_pose,relative_pose_submap);
    CommunicationAPI::toCvTransform(srv.response.camera_start_pose, cameraStartPose);

    closureID = srv.response.closure_submapID;
    closureIndex = srv.response.closure_index;
    submap_tracking_success = srv.response.submap_tracking_success; 
    submap_lcd = srv.response.submap_lcd;
    MultiViewGeometry::KeyFrameDatabase kfd(newKeyFrameIndex);
    kfd.cameraID = frame_list[newKeyFrameIndex].cameraID;
    kflist.push_back(kfd);

    std::cout<<GREEN<<"[INFO]::Upload keyframe to center successfully! Submap tracking success: "<<submap_tracking_success<<RESET<<std::endl;
  }
  else
  {
    std::cout<<RED<<"[ERROR]::Failed to upload the keyframe to center, please check the connection."<<RESET<<std::endl;
    return;
  }
#endif
  //*********************** select candidates

  std::vector<MultiViewGeometry::FrameCorrespondence> fCorrCandidate;

  for (size_t k = 0; k < candidate_keyframes_index.size(); k++)
  {
    int candidate_frame_index = candidate_keyframes_index[k];
    int candidate_cameraID = candidate_keyframes_camera[k];
    int candidate_submapID = candidate_keyframes_submap[k];
    if(candidate_submapID == frame_list[newKeyFrameIndex].submapID && candidate_cameraID == frame_list[newKeyFrameIndex].cameraID && candidate_frame_index != last_keyframe_index  )
    {
    MultiViewGeometry::FrameCorrespondence global_frame_corr(frame_list[candidate_frame_index], frame_list[newKeyFrameIndex]);
    fCorrCandidate.push_back(global_frame_corr);
    }
  }
  if(!registration_success)
  {
    MultiViewGeometry::FrameCorrespondence global_frame_corr(frame_list[last_keyframe_index], frame_list[newKeyFrameIndex]);
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
    
    registration_success_list[k] = MultiViewGeometry::FrameMatchingTwoViewRGB(fCorrCandidate[k],
                                                                              camera_para,
                                                                              sparseMatcher,
                                                                              relative_pose_from_ref_to_new_list[k],
                                                                              average_disparity_list[k],
                                                                              scale_change_ratio,
                                                                              update_keyframe_from_dense_matching,0);
                                                
    relative_pose_from_ref_to_new_list[k] = relative_pose_from_ref_to_new_list[k].inverse();

  }
      //std::cout<<"point2"<<std::endl;
  if(registration_success)
  {
    relative_pose_from_ref_to_new_list.push_back(relative_pose_from_key_to_new);
    registration_success_list.push_back(registration_success);
    average_disparity_list.push_back(average_disparity);
    fCorrCandidate.push_back(key_frame_corr);
  }

#if FOR_REBUTTAL
  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
    CorrIdList.push_back(fCorrCandidate[k]);
    relative_pose_list.push_back(relative_pose_from_ref_to_new_list[k]);
  }  
#endif
  //update camera pose based on previous results
  float min_average_disparity = 0.5;
  float min_average_disparity_submap = 0.5;
  float min_average_disparity_camera = 0.5;
  int min_index = 0;
  int min_index_submap=0;
  int min_index_camera=0; 
  
//  std::cout << "average disparity / reprojection error: ";
//if(registration_success_list.size() == 1)
//registration_success_list[0] = true;
  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
    std::cout << fCorrCandidate[k].frame_ref.frame_index << "	/"
      << average_disparity_list[k] << "	/"
      << registration_success_list[k] << std::endl;
    if (min_average_disparity > average_disparity_list[k] && registration_success_list[k] && 
      fCorrCandidate[k].sameSubmap == true &&fCorrCandidate[k].sameCamera == true 
        && fCorrCandidate[k].frame_ref.origin_index == submapOrigin[currentSubmapID])
    {
      min_average_disparity = average_disparity_list[k];
      min_index = k;
      keyframe_tracking_success = 1;
      continue;
    }

  }
//  std::cout << std::endl;

  int current_map_origin = newKeyFrameIndex;
  if (keyframe_tracking_success == 1)
  {
    std::cout<<"keyframe tracking successs!!"<<std::endl;
    frame_list[newKeyFrameIndex].tracking_success = 1;

    frame_list[newKeyFrameIndex].pose_sophus[2] = frame_list[fCorrCandidate[min_index].frame_ref.frame_index].pose_sophus[2]
            * relative_pose_from_ref_to_new_list[min_index] ;
    frame_list[newKeyFrameIndex].pose_sophus[0] = submapPoses[currentSubmapID]*frame_list[newKeyFrameIndex].pose_sophus[2];
    frame_list[newKeyFrameIndex].pose_sophus[3] = submapPosesFinal[currentSubmapID] * frame_list[newKeyFrameIndex].pose_sophus[2];

    current_map_origin = frame_list[fCorrCandidate[min_index].frame_ref.frame_index].origin_index;


  //if(oldsubmap!=newsubmap)
  //submapCorrLists.push_back(fCorrCandidate[min_index]);
  }
  else
  {
    std::cout<<RED<<"[ERROR]::Track failed."<<RESET<<std::endl;
  }
  /*
  else if()
  {
    std::cout<<"submap tracking successs!!"<<std::endl;
    std::cout<<"closure index = "<<closureIndex <<" \nrelative_pose:\n" << relative_pose_submap.matrix() <<"\nrelative_pose_frame_key_to_new:\n"<<relative_pose_from_key_to_new.matrix()<<std::endl;
    frame_list[newKeyFrameIndex].tracking_success = 1;
    //frame_list[newKeyFrameIndex].pose_sophus[3] = frame_list[closureIndex].pose_sophus[3] * relative_pose_from_key_to_new;
    //frame_list[newKeyFrameIndex].pose_sophus[0] = frame_list[closureIndex].pose_sophus[0] * relative_pose_from_key_to_new;//relative_pose_submap;
    
  }*/

  if(submap_tracking_success ==1 && is_newSubmap == 1)
  {
    std::cout<<YELLOW<<"New submap!!"<<RESET<<std::endl;
    frame_list[newKeyFrameIndex].tracking_success = 1;    
    submapPoses[currentSubmapID] = submapPoses[closureID] *frame_list[closureIndex].pose_sophus[2] * relative_pose_from_key_to_new;;
    submapPosesFinal[currentSubmapID]  =  submapPosesFinal[closureID] *frame_list[closureIndex].pose_sophus[2] * relative_pose_from_key_to_new;
    submapPosesRelativeChanges[currentSubmapID] = submapPosesFinal[currentSubmapID] * submapPoses[currentSubmapID].inverse(); 
    frame_list[newKeyFrameIndex].pose_sophus[3] =  submapPosesFinal[currentSubmapID];
    frame_list[newKeyFrameIndex].pose_sophus[0] = submapPoses[currentSubmapID];
    frame_list[newKeyFrameIndex].pose_sophus[2] = PoseSE3d();
    //std::cout<<"Submap Pose: "<<frame_list[newKeyFrameIndex].pose_sophus[0].matrix()<<std::endl;
    //std::cout<<"Final Submap Pose: "<<frame_list[newKeyFrameIndex].pose_sophus[3].matrix()<<std::endl;
  }
  if(submap_lcd)
  {
    std::cout<<BLUE<<"Detect loop closure between submaps."<<RESET<<std::endl;
    submap_lcd = true;
  }

  frame_list[newKeyFrameIndex].is_keyframe = 1;
  frame_list[newKeyFrameIndex].origin_index = current_map_origin;
  int reg_success_cnt = 0;
  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
    if (registration_success_list[k] )
    {
        reg_success_cnt ++;
    }
  }

  for (size_t k = 0; k < fCorrCandidate.size(); k++)
  {
    if (registration_success_list[k] )
    {
       if(fCorrCandidate[k].sameSubmap && fCorrCandidate[k].sameCamera)
       {
      /* std::cout<<"fcorrlist keyframe: .........................................."<<fCorrList_keyframes.size()<<std::endl;
      for(int t = 0;t!=fCorrList_keyframes.size();++t)
      std::cout<<fCorrList_keyframes[t].frame_ref.frame_index<<" "<<fCorrList_keyframes[t].frame_new.frame_index<<std::endl;

      std::cout<<"fcorrlist keyframe: .........................................."<<fCorrList_keyframes.size()<<std::endl;*/
      fCorrList_keyframes.push_back(fCorrCandidate[k]);
      imuCorrList.push_back(fCorrCandidate[k]);
      keyCorrIDs.insert(fCorrCandidate[k].frame_ref.frame_index);
      keyCorrIDs.insert(fCorrCandidate[k].frame_new.frame_index);
       }
    }
  }
        //std::cout<<"point3"<<std::endl;
    std::cout<<"update map origin, now origin is "<<current_map_origin<<std::endl;
    updateMapOrigin(fCorrCandidate, keyCorrIDs,registration_success_list,newKeyFrameIndex);
    std::cout<<"updated map origin: "<<frame_list[newKeyFrameIndex].origin_index<<std::endl;
    if(frame_list[newKeyFrameIndex].origin_index != submapOrigin[frame_list[newKeyFrameIndex].submapID])
    {
    frame_list[newKeyFrameIndex].tracking_success = 0;
    }
  TOCK("GCSLAM::GlobalRegistration");
 }

void GCSLAM::updateMapOrigin(std::vector<MultiViewGeometry::FrameCorrespondence> &fCorrCandidate,std::set<int> &keyCorrIDs,
                             std::vector<int> &registration_success_list,
                             int newKeyFrameIndex)
{


    std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist = KeyframeDataList;
    //std::vector<size_t> &kfplist = keyframeDataPtrList;
    std::vector<Frame> &frame_list = globalFrameList;
#if 0
    std::vector<int> keyFrameIndex(frame_list.size());
    for (size_t k = 0; k < frame_list.size(); k++)
    {
      keyFrameIndex[k] = -1;
    }
    
    for (size_t k = 0; k < kflist.size(); k++)
    {
      if(keyCorrIDs.find(kflist[k].keyFrameIndex) != keyCorrIDs.end())
        keyFrameIndex[k].keyFrameIndex] = k;
    }
#endif
    for (size_t k = 0; k < fCorrCandidate.size(); k++)
    {
        int ref_frame_index = fCorrCandidate[k].frame_ref.frame_index;
/*
        if (keyFrameIndex[ref_frame_index] < 0)
        {
          std::cout << "warning! ref frame is not in the current submap!" << std::endl;
          continue;
        }
*/
        if(registration_success_list[k] && fCorrCandidate[k].sameSubmap && fCorrCandidate[k].sameCamera)
        {
        std::cout<<fCorrCandidate[k].frame_ref.frame_index <<" /"<<registration_success_list[k]<<std::endl;
        int ref_origin = frame_list[ref_frame_index].origin_index;
        int current_origin = frame_list[newKeyFrameIndex].origin_index;
        frame_list[newKeyFrameIndex].origin_index = std::min(ref_origin,current_origin);
        }
    }


}

void GCSLAM::update_frame(Frame &frame_input, const PoseSE3d & start_pose)
{
    if(GCSLAM_INIT_FLAG == 0)
    {
        std::cout << "error ! gcSLAM not initialized! " << std::endl;
        return;
    }
    std::vector<MultiViewGeometry::FrameCorrespondence> &fCorrList_keyframes = keyFrameCorrLists[0];
    std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist = KeyframeDataList;
    std::vector<Frame> &frame_list = globalFrameList;
    std::set<int> &keyframeIndex = keyframeIndexInSubmap[0];
    MILD::LoopClosureDetector & lcd = mild;


    //*********************** add current frame to database
    frame_list.push_back(frame_input);
    Frame &f = frame_list.back();

    //*********************** init keyframe database
    if (kflist.size() == 0)
    {
      MultiViewGeometry::KeyFrameDatabase kfd(f.frame_index);
      kflist.push_back(kfd);
      lcd.construct_database(f.descriptor, f.frame_index, 0, 0);
      f.origin_index = f.frame_index;
      f.tracking_success = 1;
      f.is_fixed_frame = 1;
      f.is_keyframe = 1;
      f.pose_sophus[0] = start_pose;
      f.pose_sophus[1] = start_pose;
      f.pose_sophus[2] = start_pose;
      f.pose_sophus[3] = start_pose;
      keyframeIndex.insert(f.frame_index);
      return;
    }

    int add_new_key_frame_flag = 0;
    bool registration_success = 0;
    float average_disparity = 1e8;
    float scale_change_ratio = 0;


    //*********************** SparseMatcher is used for efficient binary feature matching
    MILD::SparseMatcher sparseMatcher(FEATURE_TYPE_ORB, 32, 0, 50);
    sparseMatcher.train(f.descriptor);

    int last_keyframe_index = kflist.back().keyFrameIndex;
    int anchor_frame = f.frame_index - 1;

    static int local_tracking_cnt = 0;
    PoseSE3d relative_transform_from_key_to_new;
    if(anchor_frame >= 0 && frame_list[anchor_frame].tracking_success)
    {
      relative_transform_from_key_to_new = frame_list[anchor_frame].pose_sophus[0].inverse() *
              frame_list[last_keyframe_index].pose_sophus[0];
    }

    MultiViewGeometry::FrameCorrespondence key_frame_corr(frame_list[last_keyframe_index], f);
    bool update_keyframe_from_dense_matching = 0;

    //*********************** Match two frames based on RGBD features
    registration_success = MultiViewGeometry::FrameMatchingTwoViewRGB(key_frame_corr,
                                                                      camera_para,
                                                                      sparseMatcher,
                                                                      relative_transform_from_key_to_new,
                                                                      average_disparity,
                                                                      scale_change_ratio,
                                                                      update_keyframe_from_dense_matching,
                                                                      1);
    int update_keyframe_flag = 0;

    if(!registration_success)
    {
      PoseSE3d relative_transform_from_last_to_new = PoseSE3d();
      int last_frame_index = f.frame_index - 1;
      MultiViewGeometry::FrameCorrespondence frame_corr(frame_list[last_frame_index], f);
      frame_corr.allowance_icp = true;
      bool registration_success_tmp = MultiViewGeometry::FrameMatchingTwoViewRGB(frame_corr, camera_para,
                                                              sparseMatcher,
                                                              relative_transform_from_last_to_new,
                                                              average_disparity,
                                                              scale_change_ratio,
                                                              update_keyframe_from_dense_matching,
                                                              local_tracking_cnt,1,0);
      if(registration_success_tmp == false)
      {
        std::cout<<RED<<"Matching last frame failed!"<<RESET<<std::endl;
      }
      else
      {
        relative_transform_from_key_to_new = 
          relative_transform_from_last_to_new * frame_list[last_frame_index].pose_sophus[2].inverse() * frame_list[last_keyframe_index].pose_sophus[2];
        //frame_list[last_keyframe_index].pose_sophus[2].inverse() * frame_list[last_frame_index].pose_sophus[2] * relative_transform_from_key_to_new.inverse(); 
        //ConstructCorrespondence(frame_list[last_keyframe_index], f, relative_transform_from_key_to_new, key_frame_corr);
        ConstructCorrespondence(frame_list[last_keyframe_index].pose_sophus[2].inverse() * 
          frame_list[last_frame_index].pose_sophus[2] ,key_frame_corr, frame_corr);
        average_disparity = key_frame_corr.calculate_average_disparity(camera_para);
        std::cout<<GREEN<<"Use last frame to localize!"<<RESET<<std::endl;
        std::cout<<average_disparity<<std::endl;
        update_keyframe_flag = 1;
        //f.tracking_success = true;
        registration_success = true;
      }
    }
    if((average_disparity > minimumDisparity  || (scale_change_ratio > 0.4)) && registration_success)
    {
            update_keyframe_flag = 1;
    }

    if(!registration_success)
    {
        local_tracking_cnt++;
    }

    if(local_tracking_cnt > 0)
    {
            update_keyframe_flag = 1;
    }
    PoseSE3d relative_pose_from_key_to_new = relative_transform_from_key_to_new;
    relative_pose_from_key_to_new = relative_pose_from_key_to_new.inverse();
    if (registration_success )
    {
        local_tracking_cnt = 0;
        f.tracking_success = 1;
        f.pose_sophus[2] = frame_list[last_keyframe_index].pose_sophus[2] * relative_pose_from_key_to_new;
        if(!update_keyframe_flag)
        {
          f.pose_sophus[0] = submapPoses[0] * f.pose_sophus[2];
          f.pose_sophus[3] = submapPosesFinal[0]*f.pose_sophus[2];

          f.origin_index = frame_list[last_keyframe_index].origin_index;
          kflist.back().corresponding_frames.push_back(f.frame_index);
          kflist.back().localFrameCorrList.push_back(key_frame_corr);
          kflist.back().relative_pose_from_key_to_current.push_back(relative_pose_from_key_to_new);
        }
    }

    //*********************** update keyframe
    if(update_keyframe_flag)
    {
      local_tracking_cnt = 0;
      update_keyframe(f.frame_index,
                      key_frame_corr,
                      average_disparity,
                      relative_pose_from_key_to_new,
                      registration_success);
      f.is_keyframe = 1;
      if(f.tracking_success)
      keyframeIndex.insert(f.frame_index);
      //*********************** fastBA for globally consistent pose estimation
      TICK("GCSLAM::FastBA");
      MultiViewGeometry::optimizeKeyFrameMap(fCorrList_keyframes, frame_list,keyframeIndex,
                                              kflist,submapPoses,submapPosesFinal,submapOrigin[f.submapID]);
      TOCK("GCSLAM::FastBA");
    }
}
void GCSLAM::update_frame(ros::ServiceClient &client,Frame &frame_input,int currentSubmapID,int lastSubmapID,int &closureID, int &closureIndex,
  int &needCollibration, int newsubmap)
{
    if(GCSLAM_INIT_FLAG == 0)
    {
        std::cout << "error ! gcSLAM not initialized! " << std::endl;
        return;
    }
    //std::cout<<"current SubmapID(update_frame): "<<currentSubmapID<<std::endl;    
    std::vector<MultiViewGeometry::FrameCorrespondence> &fCorrList_keyframes = keyFrameCorrLists[currentSubmapID];//frame correspondency in current submap

    std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist = KeyframeDataList;
    //std::vector<size_t> &kfplist = keyframeDataPtrList;
    std::vector<Frame> &frame_list = globalFrameList;
    std::set<int > & keyCorrIDs =  keyFrameIDCorrs[currentSubmapID];
    std::set<int> &frameIndex = frameIndexInSubmap[currentSubmapID];
    std::set<int> &keyframeIndex = keyframeIndexInSubmap[currentSubmapID];
    MILD::LoopClosureDetector & lcd = mild;
    frame_input.setSubmapID(currentSubmapID);

    //*********************** add current frame to database
    frame_list.push_back(frame_input);
    Frame &f = frame_list.back();
    assert(f.frame_index < 20000);
    frameIndex.insert(f.frame_index);
    //*********************** init keyframe database
    if (kflist.size() == 0 )
    {
      MultiViewGeometry::KeyFrameDatabase kfd(f.frame_index);
      kfd.cameraID = f.cameraID;
      //kfd.setSubmapID(currentSubmapID);
      //send the first keyframe to server and construct the lcd  database
      collaborative_fusion::Frame cf;
      CommunicationAPI::toRosFrame(f,cf);
      collaborative_fusion::LoopClosureDetection srv;
      srv.request.new_frame = cf;
      srv.request.tracking_success = 1;
      std::cout<<std::endl;
      if(client.call(srv))
      {
        //std::vector<int > loop_candidate = srv.response.frame_candidates_index;
        std::cout<<GREEN<<"[INFO]::Add first Keyframe to center!"<<RESET<<std::endl;
      }
      else
      {
        std::cout<<RED<<"[ERROR]::Fail to add the first keyframe!"<<RESET<<std::endl;
        return;
      }
      
      //std::unique_lock<std::mutex> lock(update_kflist_mutex);
      kflist.push_back(kfd);
      //lcd.construct_database(f.getDescriptor(),0,0,0);
      
      f.origin_index = f.frame_index;
      f.tracking_success = 1;
      f.is_fixed_frame = 1;
      f.is_keyframe = 1;
      keyframeIndex.insert(f.frame_index);
      window_keyframe_index.push(f.frame_index);
      std::cout<<"first frame's pose: "<<f.pose_sophus[0].log().transpose()<<std::endl;

      return;
    
    }

    int add_new_key_frame_flag = 0;
    bool registration_success = 0;
    float average_disparity = 1e8;
    float scale_change_ratio = 0;


    //*********************** SparseMatcher is used for efficient binary feature matching
    timer.Tick("tracking");
    MILD::SparseMatcher sparseMatcher(FEATURE_TYPE_ORB, 32, 0, 50);
    sparseMatcher.train(f.getDescriptor());

    int last_keyframe_index = kflist.back().keyFrameIndex;
    int anchor_frame = f.frame_index - 1;


    PoseSE3d relative_transform_from_key_to_new;
    if(anchor_frame >= 0 && frame_list[anchor_frame].tracking_success)
    {
      relative_transform_from_key_to_new = frame_list[anchor_frame].pose_sophus[3].inverse() *
              frame_list[last_keyframe_index].pose_sophus[3];
      
    }

    //std::cout<<"last_keyframe_index: "<<last_keyframe_index<<std::endl;
    bool update_keyframe_from_dense_matching = 0;
    MultiViewGeometry::FrameCorrespondence key_frame_corr(frame_list[last_keyframe_index],f);

    registration_success = MultiViewGeometry::FrameMatchingTwoViewRGB(key_frame_corr,
                                                                      camera_para,
                                                                      sparseMatcher,
                                                                      relative_transform_from_key_to_new,
                                                                      average_disparity,
                                                                      scale_change_ratio,
                                                                      update_keyframe_from_dense_matching,
                                                                      local_tracking_cnt,1,0);
    // keyframe local_0_s, local_1_s, local_2_s,..., local_n_f.
    // local_n_f -> local_{n-1}_f
    int update_keyframe_flag = 0;
    if(!registration_success)
    {
      PoseSE3d relative_transform_from_last_to_new = PoseSE3d();
      int last_frame_index = f.frame_index - 1;
      MultiViewGeometry::FrameCorrespondence frame_corr(frame_list[last_frame_index], f);
      frame_corr.allowance_icp = true;
      bool registration_success_tmp = MultiViewGeometry::FrameMatchingTwoViewRGB(frame_corr, camera_para,
                                                              sparseMatcher,
                                                              relative_transform_from_last_to_new,
                                                              average_disparity,
                                                              scale_change_ratio,
                                                              update_keyframe_from_dense_matching,
                                                              local_tracking_cnt,1,0);
      if(registration_success_tmp == false)
      {
        std::cout<<RED<<"Matching last frame failed!"<<RESET<<std::endl;
      }
      else
      {
        relative_transform_from_key_to_new = 
          relative_transform_from_last_to_new * frame_list[last_frame_index].pose_sophus[2].inverse() * frame_list[last_keyframe_index].pose_sophus[2];
        //frame_list[last_keyframe_index].pose_sophus[2].inverse() * frame_list[last_frame_index].pose_sophus[2] * relative_transform_from_key_to_new.inverse(); 
        //ConstructCorrespondence(frame_list[last_keyframe_index], f, relative_transform_from_key_to_new, key_frame_corr);
        ConstructCorrespondence(frame_list[last_keyframe_index].pose_sophus[2].inverse() * 
          frame_list[last_frame_index].pose_sophus[2] ,key_frame_corr, frame_corr);
        average_disparity = key_frame_corr.calculate_average_disparity(camera_para);
        std::cout<<GREEN<<"Use last frame to localize!"<<RESET<<std::endl;
        std::cout<<average_disparity<<std::endl;
        update_keyframe_flag = 1;
        //f.tracking_success = true;
        registration_success = true;
      }
    }

    timer.Tock("tracking");
#if FOR_REBUTTAL
    t_tracking[f.frame_index] = timer.Elapsed("tracking");
    //*********************** Match two frames based on RGBD features
#endif


 //minimumDisparity 0.1 scale_change_ratia 0.4
    if((average_disparity > minimumDisparity  || (scale_change_ratio > 0.4)) && registration_success)
    {
            update_keyframe_flag = 1;
    }
    std::cout<<"newsubmap: "<<newsubmap <<" update_keyframe_flag: "<<update_keyframe_flag<<" average_disparity: "<<average_disparity<<std::endl;
    if(newsubmap>0) 
    {
      update_keyframe_flag = 1;
      submapOrigin.push_back(f.frame_index);
    }




    //update_keyframe_flag = 1;


    //Eigen::Isometry3f relative_pose;
    PoseSE3d relative_pose_from_key_to_new = relative_transform_from_key_to_new.inverse();

    if(!registration_success)
    {
        local_tracking_cnt++;
        std::cout<<"local tracking failed: "<<local_tracking_cnt<<std::endl;
    }
    else
    
    {
      local_tracking_cnt = 0;
      f.tracking_success = 1;
    }
    if(local_tracking_cnt >=1)
    {
            update_keyframe_flag = 1;
    }

    if (registration_success  )
    {
        local_tracking_cnt = 0;
#if 0
        f.tracking_success = 1;
        f.pose_sophus[3] = frame_list[last_keyframe_index].pose_sophus[3] * relative_pose_from_key_to_new;
        //f.pose_sophus[2] = frame_list[last_keyframe_index].pose_sophus[2] * relative_pose_from_key_to_new;
        f.pose_sophus[2] = submapPosesFinal[currentSubmapID].inverse() * f.pose_sophus[3];
        f.pose_sophus[0] = submapPoses[currentSubmapID] * f.pose_sophus[2];
        f.origin_index = frame_list[last_keyframe_index].origin_index;
        kflist.back().corresponding_frames.push_back(f.frame_index);
        kflist.back().localFrameCorrList.push_back(key_frame_corr);
        kflist.back().relative_pose_from_key_to_current.push_back(relative_pose_from_key_to_new);
#else
        f.tracking_success = 1;
        
        f.pose_sophus[2] = frame_list[last_keyframe_index].pose_sophus[2] * relative_pose_from_key_to_new;
        if(!update_keyframe_flag)
        {
          f.pose_sophus[0] = submapPoses[currentSubmapID] * f.pose_sophus[2];
          f.pose_sophus[3] = submapPosesFinal[currentSubmapID]*f.pose_sophus[2];

          f.origin_index = frame_list[last_keyframe_index].origin_index;
          kflist.back().corresponding_frames.push_back(f.frame_index);
          kflist.back().localFrameCorrList.push_back(key_frame_corr);
          kflist.back().relative_pose_from_key_to_current.push_back(relative_pose_from_key_to_new);
        }
#endif      
    }



    //*********************** update keyframe
      bool detectLoopClosure = false;
      //update_keyframe_flag = 1;
    if (update_keyframe_flag)
    {

        //resetRelativePoses = false;

      local_tracking_cnt = 0;
      update_keyframe(client,f.frame_index,
                      key_frame_corr,
                      average_disparity,
                      relative_pose_from_key_to_new,
                      registration_success,currentSubmapID,closureID,closureIndex,detectLoopClosure,needCollibration,newsubmap>0);

      f.is_keyframe = 1;


       if(!f.tracking_success)
       keyframeIndex.erase(f.frame_index);

      std::cout<<"current SubmapID: "<<currentSubmapID<<" keyframe detect Loop Closure: "<<detectLoopClosure <<" need Collibration:"<<needCollibration <<std::endl;
      //if(newsubmap>0)
      //f.pose_sophus[2] = PoseSE3d();
      if(f.tracking_success)
      {
        keyframeIndex.insert(f.frame_index);
 
      }
      //*********************** fastBA for globally consistent pose estimation
      TICK("GCSLAM::FastBA_Local");
#if 1
      timer.Tick("local optimization");
      MultiViewGeometry::optimizeKeyFrameMap(fCorrList_keyframes, frame_list,keyframeIndex,
                                              kflist,submapPoses,submapPosesFinal,submapOrigin[f.submapID]);
      timer.Tock("local optimization");
#if FOR_REBUTTAL
      t_local_optimization[f.frame_index] = timer.Elapsed("local optimization");
#endif
#endif


      TOCK("GCSLAM::FastBA_Local");

        /*std::cout<<std::endl<<"global: "<<f.pose_sophus[0].log().transpose()<<std::endl<<std::endl;
        std::cout<<"submap: "<<submapPoses[currentSubmapID].log().transpose()<<std::endl;
        std::cout<<"local: "<< f.pose_sophus[2].log().transpose()<<std::endl;
        std::cout<<"computed global: "<<(submapPoses[currentSubmapID] * f.pose_sophus[2]).log().transpose()<<std::endl<<std::endl;*/

      if( newsubmap>0||detectLoopClosure)
      {
#if 0
      std::cout<<"Start to optimize submaps!"<<std::endl;
      std::cout<<"Preintegrate submap correnspondency!"<<std::endl;
      //resetRelativeChanges();
      //resetRelativePoses = true;
      for( int i = submapCorrLists.size() -1; i>=0;--i)
      {

       
         if(submapCorrLists[i].use_icp)
         submapCorrLists[i].preIntegrateICPInterSubmap( MultiViewGeometry::g_para.icp_weight);
         else
         submapCorrLists[i].preIntegrateInterSubmap();    
      }
      TICK("GCSLAM::FastBA_Global");
       MultiViewGeometry::optimizeSubmap(submapCorrLists, frame_list,keyframeIndexInSubmap, kflist,submapPoses,submapPosesRelativeChanges,submapPosesFinal,0);
      TOCK("GCSLAM::FastBA_Global");    
#endif
  
      }

    }

}

void GCSLAM::toUpdateFrame(std::vector<collaborative_fusion::UpdateFrame> &vuf)
{
  for(auto id = activeSubmaps.begin(); id != activeSubmaps.end();++id)
  {
      
    std::set<int> &keyframeIndex = keyframeIndexInSubmap[*id];
    for(auto i = keyframeIndex.begin(); i != keyframeIndex.end(); ++i)
    {
      collaborative_fusion::UpdateFrame uf;
      CommunicationAPI::toUpdateFrame(globalFrameList[*i],uf);
      vuf.push_back(uf);
    }
  }
}
void GCSLAM::requestOptimization(ros::ServiceClient &client,bool is_newSubmap,int lastSubmap)
{

  std::vector<collaborative_fusion::UpdateFrame> updated_frames;
  toUpdateFrame(updated_frames);
  collaborative_fusion::GlobalOptimization srv;
  srv.request.camera_id  =  globalFrameList.back().cameraID;
  srv.request.updated_frames = updated_frames;
  srv.request.is_newsubmap = is_newSubmap;
  CommunicationAPI::toRosVec3f(floor_normals, srv.request.floor_normals);
  if(is_newSubmap)
  {
  srv.request.last_submap_id = lastSubmap;
  PoseSE3d submap_relative_pose = submapPosesFinal[lastSubmap].inverse() * submapPosesFinal.back();;
  CommunicationAPI::toRosTransform(submap_relative_pose,srv.request.submap_relative_pose); 
  }
  if(client.call(srv))
  {
    std::cout<<GREEN<<"[INFO]::Global Optimization..."<<RESET<<std::endl;
    CommunicationAPI::toCvPoseList(srv.response.submap_poses, submapPosesRelativeChanges);
    for(int i = 0;i!=submapPosesRelativeChanges.size();++i)
    {
    submapPosesFinal[i] =   submapPosesRelativeChanges[i] * submapPoses[i];
    }
    UpdateAllKeyframe();
  }
  else
  {
    std::cout<<RED<<"[ERROR]::Global Optimization failed, please check the connection."<<RESET<<std::endl;
  }  
}
