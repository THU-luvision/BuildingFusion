#ifndef GCSLAM_H
#define GCSLAM_H
#include "MultiViewGeometry.h"
#include "ICPRegistration.h"
#include <thread>
#include <mutex>
#include <iostream>
#include "../CommunicationAPI.h"
#include <collaborative_fusion/UpdateFrame.h>
#include "../Tools/TickTock.h"
/*
#define RESET   "\033[0m"
#define BLACK   "\033[30m"      
#define RED     "\033[31m"      
#define GREEN   "\033[32m"      
#define YELLOW  "\033[33m" 
*/

#define FOR_REBUTTAL 0

#if FOR_REBUTTAL
//running time of each components
extern std::map<int, float> t_tracking;
extern std::map<int, float> t_local_optimization;
extern std::vector<MultiViewGeometry::FrameCorrespondence> CorrIdList;
extern PoseSE3dList relative_pose_list;
#endif


class GCSLAM
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static std::mutex update_kflist_mutex;
    static bool allInitialized;


    GCSLAM()
    {

        mild.init(FEATURE_TYPE_ORB,16,0);

        GCSLAM_INIT_FLAG = 0;
        minimumDisparity = 0.001;
        salientScoreThreshold = 1.0;
        minSalientScoreThreshold = 0.5;
        maxCandidateNum = 8;
        maxColliCandidateNum = 3;
        GCSLAM_INIT_FLAG = 1;
        
        //mild.init(FEATURE_TYPE_ORB, 16, 0);
        //KeyframeDataList.reserve(maximum_frame_num);
        keyFrameCorrLists=std::vector<std::vector<MultiViewGeometry::FrameCorrespondence>>();
        keyFrameCorrLists.push_back(std::vector<MultiViewGeometry::FrameCorrespondence>());
        keyFrameIDCorrs = std::vector<std::set<int>>();
        keyFrameIDCorrs.push_back(std::set<int>());
        frameIndexInSubmap = std::vector<std::set<int>>();
        frameIndexInSubmap.push_back(std::set<int>());
        keyframeIndexInSubmap = std::vector<std::set<int>>();
        keyframeIndexInSubmap.push_back(std::set<int>());
        R<<1,0,0,0,0,1,0,-1,0;
        activeSubmaps.insert(0);
        submapPoses = PoseSE3dList();
        submapPoses.push_back(Sophus::SE3d());
        submapPosesFinal = PoseSE3dList();
        submapPosesFinal.push_back(Sophus::SE3d());
        submapInitialized = std::vector<bool>();
        submapInitialized.push_back(true);
        submapOrigin = std::vector<int>();
        submapOrigin.push_back(0);
        submapPosesRelativeChanges = PoseSE3dList();
        submapPosesRelativeChanges.push_back(Sophus::SE3d());
        centerCoord = Point3fList();
        centerCoord.push_back(Eigen::Vector3f::Zero());

    }
    void setCameraModel(const  MultiViewGeometry::CameraPara &camera_para_t)
    {
        camera_para =  camera_para_t;
    }

    /* GCSLAM( GCSLAM &&gc):camera_para(gc.camera_para),KeyframeDataList(gc.KeyframeDataList),cameraCorrLists(gc.cameraCorrLists),mild(gc.mild),cameraStartPoses(gc.cameraStartPoses),isInitialized(gc.isInitialized)
    {
        GCSLAM_INIT_FLAG = 0;
        minimumDisparity = 0.1;
        salientScoreThreshold = 1.0;
        maxCandidateNum = 5;
        GCSLAM_INIT_FLAG = 1;
        //mild.init(FEATURE_TYPE_ORB, 16, 0);
        //KeyframeDataList.reserve(maximum_frame_num);
        keyFrameCorrLists=std::vector<std::vector<MultiViewGeometry::FrameCorrespondence>>();
        keyFrameCorrLists.push_back(std::vector<MultiViewGeometry::FrameCorrespondence>());
        keyFrameIDCorrs = std::vector<std::set<int>>();
        keyFrameIDCorrs.push_back(std::set<int>());
        frameIndexInSubmap = std::vector<std::set<int>>();
        frameIndexInSubmap.push_back(std::set<int>());
        keyframeIndexInSubmap = std::vector<std::set<int>>();
        keyframeIndexInSubmap.push_back(std::set<int>());
        
        globalFrameList.reserve(gc.globalFrameList.capacity());
        submapPoses = PoseSE3dList();
        submapPoses.push_back(Sophus::SE3d());
        submapPosesRelativeChanges = PoseSE3dList();
        submapPosesRelativeChanges.push_back(Sophus::SE3d());

    }*/



    /* void finalBA(std::vector<Frame> &frame_list,int currentSubmapID)
    {

        std::vector<MultiViewGeometry::FrameCorrespondence> &fCorrList_keyframes = keyFrameCorrLists[currentSubmapID];
        std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist = KeyframeDataList;
        initGraphHuberNorm(fCorrList_keyframes,frame_list);
        MultiViewGeometry::optimizeKeyFrameMap(fCorrList_keyframes, frame_list ,keyframeIndexInSubmap[currentSubmapID],kflist, submapPoses,0);
    }*/


    void select_closure_candidates(Frame &f, std::vector<int> &candidate_frame_index,bool inside_submap = false);
    void toUpdateFrame(std::vector<collaborative_fusion::UpdateFrame> &vuf);
    void update_keyframe(ros::ServiceClient &client,int newKeyFrameIndex,
                         MultiViewGeometry::FrameCorrespondence &key_frame_corr,
                         float average_disparity,
                         PoseSE3d relative_pose_from_key_to_new,
                         int registration_success,int currentSubmapID,int &closureID,int &closureIndex,bool &detectLoopClosure,int &needCollibration,bool newSubmap);

    /*
    frame_input will be added into frame database, and matched with previous keyframes
    if the disparity between frame_input and previous keyframe is larger than a threshold, frame_input is recogonized as a new keyframe
    FastBA is activated if new keyframe is inserted, and all previous frames' poses are updated
    */
    void update_frame(ros::ServiceClient &client,Frame &frame_input,int currentSubmapID,int lastSubmapID,int &closureID,int &closureIndex,
        int &needCollibration,int newSubmap = -1);

    void update_keyframe(int newKeyFrameIndex,
                         MultiViewGeometry::FrameCorrespondence &key_frame_corr,
                         float average_disparity,
                         PoseSE3d relative_pose_from_key_to_new,
                         int registration_success);

    /*
    frame_input will be added into frame database, and matched with previous keyframes
    if the disparity between frame_input and previous keyframe is larger than a threshold, frame_input is recogonized as a new keyframe
    FastBA is activated if new keyframe is inserted, and all previous frames' poses are updated
    */
    void update_frame(Frame &frame_input, const PoseSE3d & start_pose = PoseSE3d());

    void updateMapOrigin(std::vector<MultiViewGeometry::FrameCorrespondence> &fCorrCandidate,
                         std::set<int> &keyCorrIDs,std::vector<int> &registration_success_list ,int newKeyFrameIndex);
    inline const std::vector<MultiViewGeometry::KeyFrameDatabase> & GetKeyframeDataList() {return KeyframeDataList;}
    /*
    void optimizeCollibration(int needCollibration)
    {
      if(needCollibration)
      {
      std::cout<<"Start to collibrate!"<<std::endl;
      std::cout<<"Preintegrate camera correnspondency!"<<std::endl;
      //resetRelativeChanges();
      //resetRelativePoses = true;
      for( int i = cameraCorrLists.size() -1; i>=0;--i)
      {
         if(cameraCorrLists[i].use_icp)
         cameraCorrLists[i].preIntegrateICPInterCamera( MultiViewGeometry::g_para.icp_weight);
         else
         cameraCorrLists[i].preIntegrateInterCamera();
      }

      MultiViewGeometry::optimizeCollibration(cameraCorrLists,cameraStartPoses,0);
      //TOCK("GCSLAM::FastBA_COLLI");              
      }
    }*/

    size_t GetOccupyMemory()
    {

        size_t framelist = 0, keyframecorr = 0,keyframedata = 0,others = 0,keyframelist = 0;
        others += sizeof(float)*2 + sizeof(int )*2 + sizeof(camera_para);
        for(auto i = globalFrameList.begin();i!=globalFrameList.end();++i)
        {
        
        framelist += i->GetOccupiedMemorySize()+sizeof(Frame);
        if(i->is_keyframe)
        keyframelist += i->GetOccupiedMemorySize()+sizeof(Frame);
        }//framelist += globalFrameList.capacity() *sizeof(Frame);

        for(int i = 0;i!= keyFrameIDCorrs.size();++i)
        {
        others += keyFrameIDCorrs[i].size()*sizeof(int);
        others += frameIndexInSubmap[i].size()*sizeof(int);
        others += keyframeIndexInSubmap[i].size()*sizeof(int);
        }

        for(auto i = keyFrameCorrLists.begin();i!=keyFrameCorrLists.end();++i)
        for(auto j = i->begin();j!=i->end(); ++j)
        keyframecorr += j->getOccupyMemory() + sizeof(MultiViewGeometry::FrameCorrespondence);


        //for(auto i = KeyframeDataList.begin();i!=KeyframeDataList.end();++i)
        //keyframedata += i->getOccupyMemory() + sizeof(MultiViewGeometry::KeyFrameDatabase);

        std::cout<<"GlobalFrame: "<<framelist/1024/1024<<std::endl;
        std::cout<<"KeyFrame: "<<keyframelist/1024/1024<<std::endl;
        std::cout<<"keyFrameCorr: "<<keyframecorr/1024/1024<<std::endl;
        std::cout<<"keyFrameDatabase: "<<keyframedata/1024/1024<<std::endl;
        std::cout<<"others: "<<others/1024/1024<<std::endl;
        return others+framelist+keyframecorr+keyframedata;
    }

    void SetMinimumDisparity(float inputMinDisp) { minimumDisparity = inputMinDisp; }
    void SetMaxCandidateNum(int inputMaxCandidateNum) {maxCandidateNum = inputMaxCandidateNum; }
    void SetSalientScoreThreshold(float Threshold) {salientScoreThreshold = Threshold; }
    int  addNewSubmap()
    {
        
        keyFrameCorrLists.push_back(std::vector<MultiViewGeometry::FrameCorrespondence>());
        keyFrameIDCorrs.push_back(std::set<int>());
        frameIndexInSubmap.push_back(std::set<int>());
        keyframeIndexInSubmap.push_back(std::set<int>());
        submapPoses.push_back(Sophus::SE3d());
        submapPosesRelativeChanges.push_back(PoseSE3d());
        centerCoord.push_back(Eigen::Vector3f::Zero());
        submapPosesFinal.push_back(Sophus::SE3d());
        submapInitialized.push_back(false);
        activeSubmaps.insert(keyFrameCorrLists.size()-1);
        return keyFrameCorrLists.size()-1;
    }
    void killSubmap(int id)
    {
        activeSubmaps.erase(id);
    }
    void requestOptimization(ros::ServiceClient &client,bool is_newSubmap,int lastSubmap);
    void saveFrames(int submapID,int last_keyframe_index,const std::string &filepath = "./")
    {
        std::string final_path = filepath ;
        
        for(auto i = frameIndexInSubmap[submapID].begin();i!=frameIndexInSubmap[submapID].end();++i)
        {
            if(globalFrameList[*i].frame_index >= last_keyframe_index)
            break;
            //if(!globalFrameList[*i].is_keyframe)
            globalFrameList[*i].saveImages(final_path);
            //else  globalFrameList[*i].saveKeyFrame(filepath); 
        }
    }
    void saveFrames(int submapID, const std::string &filepath="./")
    {
        std::string final_path = filepath ;

        for(auto i = frameIndexInSubmap[submapID].begin();i!=frameIndexInSubmap[submapID].end();++i)
        {
            globalFrameList[*i].saveImages(final_path);
        }
    }

    inline void resetRelativeChanges()
    {
        for(int i = 0;i!=submapPosesRelativeChanges.size(); ++i)
            submapPosesRelativeChanges[i] = PoseSE3d();
    }
    int preintegration(Frame  *newkeyframe);
    void setCameraID(int _cameraID)
    {
        cameraID = _cameraID;
    }
    void UpdateAllKeyframe()
    {
        for(int i = 0; i != KeyframeDataList.size(); ++i)
        {
            int index = KeyframeDataList[i].keyFrameIndex;
            int submapID = globalFrameList[index].submapID;
            globalFrameList[index].pose_sophus[3] =
                submapPosesFinal[submapID] * globalFrameList[index].pose_sophus[2];
        }
    }
    /*void loadFrames(int submapID,const std::string &filepath="./")
    {
        std::string final_path = filepath + "frames/";
        for(auto i = frameIndexInSubmap[submapID].begin();i!=frameIndexInSubmap[submapID].end();++i)
        {
            globalFrameList[*i].loadImages(final_path);
        }
    }*/
    std::vector<Frame> globalFrameList;                 // database for all frames
    std::vector<std::vector<MultiViewGeometry::FrameCorrespondence>> keyFrameCorrLists; // database for matched keyframe pairs
    std::vector<std::set<int>> keyFrameIDCorrs;
    std::vector<std::set<int>> frameIndexInSubmap;
    std::vector<std::set<int>> keyframeIndexInSubmap;
    std::vector<MultiViewGeometry::FrameCorrespondence> submapCorrLists;
    std::vector<MultiViewGeometry::FrameCorrespondence> imuCorrList;
    PoseSE3dList submapPoses;
    PoseSE3dList submapPosesRelativeChanges;
    PoseSE3dList submapPosesFinal;
    Point3fList centerCoord;
    tool::Timer timer;

    std::vector<MultiViewGeometry::KeyFrameDatabase>  KeyframeDataList;   // database for all keyframes
    void UpdateCenterCoord(int submap_id)
    {
        centerCoord[submap_id] = Eigen::Vector3f::Zero();
        Eigen::Vector3f tmpCoord;
        for(auto iter = keyframeIndexInSubmap[submap_id].begin();iter!=keyframeIndexInSubmap[submap_id].end(); ++iter)
        {
            //global pose
            tmpCoord = ((globalFrameList[*iter].pose_sophus[3].matrix()).block<3,1>(0,3)).cast<float>();
            centerCoord[submap_id] = centerCoord[submap_id] + R*tmpCoord;
        }
        centerCoord[submap_id] /= (float)keyframeIndexInSubmap[submap_id].size();
    }    
    void UpdateCenterCoord()
    {
        for(int i = 0; i != centerCoord.size(); ++i)
        {
            UpdateCenterCoord(i);
        }
    }

    Eigen::Vector3f GetCurrentPosition()
    {
        return  R * (/*cameraStartPose */ globalFrameList.back().pose_sophus[3]).translation().cast<float>();
    }
    Eigen::Vector3f GetFramePosition(int frame_id)
    {
        return R * (/*cameraStartPose */ globalFrameList[frame_id].pose_sophus[3]).translation().cast<float>();
    }
    std::vector<bool> submapInitialized;
    std::vector<int> submapOrigin;
    std::queue<int> window_keyframe_index;
    bool resetRelativePoses = false;
    std::set<int > activeSubmaps;
    int GCSLAM_INIT_FLAG;
    PoseSE3d cameraStartPose = PoseSE3d();
    MILD::LoopClosureDetector  mild;                                       // interface for loop closure detector
    MultiViewGeometry::CameraPara  camera_para;                            // camera parameters
    int local_tracking_cnt = 0;
    float minimumDisparity;        // minimum disparity to update keyframes
    float salientScoreThreshold;   // minimum salient score for loop closures
    float minSalientScoreThreshold;
    int maxCandidateNum;         // maximum closure candidate num
    int maxColliCandidateNum;
    int cameraID;
    bool submap_lcd = false;
    Eigen::Matrix3f R;
    Point3fList floor_normals;
};

#endif

