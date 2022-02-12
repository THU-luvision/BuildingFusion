//#include "header.h"
#ifndef SERVER_MILD
#define SERVER_MILD


#include <ros/ros.h>
#include <ros/console.h>
#include <collaborative_fusion/FramePose.h>
#include <collaborative_fusion/FrameDescriptor.h>
#include <collaborative_fusion/FrameKeyPoints.h>
#include <collaborative_fusion/FrameLocalPoints.h>
#include <collaborative_fusion/Frame.h>
#include <collaborative_fusion/LoopClosureDetection.h>
#include <vector>
#include "../CommunicationAPI.h"
#include "../GCSLAM/frame.h"
#include "../GCSLAM/MILD/loop_closure_detector.hpp"
#include "../GCSLAM/MILD/BayesianFilter.hpp"
#include "../GCSLAM/MILD/sparse_match.hpp"
#include "ServerFrame.h"

namespace Server
{
class ServerMild 
{
    public:
    MILD::LoopClosureDetector mild;
    ServerMild()=default;
    void init()
    {
        mild.init(FEATURE_TYPE_ORB,16,0);
    }
    void reset()
    {
        mild.reset();
    }
    float salientScoreThreshold = 1.0;
    float minSalientScoreThreshold = 0.5;
    int maxCandidateNum = 6;
    int maxColliCandidateNum = 5;
    int size(){return mild.getDatabaseSize();}
    void select_closure_candidates(ServerFrame & f, std::vector<int> &candidate_frame_index,std::vector<int> &candidate_frame_submap,std::vector<int> &candidate_frame_camera);
};
//static ServerMild server_mild;
}
#endif