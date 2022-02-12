#include <iostream>
#include <opencv/cv.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <list>
#include <omp.h>
#include <stdio.h>
#include <ctime>

#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <thread>
#include "GCSLAM/frame.h"
#include "GCSLAM/MILD/loop_closure_detector.hpp"
#include "GCSLAM/MILD/BayesianFilter.hpp"
#include "GCSLAM/MultiViewGeometry.h"
#include "GCSLAM/GCSLAM.h"


#include "GCFusion/MapMaintain.hpp"
#include "GCFusion/MobileGUI.hpp"
#include "GCFusion/MobileFusion.h"


#include "BasicAPI.h"
#include "CHISEL/src/open_chisel/ChunkManager.h"
#include "CHISEL/src/open_chisel/Chisel.h"
#include "CHISEL/src/open_chisel/ProjectionIntegrator.h"
#include "CHISEL/src/open_chisel/camera/PinholeCamera.h"
#include "CHISEL/src/open_chisel/Stopwatch.h"

#include "Tools/LogReader.h"
#include "Tools/LiveLogReader.h"
#include "Tools/RawLogReader.h"
//#include "Tools/RealSenseInterface.h"

#include <collaborative_fusion/Registration.h>
#include <collaborative_fusion/WithDraw.h>
#include "CommunicationAPI.h"
#include <unistd.h>
#define MULTI_THREAD 1

#define INPUT_SOURCE_DATABASE 0
#define INPUT_SOURCE_OPENNI 1
#define INPUT_SOURCE_REALSENSE 2
int camera_step = 0;

bool should_quit = false;
bool server_mesh_visualization = false;
void thread_sim_submap(MobileFusion &gcFusion)
{
    while(true)
    {
    gcFusion.simSubmap();
    usleep(1000);
    if(gcFusion.quitLockFusion) break;    
    }

    std::cout<<"Mesh simplification thread quit."<<std::endl;
}
void thread_kill_reactivated_submap(MobileFusion &gcFusion)
{
    while(true)
    {
    gcFusion.killReactiveSubmap();
    usleep(1000);
    if(gcFusion.quitLockFusion) break;    
    }

    std::cout<<"kill reactivated submap thread quit."<<std::endl;
}
void thread_mesh_visualization(MobileFusion &gcFusion, ros::ServiceClient &client_visualization)
{
    // used for putting the mesh visualization to backend
    //std::vector<unsigned char> buffer;

    
    /*
    for(int i = 0; i < buffer.size();++i)
    std::cout<<buffer[i];
    std::cout<<std::endl;
    */
    while(true)
    {
        if(server_mesh_visualization)
        {
            collaborative_fusion::MeshVisualization srv;
            //std::vector<unsigned char> buffer;
            std::vector<chisel::PointCloud> pcds;

            std::vector<std::vector<int>> rgb_colors;
            std::vector<std::vector<int>> semantic_colors;
            int point_count = gcFusion.chiselMap->GetFullPCD(pcds, rgb_colors, semantic_colors);
            srv.request.camera_id = gcFusion.cameraID;
            //srv.request.point_count = point_count;
            CommunicationAPI::toRosPointCloud(pcds, rgb_colors, semantic_colors, srv.request.pcds);
            //= buffer;

            if(client_visualization.call(srv))
            {
                //std::vector<int > loop_candidate = srv.response.frame_candidates_index;
                std::cout<<GREEN<<"[INFO]::Add meshes to center!"<<RESET<<std::endl;
            }
            else
            {
                std::cout<<RED<<"[ERROR]::Fail to add the meshes!"<<RESET<<std::endl;
            }
            server_mesh_visualization = false;
        }
        if(gcFusion.quitLockFusion) break;
        usleep(10000);
    }
    std::cout<<"Server sparse model visualization thread quit."<<std::endl;
}

void thread_register_room(MobileFusion &gcFusion, ros::ServiceClient &client_room)
{
    // used for room registration
    while(true)
    {
        //std::cout<<"room_registration: "<<gcFusion.room_registration<<std::endl;
        if(gcFusion.room_registration != -1)
        {
            
            int lastRoomID = gcFusion.room_registration;
            gcFusion.ExtractRoomModel(lastRoomID);
            WritePLYFromChiselPcdWithLabel(std::to_string(gcFusion.cameraID)+"_room_"+std::to_string(lastRoomID)+".ply", 
                gcFusion.room_pcds[lastRoomID]);
            chisel::SaveFPCDASCII(std::to_string(gcFusion.cameraID)+"_room_"+std::to_string(lastRoomID)+".fpcd", 
                gcFusion.room_pcds[lastRoomID]);
            gcFusion.RegisterRoom(client_room, lastRoomID);
            gcFusion.resetBuilding();
            gcFusion.room_registration = -1;
            
        }
        if(gcFusion.quitLockFusion) break;        
        usleep(1000);
    }
    std::cout<<"Room Registration thread quit."<<std::endl;
}


void thread_quit()
{
    char c;
    c=getchar();
    while(true)
    {
        c=getchar();
        if(c == '\n')
        {
        //should_quit = true;
        break;
        }
    }
}
void thread_save_submap(MobileFusion &gcFusion)
{
    while(true)
    {

    gcFusion.saveSubmap();
    usleep(1000);
    if(gcFusion.quitLockFusion) break;
    }
    std::cout<<"TSDF field saving thread quit."<<std::endl;
}   
void thread_save_frames(MobileFusion &gcFusion)
{
    while(true)
    {

    gcFusion.saveFrames();
    usleep(1000);
    if(gcFusion.quitLockFusion) break;
    }
    std::cout<<"Frame saving thread quit."<<std::endl;
}

void getPosePart(ros::ServiceClient &client,Frame &f ,MobileFusion &gcFusion,int feature_num, LogReader * logReader,MultiViewGeometry::CameraPara &camera,
bool &newSubmapFlag, int &currentSubMapID, int &lastSubMapID,int integrateLocalFrameNum,int i,std::vector<std::string> &rgb_files, std::vector<std::string> &depth_files, std::vector<double> &time_stamp,int sensorType = 0)
{

    TICK("System::FrameTime::1::LoadRawData");

    if(sensorType == INPUT_SOURCE_OPENNI) BasicAPI::LoadOnlineOPENNIData(f, logReader, camera);
    if(sensorType == INPUT_SOURCE_DATABASE) 
    {
        if(i >= rgb_files.size())
        return;
        BasicAPI::LoadRawData(i,f,rgb_files,depth_files,time_stamp, camera);
    }
    TOCK("System::FrameTime::1::LoadRawData");
    f.frame_index = i;
    f.cameraID = gcFusion.cameraID ;
    TICK("System::FrameTime::3::ExtractFeatures");
    BasicAPI::detectAndExtractFeatures(f,feature_num,camera);
    BasicAPI::extractNormalMapSIMD(f.getRefinedDepth(), f.normal_map,camera.c_fx,camera.c_fy,camera.c_cx, camera.c_cy);
    TOCK("System::FrameTime::3::ExtractFeatures");                

    std::cout<<"update frame ! "<<f.frame_index<<std::endl;
    TICK("System::FrameTime::2::UpdateFrame");
    //std::cout<<"current SubmapID(getPosePart): "<<currentSubMapID<<std::endl;

    if(newSubmapFlag)
        gcFusion.gcSLAM.update_frame(client,f,currentSubMapID,lastSubMapID,
        gcFusion.closureID,gcFusion.closureIndex,gcFusion.needToCollibrate,currentSubMapID);
    else
        gcFusion.gcSLAM.update_frame(client,f,currentSubMapID,lastSubMapID,
        gcFusion.closureID,gcFusion.closureIndex,gcFusion.needToCollibrate);
    TOCK("System::FrameTime::2::UpdateFrame");        


 
    Frame &frame_current = gcFusion.gcSLAM.globalFrameList.back();
            //std::cout<<"point1"<<std::endl;            
    if(frame_current.tracking_success &&!frame_current.is_keyframe)
        {
            
            int keyframeIndex = gcFusion.gcSLAM.GetKeyframeDataList().back().keyFrameIndex;

            BasicAPI::refineKeyframesSIMD(gcFusion.gcSLAM.globalFrameList[keyframeIndex],frame_current,camera);
                 //std::cout<<"point3"<<std::endl;
            BasicAPI::refineNewframesSIMD(gcFusion.gcSLAM.globalFrameList[keyframeIndex],frame_current,camera);
            
        }

    BasicAPI::refineDepthUseNormalSIMD((float *)frame_current.normal_map.data, (float *)frame_current.getRefinedDepth().data,camera.c_fx,camera.c_fy, camera.c_cx, camera.c_cy,
                                 camera.width, camera.height);

    if(frame_current.is_keyframe && frame_current.tracking_success == true)
    {

        BasicAPI::checkColorQuality(frame_current.normal_map,frame_current.colorValidFlag,camera.c_fx,camera.c_fy,camera.c_cx, camera.c_cy);
        gcFusion.clearRedudentFrameMemory(integrateLocalFrameNum);

        if(frame_current.tracking_success && gcFusion.add_new_submap == 0 || gcFusion.closureID != -1)
        gcFusion.need_to_tsdfFusion = 1;
    }
}  
