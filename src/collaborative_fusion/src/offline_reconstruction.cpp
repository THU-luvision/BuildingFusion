#include "offline_reconstruction.h"
using namespace std;
using namespace cv;
#define GUI 0
void OfflineReconstruction(const std::string &base_path, const std::string &associate, 
    const Eigen::Matrix4f & start_pose, const std::string &final_generated_model)
{
    int showCaseMode = 1;
    std::cout<<"base_path: "<<base_path<<std::endl;
    std::cout<<"associate: "<<associate<<std::endl;
    std::cout<<"final_generated_model: "<<final_generated_model<<std::endl;
    std::cout<<"start_pose: "<<start_pose<<std::endl;
    float ipnutVoxelResolution = 0.00625;
    int sensorType = 0;
    std::string global_parameters_file = "./settings.yaml";
    BasicAPI::loadGlobalParameters(MultiViewGeometry::g_para, global_parameters_file);
    vector <string> rgb_files;
    vector <string> depth_files;
    vector <string> rgb_paths;
    vector <string> depth_paths;
    Eigen::MatrixXd ground_truth;
    vector<double> time_stamp;
    std::vector<float> processingTimePerFrame;
    PoseSE3d start_pose_se(start_pose.block<3,3>(0,0).cast<double>(),start_pose.block<3,1>(0,3).cast<double>());
    MultiViewGeometry::CameraPara camera;

    LogReader *logReader = NULL;


    int rs2active = 0;

    BasicAPI::initOfflineData(base_path, associate ,rgb_files, depth_files, time_stamp, ground_truth, camera);
    std::cout<<"frame count: "<<rgb_files.size()<<std::endl;
    MultiViewGeometry::g_para.step_len = rgb_files.size();
    if(rgb_files.size() < 3500) 
    MultiViewGeometry::g_para.closure_key_points = 0;
    cout << "begin init rendering" << endl;
#if GUI
    MobileGUI gui(showCaseMode);
    MobileFusion gcFusion(camera);
#else 
    MobileFusion gcFusion(camera, 0, false);    
#endif

    gcFusion.initChiselMap(ipnutVoxelResolution,
                           MultiViewGeometry::g_para.far_plane_distance);

    cout << "begin init gcSLAM" << endl;
    int maxFrameNum = 20000;
    gcFusion.initGCSLAM(MultiViewGeometry::g_para);

    int i = 0;
    pangolin::GlTexture imageTexture(640,480,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);


#if 0
    boost::thread map_thread(boost::bind(&MobileFusion::MapManagement,&gcFusion));
#endif

    float integrateLocalFrameNum = 6;

    int portableDeviceFlag = 0;
    // for portable devices
    if(std::thread::hardware_concurrency() < 4)
    {
        portableDeviceFlag = 1;
        integrateLocalFrameNum = 3;
    }

    printf("%d\r\n", logReader != NULL);
#if GUI
    while(!pangolin::ShouldQuit() && (i < rgb_files.size() || (logReader != NULL || rs2active)))
    {
        if(!gui.pause->Get() || pangolin::Pushed(*gui.step))
#else
    while( i < rgb_files.size() )
    {
        if(true)
#endif
        {
            TICK("System::1::FrameTime");
            Frame f;

            TICK("System::FrameTime::1::LoadRawData");
            BasicAPI::LoadRawData(i,f,rgb_files,depth_files,time_stamp, camera);
            f.frame_index = i;
            f.submapID = 0;
            TOCK("System::FrameTime::1::LoadRawData");

            TICK("System::FrameTime::2::UpdateFrame");
            int feature_num = MultiViewGeometry::g_para.max_feature_num;
            if(portableDeviceFlag)  feature_num = 600;
            BasicAPI::detectAndExtractFeatures(f,feature_num,camera);
            BasicAPI::extractNormalMapSIMD(f.refined_depth, f.normal_map,camera.c_fx,camera.c_fy,camera.c_cx, camera.c_cy);
            gcFusion.gcSLAM.update_frame(f, start_pose_se);
            Frame &frame_current = gcFusion.gcSLAM.globalFrameList.back();
            if(frame_current.tracking_success &&!frame_current.is_keyframe)
            {
                int keyframeIndex = gcFusion.gcSLAM.GetKeyframeDataList().back().keyFrameIndex;
                BasicAPI::refineKeyframesSIMD(gcFusion.gcSLAM.globalFrameList[keyframeIndex],frame_current,camera);
                BasicAPI::refineNewframesSIMD(gcFusion.gcSLAM.globalFrameList[keyframeIndex],frame_current,camera);
            }
            BasicAPI::refineDepthUseNormalSIMD((float *)frame_current.normal_map.data, (float *)frame_current.refined_depth.data,camera.c_fx,camera.c_fy, camera.c_cx, camera.c_cy,
                                 camera.width, camera.height);
            TOCK("System::FrameTime::2::UpdateFrame");
            i++;



            if(frame_current.is_keyframe )
            {
                BasicAPI::checkColorQuality(frame_current.normal_map,frame_current.colorValidFlag,camera.c_fx,camera.c_fy,camera.c_cx, camera.c_cy);
                gcFusion.clearRedudentFrameMemory(integrateLocalFrameNum);
#if GUI
#if 0
                gcFusion.updateGlobalMap(gcFusion.gcSLAM.globalFrameList.size(),gcFusion.gcSLAM.globalFrameList.size() - 1);
#else

                gcFusion.tsdfFusion(gcFusion.gcSLAM.globalFrameList,
                                    gcFusion.gcSLAM.globalFrameList.size() - 1,
                                    gcFusion.gcSLAM.GetKeyframeDataList(),
                                    gcFusion.gcSLAM.frameIndexInSubmap[0],                                    
                                    gcFusion.gcSLAM.KeyframeDataList.size() - 2,
                                    0);
                gcFusion.GetFullMeshes(); 
#endif
#endif
            }
            imageTexture.Upload(gcFusion.gcSLAM.globalFrameList.back().rgb.data,GL_RGB,GL_UNSIGNED_BYTE);
       


            }

/*            


            float memoryConsumption = 0;
            for(int k = 0; k < gcFusion.gcSLAM.globalFrameList.size(); k++)
            {
                memoryConsumption += gcFusion.gcSLAM.globalFrameList[k].GetOccupiedMemorySize();
            }
            cout << "memory for frames: " << memoryConsumption / 1024 / 1024 << " " << memoryConsumption / 1024 / 1024 /gcFusion.gcSLAM.globalFrameList.size() << endl;
            TOCK("System::1::FrameTime");
            printf("frame %d time: %f\r\n",gcFusion.gcSLAM.globalFrameList.back().frame_index,Stopwatch::getInstance().getTiming("System::1::FrameTime"));
            processingTimePerFrame.push_back(Stopwatch::getInstance().getTiming("System::1::FrameTime"));
            
*/

#if GUI
            TICK("System::2::GUI");
            if(gui.followPose->Get())
            {
                Eigen::Matrix4f currPose;
                if(gcFusion.gcSLAM.globalFrameList.back().tracking_success && gcFusion.gcSLAM.globalFrameList.back().origin_index == 0)
                {
                    currPose = gcFusion.gcSLAM.globalFrameList.back().pose_sophus[0].matrix().cast<float>();
                }
                gui.setModelView(currPose, camera.c_fy < 0);

            }
            gui.PreCall();
            if(gui.drawGlobalModel->Get())
            {
                gui.drawCamera(gcFusion.gcSLAM.cameraStartPose * gcFusion.gcSLAM.globalFrameList.back().pose_sophus[3],"red");
                
                gcFusion.MobileShow(gui.s_cam.GetProjectionModelViewMatrix(),
                                    VERTEX_WEIGHT_THRESHOLD,
                                    //gui.drawUnstable->Get(),
                                    gui.drawNormals->Get(),
                                    gui.drawColors->Get(),
                                    gui.drawSemanticLabel->Get(),
                                    gui.drawInstanceLabel->Get());
            }
            gui.DisplayImg(gui.RGB,&imageTexture);
            gui.PostCall();
            TOCK("System::2::GUI");
#endif
    }
    
    char fileName[2560];
    memset(fileName,0,2560);
/*

    sprintf(fileName,"%s/trajectory.txt",base_path.c_str());
    BasicAPI::saveTrajectoryFrameList(gcFusion.gcSLAM.globalFrameList,fileName);

    float total_time = 0;
    for(int i = 0; i < processingTimePerFrame.size(); i++)
    {
        total_time += processingTimePerFrame[i];
    }
    cout << "average processing time per frame: " << total_time / processingTimePerFrame.size() << endl;
    memset(fileName,0,2560);
    sprintf(fileName,"%s/OnlineModel_%dmm.ply",base_path.c_str(),(int)(1000 *(gcFusion.GetVoxelResolution())));
    cout << "saving online model to:    " << fileName << endl;
    gcFusion.chiselMap->SaveAllMeshesToPLY(fileName);
    gcFusion.chiselMap->Reset();
*/

    cout <<"offline re-integrate all frames" << endl;

    TICK("Final::IntegrateAllFrames");
    for(int i = 0; i < gcFusion.gcSLAM.globalFrameList.size(); i++)
    {
        if(gcFusion.gcSLAM.globalFrameList[i].tracking_success)
        gcFusion.IntegrateFrame(gcFusion.gcSLAM.globalFrameList[i]);
    }
    std::cout<<gcFusion.gcSLAM.globalFrameList[0].pose_sophus[3].matrix()<<std::endl;
    TOCK("Final::IntegrateAllFrames");
    gcFusion.chiselMap->UpdateMeshes(gcFusion.cameraModel);
    
    gcFusion.UpdateSegmentationGlobal();
    auto mesh_ptr = gcFusion.chiselMap->GetMeshesGlobalSemantic();
    //to avoid the bounding float result
    WritePLYFromChiselMesh(final_generated_model, mesh_ptr);
    BasicAPI::saveTrajectoryFrameListMatrix(gcFusion.gcSLAM.globalFrameList, base_path);
    gcFusion.chiselMap->SaveLabels(final_generated_model, mesh_ptr->compact_labels);
    gcFusion.chiselMap->Reset();
    return;
}


void OfflineReconstruction(const std::string &base_path, const std::string &last_associate, const std::string &associate, 
   const std::string &next_associate, const Eigen::Matrix4f & start_pose, const std::string &final_generated_model)
{
    int showCaseMode = 1;
    std::cout<<"base_path: "<<base_path<<std::endl;
    std::cout<<"last_associate: "<<last_associate<<std::endl;
    std::cout<<"associate: "<<associate<<std::endl;
    std::cout<<"next_associate: "<<next_associate<<std::endl;
    std::cout<<"final_generated_model: "<<final_generated_model<<std::endl;
    std::cout<<"start_pose: "<<start_pose<<std::endl;
    float ipnutVoxelResolution = 0.00625;
    int sensorType = 0;
    std::string global_parameters_file = "./settings.yaml";
    BasicAPI::loadGlobalParameters(MultiViewGeometry::g_para, global_parameters_file);
    vector <string> last_rgb_files;
    vector <string> last_depth_files;
    vector <string> rgb_files;
    vector <string> depth_files;
    vector <string> next_rgb_files;
    vector <string> next_depth_files;
    vector <string> rgb_paths;
    vector <string> depth_paths;
    Eigen::MatrixXd ground_truth;
    vector<double> time_stamp;
    std::vector<float> processingTimePerFrame;
    PoseSE3d start_pose_se(start_pose.block<3,3>(0,0).cast<double>(),start_pose.block<3,1>(0,3).cast<double>());
    MultiViewGeometry::CameraPara camera;

    LogReader *logReader = NULL;


    int rs2active = 0;
    BasicAPI::initOfflineData(base_path, last_associate ,last_rgb_files, last_depth_files, time_stamp, ground_truth, camera);
    BasicAPI::initOfflineData(base_path, associate ,rgb_files, depth_files, time_stamp, ground_truth, camera);
    BasicAPI::initOfflineData(base_path, next_associate ,next_rgb_files, next_depth_files, time_stamp, ground_truth, camera);
    
    rgb_files.insert(rgb_files.begin(), last_rgb_files.begin(), last_rgb_files.end());
    depth_files.insert(depth_files.begin(), last_depth_files.begin(), last_depth_files.end());
    

    rgb_files.insert(rgb_files.end(), next_rgb_files.begin(), next_rgb_files.end());
    depth_files.insert(depth_files.end(), next_depth_files.begin(), next_depth_files.end());

    std::cout<<"frame count: "<<rgb_files.size()<<std::endl;
    MultiViewGeometry::g_para.step_len = rgb_files.size();
    if(rgb_files.size() < 3500) 
    MultiViewGeometry::g_para.closure_key_points = 0;
    cout << "begin init rendering" << endl;
#if GUI
    MobileGUI gui(showCaseMode);
    MobileFusion gcFusion(camera);
#else 
    MobileFusion gcFusion(camera, 0, false);    
#endif

    gcFusion.initChiselMap(ipnutVoxelResolution,
                           MultiViewGeometry::g_para.far_plane_distance);

    cout << "begin init gcSLAM" << endl;
    int maxFrameNum = 20000;
    gcFusion.initGCSLAM(MultiViewGeometry::g_para);

    int i = 0;
    pangolin::GlTexture imageTexture(640,480,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);


#if 0
    boost::thread map_thread(boost::bind(&MobileFusion::MapManagement,&gcFusion));
#endif

    float integrateLocalFrameNum = 6;

    int portableDeviceFlag = 0;
    // for portable devices
    if(std::thread::hardware_concurrency() < 4)
    {
        portableDeviceFlag = 1;
        integrateLocalFrameNum = 3;
    }

    printf("%d\r\n", logReader != NULL);
#if GUI
    while(!pangolin::ShouldQuit() && (i < rgb_files.size() || (logReader != NULL || rs2active)))
    {
        if(!gui.pause->Get() || pangolin::Pushed(*gui.step))
#else
    while( i < rgb_files.size() )
    {
        if(true)
#endif
        {
            TICK("System::1::FrameTime");
            Frame f;

            TICK("System::FrameTime::1::LoadRawData");
            BasicAPI::LoadRawData(i,f,rgb_files,depth_files,time_stamp, camera);
            f.frame_index = i;
            f.submapID = 0;
            TOCK("System::FrameTime::1::LoadRawData");

            TICK("System::FrameTime::2::UpdateFrame");
            int feature_num = MultiViewGeometry::g_para.max_feature_num;
            if(portableDeviceFlag)  feature_num = 600;
            BasicAPI::detectAndExtractFeatures(f,feature_num,camera);
            BasicAPI::extractNormalMapSIMD(f.refined_depth, f.normal_map,camera.c_fx,camera.c_fy,camera.c_cx, camera.c_cy);
            gcFusion.gcSLAM.update_frame(f, start_pose_se);
            Frame &frame_current = gcFusion.gcSLAM.globalFrameList.back();
            if(frame_current.tracking_success &&!frame_current.is_keyframe)
            {
                int keyframeIndex = gcFusion.gcSLAM.GetKeyframeDataList().back().keyFrameIndex;
                BasicAPI::refineKeyframesSIMD(gcFusion.gcSLAM.globalFrameList[keyframeIndex],frame_current,camera);
                BasicAPI::refineNewframesSIMD(gcFusion.gcSLAM.globalFrameList[keyframeIndex],frame_current,camera);
            }
            BasicAPI::refineDepthUseNormalSIMD((float *)frame_current.normal_map.data, (float *)frame_current.refined_depth.data,camera.c_fx,camera.c_fy, camera.c_cx, camera.c_cy,
                                 camera.width, camera.height);
            TOCK("System::FrameTime::2::UpdateFrame");
            i++;



            if(frame_current.is_keyframe )
            {
                BasicAPI::checkColorQuality(frame_current.normal_map,frame_current.colorValidFlag,camera.c_fx,camera.c_fy,camera.c_cx, camera.c_cy);
                gcFusion.clearRedudentFrameMemory(integrateLocalFrameNum);
#if GUI
#if 0
                gcFusion.updateGlobalMap(gcFusion.gcSLAM.globalFrameList.size(),gcFusion.gcSLAM.globalFrameList.size() - 1);
#else

                gcFusion.tsdfFusion(gcFusion.gcSLAM.globalFrameList,
                                    gcFusion.gcSLAM.globalFrameList.size() - 1,
                                    gcFusion.gcSLAM.GetKeyframeDataList(),
                                    gcFusion.gcSLAM.frameIndexInSubmap[0],                                    
                                    gcFusion.gcSLAM.KeyframeDataList.size() - 2,
                                    0);
                gcFusion.GetFullMeshes(); 
#endif
#endif
            }
            imageTexture.Upload(gcFusion.gcSLAM.globalFrameList.back().rgb.data,GL_RGB,GL_UNSIGNED_BYTE);
       


            }

/*            


            float memoryConsumption = 0;
            for(int k = 0; k < gcFusion.gcSLAM.globalFrameList.size(); k++)
            {
                memoryConsumption += gcFusion.gcSLAM.globalFrameList[k].GetOccupiedMemorySize();
            }
            cout << "memory for frames: " << memoryConsumption / 1024 / 1024 << " " << memoryConsumption / 1024 / 1024 /gcFusion.gcSLAM.globalFrameList.size() << endl;
            TOCK("System::1::FrameTime");
            printf("frame %d time: %f\r\n",gcFusion.gcSLAM.globalFrameList.back().frame_index,Stopwatch::getInstance().getTiming("System::1::FrameTime"));
            processingTimePerFrame.push_back(Stopwatch::getInstance().getTiming("System::1::FrameTime"));
            
*/

#if GUI
            TICK("System::2::GUI");
            if(gui.followPose->Get())
            {
                Eigen::Matrix4f currPose;
                if(gcFusion.gcSLAM.globalFrameList.back().tracking_success && gcFusion.gcSLAM.globalFrameList.back().origin_index == 0)
                {
                    currPose = gcFusion.gcSLAM.globalFrameList.back().pose_sophus[0].matrix().cast<float>();
                }
                gui.setModelView(currPose, camera.c_fy < 0);

            }
            gui.PreCall();
            if(gui.drawGlobalModel->Get())
            {
                gui.drawCamera(gcFusion.gcSLAM.cameraStartPose * gcFusion.gcSLAM.globalFrameList.back().pose_sophus[3],"red");
                
                gcFusion.MobileShow(gui.s_cam.GetProjectionModelViewMatrix(),
                                    VERTEX_WEIGHT_THRESHOLD,
                                    //gui.drawUnstable->Get(),
                                    gui.drawNormals->Get(),
                                    gui.drawColors->Get(),
                                    gui.drawSemanticLabel->Get(),
                                    gui.drawInstanceLabel->Get());
            }
            gui.DisplayImg(gui.RGB,&imageTexture);
            gui.PostCall();
            TOCK("System::2::GUI");
#endif
    }
    
    char fileName[2560];
    memset(fileName,0,2560);
/*

    sprintf(fileName,"%s/trajectory.txt",base_path.c_str());
    BasicAPI::saveTrajectoryFrameList(gcFusion.gcSLAM.globalFrameList,fileName);

    float total_time = 0;
    for(int i = 0; i < processingTimePerFrame.size(); i++)
    {
        total_time += processingTimePerFrame[i];
    }
    cout << "average processing time per frame: " << total_time / processingTimePerFrame.size() << endl;
    memset(fileName,0,2560);
    sprintf(fileName,"%s/OnlineModel_%dmm.ply",base_path.c_str(),(int)(1000 *(gcFusion.GetVoxelResolution())));
    cout << "saving online model to:    " << fileName << endl;
    gcFusion.chiselMap->SaveAllMeshesToPLY(fileName);
    gcFusion.chiselMap->Reset();
*/
#if GUI
    for(int i = 0; i != gcFusion.chiselMap->chunkManagers.size(); ++i)
    {
        gcFusion.chiselMap->SaveMeshesToPLYBySubmap(i, "./model/by_submap/"+std::to_string(gcFusion.cameraID)+"/"+std::to_string(i)+".ply");
    }
#else
    cout <<"offline re-integrate all frames" << endl;

    TICK("Final::IntegrateAllFrames");
    for(int i = 0; i < gcFusion.gcSLAM.globalFrameList.size(); i++)
    {
        if(gcFusion.gcSLAM.globalFrameList[i].tracking_success)
        //if(i % 10 == 0)
        gcFusion.IntegrateFrame(gcFusion.gcSLAM.globalFrameList[i]);
    }
    std::cout<<gcFusion.gcSLAM.globalFrameList[0].pose_sophus[3].matrix()<<std::endl;
    TOCK("Final::IntegrateAllFrames");
    gcFusion.chiselMap->UpdateMeshes(gcFusion.cameraModel);
    gcFusion.UpdateSegmentationGlobal();
    auto mesh_ptr = gcFusion.chiselMap->GetMeshesGlobalSemantic();
    //to avoid the bounding float result
    WritePLYFromChiselMesh(final_generated_model, mesh_ptr);
    BasicAPI::saveTrajectoryFrameListMatrix(gcFusion.gcSLAM.globalFrameList, base_path);
    gcFusion.chiselMap->SaveLabels(final_generated_model, mesh_ptr->compact_labels);
    gcFusion.chiselMap->Reset();
#endif
    return;
}