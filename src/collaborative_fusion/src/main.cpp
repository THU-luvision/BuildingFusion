#include "header.h"
#include <csignal>


#define WITHDRAW 1


static volatile int keepRunning = 1; 
void sig_handler( int sig )
{
    if ( sig == SIGINT)
    {
        keepRunning = 0;
    }
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "collaborative_fusion_client");   
    ros::NodeHandle n;
    ros::ServiceClient client_d = n.serviceClient<collaborative_fusion::LoopClosureDetection>("loop_closure_detection");
    ros::ServiceClient client_o = n.serviceClient<collaborative_fusion::GlobalOptimization>("global_optimization");
    ros::ServiceClient client_v = n.serviceClient<collaborative_fusion::MeshVisualization>("mesh_visualization");   
    ros::ServiceClient client_r = n.serviceClient<collaborative_fusion::Registration>("registration");
    ros::ServiceClient client_room = n.serviceClient<collaborative_fusion::RoomRegistration>("room_registration");
    //cv::Mat patch_mat;
    std::vector<float> calib_para;


    int showCaseMode = 0;
    std::string basepath;
    
    float inputVoxelResolution = 0.00625;
    int sensorType = 0;
    
    int active_submap_number = 2;
    bool datasave = false;
    double compact_ratio = 100000;

    if(argc==2 && std::string(argv[1]) =="-help")
    {
        std::cout<<"usage 1(offline): ./FlashFusion <datasetpath> <settings> <resolution> <isonline> <compact_ratio/target_num> <step length> <active submap number>"<<std::endl;
        std::cout<<"usage 2(offline): ./FlashFusion <datasetpath>(other settings are default.)"<<std::endl;
        std::cout<<"usage 3(online): ./FlashFusion"<<std::endl;
        std::cout<<"usage 4(online): ./FlashFusion -a(acquire data)"<<std::endl;
        return 0;
    }
    else if(argc==2 && std::string(argv[1]) =="-save")
    datasave = true;
    BasicAPI::parseInput(argc,argv,showCaseMode,inputVoxelResolution,basepath,MultiViewGeometry::g_para,sensorType,compact_ratio,active_submap_number);
    size_t threadIndex = 0;

    collaborative_fusion::Registration srv_register;
    srv_register.request.ready = true;
    srv_register.request.base_path = basepath;
    int registration_camera_id;
    int currentRoomID = -1, lastRoomID = -1;
    int needRoomDetection = 0;
    if(client_r.call(srv_register))
    {
        //std::cout << GREEN<<"[ERROR]::Register successfully. CameraID: "<<registration_camera_id<<RESET<<std::endl;
        registration_camera_id = srv_register.response.camera_id;
        calib_para = srv_register.response.calib_parameters;
        std::cout << GREEN<<"[INFO]::Register successfully. CameraID: "<<registration_camera_id<<RESET<<std::endl;
    }
    else
    {
        std::cout << RED<<"[ERROR]::Fail to register."<<RESET<<std::endl;
        return 1;
    }
    float float_r = 0, float_g = 0, float_b = 0;
    if(registration_camera_id % 3 == 0)
    {
        float_r = 1;
    }
    else if(registration_camera_id % 3 == 1)
    {
        float_b = 1;
    }
    else
    {
        float_g = 1;
    }

    std::vector<std::string > rgb_files;
    std::vector<std::string > depth_files;
    std::vector<std::string > rgb_relative_paths;
    std::vector<std::string > depth_relative_paths;
    Eigen::MatrixXd ground_truth;
    std::vector<double> time_stamp;

    std::vector<float> processingTimePerFrame;
    std::vector<float> memories;
    std::vector<double> tsdf_time;

    //std::vector<int> feature_points;

    std::ofstream ofs("./dataset/associate.txt");

    //rs2::pipeline pipe;
 
    LogReader * logReader =nullptr ;    //TOCK("System::FrameTime::2::UpdateFrame");


    if(sensorType == INPUT_SOURCE_DATABASE) 
    { 
        BasicAPI::initOfflineData(basepath, rgb_files, depth_files,
            rgb_relative_paths, depth_relative_paths ,time_stamp, ground_truth, MultiViewGeometry::g_camera, calib_para);

    }
    if(sensorType == INPUT_SOURCE_OPENNI)       //
    {
        BasicAPI::initOpenNICamera(logReader,MultiViewGeometry::g_camera);
    }
    //if(sensorType == INPUT_SOURCE_REALSENSE)    rs2active = BasicAPI::initRS2Camera(pipe,camera);
    //if(sensorType == INPUT_SOURCE_REALSENSE)    rs2active = BasicAPI::initRS2Camera(pipe,camera);

    //--------------------------------------------------    READ IMU DATA
  
    //IMU_data_raw_mcam.resize(MultiViewGeometry::g_para.camera_num);

    // if(G_parameter.on_line_ros)
    // {
    //     // pthread_t id;
    //     // int ret = pthread_create(&id, NULL, BasicAPI::get_xtion_imu_data, NULL);  
    //     // number_limit=100000;
    // }
    // else

    if(sensorType == INPUT_SOURCE_DATABASE)
    {

            // basepath_imu = (char*)G_parameter.dataset_route.c_str();

            std::string basepath_imu;
            char fileName[256];
            memset(fileName, '\0', 256);
            sprintf(fileName, "%s", basepath.c_str());\        
            basepath_imu=fileName;      

        // BasicAPI::initOfflineData(basepath, rgb_files, depth_files, time_stamp, ground_truth);
        // number_limit=rgb_files.size()-1;
    }
    //--------------------------------------------------    READ IMU DATA

    std::cout << "begin init rendering" << std::endl;

    int width = g_camera.GetWidth();
    int height = g_camera.GetHeight();
    std::cout<<"width/height: "<<width <<"/"<<height<<std::endl;
    MobileGUI gui(showCaseMode, width, height);            

    MobileFusion gcFusion(MultiViewGeometry::g_camera,registration_camera_id);

    //int maxFrameNum = 20000;
    gcFusion.initChiselMap(inputVoxelResolution,
                           MultiViewGeometry::g_para.far_plane_distance,compact_ratio);

    std::cout<<"init ChiselMap!"<<std::endl;
    gcFusion.initGCSLAM(MultiViewGeometry::g_para);
    gcFusion.setActiveSubmapNumber(active_submap_number);

    int i = 0;
    int key_frame_num = 0;
    std::cout<<"width/height: "<<width <<"/"<<height<<std::endl;
    pangolin::GlTexture imageTexture(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture depthTexture(width,height,GL_RGB,true,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture patchTexture(width,height,GL_RGB,false,0,GL_RGB,GL_UNSIGNED_BYTE);
    pangolin::GlTexture cellTexture(width, height, GL_RGB, false,0,GL_RGB,GL_UNSIGNED_BYTE);
    bool newSubmapFlag=false;
    size_t TSDFfusionTime=0;
    //std::vector<boost::thread> map_threads(MultiViewGeometry::g_para.camera_num);


#if MULTI_THREAD
    boost::thread sim_thread;
    std::thread save_thread;
    std::thread saveFrames_thread;
    boost::thread map_thread = boost::thread(boost::bind(&MobileFusion::MapManagement,&gcFusion));
    std::thread roomReg_thread = std::thread(thread_register_room, std::ref(gcFusion), std::ref(client_room));
    std::thread meshVis_thread = std::thread(thread_mesh_visualization, std::ref(gcFusion), std::ref(client_v));
    sim_thread = boost::thread(thread_sim_submap,std::ref(gcFusion));
    save_thread = std::thread(thread_save_submap,std::ref(gcFusion));
    saveFrames_thread = std::thread(thread_save_frames,std::ref(gcFusion));
    //std::thread kill_reactivated_thread = std::thread(thread_kill_reactivated_submap, std::ref(gcFusion));
    /* boost::thread map_thread(boost::bind(&MobileFusion::MapManagement,&gcFusion));
    boost::thread  sim_thread(thread_sim_submap,std::ref(gcFusion));
    std::thread  save_thread(thread_save_submap, std::ref(gcFusion));
    std::thread saveFrames_thread(thread_save_frames, std::ref(gcFusion));
    */
    //std::thread quit_thread(thread_quit);
#endif

    float integrateLocalFrameNum = 6;

    int portableDeviceFlag = 0;
    // for portable devices
    if(std::thread::hardware_concurrency() < 4)
    {
        portableDeviceFlag = 1;
        integrateLocalFrameNum = 3;
    }
    gcFusion.setIntegrateLocalFrameNum(integrateLocalFrameNum);
    int currentSubMapID=0;
    int lastSubmapID=-1;

    int last_keyframe_index = -1; 
    int feature_num = MultiViewGeometry::g_para.max_feature_num;
    if(portableDeviceFlag)  feature_num = 600;
    int max_len=rgb_files.size();
    int RoomDetectionFrame = -1;
    tool::Timer over_all_timer;
    over_all_timer.Tick("overall");
    while(!pangolin::ShouldQuit()&&!should_quit && (i < max_len || (logReader != NULL )))
    {
        //boost::unique_lock<boost::mutex> lock(add_process_mutex);        
        Frame frame;
        std::cout<<"width/height: "<<width <<"/"<<height<<std::endl;
        if(!gui.pause->Get() || pangolin::Pushed(*gui.step))
        {
            TICK("System::1::FrameTime");

            std::cout<<"frame process!"<<std::endl;


            //TICK("System::FrameTime::2::UpdateFrame");
            //if( i >500 && i <=550);
            //else
            gcFusion.resetNeedToTSDFFusion();              
            //gcFusion.gcSLAM.timer.Reset();
            getPosePart(client_d,frame ,gcFusion,feature_num, logReader,MultiViewGeometry::g_camera,
                newSubmapFlag,currentSubMapID,lastSubmapID,integrateLocalFrameNum,i,rgb_files,depth_files,time_stamp,sensorType );
            if(gcFusion.needToCollibrate == 1)
            {
                gcFusion.GetAllFloorNormals();
                gcFusion.gcSLAM.requestOptimization(client_o,newSubmapFlag,lastSubmapID);
            }
            
            TOCK("System::1::FrameTime");

            Frame &frame_current = gcFusion.gcSLAM.globalFrameList.back(); 

#if MULTI_THREAD
            
            if(gcFusion.gcSLAM.submap_lcd && currentRoomID == -1)
            {
                needRoomDetection = true;
                if(RoomDetectionFrame == -1)
                RoomDetectionFrame = frame_current.frame_index;
            }  
            if(newSubmapFlag == true && frame_current.tracking_success)
            newSubmapFlag = false;
            else if(frame_current.is_keyframe && frame_current.tracking_success && gcFusion.in_tsdfFusion.try_lock() )
            {
                //currentRoomID is used for addNewSubmap in gcSLAM    
                currentRoomID = gcFusion.GetCurrentRoomID();
                gcFusion.in_tsdfFusion.unlock();
                std::cout<<"Current Room ID: "<<currentRoomID<<" Last Room ID: "<<lastRoomID<<std::endl;
                if((TSDFfusionTime >0 && TSDFfusionTime% MultiViewGeometry::g_para.step_len == 0 
                    || (currentRoomID  == -1 && lastRoomID != -1)) && gcFusion.gcSLAM.keyframeIndexInSubmap[currentSubMapID].size()>3)
                {
                    newSubmapFlag = true;
                }
                //else
                //newSubmapFlag = false;
                if(needRoomDetection)
                {
                    gcFusion.updateGlobalMap(newSubmapFlag, currentSubMapID, needRoomDetection, gcFusion.closureID, RoomDetectionFrame);
                    needRoomDetection =false;
                    RoomDetectionFrame = -1;
                }
                else
                gcFusion.updateGlobalMap(newSubmapFlag, currentSubMapID, needRoomDetection, gcFusion.closureID);
                

                if(newSubmapFlag)
                {
                    TSDFfusionTime = 0;
                    lastSubmapID= currentSubMapID;
                    currentSubMapID = gcFusion.gcSLAM.addNewSubmap();
                }
                lastRoomID = currentRoomID;                
                TSDFfusionTime += 1;

            }
                

#else            
            gcFusion.gcSLAM.timer.Tick("TSDF fusion and get full meshes");
            if(frame_current.is_keyframe && frame_current.tracking_success )
            {  

                clock_t start = clock();
                
                std::cout<<" submap id: "<<gcFusion.closureID
                    <<" closure_index: "<<gcFusion.closureIndex<<std::endl;
/*
                if( gcFusion.closureID >= 0 && std::fabs(currentSubMapID -  gcFusion.closureID ) > 2 )
                { 
                    int closureid = gcFusion.closureID;
                    int closureindex = gcFusion.closureIndex;

                    gcFusion.gcSLAM.timer.Tick("load chunks from outside");
                    gcFusion.loadSubmap(closureid);
                    gcFusion.gcSLAM.timer.Tock("load chunks from outside");
                              
                    std::cout<<"integrate two submaps, in tsdf fusion!"<<std::endl;            
                    Frame &kfref = gcFusion.gcSLAM.globalFrameList[closureindex];
                    Frame &kfnew = frame_current;

                    PoseSE3d poseref = gcFusion.gcSLAM.submapPosesRelativeChanges[currentSubMapID].inverse() * kfref.pose_sophus[3] ;
                    PoseSE3d posenew = gcFusion.gcSLAM.submapPosesRelativeChanges[closureid].inverse()  * kfnew.pose_sophus[3] ;
                    chisel::ChunkMap commonChunks;
                    
                    gcFusion.ReIntegrateKeyframeOtherSubmap(currentSubMapID,kfref,poseref,commonChunks);

                    gcFusion.gcSLAM.timer.Tick("integrate two chunk maps");
                    gcFusion.integrateTwoChunkMap(currentSubMapID,closureid,commonChunks,gcFusion.chiselMap->GetChunkMap(closureid));
                    gcFusion.gcSLAM.timer.Tock("integrate two chunk maps");

                    gcFusion.gcSLAM.timer.LogAll();



                    //ReIntegrateKeyframeOtherSubmap(closureCameraid,closureid,kfnew,posenew,commonChunks);
                    //integrateTwoChunkMap(closureCameraid,cameraID,closureid,currentSubMapID,commonChunks,chiselMap->GetChunkMap(currentSubMapID));
                    //chiselMap->clearGlobalMeshes();            
                    //chiselMap->UpdateMeshes(cameraModel,mainSubmapIDs[closureCameraid],referenceSubmapIDs[closureCameraid]);
                    //global_vertex_data_need_to_update = 1;          
                }

*/                
/*
                if( gcFusion.closureID >= 0 && std::fabs(currentSubMapID -  gcFusion.closureID ) > 2 )
                {
                    int closureid = gcFusion.closureID;
                    gcFusion.reactivateSubmap(closureid, frame_current.frame_index);
                }
*/
                if(!newSubmapFlag)
                gcFusion.tsdfFusion(gcFusion.gcSLAM.globalFrameList,
                                    gcFusion.gcSLAM.globalFrameList.size() - 1,
                                    gcFusion.gcSLAM.GetKeyframeDataList(),
                                    gcFusion.gcSLAM.frameIndexInSubmap[currentSubMapID],                                    
                                    gcFusion.gcSLAM.KeyframeDataList.size() - 2,
                                    currentSubMapID);


                           

                
                tsdf_time.push_back((double)(clock()-start)/(CLOCKS_PER_SEC)*1000);
            
                gcFusion.GetFullMeshes();   
                //gcFusion.DetectPlaneAndBuildDcel();  
                if(gcFusion.gcSLAM.submap_lcd && currentRoomID == -1)
                {
                    //char c = getchar();
                    gcFusion.resetBuilding();    
                    gcFusion.DetectPlaneAndBuildDcel();              
                    gcFusion.prepareBuilding();
                    gcFusion.SeparateRoom();
                    //Just a test
                    /*
                    if(gcFusion.room_to_submap.size())
                    {
                        gcFusion.ExtractRoomModel(0);
                        gcFusion.RegisterRoom(0);
                        gcFusion.RegisterRoom(0);
                    }
                    */
                }
                gcFusion.UpdatePatchImages();
                currentRoomID = gcFusion.GetCurrentRoomID();
                gcFusion.current_room_id = currentRoomID;
                std::cout<<"Current Room ID: "<<currentRoomID<<" Last Room ID: "<<lastRoomID<<std::endl;
                if(TSDFfusionTime >0&&TSDFfusionTime% MultiViewGeometry::g_para.step_len == 0 
                    || (currentRoomID  == -1 && lastRoomID != -1))
                {
                     std::cout<<"current room id: "<<currentRoomID<<" last room id: "<<lastRoomID<<" Tsdf fusion time: "<<TSDFfusionTime<<std::endl;
                    TSDFfusionTime = 0;
                    lastSubmapID =currentSubMapID;
                    gcFusion.gcSLAM.addNewSubmap();
                    currentSubMapID = gcFusion.addNewSubmap();
                    newSubmapFlag = true;
                    int wait_to_kill = gcFusion.referenceSubmapID;
                    gcFusion.referenceSubmapID = gcFusion.mainSubmapID;
                    gcFusion.mainSubmapID =    currentSubMapID; 
                    if(wait_to_kill >= 0)
                    {
                    gcFusion.saveFrames(wait_to_kill);
                    gcFusion.simplifySubmap(wait_to_kill);
                    gcFusion.saveSubmap(wait_to_kill);
                    }

                    if(currentRoomID  == -1 && lastRoomID != -1)
                    {   
                        gcFusion.room_to_submap[lastRoomID].push_back(gcFusion.referenceSubmapID);
                        gcFusion.unlabeled_submap.erase(gcFusion.referenceSubmapID);
                        gcFusion.submap_to_room[gcFusion.referenceSubmapID] = lastRoomID; 
                        gcFusion.ExtractRoomModel(lastRoomID);
                        auto dense_pcd = gcFusion.chiselMap->ExtractRoomModel(gcFusion.room_to_submap[lastRoomID], gcFusion.gcSLAM.submapPosesRelativeChanges);
                        chisel::SaveFPCDASCII("room_"+std::to_string(lastRoomID)+".fpcd", gcFusion.room_pcds[lastRoomID]);
                        //chisel::SaveFPCDASCII("dense_room_"+std::to_string(lastRoomID)+".fpcd", dense_pcd);
                        //gcFusion.chiselMap->SaveRoomMeshesToPLY("room_"+std::to_string(lastRoomID)+"_mesh.ply", gcFusion.room_to_submap[lastRoomID], gcFusion.gcSLAM.submapPosesRelativeChanges, gcFusion.gcSLAM.cameraStartPose);
                        gcFusion.RegisterRoom(client_room, lastRoomID);
                        gcFusion.resetBuilding();
                        
                    }
                    else if(currentRoomID != -1)
                    {
                        gcFusion.room_to_submap[currentRoomID].push_back(gcFusion.referenceSubmapID);
                        gcFusion.unlabeled_submap.erase(gcFusion.referenceSubmapID);  
                        gcFusion.submap_to_room[gcFusion.referenceSubmapID] = currentRoomID;  
                        std::cout<<"Add a new submap to room "<<currentRoomID<<std::endl;                
                    }
                    
                    //if(currentSubMapID > active_submap_number - 1)
                    //gcFusion.killSubmap(currentSubMapID - active_submap_number);
                }
                else
                { 
                    newSubmapFlag = false;
                }
                lastRoomID = currentRoomID;
                gcFusion.last_room_id = lastRoomID;
                gcFusion.gcSLAM.timer.LogAll();
                TSDFfusionTime += 1;
            }
            gcFusion.gcSLAM.timer.Tock("TSDF fusion and get full meshes");
            gcFusion.gcSLAM.timer.LogAll();
#endif            

            if(newSubmapFlag)
            {
#if MULTI_THREAD

                server_mesh_visualization = true;
#else
                collaborative_fusion::MeshVisualization srv;
            //std::vector<unsigned char> buffer;
                chisel::PointCloud pcd;
                std::vector<int> rgb_colors;
                std::vector<int> semantic_colors;
                int point_count = gcFusion.chiselMap->GetFullPCD(pcd, rgb_colors, semantic_colors, gcFusion.gcSLAM.submapPosesRelativeChanges);
                srv.request.camera_id = gcFusion.cameraID;
                //srv.request.point_count = point_count;
                CommunicationAPI::toRosPointCloud(pcd, rgb_colors, semantic_colors, srv.request.pcd);
                if(client_v.call(srv))
                {
                    //std::vector<int > loop_candidate = srv.response.frame_candidates_index;
                    std::cout<<GREEN<<"[INFO]::Add meshes to center!"<<RESET<<std::endl;
                }
                else
                {
                    std::cout<<RED<<"[ERROR]::Fail to add the meshes!"<<RESET<<std::endl;
                }
#endif
            }

        }
        TICK("System::2::GUI");
        //std::cout<<GREEN<< "Upload image." <<std::endl;
        imageTexture.Upload(frame.getRgb().data,GL_RGB,GL_UNSIGNED_BYTE);
        depthTexture.Upload(frame.getRefinedDepth().data,GL_RGBA,GL_UNSIGNED_BYTE);
        //patchTexture.Upload(gcFusion.patch_image.data, GL_RGB,GL_UNSIGNED_BYTE);
        //cellTexture.Upload(gcFusion.cell_image.data, GL_RGB, GL_UNSIGNED_BYTE);
        //std::cout<<GREEN<< "Upload image....." <<std::endl;
        i++;
        if(gui.followPose->Get())
        {
            //std::cout<<"followPose"<<std::endl;
            //getchar();
            Eigen::Matrix4f currPose;
            
            if(gcFusion.gcSLAM.globalFrameList.back().tracking_success)
            {
                currPose = (gcFusion.gcSLAM.cameraStartPose * gcFusion.gcSLAM.globalFrameList.back().pose_sophus[3]).matrix().cast<float>();
            //std::cout<<currPose<<std::endl;            
            }
            gui.setModelView(currPose, MultiViewGeometry::g_camera.c_fy < 0);

        }
        gui.PreCall();

        
        if(gui.drawGlobalModel->Get())
        {
            //std::cout<<gcFusion.gcSLAM.cameraStartPose.matrix()<<std::endl;
            
            gui.drawCamera(gcFusion.gcSLAM.cameraStartPose * gcFusion.gcSLAM.globalFrameList.back().pose_sophus[3],float_r, float_g, float_b);
            
            gcFusion.MobileShow(gui.s_cam.GetProjectionModelViewMatrix(),
                                VERTEX_WEIGHT_THRESHOLD,
                                //gui.drawUnstable->Get(),
                                gui.drawNormals->Get(),
                                gui.drawColors->Get(),
                                gui.drawSemanticLabel->Get(),
                                gui.drawInstanceLabel->Get());
            
        }
        //std::cout<<"display:"<<std::endl;
        //getchar();
        gui.DisplayImg(gui.RGB,&imageTexture);
        gui.DisplayImg(gui.DepthNorm, &depthTexture);
        //gui.DisplayImg(gui.Patches, &patchTexture);
        //gui.DisplayImg(gui.Cells, &cellTexture);

        gui.PostCall();
        TOCK("System::2::GUI");
        // printf("Frame time: %f %f %f %f %f\r\n",Stopwatch::getInstance().getTiming("System::1::FrameTime"), Stopwatch::getInstance().getTiming("System::FrameTime::1::LoadRawData"),
        //     Stopwatch::getInstance().getTiming("System::FrameTime::2::UpdateFrame"), Stopwatch::getInstance().getTiming("System::FrameTime::3::ExtractFeatures"),
        //         Stopwatch::getInstance().getTiming("System::2::GUI"));
    over_all_timer.Tock("overall");
    std::cout<<"Running time: "<< over_all_timer.Elapsed("overall")<<"\n processed_frame: "<< gcFusion.gcSLAM.globalFrameList.size()<<"\n frequency: "
        <<gcFusion.gcSLAM.globalFrameList.size() /(over_all_timer.Elapsed("overall") / 1000.0) <<"Hz"<<std::endl;

#if 0
        if(i>0 && i%5000 ==0)
        {

        std::set<int > activeSubmaps;
        activeSubmaps.insert(gcFusion.referenceSubmapID);
        activeSubmaps.insert(gcFusion.mainSubmapID);
        std::cout<<"save model..."<<std::endl;
        gcFusion.chiselMap->SaveAllMeshesToPLY("./model_singlethread"+std::to_string(i)+".ply",activeSubmaps,gcFusion.gcSLAM.submapPosesRelativeChanges);
        std::cout << "program finish" << std::endl;
        }
#endif    
    }

#if MULTI_THREAD
    std::cout<<"Finish online scanning. "<<std::endl;

    server_mesh_visualization = true;
    gcFusion.quit();
    gcFusion.updateGlobalMap(newSubmapFlag,currentSubMapID,false,-1);
    map_thread.join();//done
    sim_thread.join();//done
    //kill_reactivated_thread.join();
    save_thread.join();//done
    roomReg_thread.join();//done
    meshVis_thread.join();//done
    saveFrames_thread.join();//done
    std::cout<<"all threads quit."<<std::endl;
#endif
    over_all_timer.Tock("overall");

/*
    for(int i = 0; i != gcFusion.gcSLAM.submapPoses.size(); ++i)
    {
        std::vector<int> submap;
        submap.push_back(i);
        auto submap_pcd = gcFusion.chiselMap->ExtractRoomModelSparse(submap);
        std::string filename = "./for_normal_optimization/"+std::to_string(i)+".fpcd";
        chisel::SaveFPCDASCII(filename,submap_pcd);
    }
*/


    //BasicAPI::saveTrajectoryFrameList(gcFusion.gcSLAM.globalFrameList, "./trajectories/"+std::to_string(gcFusion.cameraID)+".txt");
    BasicAPI::saveTrajectoryFrameList(gcFusion.gcSLAM.globalFrameList, basepath+"/trajectory_submap.txt");
    std::set<int > activeSubmaps;
    activeSubmaps.insert(gcFusion.referenceSubmapID);
    activeSubmaps.insert(gcFusion.mainSubmapID);
    gcFusion.chiselMap->SimplifyAllSemantic();
    std::cout<<"save model..."<<std::endl;
    //gcFusion.saveTimeConsuming("./time_consuming.txt");
    for(int i = 0; i != gcFusion.chiselMap->chunkManagers.size(); ++i)
    {
        gcFusion.chiselMap->SaveMeshesToPLYBySubmap(i, "./model/by_submap/"+std::to_string(gcFusion.cameraID)+"/"+std::to_string(i)+".ply");
    }
    /*
    for(int i = 0; i != gcFusion.chiselMap->chunkManagers.size(); ++i)
    {
        std::string filename = "./Map/"+std::to_string(gcFusion.cameraID)+"/";
        gcFusion.chiselMap->SaveMeshesToPLYBySubmapDense(i,gcFusion.gcSLAM.submapPosesRelativeChanges[i],"./model/by_submap/"+std::to_string(gcFusion.cameraID)+"/dense_"+std::to_string(i)+".ply", filename);
    }*/
    gcFusion.SaveAllSubmap();
    
    gcFusion.chiselMap->SaveAllMeshesToPLY("./model/"+std::to_string(gcFusion.cameraID)+"_online_model.ply", activeSubmaps,
        gcFusion.gcSLAM.submapPosesRelativeChanges, gcFusion.gcSLAM.cameraStartPose);
    /*gcFusion.chiselMap->SaveAllMeshesToPLYSemantic("./model/"+std::to_string(gcFusion.cameraID)+"_semantic_model.ply",activeSubmaps,
        gcFusion.gcSLAM.submapPosesRelativeChanges, gcFusion.gcSLAM.cameraStartPose);
    */
    //used for room offline-reconstruction 
    gcFusion.SaveRoomAssociate(rgb_relative_paths, depth_relative_paths);
    gcFusion.SaveSubmapAssociate(rgb_relative_paths, depth_relative_paths);


    //this part is used for comparison between submap optimization and keyframe optimization.

#if FOR_REBUTTAL
    //save keyframe information
    // std::map<std::pair<int, int>, Eigen::Matrix4d> submap_corr_id;

    // { 
        
    //     for(int i = 0; i != gcFusion.gcSLAM.keyFrameCorrLists[0].size(); ++i)
    //     {
    //         auto &corr = gcFusion.gcSLAM.keyFrameCorrLists[0][i];
    //         int ref_id = corr.frame_ref.frame_index / 50;
    //         int new_id = corr.frame_new.frame_index / 50;
    //         if(ref_id == new_id || corr.sum_weight <= 0 || new_id == (gcFusion.gcSLAM.globalFrameList.size() / 50)  ) continue;
    //         std::pair<int, int> corr_id = std::make_pair(ref_id, new_id);

    //         if(submap_corr_id.find(corr_id) == submap_corr_id.end() )
    //         {
    //             Eigen::Matrix4d relative_pose = (gcFusion.gcSLAM.globalFrameList[ref_id * 50].pose_sophus[3].inverse() * gcFusion.gcSLAM.globalFrameList[new_id * 50].pose_sophus[3] ).matrix();

    //             submap_corr_id[corr_id] = relative_pose;
    //         }
            
    //     }
    //     std::ofstream ofs_reg(basepath + "/result.log");
    //     for(auto iter = submap_corr_id.begin(); iter != submap_corr_id.end(); ++iter)
    //     {
    //         ofs_reg<<iter->first.first<<" "<<iter->first.second<<" "<<gcFusion.gcSLAM.globalFrameList.size() / 50<<std::endl;
    //         ofs_reg<<iter->second<<std::endl;
    //     }
    //     ofs_reg.close();
    // }

    // {
    //     //std::map<std::pair<int, int>, Eigen::Matrix4d> submap_corr_id;
    //     for(int i = 0; i != CorrIdList.size(); ++i)
    //     {
    //         auto &corr = CorrIdList[i];
    //         int ref_id = corr.frame_ref.frame_index / 50;
    //         int new_id = corr.frame_new.frame_index / 50;
    //         if(ref_id == new_id || new_id == (gcFusion.gcSLAM.globalFrameList.size() / 50)  ) continue;
    //         std::pair<int, int> corr_id = std::make_pair(ref_id, new_id);

    //         if(submap_corr_id.find(corr_id) == submap_corr_id.end() )
    //         {
    //             PoseSE3d T1 = gcFusion.gcSLAM.globalFrameList[corr.frame_ref.frame_index].pose_sophus[3].inverse() * gcFusion.gcSLAM.globalFrameList[ref_id * 50].pose_sophus[3];
    //             PoseSE3d T2 = gcFusion.gcSLAM.globalFrameList[corr.frame_new.frame_index].pose_sophus[3].inverse() * gcFusion.gcSLAM.globalFrameList[new_id * 50].pose_sophus[3];
    //             PoseSE3d K = relative_pose_list[i];
    //             Eigen::Matrix4d relative_pose = (T1.inverse() * K * T2).matrix();

    //             submap_corr_id[corr_id] = relative_pose;
    //         }
            
    //     }
    //     std::ofstream ofs_reg(basepath + "/result_before_pruning.log");
    //     for(auto iter = submap_corr_id.begin(); iter != submap_corr_id.end(); ++iter)
    //     {
    //         ofs_reg<<iter->first.first<<" "<<iter->first.second<<" "<<gcFusion.gcSLAM.globalFrameList.size() / 50<<std::endl;
    //         ofs_reg<<iter->second<<std::endl;
    //     }
    //     ofs_reg.close();
    // }



    int keyframe_number = 0;
    for(int i = 0; i != gcFusion.gcSLAM.globalFrameList.size(); ++i)
    {
        //save the frame_index, poses, and submap_id for each keyframe
        auto &keyframe = gcFusion.gcSLAM.globalFrameList[i];
        if(keyframe.is_keyframe && keyframe.tracking_success)
        {
            ++keyframe_number;
        }
    }

    {
        std::ofstream file_keyframe("./keyframes_info.txt", std::ios::binary);
        
        std::vector<double> buffer;
        buffer.push_back(keyframe_number);

        for(int i = 0; i != gcFusion.gcSLAM.globalFrameList.size(); ++i)
        {
            //save the frame_index, poses, and submap_id for each keyframe
            auto &keyframe = gcFusion.gcSLAM.globalFrameList[i];
            if(keyframe.is_keyframe && keyframe.tracking_success)
            {
                buffer.push_back(keyframe.frame_index);
                buffer.push_back(keyframe.submapID);
                {
                    Eigen::Matrix3d R = keyframe.pose_sophus[3].rotationMatrix();
                    Eigen::Vector3d t = keyframe.pose_sophus[3].translation();
                    BasicAPI::EigenMatrixToStdVector(R, buffer);
                    BasicAPI::EigenMatrixToStdVector(t, buffer);

                }

                {
                    Eigen::Matrix3d R = keyframe.pose_sophus[2].rotationMatrix();
                    Eigen::Vector3d t = keyframe.pose_sophus[2].translation();
                    BasicAPI::EigenMatrixToStdVector(R, buffer);
                    BasicAPI::EigenMatrixToStdVector(t, buffer);
                }
            }
        }


        file_keyframe.write((char *) &buffer[0],sizeof(double) * buffer.size());
            //std::cout<<buffer[0]<<" "<<bufferSize<<std::endl;
        file_keyframe.close();

    }


    int keyframe_corr_number = 0;
    for(int i = 0; i != gcFusion.gcSLAM.keyFrameCorrLists.size(); ++i)
    {
        auto &frame_corr_in_submap = gcFusion.gcSLAM.keyFrameCorrLists[i];
        for(int j = 0; j != frame_corr_in_submap.size(); ++j)
        {

            ++keyframe_corr_number;
        }
    }

    {
        std::ofstream file_keyframe_corr("./keyframe_corrs.txt", std::ios::binary);

        std::vector<double> buffer;
        buffer.push_back(keyframe_corr_number);

        for(int i = 0; i != gcFusion.gcSLAM.keyFrameCorrLists.size(); ++i)
        {
            auto &frame_corr_in_submap = gcFusion.gcSLAM.keyFrameCorrLists[i];
            for(int j = 0; j != frame_corr_in_submap.size(); ++j)
            {
                // save the ref, new index, save the sum_value

                buffer.push_back(frame_corr_in_submap[j].frame_ref.frame_index);
                buffer.push_back(frame_corr_in_submap[j].frame_new.frame_index);


                BasicAPI::EigenMatrixToStdVector(frame_corr_in_submap[j].sum_p_ref, buffer);
                BasicAPI::EigenMatrixToStdVector(frame_corr_in_submap[j].sum_p_new, buffer);
                BasicAPI::EigenMatrixToStdVector(frame_corr_in_submap[j].sum_p_ref_new, buffer);
                BasicAPI::EigenMatrixToStdVector(frame_corr_in_submap[j].sum_p_ref_ref, buffer);
                BasicAPI::EigenMatrixToStdVector(frame_corr_in_submap[j].sum_p_new_new, buffer);
                BasicAPI::EigenMatrixToStdVector(frame_corr_in_submap[j].sum_p_new_ref, buffer);
                buffer.push_back(frame_corr_in_submap[j].sum_weight);
            }
        }
        file_keyframe_corr.write((char *) &buffer[0],sizeof(double) * buffer.size());
        file_keyframe_corr.close();
    }



//     static std::map<int, float> t_tracking;

// static std::map<int, float> t_local_optimization;
// static std::map<int, float> t_tsdf_fusion;
// static std::map<int, float> t_segmentation;
// static std::map<int, float> t_mesh;
// static std::map<int, float> t_room_detection;
    std::ofstream file_tracking("./running_time/tracking.txt");
    for(auto iter = t_tracking.begin(); iter != t_tracking.end(); ++iter)
    {
        file_tracking << iter->first<<" "<<iter->second<<std::endl;
    }    
    file_tracking.close();    
    std::ofstream file_memory("./running_time/memory.txt");
    for(auto iter = m_gcfusion.begin(); iter != m_gcfusion.end(); ++iter)
    {
        file_memory << iter->first<<" "<<iter->second<<std::endl;
    }    
    file_memory.close();    
    std::ofstream file_local_optimization("./running_time/local_optimization.txt");
    for(auto iter = t_local_optimization.begin(); iter != t_local_optimization.end(); ++iter)
    {
        file_local_optimization << iter->first<<" "<<iter->second<<std::endl;
    }    
    file_local_optimization.close();

    std::ofstream file_tsdf_fusion("./running_time/tsdf_fusion.txt");
    for(auto iter = t_tsdf_fusion.begin(); iter != t_tsdf_fusion.end(); ++iter)
    {
        file_tsdf_fusion << iter->first<<" "<<iter->second<<std::endl;
    }    
    file_tsdf_fusion.close();

    std::ofstream file_segmentation("./running_time/segmentation.txt");
    for(auto iter = t_segmentation.begin(); iter != t_segmentation.end(); ++iter)
    {
        file_segmentation << iter->first<<" "<<iter->second<<std::endl;
    }    
    file_segmentation.close();

    std::ofstream file_mesh("./running_time/mesh.txt");
    for(auto iter = t_mesh.begin(); iter != t_mesh.end(); ++iter)
    {
        file_mesh << iter->first<<" "<<iter->second<<std::endl;
    }    
    file_mesh.close();

    std::ofstream file_room_detection("./running_time/room_detection.txt");
    for(auto iter = t_room_detection.begin(); iter != t_room_detection.end(); ++iter)
    {
        file_room_detection << iter->first<<" "<<iter->second<<std::endl;
    }    
    file_room_detection.close();
#endif

if(g_para.final_integration > 0)
{

    if(g_para.final_integration == 2)
    {
        std::cout <<"offline re-integrate all frames" << std::endl;
        //TICK("Final::IntegrateAllFrames");
        gcFusion.chiselMap->ClearGlobalChunkManager();
        for(int i = 0; i < gcFusion.gcSLAM.globalFrameList.size(); i++)
        {
            gcFusion.gcSLAM.globalFrameList[i].pose_sophus[3] =gcFusion.gcSLAM.cameraStartPose *  
                gcFusion.gcSLAM.submapPosesRelativeChanges[gcFusion.gcSLAM.globalFrameList[i].submapID] *  gcFusion.gcSLAM.globalFrameList[i].pose_sophus[0];
            // if(gcFusion.gcSLAM.globalFrameList[i].is_keyframe)
            gcFusion.IntegrateFrame(gcFusion.gcSLAM.globalFrameList[i]);
        }
        //TOCK("Final::IntegrateAllFrames");
    }
    else if(g_para.final_integration == 1)
    {
        std::cout<<"offline re-integrate all submaps"<<std::endl;
        gcFusion.FinalIntegration();
    }

    //gcFusion.chiselMap->globalChunkManager.saveSubmapB("./globalMap.map");
    gcFusion.chiselMap->UpdateMeshes(gcFusion.cameraModel);
    
    gcFusion.chiselMap->SaveMeshesToPLYGlobal("./model/"+std::to_string(gcFusion.cameraID)+"_final_model.ply", gcFusion.gcSLAM.cameraStartPose);
    //gcFusion.chiselMap->Reset();
    gcFusion.chiselMap->Reset();
}

#if WITHDRAW
    //TODO: Add withdraw function to withdraw the mesh which is not good enough.
    std::cout<<"Do you want to withdraw the mesh?(y/n)"<<std::endl;
    std::string withdraw;
    std::cin>>withdraw;
    while(withdraw != "y" && withdraw != "n")
    {
        std::cout<<withdraw<<" is not a valid input."<<std::endl;
        std::cout<<"input y for yes, n for no."<<std::endl;
        cin>>withdraw;
    }

    ros::ServiceClient client_w = n.serviceClient<collaborative_fusion::WithDraw>("withdraw_mesh");
    collaborative_fusion::WithDraw srv_withdraw;
    srv_withdraw.request.camera_id = gcFusion.cameraID;
    srv_withdraw.request.withdraw = (withdraw == "y");
    if(client_w.call(srv_withdraw))
    {
        std::cout << GREEN<<"Program finish. "<<RESET<<std::endl;
    }
    else
    {
        std::cout << RED<<"[ERROR]::Fail to withdraw."<<RESET<<std::endl;
    }
#endif
    std::cout<<"Running time: "<< over_all_timer.Elapsed("overall")<<"\n processed_frame: "<< gcFusion.gcSLAM.globalFrameList.size()<<"\n frequency: "
        <<gcFusion.gcSLAM.globalFrameList.size() /(over_all_timer.Elapsed("overall") / 1000.0) <<"Hz"<<std::endl;

    std::cout<<"Press ctrl+c to stop..."<<std::endl;
    signal( SIGINT, sig_handler );

    while( keepRunning );
    /*
    write something about final mobile show
    */

    //std::cout<<"Memory: \n"<<gcFusion.GetOccupyMemory()/1024.0/1024/1024<<"GB"<<std::endl;
    /*std::cout<<"gcFusion memory(MB):\n"<<(gcm = gcFusion.getOccupyMemory()/1024.0/1024.0)<<std::endl;
    std::cout<<"GUI memory(MB): "<<1.8<<std::endl;
    size_t memoryLocalVara = 0;
    for(int j = 0;j!=rgb_files.size();++j)
    memoryLocalVara += rgb_files[j].capacity();
    for(int j = 0;j!=depth_files.size();++j)
    memoryLocalVara += depth_files[j].capacity();
    memoryLocalVara += (time_stamp.capacity()+processingTimePerFrame.capacity())*sizeof(double);
    memoryLocalVara += sizeof(camera);
    memoryLocalVara+= sizeof(double)*ground_truth.size();
    memoryLocalVara /= 1024.0*1024.0;
    std::cout<<"other memory LocalVara: "<<memoryLocalVara<<std::endl;
    //std::cout<<"Gui: "<<400<<std::endl;
    std::cout<<"All memory(GB): "<< (gcm+memoryLocalVara)/1024.0+1.8<<std::endl;
    /*
    */
    /*char fileName[2560];
    memset(fileName,0,2560);
    sprintf(fileName,"%s/trajectory.txt",basepath.c_str());
    BasicAPI::saveTrajectoryFrameList(gcFusion.gcSLAM.globalFrameList,fileName);

    float total_time = 0;
    for(int i = 0; i < processingTimePerFrame.size(); i++)
    {
        total_time += processingTimePerFrame[i];
    }
    std::cout << "average processing time per frame: " << total_time / processingTimePerFrame.size() << std::endl;
    memset(fileName,0,2560);
    sprintf(fileName,"%s/OnlineModel_%dmm.ply",basepath.c_str(),(int)(1000 *(gcFusion.GetVoxelResolution())));
    std::cout << "saving online model to:    " << fileName << std::endl;
    gcFusion.chiselMap->SaveAllMeshesToPLY(fileName);
    gcFusion.chiselMap->Reset();

    std::cout <<"offline re-integrate all frames" << std::endl;
    TICK("Final::IntegrateAllFrames");
    for(int i = 0; i < gcFusion.gcSLAM.globalFrameList.size(); i++)
    {
        gcFusion.IntegrateFrame(gcFusion.gcSLAM.globalFrameList[i]);
    }
    TOCK("Final::IntegrateAllFrames");
    gcFusion.chiselMap->UpdateMeshes(gcFusion.cameraModel,0);
    memset(fileName,0,2560);
    sprintf(fileName,"%s/finalModelAllframes_%dmm.ply",basepath.c_str(),(int)(1000 *(gcFusion.GetVoxelResolution())));
    std::cout << "saving offline model to:    " << fileName << std::endl;
    gcFusion.chiselMap->SaveAllMeshesToPLY(fileName);
    gcFusion.chiselMap->Reset();



    for(int i = 0; i < gcFusion.gcSLAM.KeyframeDataList.size(); i++)
    {
        MultiViewGeometry::KeyFrameDatabase kfd = gcFusion.gcSLAM.KeyframeDataList[i];
        Frame &f = gcFusion.gcSLAM.globalFrameList[kfd.keyFrameIndex];

        Eigen::MatrixXf transform = f.pose_sophus[0].matrix().cast<float>();
        transform = transform.inverse();
        Eigen::Matrix3f r = transform.block<3,3>(0,0);
        Eigen::MatrixXf t = transform.block<3,1>(0,3);

        memset(fileName,0,2560);
        sprintf(fileName,"%s/texture/%06d.cam",basepath.c_str(),i);
        FILE * fp = fopen(fileName,"w+");
        fprintf(fp,"%f %f %f %f %f %f %f %f %f %f %f %f\r\n",
                t(0),t(1),t(2),
                r(0,0),r(0,1),r(0,2),
                r(1,0),r(1,1),r(1,2),
                r(2,0),r(2,1),r(2,2));
        fclose(fp);
        memset(fileName,0,2560);
        sprintf(fileName,"%s/texture/%06d.png",basepath.c_str(),i);
        cv::imwrite(fileName,f.getRgb());

    }*/



}
