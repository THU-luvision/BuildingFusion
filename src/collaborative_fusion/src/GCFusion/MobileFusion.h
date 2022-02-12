#ifndef MOBILEFUSION_H
#define MOBILEFUSION_H



#include "../GCSLAM/frame.h"
#include "../GCSLAM/MultiViewGeometry.h"
#include "../BasicAPI.h"
#include "../GCSLAM/GCSLAM.h"
#include "MapMaintain.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <sys/stat.h>

#include "../Shaders/Shaders.h"

#include <pangolin/pangolin.h>
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <GL/freeglut.h>
#include <GL/glut.h>
#include <GL/glew.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
//#include <boost/thread/recursive_mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/filesystem.hpp>
#include <open_chisel/Chisel.h>
#include <open_chisel/ProjectionIntegrator.h>
#include <open_chisel/ChunkManager.h>
#include <open_chisel/io/PLY.h>
#include <open_chisel/camera/PinholeCamera.h>
#include <open_chisel/camera/DepthImage.h>
#include <open_chisel/camera/ColorImage.h>
#include <open_chisel/truncation/QuadraticTruncator.h>
#include <open_chisel/weighting/ConstantWeighter.h>
#include <atomic>
#include <queue>
#include <collaborative_fusion/RoomRegistration.h>
#include "model.h"
#include "torch/torch.h"
#include "cnpy.h"
#include "../RoomLCD/utils.h"
#include "../RoomLCD/PlaneFittingModel.hpp"
#include "../RoomDetection/Building.h"
#include "RoomOptimization.h"
#include "../IO/RPLYReader.h"
// #define MAX_MOBILEFUSION_IMAGE_WIDTH  640
// #define MAX_MOBILEFUSION_IMAGE_HEIGHT 480

#define GLOBAL_MODLE_VERTEX_NUM (1024*1024*50)
#define ADD_VERTEX_NUM_EACH_TIME (1024*1024*9)
#define VERTEX_WEIGHT_THRESHOLD 3
#define SUBMAP_REINTEGRATION 0
#define VOXEL_SIZE 10
#define DETECT_ROOM 1


typedef float DepthData;
typedef uint8_t ColorData;


typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Mat4List;
typedef std::vector<Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > Mat3List;
typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vec3List;
typedef Eigen::Vector3i RGBAPixel;
typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > RGBAPixelList;
#define INTEGRATE_ALL 1
#define FUSE_COLOR  0
//static std::atomic_flag fusion_control = ATOMIC_FLAG_INIT; 

#if FOR_REBUTTAL
extern std::map<int, float> t_tsdf_fusion;
extern std::map<int, float> t_segmentation;
extern std::map<int, float> t_mesh;
extern std::map<int, float> t_room_detection;
extern std::map<int, float> m_gcfusion;
#endif

class MobileFusion
{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GLuint vbo;
    //GLuint vbo_point_cloud;
    //GLuint vbo_data_transfer;
    //GLuint feedback_vbo;
    //GLuint unstable_vbo;
    std::shared_ptr<Shader> drawProgram;
    std::shared_ptr<Shader> drawPhongLighting;
    std::shared_ptr<Shader> drawVoxelHashingStyle;
    GRANSAC::RANSAC<PlaneFittingModel, 8> Estimator;

    cv::Mat patch_image;
    cv::Mat cell_image;
    DCEL dcel;
    Building building;
    int interval_threshold = 80;
    //VertexElement *currentObservationBuffer;          //As a simple version, global model vertices are maintained at CPU for simplicity
    std::mutex global_show_lock;
    int byte_of_each_point = 30;
    bool use_gui = 1;
    //float * tsdf_visualization_buffer;
    unsigned char * tsdf_visualization_buffer;
    std::vector<unsigned char> real_buffer;// can be allocated flexibly
    //int visualize_vertex_num;
    //int valid_vertex_cnt;
    int need_to_tsdfFusion=0;                        // update vertex data;
    std::mutex in_tsdfFusion;
    int add_new_submap=0;
    int global_vertex_data_updated = 0;
    int global_vertex_data_need_to_update = 0;
    //std::vector<int> verticesHashTable;

    //std::vector<Mat4List> PrePoseList;

    chisel::ChiselPtr chiselMap;
    //std::shared_ptr<chisel::DepthImage<DepthData> > lastDepthImage;
    //std::shared_ptr<chisel::ColorImage<ColorData> > lastColorImage;
    chisel::ProjectionIntegrator projectionIntegrator;
    chisel::PinholeCamera cameraModel;

    //std::vector<Mat4List> IntegratePoseList;
    //for tsdf fusion
    //std::vector<ChunkIDList> chunksIntersectings;

    //std::vector<std::vector<bool>> needsUpdateFlag;
    //std::vector<std::vector<bool>> newChunkFlags;
    //Frame * lastKeyframe;
    //std::vector<std::pair<int,int>> waitToIntegrate;//for loop closre, other submap or camera 
    boost::condition_variable update_globalMap_cv;
    boost::mutex update_globalMap_mutex;
    std::queue<int> validFrameNums;
    std::queue<int> fuseKeyframeIds;
    std::queue<int> roomDetectionFlags;
    //int fuseKeyframeId;
    std::queue<int> currentSubMapIDs;
    std::queue<int> closureIDs;
    //std::queue<int> closureCameraIDs;
    std::queue<int> closureIndexs;
    int mainSubmapID=0;
    int referenceSubmapID=-1;
    std::queue<int> shouldAddNewSubmaps;
    std::queue<int> integrateKeyframeIDs;
    int closureID=-1;
    //int closureCameraID=-1;
    int closureIndex=-1;
    std::map<int, string> time_consuming;
    int add_vertex_num = 0;
    //int currentSubMapID;
    int active_submap_number;
    int integrateLocalFrameNum = 3;
    size_t amount_tsdf_vertice_num = 0;
    //bool shouldAddNewSubmap = false;
    std::queue<size_t> waitToSim;
    std::queue<size_t> waitToSave;
    std::queue<size_t> waitToSaveF;
    //if last visited keyframe index - current keyframe index > 10, kill this submap
    std::map<int, int> reactivated_submaps;//submap id, last_visited keyframe index


    int needToCollibrate;
    int needToOptimizeSubmap;
    bool quitLockFusion = false;
    int current_room_id = -1;
    int last_room_id = -1;
    int room_registration = -1;
    //std::vector<std::queue<PoseSE3dList>> relativeChangesQueue;
    int cameraID;
    GCSLAM gcSLAM;
    std::vector<std::vector<int>> room_to_submap;
    std::vector<int> submap_to_room;
    std::shared_ptr<LearningBWDenseUNet> unet;
    MultiViewGeometry::CameraPara camera;
    std::vector<LinePatch> wall_lines;
    SceneDatabase scene_database;//used for room loop closure detection
    std::set<int> unlabeled_submap;// submaps which don't belong to any room
    std::vector<chisel::PcdPtr> room_pcds;
    std::vector<float> room_area_s;
    std::vector<RoomCorrespondence> room_correspondences;
    inline void setIntegrateLocalFrameNum(int _integrateLocalFrameNum)
    {
        integrateLocalFrameNum = _integrateLocalFrameNum;
    }
    void killReactiveSubmap()
    {
        std::vector<int> tmp_submaps;
        int current_keyframe_id = gcSLAM.globalFrameList.size()-1;
        for(auto iter = reactivated_submaps.begin(); iter != reactivated_submaps.end();++iter)
        {
            if(current_keyframe_id - iter->second > interval_threshold)
            {
                saveSubmap(iter->first);
                simplifySubmap(iter->first);
                tmp_submaps.push_back(iter->first);
            }
        }

        for(int i = 0; i < tmp_submaps.size(); ++i)
        {
            reactivated_submaps.erase(tmp_submaps[i]);
        }
    }
    void FinalIntegration(const std::string &filepath = "./")
    {
        std::string filename = filepath+"Map/"+std::to_string(cameraID)+"/";
        chiselMap->FinalIntegrationToGlobalChunkManager(projectionIntegrator, gcSLAM.submapPosesRelativeChanges,filename);
    }
    int UpdateSegmentationGlobal();
    void SaveAllSubmap(const std::string &filepath = "./")
    {
        std::string filename = filepath+"Map/"+std::to_string(cameraID)+"/";
        chiselMap->SaveAllSubmap(filename);
    }
    void reactivateSubmap(int submap_id, int keyframe_index)
    {
        std::cout<<"Re-activate submap: "<<submap_id<<std::endl;
        if(submap_id < 0)
        return;
        if(submap_id > chiselMap->chunkManagers.size())
        return;
        reactivated_submaps[submap_id] = keyframe_index;
        if(chiselMap->chunkManagers[submap_id].isValid)
        return;

        //gcSLAM.timer.Tick("load chunks from outside");
        loadSubmap(submap_id);
        //gcSLAM.timer.Tock("load chunks from outside");
    }
    void Planes2Lines(std::vector<PlanePatch> &planes , std::vector<LinePatch> &lines)
    {
        lines.clear();
        //lines.resize(planes.size());
        for(int i = 0; i != planes.size(); ++i)
        {
            Point2fList items(planes[i].items.size());

            for(int j = 0; j != planes[i].items.size(); ++j)
            {
                items[j] = planes[i].items[j].block<2,1>(0,0);
            }
            
            auto result = FitLine(items);
            Eigen::Vector3f tmp_line;
            tmp_line.block<2,1>(0,0) = std::get<0>(result);
            tmp_line(2) = std::get<1>(result);
            bool is_merged = false;
            for(int j = 0; j != lines.size(); ++j)
            {
                if(CompareTwoLine(tmp_line, lines[j].rep))
                {
                    is_merged = true;
                    lines[j].items.insert(lines[j].items.end(), items.begin(), items.end());
                    auto merged_result = FitLine(lines[j].items);
            
                    lines[j].rep.block<2,1>(0,0)  = std::get<0>(merged_result);
                    lines[j].rep(2) = std::get<1>(merged_result);
                    break;
                }
            }

            if(is_merged == false)
            {
                lines.push_back(LinePatch());
                lines.back().items = std::move(items);
                lines.back().rep = tmp_line;
            }
            /*
            lines[i].rep(0) = planes[i].rep(0);
            lines[i].rep(1) = planes[i].rep(1);
            lines[i].rep(2) = planes[i].rep(3);
            */
        }
    }
    inline void setCameraID(int _cameraID)
    {
        cameraID = _cameraID;
    }
    inline void prepareDCEL(float r)
    {
        //Eigen::Vector2f start_point = Eigen::Vector2f::Zero();
        Eigen::Vector2f start_point = (gcSLAM.R * (gcSLAM.globalFrameList.back().pose_sophus[3].translation().cast<float>())).head<2>();
        dcel.InitialWithBB(Eigen::Vector2f(-r,-r) + start_point, Eigen::Vector2f(r, -r) + start_point, 
            Eigen::Vector2f(r, r) + start_point, Eigen::Vector2f(-r, r) + start_point);
    }
    inline void resetDCEL()
    {
        dcel.Reset();
    }
    inline void prepareBuilding()
    {
        building.dcel = std::make_shared<DCEL>(dcel);
        building.lines = wall_lines;
    }
    inline void resetBuilding()
    {
        building.Reset();
    }
    int SeparateRoom(int frame_index = -1)
    {
        if(dcel.faces.size() < 7)
        return 0;
        std::vector<std::vector<int>> _room_to_submap;
        std::cout<<"Start to Separate Room..."<<std::endl;

        gcSLAM.UpdateCenterCoord();
        std::cout<<"Set embedding dimension"<<std::endl;
        building.SetEmbeddingDimension();
        std::cout<<"Compute weights..."<<std::endl;
        building.ComputeWeightsForEachEdge();
        std::cout<<"Compute embedding..."<<std::endl;
        building.ComputeEmbedding();
        std::cout<<"Separate room..."<<std::endl;
        std::vector<float> room_areas;
        auto submap_center = gcSLAM.centerCoord;
        building.SeparateRoomUsingCameraCoord(gcSLAM.centerCoord,
            _room_to_submap, unlabeled_submap, room_areas);

        if(frame_index == -1)
            frame_index = gcSLAM.globalFrameList.size()-1;
        int start_room_id = room_to_submap.size();
        if(building.IsInRoom(gcSLAM.GetFramePosition(frame_index), 0))
        {
            for(int i = 0; i != _room_to_submap.size(); ++i)
            {
                for(int j = 0; j != _room_to_submap[i].size(); ++j)
                {
                    int room_id = i + start_room_id;
                    unlabeled_submap.erase(_room_to_submap[i][j]);
                    submap_to_room[_room_to_submap[i][j]] = room_id;
                }
                //Now we don't need it
                //room_to_droom[room_id] = i;
            }
            int has_new_room = _room_to_submap.size();
            room_to_submap.insert(room_to_submap.end(), 
                _room_to_submap.begin(), _room_to_submap.end());
            room_pcds.resize(room_to_submap.size());
            room_area_s.insert(room_area_s.end(), room_areas.begin(), room_areas.end());
#if 0
            cv::Mat room_img = building.RoomImage();
            cv::imwrite("./"+std::to_string(cameraID)+"_Room_image.png", room_img);
#endif
            return has_new_room;
        }
        else if(_room_to_submap.size())
        {
            std::cout<<RED<<"Wrong Room Detection: The room doesn't contain current camera position!"<<RESET<<std::endl;
        }

        resetBuilding();
        resetDCEL();
        return 0;
    }
    int GetCurrentRoomID()
    {
        if(room_to_submap.size() == 0)
        {
        std::cout<<"the size of room_to_submap is 0."<<std::endl;
        return -1;
        }
        if(building.rooms.size() == 0)
        {
        std::cout<<"the size of room in building is 0."<<std::endl;
        return -1;
        }
        std::cout<<"the size of room_in building is "<<building.rooms.size()<<std::endl;
        if(building.IsInRoom(gcSLAM.GetCurrentPosition(), 0))
        return room_to_submap.size()-1;
        else return -1;
    }
    int GetFrameRoomID(int frame_id)
    {
        if(room_to_submap.size() == 0)
        {
        std::cout<<"the size of room_to_submap is 0."<<std::endl;
        return -1;
        }
        if(building.rooms.size() == 0)
        {
        std::cout<<"the size of room in building is 0."<<std::endl;
        return -1;
        }
        std::cout<<"the size of room_in building is "<<building.rooms.size()<<std::endl;
        if(building.IsInRoom(gcSLAM.GetFramePosition(frame_id), 0))
        return room_to_submap.size()-1;
        else return -1;        
    }
    void DetectPlaneAndBuildDcel(int closure_id = 0)
    {
        Point3fList points, normals, f_points;
        std::vector<double> residuals;
        std::vector<PlanePatch> planes;
        //std::vector<LinePatch> lines;
        std::set<int> tmp_submap;
        //if(closure_id >= 1)
        //closure_id -= 1;
        for(auto iter = unlabeled_submap.begin(); iter!= unlabeled_submap.end(); ++iter)
        if(*iter >= closure_id)
            tmp_submap.insert(*iter);
        chiselMap->GetPointCloud(points,normals,residuals, tmp_submap);
        /*chiselMap->GetFloorPCD(f_points, unlabeled_submap);
        Eigen::Vector3f f_n;
        auto result = FitPlane(f_points);
        f_n = std::get<0>(result);
        if(f_n(2) < 0) f_n = -f_n;
    //std::cout<<n(0)<<" "<<n(1)<<" "<<n(2)<<" "<<d<<std::endl;
        Eigen::Matrix3f W = f_n * Eigen::Vector3f(0,0,1).transpose();
        std::cout<<"floor normal: "<<f_n(0)<<" "<<f_n(1)<<" "<<f_n(2)<<std::endl;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto UT = svd.matrixU().transpose();
        auto V = svd.matrixV();

        gcSLAM.R =  V*UT;
        std::cout<<"points: "<<points.size()<<std::endl;
        for(int i = 0; i != points.size(); ++i)
        {
            points[i] =gcSLAM.R * points[i];
        }*/
        if(points.size() > 100)
        {
            //chiselMap->SavePointCloudToPLY("chunk_pointcloud.ply", points, normals);
            //gcSLAM.timer.Tick("PlaneDetection");
            PlaneDetection(points,normals,residuals,planes);
            //gcSLAM.timer.Tock("PlaneDetection");


                

            //gcSLAM.timer.Tick("IncrementLine");
            Planes2Lines(planes, wall_lines);
            std::cout<<"We detect "<<wall_lines.size()<<" lines."<<std::endl;
            for(int i = 0; i != wall_lines.size(); ++i)
            {
                std::cout<<wall_lines[i].rep(0)<<" "<<wall_lines[i].rep(1)<<" "
                    <<wall_lines[i].rep(2)<<" "<<wall_lines[i].items.size()<<std::endl;
                dcel.IncrementLine(wall_lines[i].rep);
            }
            
            //gcSLAM.timer.Tock("IncrementLine");

        }
    }
    void UpdatePatchImages(int frame_index = -1)
    {
        if(frame_index < 0)
        frame_index = gcSLAM.globalFrameList.size()-1;
        patch_image = visualization::PatchImage(wall_lines,gcSLAM.GetFramePosition(frame_index).block<2,1>(0,0));
        
        cell_image = dcel.Draw(gcSLAM.GetFramePosition(frame_index).block<2,1>(0,0), gcSLAM.centerCoord);
    }
    void ExtractRoomModel(int room_id)
    {
        
        if(room_id > room_to_submap.size()) 
        {
            std::cout<<"Room does not exist!"<<std::endl;
            return;
        }
        //room_pcds[room_id] = chiselMap->ExtractRoomModel(room_to_submap[room_id], gcSLAM.submapPosesRelativeChanges);
        room_pcds[room_id] = chiselMap->ExtractRoomModelSparse(room_to_submap[room_id]);
#if 0
        room_pcds[room_id]->Rotate(gcSLAM.R.inverse());
#endif        
    }
    void OptimizeRoomAndSubmap()
    {
        //update all the pose of keyframes
        std::cout<<"Optimize Room and submap!"<<std::endl;
        CombiningOptimization(room_correspondences, gcSLAM.submapCorrLists,
            gcSLAM.submapPosesRelativeChanges, gcSLAM.submapPosesFinal,
             room_to_submap, submap_to_room);
        gcSLAM.UpdateAllKeyframe();
    }
    void SaveRoomAssociate(const std::vector<std::string> &rgb_paths, const std::vector<std::string> &depth_paths)
    {

        for(int i = 0 ; i != room_to_submap.size(); ++i)
        {
            std::cout<<"save room "<<i<<"..."<<std::endl;
            std::ofstream ofs("./room_associate/"+std::to_string(cameraID)+"/room_"+std::to_string(i)+"_associate.txt");
            int re_index = 0;
            std::sort(room_to_submap[i].begin(), room_to_submap[i].end());
            for(int j = 0; j != room_to_submap[i].size(); ++j)
            {
                
                int submap_id = room_to_submap[i][j];
                std::cout<<"processing on submap "<<submap_id<<std::endl;
                for(auto frame_index = gcSLAM.frameIndexInSubmap[submap_id].begin(); 
                    frame_index != gcSLAM.frameIndexInSubmap[submap_id].end(); ++frame_index)
                {
                    ofs<<re_index<<" "<<rgb_paths[*frame_index]<<" "<<re_index<<" "<<depth_paths[*frame_index]<<"\n";
                    re_index += 1;
                }
            }
            ofs.close();
        }
        
    }
    void SaveSubmapAssociate(const std::vector<std::string> &rgb_paths, const std::vector<std::string> &depth_paths)
    {
        for(int i = 0; i != submap_to_room.size(); ++i)
        {

                std::cout<<"save submap "<<i<<"..."<<std::endl;
                std::ofstream ofs("./submap_associate/"+std::to_string(cameraID)+"/submap_"+std::to_string(i)+"_associate.txt");
                int re_index = 0;
                int submap_id = i;

                for(auto frame_index = gcSLAM.frameIndexInSubmap[submap_id].begin(); 
                    frame_index != gcSLAM.frameIndexInSubmap[submap_id].end(); ++frame_index)
                {
                    ofs<<re_index<<" "<<rgb_paths[*frame_index]<<" "<<re_index<<" "<<depth_paths[*frame_index]<<"\n";
                    re_index += 1;
                }
                ofs.close();  
        }
        
    }
    void RegisterRoom(int room_id)
    {
        //gcSLAM.timer.Tick("Construct graph");
        auto graph_new = get_graph(*room_pcds[room_id],room_id);
        //gcSLAM.timer.Tock("Construct graph");
        auto obj_pcd = graph_new.visualize_to_ply();
        WritePLYFromChiselPcdWithLabel("object_0.ply", obj_pcd);
        std::vector<RoomCorrespondence> room_correspondences_tmp;
        //gcSLAM.timer.Tick("room LCD query");
        scene_database.query(graph_new, room_correspondences_tmp);
        //gcSLAM.timer.Tock("room LCD query");
        
        //gcSLAM.timer.Tick("room LCD insert");
        scene_database.insert(graph_new);
        //gcSLAM.timer.Tock("room LCD insert");
        // lack of a parameter, trash code
        //for(int i = 0 ; i!= room_correspondences_tmp.size(); ++i)
            //room_correspondences_tmp[i].preIntegrate();
        //room_correspondences.insert(room_correspondences.end(), 
        //    room_correspondences_tmp.begin(), room_correspondences_tmp.end());
    }

    void RegisterRoom(ros::ServiceClient &client, int room_id)
    {
        collaborative_fusion::RoomRegistration srv;
        CommunicationAPI::toRosRoomInfo(cameraID, room_id, 
            room_to_submap[room_id], room_pcds[room_id], srv.request.room_info); 
        srv.request.room_info.room_area = room_area_s[room_id];
        //std::cout<<std::endl;
        //auto copy = gcSLAM.submapPosesRelativeChanges;
        if(client.call(srv))
        {
            std::cout<<"Register room, area: "<<srv.request.room_info.room_area<<std::endl;
            std::cout<<GREEN<<"[Info]::Finish Room Registration."<<RESET<<std::endl;
            CommunicationAPI::toCvPoseList(srv.response.submap_poses, gcSLAM.submapPosesRelativeChanges);
            
            for(int i = 0;i!=gcSLAM.submapPosesRelativeChanges.size();++i)
            {
                //gcSLAM.submapPosesRelativeChanges[i] =gcSLAM.submapPoses[i] * gcSLAM.submapPosesRelativeChanges[i];
                gcSLAM.submapPosesFinal[i] = gcSLAM.submapPosesRelativeChanges[i] * gcSLAM.submapPoses[i];
            }
            gcSLAM.UpdateAllKeyframe();
            //global_vertex_data_need_to_update = 1;
        }
        else
             std::cout<<RED<<"[ERROR]::Room Registration failed, please check the connection."<<RESET<<std::endl;
/*
        for(int i = 0;i!=gcSLAM.submapPosesRelativeChanges.size();++i)
        {
            std::cout<<i<<std::endl;
            std::cout<<gcSLAM.submapPosesRelativeChanges[i].matrix()<<std::endl<<std::endl;
            std::cout<<copy[i].matrix()<<std::endl<<std::endl;
        }
*/
    }
    int addNewSubmap()
    {
        int ID = chiselMap->addNewSubmap();
        //std::cout<<"gcslam: add  new map!"<<std::endl;
        //gcSLAM.addNewSubmap();
        unlabeled_submap.insert(ID);
        submap_to_room.push_back(-1);
        return ID;
    }
    void GetAllFloorNormals()
    {
        auto &normals = gcSLAM.floor_normals;
        Eigen::Vector3f default_value, floor;

        default_value << 999.0,0,0; 
        
        normals.resize(gcSLAM.submapPoses.size(),default_value);
        int start = 0;
        if(normals.size() >= 7)
        start = normals.size() - 7;
        if(normals.size() > 3)
        {
            for(int i = start; i < normals.size() - 3; ++i)
            {
                //normals[i] = chiselMap->GetFloorNormal(i);
                if(normals[i](0) == 999.0)
                {
                    normals[i] = GetFloorNormal(i);
                    if(normals[i](1) > 0)
                    normals[i] = -normals[i];
                }
                std::cout<<normals[i].transpose()<<std::endl;
            }
        }
    }

    Eigen::Vector3f GetFloorNormal(int submap_id)
    {
        //Use Ransac to get floor normal
        Point3fList floor_points;
        int count = chiselMap->GetFloorPCD(submap_id, floor_points);
        std::cout<<"floor: "<<submap_id<<" "<<count<<std::endl;
        //chiselMap->SavePointCloudToPLY(std::to_string(submap_id) + "_chunk_pointcloud.ply", floor_points, floor_points);
        Eigen::Vector3f normal;
        normal<<999.0,0,0;
        
        if(count >= 20)
        {
            //should use ransac for robustness.
            std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
            Point3fList _8_points;
            for(int i = 0; i != floor_points.size(); ++i)
            {
                CandPoints.push_back(std::make_shared<GPoint3f>(floor_points[i]));
            }
            Estimator.Estimate(CandPoints);
            auto bestModel = Estimator.GetBestModel();
            auto bestPoints = bestModel->GetModelParams(); 
            for(int i = 0; i != 8; ++i )
            {
                _8_points.push_back(std::dynamic_pointer_cast<GPoint3f>(bestPoints[i])->p);
            }
            std::cout<<Estimator.GetBestInliers().size()<<std::endl;
            //auto bestInliers = Estimator.GetBestInliers();
            auto result = FitPlane(_8_points);
            normal = std::get<0>(result);
        }
        return normal;
    }




    void loadSubmap(int submapID,const std::string &filepath="./")
    {
        if(submapID < 0) return;
        std::string filename = filepath+"Map/"+std::to_string(cameraID)+"/";
        //std::cout<<chiselMap->chunkManagers.size()<<std::endl;
        chiselMap->loadSubmap(submapID,filename);
    }
    // clear redudent memory stored in frames, including depth, normal, color, features.
    void resetNeedToTSDFFusion()
    {

            need_to_tsdfFusion = 0;
            needToCollibrate = 0;
            needToOptimizeSubmap = 0;
            closureID = -1;
            //closureCameraID = -1; 
            closureIndex = -1;

    }
    void clearRedudentFrameMemory(int integrateLocalFrameNum)
    {

        int keyFrameNum = gcSLAM.KeyframeDataList.size();
        if(keyFrameNum > 1)
        {
            int index = keyFrameNum - 2;
            MultiViewGeometry::KeyFrameDatabase kd = gcSLAM.KeyframeDataList[index];

            float inc = 0;
            for(int k = 0; k < kd.corresponding_frames.size(); k++)
            {
                if(k < inc - 1e-4)
                {
                    gcSLAM.globalFrameList[kd.corresponding_frames[k]].clear_memory();
                    continue;
                }
                inc += kd.corresponding_frames.size() / float(integrateLocalFrameNum);
                gcSLAM.globalFrameList[kd.corresponding_frames[k]].clear_redudent_memoery();
            }

            gcSLAM.globalFrameList[kd.keyFrameIndex].clear_keyframe_memory();
        }
    }
    float getOccupiedMemory()
    {
        float result_memory = /* gcSLAM.GetOccupyMemory() / 1024.0 / 1024.0 +*/ chiselMap->GetOccupyMemory() / 1024.0 / 1024.0;
        //std::cout<<"------------------------------------------------- "<<result_memory<<std::endl;
        return result_memory;
    }
    void simSubmap(const std::string &filepath = "./")
    {
        
        if(!waitToSim.empty())
        {
            size_t submapID = waitToSim.front();
            waitToSim.pop();
            if(submapID == mainSubmapID || submapID == referenceSubmapID)
            return ;
            std::cout<< "sim submap "<<submapID <<"!"<<std::endl;
            simplifySubmap(submapID);
            //waitToSave.push(submapID);

        }
        else ;
    }
    void saveSubmap(const std::string &filepath = "./")
    {

        if(!waitToSave.empty())
        {

            size_t submapID = waitToSave.front();
            waitToSave.pop();
            if(submapID == mainSubmapID || submapID == referenceSubmapID)
            return ;
            std::cout<< "save submap "<<submapID <<"!"<<std::endl;
            saveSubmap(submapID,filepath);

        }
        else ;
    }

    void saveFrames(const std::string &filepath = "./")
    {
        if(!waitToSaveF.empty())
        {
            size_t submapID = waitToSaveF.front();
            if(submapID == mainSubmapID || submapID == referenceSubmapID)
            return ;
            waitToSaveF.pop();
            std::cout<<"save frames "<<submapID<<"..."<<std::endl;
            saveFrames(submapID,filepath);
        }
    }
    void updateGlobalMap(bool newSubmapFlag, size_t _currentSubMapID, 
        int room_detection_flag, int _closureID, int roomDetectionFrame = -1)
    {

        //quitLockFusion = quit;
        //bool needToNotify = false;
        in_tsdfFusion.lock();
        std::cout<<"multi thread tsdf fusion..."<< need_to_tsdfFusion<<std::endl;
        //needToNotify = true;
        int inputValidFrameNum  = gcSLAM.globalFrameList.size();
        int inputFuseKeyframeId = gcSLAM.globalFrameList.size()-1;
        if(roomDetectionFrame != -1)
        {
            inputFuseKeyframeId = roomDetectionFrame;
        }
        int currentSubMapID = _currentSubMapID;
        int _shouldAddNewSubmap =newSubmapFlag;
        //int _closureID = closureID;
        //int _closureCameraID = closureCameraID;
        //int _closureIndex = closureIndex;
        //if(TSDFFusion_time%step_len == 0) 
        //_shouldAddNewSubmap = 1;
        /*
        else if( closureID != -1 && std::fabs(closureID -  mainSubmapID) > 2)
        {
        
        std::cout<<"------------------------------closureID: "<<closureID<<std::endl;
        _shouldAddNewSubmap = 2; 
        loadSubmap(closureID);
        }*/

        validFrameNums.push(inputValidFrameNum);
        closureIDs.push(_closureID);
        //closureCameraIDs.push(_closureCameraID);
        //closureIndexs.push(_closureIndex);
        fuseKeyframeIds.push(inputFuseKeyframeId);
        currentSubMapIDs.push(currentSubMapID);
        shouldAddNewSubmaps.push(newSubmapFlag);
        integrateKeyframeIDs.push(gcSLAM.KeyframeDataList.size() - 2);
        roomDetectionFlags.push(room_detection_flag);
        //if(gcSLAM.resetRelativePoses)
       // relativeChangesQueue.push(gcSLAM.submapPosesRelativeChanges);
       // else
       // relativeChangesQueue.push(PoseSE3dList());

        
        //if(needToNotify)
        update_globalMap_cv.notify_one();
//        boost::unique_lock<boost::mutex> lock(update_globalMap_mutex);
    }
    bool fusionDone()
    {
        return fuseKeyframeIds.empty()||validFrameNums.empty();
    }
    void finishAllRemnantFusion()
    {
        int i = 5;
        while(!fusionDone()&&i)
        {
            finishRemnantFusion();
            --i;
        }
    }
    void changeMainSubmap(int newSubMapID)
    {
            std::cout<<"change mainSubmap to new submap "<<newSubMapID<<", in tsdf fusion!"<<std::endl;
            int wait_to_kill = referenceSubmapID;
            referenceSubmapID = mainSubmapID;
            mainSubmapID = newSubMapID;
            
            if(wait_to_kill >= 0 && wait_to_kill != referenceSubmapID && wait_to_kill != mainSubmapID)
            {
            //std::cout<<"add to wait list: wait_to_simplfiy and wait_to_save!"<<std::endl;
            waitToSim.push(wait_to_kill);
            waitToSave.push(wait_to_kill);
            waitToSaveF.push(wait_to_kill);
            chiselMap->deactivateSubmap(wait_to_kill);
            }
    }
    void finishRemnantFusion()
    {

            if(validFrameNums.empty()) 
            {
                return;

            }
            int validFrameNum = validFrameNums.front();
            int fuseKeyframeId = fuseKeyframeIds.front();
            int currentSubMapID = currentSubMapIDs.front();
            int shouldAddNewSubmap = shouldAddNewSubmaps.front();
            int integrateKeyframeID = integrateKeyframeIDs.front();
            int closureid = closureIDs.front();
            //int closureCameraid = closureCameraIDs.front();
            //int closureindex = closureIndexs.front();
            int roomDetectionFlag = roomDetectionFlags.front();
            //mainSubmapID = currentSubMapID;
            //PoseSE3dList submapPose = relativeChangesQueue.front();
            /*
            int validFrameNum = validFrameNums.back();
            int fuseKeyframeId = fuseKeyframeIds.back();
            int currentSubMapID = currentSubMapIDs.back();
            bool shouldAddNewSubmap = shouldAddNewSubmaps.back();
            int integrateKeyframeID = integrateKeyframeIDs.back();
            */
            std::cout<<"cameraID: "<<cameraID<<" shouldAddNewSubmap: "<<shouldAddNewSubmap<<" fuseKeyframeId: "<<fuseKeyframeId<<" integrateKeyframeID: "<<integrateKeyframeID<<std::endl; 
            std::cout<<"current submapID: "<<currentSubMapID<< " main submap id: "<<mainSubmapID<<std::endl;
            if(currentSubMapID != mainSubmapID)
            {
                std::cout<<RED<<"ERROR: current submap is not the main submap."<<RESET<<std::endl;
                exit(1);
            }
            validFrameNums.pop();

            fuseKeyframeIds.pop();

            currentSubMapIDs.pop();

            shouldAddNewSubmaps.pop();

            integrateKeyframeIDs.pop();
            roomDetectionFlags.pop();
            closureIDs.pop();
            //closureCameraIDs.pop();
            //closureIndexs.pop();

/*  
如果遇到了回环同时应该增加新的子图 那么就取消新建子图
*/
            //int lastChunkCount = chiselMap->GetChunkCount(currentSubMapID);
            //std::cout<<"old chunk count: "<<lastChunkCount<<std::endl;
            /*
            if(shouldAddNewSubmap == 2)
            {
                std::cout<<"integrate two submaps, in tsdf fusion!"<<std::endl;            
                Frame &kfref = gcSLAM.globalFrameList[closureindex];
                Frame &kfnew = gcSLAM.globalFrameList[fuseKeyframeId];

                PoseSE3d poseref = gcSLAM.submapPosesRelativeChanges[currentSubMapID].inverse() * kfref.pose_sophus[3] ;
                PoseSE3d posenew = gcSLAM.submapPosesRelativeChanges[closureid].inverse()  * kfnew.pose_sophus[3] ;
                chisel::ChunkMap commonChunks;
                
                ReIntegrateKeyframeOtherSubmap(currentSubMapID,kfref,poseref,commonChunks);
                integrateTwoChunkMap(currentSubMapID,closureid,commonChunks,chiselMap->GetChunkMap(closureid));

                //ReIntegrateKeyframeOtherSubmap(closureCameraid,closureid,kfnew,posenew,commonChunks);
                //integrateTwoChunkMap(closureCameraid,cameraID,closureid,currentSubMapID,commonChunks,chiselMap->GetChunkMap(currentSubMapID));
                chiselMap->clearGlobalMeshes();            
                //chiselMap->UpdateMeshes(cameraModel,mainSubmapIDs[closureCameraid],referenceSubmapIDs[closureCameraid]);
                //global_vertex_data_need_to_update = 1;            
            }
            else 
            {

            }*/
            //assert(currentSubMapID == mainSubmapID);
#if SUBMAP_REINTEGRATION
            if(closureid >= 0 && std::fabs(currentSubMapID -  closureid) > 2)
            {
                std::cout<<"re-activate submap "<<closureid<<std::endl;
                reactivateSubmap(closureid, fuseKeyframeId);
            }
#endif
            TICK("MobileFusion::TSDFFusion");
            tsdfFusion(gcSLAM.globalFrameList,
                                    fuseKeyframeId,
                                    gcSLAM.GetKeyframeDataList(),
                                    gcSLAM.frameIndexInSubmap[currentSubMapID],
                                    integrateKeyframeID,currentSubMapID);
            GetFullMeshes();
            std::cout<<"__________FINISH TSDFFUSION______________"<<std::endl;
            TOCK("MobileFusion::TSDFFusion");

            
            //if we detect loop closure, we would change the loop closure submap as the main submap and save the old reference submap. 
            

#if DETECT_ROOM
            if(roomDetectionFlag && current_room_id == -1)
            {
                std::cout<<"Try to separate room..."<<std::endl;
                ftimer.Tick("room detection");
                int has_new_room = DetectAndSeparate(fuseKeyframeId, closureid);
                ftimer.Tock("room detection");
#if FOR_REBUTTAL
                t_room_detection[fuseKeyframeId] = ftimer.Elapsed("room detection");
#endif
                UpdatePatchImages(fuseKeyframeId);
                //not sure if this is necessary
                if(has_new_room != 0) unlabeled_submap.clear();
            }
#endif
            current_room_id = GetFrameRoomID(fuseKeyframeId);
            std::cout<<"last room ID(gcFusion): "<<last_room_id
                <<" current_room_id: "<<current_room_id<<" new submap: "<<shouldAddNewSubmap<<std::endl;
            if(shouldAddNewSubmap == 1)
            {
                //here we separate the insertion of submap in gcFusion and gcSLAM.
                // because gcSLAM is always faster than gcFusion
                currentSubMapID = addNewSubmap();
                changeMainSubmap(currentSubMapID);
                std::cout<<"Add new submap for gcfusion, mainSubmapID: "<<currentSubMapID<<std::endl;

                if(current_room_id != -1)
                {
                    room_to_submap[current_room_id].push_back(referenceSubmapID);
                    unlabeled_submap.erase(referenceSubmapID);  
                    submap_to_room[referenceSubmapID] = current_room_id;  
                    std::cout<<"Add a new submap to room "<<current_room_id<<std::endl;                
                }
                else if(last_room_id != -1 && current_room_id == -1)
                {
                    //room registration
                    room_to_submap[last_room_id].push_back(referenceSubmapID);
                    unlabeled_submap.erase(referenceSubmapID);  
                    submap_to_room[referenceSubmapID] = last_room_id;  
                    std::cout<<"Add a new submap to room "<<last_room_id<<std::endl;   
                    //room_registration = last_room_id;
                }
            }
            if(last_room_id != -1 && current_room_id == -1)
            {
                std::cout<<GREEN<<"Prepare to register room "<<last_room_id<<RESET<<std::endl;
                room_registration = last_room_id;
            }

            last_room_id = current_room_id;
            //in_tsdfFusion.unlock();
    }
    int DetectAndSeparate(int frame_index = -1, int closure_id = 0)
    {
        resetBuilding();    
        resetDCEL();
        prepareDCEL(20.0);
        DetectPlaneAndBuildDcel(closure_id);              
        prepareBuilding();
        return SeparateRoom(frame_index);
    }
    size_t GetFullPCD(std::vector<unsigned char > &buffer);
    void quit()
    {
        quitLockFusion = true;
        update_globalMap_cv.notify_one();
    }
    inline void MapManagement()
    {
        //int No = 0;
        while(true)
        {
            //dead lock
            boost::unique_lock<boost::mutex> lock(update_globalMap_mutex);
            std::cout<<"Wait for notification...."<<std::endl;
            update_globalMap_cv.wait(lock);
            if(quitLockFusion)
            {
                std::cout<<"quit lock fusion! "<<std::endl;
                break;
            }
            
            //while(fusion_control.test_and_set()) {}
       
            finishAllRemnantFusion();

            //TOCK("MobileFusion::TSDFFusion");
            //GetFullMeshes();   
            //std::ostringstream oss;
            Stopwatch::getInstance().printAll();   
            //time_consuming[gcSLAM.globalFrameList.size() - 1] = oss.str();         
            //usleep(1000);
            in_tsdfFusion.unlock();
        }
        std::cout<<"TSDF Fusion thread quit."<<std::endl;
    }

    void setActiveSubmapNumber(int _active_submap_number)
    {
        active_submap_number = _active_submap_number;
    }
    void saveTimeConsuming(const std::string &filename)
    {
        std::ofstream ofs(filename);
        for(auto iter = time_consuming.begin(); iter != time_consuming.end(); ++iter)
        {
            ofs<<"Frame "<<iter->first<<":\n"<<iter->second;
        }
        ofs.close();
    }
     void ReIntegrateKeyframeOtherSubmap(int currentSubMapID,Frame &kf, PoseSE3d &pose,chisel::ChunkMap &commonChunks)
    {
    /* integrateflag = 0 means deintegrate */
        int totalPixelNum =  cameraModel.GetWidth() * cameraModel.GetHeight();
        float cx = cameraModel.GetCx();
        float cy = cameraModel.GetCy();
        float fx = cameraModel.GetFx();
        float fy = cameraModel.GetFy();
        int width = cameraModel.GetWidth();
        int height = cameraModel.GetHeight();
        chisel::Transform lastPose;
        //
        //PoseSE3d _pose = PoseSE3d();
        lastPose =pose.matrix().cast<float>();
        ChunkIDList localChunksIntersecting;
        std::vector<void  *> localChunksPtr;
        std::vector<bool> localNeedsUpdateFlag;
        std::vector<bool> localNewChunkFlag;

        float *depthImageData;
        if(colorImageData == nullptr)
        colorImageData = new unsigned char[totalPixelNum * 4];


        unsigned char *colorValid = (unsigned char *)kf.getColorValidFlag().data;
        depthImageData = (float *) kf.getRefinedDepth().data;

#if 1
        cv::Mat &rgb = kf.getRgb();
        for(int i = 0; i < height; i++)
        {
            for(int j = 0; j < width ; j++)
            {
                int pos = i * width + j;
            //std::cout << "start to PrepareIntersectChunks"<<std::endl;//colorValid[pos]<< std::endl; 
                colorImageData[pos*4 + 0] = rgb.at<unsigned char>(pos*3+0);
                colorImageData[pos*4 + 1] = rgb.at<unsigned char>(pos*3+1);
                colorImageData[pos*4 + 2] = rgb.at<unsigned char>(pos*3+2);
                colorImageData[pos*4 + 3] = 1;
  
                if(!colorValid[pos])
                {
                    colorImageData[pos*4 + 0] = 0;
                    colorImageData[pos*4 + 1] = 0;
                    colorImageData[pos*4 + 2] = 0;
                    colorImageData[pos*4 + 3] = 0;
                }
            }

        }
#endif

            //std::cout << "start to PrepareIntersectChunks" << std::endl;
            TICK("CHISEL::Reintegration::1::prepareIntersectChunks");

            chiselMap->PrepareIntersectChunks(projectionIntegrator,
                                              depthImageData,
                                              lastPose,
                                              cameraModel,
                                              localChunksIntersecting,
                                              localNeedsUpdateFlag,
                                              localNewChunkFlag,currentSubMapID);
    //        chiselMap->GetSearchRegion(searchArea,cameraModel,lastPose);
            //std::cout<<"prepareIntersectChunks"<<std::endl;
            TOCK("CHISEL::Reintegration::1::prepareIntersectChunks");


        //std::cout << "IntegrateDepthScanColor" << std::endl;
        TICK("CHISEL::Reintegration::2::IntegrateKeyDepthAndColor");

        chiselMap->IntegrateDepthScanColorGetSharedChunks(projectionIntegrator,
                                           lastPose,
                                           localChunksIntersecting,
                                           currentSubMapID,commonChunks);
        //std::cout<<"integrate keyframe depth and color"<<std::endl;
        TOCK("CHISEL::Reintegration::2::IntegrateKeyDepthAndColor");
        //std::cout << "IntegrateDepthScanColor done!" << std::endl;

#if 0
        TICK("CHISEL::Reintegration::3::IntegrateLocalDepth");

        // only integrate ten frames evenly distributed in this keyframe
        //std::cout<<"integrate local depth "<<kfDatabase.corresponding_frames.size()<<std::endl;
        int local_frame_num = kfDatabase.corresponding_frames.size();
        //if(    kfDatabase.corresponding_frames.size() > integrateLocalFrameNum) local_frame_num = 6;
        int integrated = 0;
        for(int i = 0; i < local_frame_num && integrated < integrateLocalFrameNum; i++)
        {
            Frame & local_frame = frame_list[kfDatabase.corresponding_frames[i]];
        //std::cout<<"integrate local depth "<<kfDatabase.corresponding_frames.size() <<" "<< local_frame.frame_index<<std::endl;            
            if(local_frame.submapID != currentSubMapID || local_frame.refined_depth.empty())
            {
                continue;
            }
            integrated += 1;
//            printf("integrating frame: %d %d\r\n",local_frame.frame_index, integrateFlag);
            if(integrateFlag == 1)
            {
                lastPose = local_frame.pose_sophus[0].matrix().cast<float>();
                local_frame.pose_sophus[1] = local_frame.pose_sophus[0];
            }
            else if(integrateFlag == 0)
            {

                lastPose = local_frame.pose_sophus[1].matrix().cast<float>();

            }
            depthImageData = (float *) local_frame.getRefinedDepth().data;

        //std::cout<<"integrate local depth "<<local_frame.getRefinedDepth().dataend - local_frame.getRefinedDepth().datastart<<std::endl;
            chiselMap->IntegrateDepthScanColor(projectionIntegrator,
                                               depthImageData,
                                               NULL,
                                               lastPose,
                                               cameraModel,
                                               localChunksIntersecting,
                                               localNeedsUpdateFlag,
                                               integrateFlag,nullptr,mainSubmapID,referenceSubmapID);

        }
        //std::cout<<"integrate local depth done!"<<std::endl;
        TOCK("CHISEL::Reintegration::3::IntegrateLocalDepth");

        TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks");
        chiselMap->FinalizeIntegrateChunks(localChunksIntersecting,localNeedsUpdateFlag,localNewChunkFlag,kf.validChunks,currentSubMapID);
        TOCK("CHISEL::Reintegration::4::FinalizeIntegrateChunks");
#endif
    }
    void ReIntegrateKeyframeInMultiSubmap(std::vector<Frame> &frame_list,int keyFrameIndex,
        std::vector<int> &correspondent_submaps)
    {
    /* integrateflag = 0 means deintegrate */
        Frame &kf = frame_list[keyFrameIndex];
        int currentSubMapID = mainSubmapID;
        
        if(kf.submapID != mainSubmapID)
        {
        std::cout<<"kf submapID: "<<kf.submapID<<" mainSubmapID: "<<mainSubmapID<<std::endl;
        std::cout<<"kf is not in the current submap!"<<std::endl;
        return;
        }

        chisel::Transform globalPose;
        chisel::Transform relative_transform;
        ChunkIDList localChunksIntersecting;
        std::vector<void  *> localChunksPtr;
        std::vector<bool> localNeedsUpdateFlag;

        globalPose = kf.pose_sophus[3].matrix().cast<float>();

        float *depthImageData;
        depthImageData = (float *) kf.getRefinedDepth().data;
        chiselMap->PrepareIntersectChunksGlobal(projectionIntegrator,
                                            depthImageData,
                                            globalPose,
                                            cameraModel,
                                            localChunksIntersecting,
                                            localNeedsUpdateFlag);
    //        chiselMap->GetSearchRegion(searchArea,cameraModel,lastPose);
            //std::cout<<"prepareIntersectChunks"<<std::endl;


        //std::cout << "IntegrateDepthScanColor" << std::endl;
        TICK("CHISEL::ReintegrationGlobal::2::IntegrateKeyDepthAndColor");
        std::cout<<"chunk intersecting: "<<localChunksIntersecting.size()<<std::endl;
        for(int i = 0; i != correspondent_submaps.size(); ++i)
        {
            int submap_id = correspondent_submaps[i];
            relative_transform = gcSLAM.submapPosesRelativeChanges[submap_id].matrix().cast<float>();
            std::cout<<"submap id: "<<submap_id<<std::endl;
            //integrate each submap's chunk map
            chiselMap->IntegrateDepthScanColorGlobal(projectionIntegrator,
                relative_transform, localChunksIntersecting, localNeedsUpdateFlag, submap_id);
        }
        TOCK("CHISEL::ReintegrationGlobal::2::IntegrateKeyDepthAndColor");

        TICK("CHISEL::ReintegrationGlobal::4::FinalizeIntegrateChunks");

        for(int i = 0; i != correspondent_submaps.size(); ++i)
        {
            int submap_id = correspondent_submaps[i];
            relative_transform = gcSLAM.submapPosesRelativeChanges[submap_id].matrix().cast<float>();
            chiselMap->FinalizeIntegrateChunksGlobal(projectionIntegrator,
                relative_transform, localChunksIntersecting, localNeedsUpdateFlag, submap_id);
        }
    
        TOCK("CHISEL::ReintegrationGlobal::4::FinalizeIntegrateChunks");
    }
    void ReIntegrateKeyframe(std::vector<Frame> &frame_list,const MultiViewGeometry::KeyFrameDatabase &kfDatabase, const int integrateFlag)
    {
    /* integrateflag = 0 means deintegrate */
        Frame &kf = frame_list[kfDatabase.keyFrameIndex];
        int currentSubMapID = mainSubmapID;
        
        if(kf.submapID != mainSubmapID)
        {
        std::cout<<"kf submapID: "<<kf.submapID<<" mainSubmapID: "<<mainSubmapID<<std::endl;
        std::cout<<"kf is not in the current submap!"<<std::endl;
        return;
        }
        int totalPixelNum =  cameraModel.GetWidth() * cameraModel.GetHeight();
        float cx = cameraModel.GetCx();
        float cy = cameraModel.GetCy();
        float fx = cameraModel.GetFx();
        float fy = cameraModel.GetFy();
        int width = cameraModel.GetWidth();
        int height = cameraModel.GetHeight();
        chisel::Transform lastPose;
        ChunkIDList localChunksIntersecting;
        std::vector<void  *> localChunksPtr;
        std::vector<bool> localNeedsUpdateFlag;
        std::vector<bool> localNewChunkFlag;

        if(integrateFlag == 1)
        {
            lastPose = kf.pose_sophus[0].matrix().cast<float>();
            kf.pose_sophus[1] = kf.pose_sophus[0];
        }
        else if(integrateFlag == 0)
        {

            lastPose = kf.pose_sophus[1].matrix().cast<float>();
            localChunksIntersecting = kf.validChunks;
            localChunksPtr = kf.validChunksPtr;
            for(int i = 0; i < localChunksIntersecting.size(); i++)
            {
                localNeedsUpdateFlag.push_back(true);
                localNewChunkFlag.push_back(false);
            }
        }
        float *depthImageData;
        if(colorImageData == nullptr)
        colorImageData = new unsigned char[totalPixelNum * 4];
        unsigned char *colorValid = (unsigned char *)kf.getColorValidFlag().data;
        depthImageData = (float *) kf.getRefinedDepth().data;

#if 1
        cv::Mat &rgb = kf.getRgb();
        for(int i = 0; i < height; i++)
        {
            for(int j = 0; j < width ; j++)
            {
                int pos = i * width + j;
            //std::cout << "start to PrepareIntersectChunks"<<std::endl;//colorValid[pos]<< std::endl; 
                colorImageData[pos*4 + 0] = rgb.at<unsigned char>(pos*3+0);
                colorImageData[pos*4 + 1] = rgb.at<unsigned char>(pos*3+1);
                colorImageData[pos*4 + 2] = rgb.at<unsigned char>(pos*3+2);
                colorImageData[pos*4 + 3] = 1;
  
                if(!colorValid[pos])
                {
                    colorImageData[pos*4 + 0] = 0;
                    colorImageData[pos*4 + 1] = 0;
                    colorImageData[pos*4 + 2] = 0;
                    colorImageData[pos*4 + 3] = 0;
                }
            }

        }
#endif


        if(integrateFlag == 1)
        {
            std::cout << "start to PrepareIntersectChunks" << std::endl;
            TICK("CHISEL::Reintegration::1::prepareIntersectChunks");

            chiselMap->PrepareIntersectChunks(projectionIntegrator,
                                              depthImageData,
                                              lastPose,
                                              cameraModel,
                                              localChunksIntersecting,
                                              localNeedsUpdateFlag,
                                              localNewChunkFlag,currentSubMapID);
    //        chiselMap->GetSearchRegion(searchArea,cameraModel,lastPose);
            //std::cout<<"prepareIntersectChunks"<<std::endl;
            TOCK("CHISEL::Reintegration::1::prepareIntersectChunks");
        }

        //std::cout << "IntegrateDepthScanColor" << std::endl;
        TICK("CHISEL::Reintegration::2::IntegrateKeyDepthAndColor");

        chiselMap->IntegrateDepthScanColor(projectionIntegrator,
                                           depthImageData,
                                           colorImageData,
                                           lastPose,
                                           cameraModel,
                                           localChunksIntersecting,
                                           localNeedsUpdateFlag,
                                           integrateFlag,nullptr,mainSubmapID,referenceSubmapID);
        //std::cout<<"integrate keyframe depth and color"<<std::endl;
        TOCK("CHISEL::Reintegration::2::IntegrateKeyDepthAndColor");
        std::cout << "IntegrateDepthScanColor done!" << std::endl;

#if INTEGRATE_ALL
        TICK("CHISEL::Reintegration::3::IntegrateLocalDepth");

        // only integrate ten frames evenly distributed in this keyframe
        //std::cout<<"integrate local depth "<<kfDatabase.corresponding_frames.size()<<std::endl;
        int local_frame_num = kfDatabase.corresponding_frames.size();
        //if(    kfDatabase.corresponding_frames.size() > integrateLocalFrameNum) local_frame_num = 6;
        int integrated = 0;
        for(int i = 0; i < local_frame_num && integrated < integrateLocalFrameNum; i++)
        {
            Frame & local_frame = frame_list[kfDatabase.corresponding_frames[i]];
        //std::cout<<"integrate local depth "<<kfDatabase.corresponding_frames.size() <<" "<< local_frame.frame_index<<std::endl;            
            if(local_frame.submapID != currentSubMapID || local_frame.refined_depth.empty())
            {
                continue;
            }
            integrated += 1;
//            printf("integrating frame: %d %d\r\n",local_frame.frame_index, integrateFlag);
            if(integrateFlag == 1)
            {
                lastPose = local_frame.pose_sophus[0].matrix().cast<float>();
                local_frame.pose_sophus[1] = local_frame.pose_sophus[0];
            }
            else if(integrateFlag == 0)
            {

                lastPose = local_frame.pose_sophus[1].matrix().cast<float>();

            }
            depthImageData = (float *) local_frame.getRefinedDepth().data;

            //std::cout<<"integrate local depth "<<local_frame.getRefinedDepth().dataend - local_frame.getRefinedDepth().datastart<<std::endl;
            chiselMap->IntegrateDepthScanColor(projectionIntegrator,
                                               depthImageData,
                                               NULL,
                                               lastPose,
                                               cameraModel,
                                               localChunksIntersecting,
                                               localNeedsUpdateFlag,
                                               integrateFlag,nullptr,mainSubmapID,referenceSubmapID);

        }
        //std::cout<<"integrate local depth done!"<<std::endl;
        TOCK("CHISEL::Reintegration::3::IntegrateLocalDepth");
#endif



        TICK("CHISEL::Reintegration::4::FinalizeIntegrateChunks");
        if(integrateFlag == 1)
        {
            chiselMap->FinalizeIntegrateChunks(localChunksIntersecting,localNeedsUpdateFlag,localNewChunkFlag,kf.validChunks,currentSubMapID);
        }
        else if(integrateFlag == 0)
        {

            std::vector<void *> localChunksPtrValid;
            ChunkIDList localValidChunks;
            chiselMap->FinalizeIntegrateChunks(localChunksIntersecting,localNeedsUpdateFlag,localNewChunkFlag,localValidChunks,currentSubMapID);
            kf.validChunks.clear();
        }
        
        std::cout<<"finalize integrate chunks"<<std::endl;
        TOCK("CHISEL::Reintegration::4::FinalizeIntegrateChunks");
        //delete colorImageData;
    }

    MobileFusion(const MultiViewGeometry::CameraPara &_camera,int _cameraID = 0, bool _use_gui = true):camera(_camera)
    {
        int argc = 1;
        weight_count = 0;
        char ProjectName[256] = "MobileFusion";
        char *argv = ProjectName;

        cameraID = _cameraID;

        //validFrameNum = 0;
        //fuseKeyframeId = 0;
        /*
        tsdf_visualization_buffer = new float[GLOBAL_MODLE_VERTEX_NUM*12];
        memset(tsdf_visualization_buffer,0,GLOBAL_MODLE_VERTEX_NUM*12 * sizeof(float));
        */


        Estimator.Initialize(MultiViewGeometry::g_para.room_lcd_ransac_threshold, 100);
        use_gui = _use_gui;
        if(use_gui)
        {
            real_buffer.resize(GLOBAL_MODLE_VERTEX_NUM*byte_of_each_point, 0);
            tsdf_visualization_buffer = &real_buffer[0];
            glutInit(&argc, &argv);
            glutInitDisplayMode(GLUT_SINGLE);
            GLenum err=glewInit();
            if(err!=GLEW_OK) {
            // Problem: glewInit failed, something is seriously wrong.
            //std::cout << "glewInit failed: " << glewGetErrorString(err) << std::endl;
            exit(0);
            }
            glGenBuffers(1, &vbo);
            glBindBuffer(GL_ARRAY_BUFFER,vbo);
            glBufferData(GL_ARRAY_BUFFER, GLOBAL_MODLE_VERTEX_NUM * byte_of_each_point, &tsdf_visualization_buffer[0], GL_DYNAMIC_DRAW);
            drawVoxelHashingStyle = loadProgramFromFile("draw_feedback_VoxelHashing.vert","draw_feedback_VoxelHashing.frag");
            //unet = std::make_shared<LearningBWDenseUNet>(std::string("./src/collaborative_fusion/src/scn_cpp/weight"));
        }
        unet = std::make_shared<LearningBWDenseUNet>(std::string("./src/collaborative_fusion/src/scn_cpp/weight"));
        //glBindBuffer(GL_ARRAY_BUFFER, 0);

        //glGenBuffers(1, &vbo_data_transfer);
        //glGenBuffers(1, &unstable_vbo);
        //glGenTransformFeedbacks(1, &feedback_vbo);
        //glGenBuffers(1, &vbo_point_cloud);
        gcSLAM.setCameraModel(_camera);
        gcSLAM.setCameraID(cameraID);

        int maximum_frame_num = MultiViewGeometry::g_para.maximum_frame_num;


        
        unlabeled_submap.insert(0);//initialize the unlabeled submap.
        submap_to_room.push_back(-1);
        patch_image = cv::Mat::zeros(cameraModel.GetHeight(), cameraModel.GetWidth(), CV_8UC3);
        cell_image = cv::Mat::zeros(cameraModel.GetHeight(), cameraModel.GetWidth(), CV_8UC3);
    }

    void IntegrateFrame( Frame & frame_ref);
    int GetFullMeshes();
    void GetFullMeshes(std::vector<unsigned char> &buffer);
    int tsdfFusion(std::vector<Frame> &frame_list,
                              int CorrKeyframeIndex,
                              const std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist,const std::set<int> &frameIndex,
                              int integrateKeyframeID,int currentSubMapID);
    void initGCSLAM(const MultiViewGeometry::GlobalParameters para)
    {
        //gcSLAM.init(maximum_frame_num, camera);

        gcSLAM.SetMinimumDisparity(para.minimum_disparity);
        gcSLAM.SetSalientScoreThreshold(para.salient_score_threshold);
        gcSLAM.SetMaxCandidateNum(para.maximum_keyframe_match_num);
        gcSLAM.globalFrameList.reserve(para.maximum_frame_num);
        gcSLAM.KeyframeDataList.reserve(para.maximum_frame_num/2);
        
    }
    void rebind_buffer()
    {
        tsdf_visualization_buffer = &real_buffer[0];
    }
    void initChiselMap(float ipnutVoxelResolution,float farPlaneDist = 3,double compact_ratio = 0.5 )
    {
        std::cout << "init chisel map..."<<std::endl;
        float fx = camera.c_fx;
        float fy = camera.c_fy;
        float cx = camera.c_cx;
        float cy = camera.c_cy;
        int width = camera.width;
        int height = camera.height;

#if 1
        float truncationDistConst=  0.001504;
        float truncationDistLinear=  0.00152;
        float truncationDistQuad=  0.0019;
        float truncationDistScale=  6.0;
#else
        float truncationDistConst=  0.01;
        float truncationDistLinear=  0.01;
        float truncationDistQuad=  0.01;
        float truncationDistScale=  1.0;
#endif
        float weight=  1;
        bool useCarving=  true;
        float carvingDist=  0.05;
        float nearPlaneDist=  0.01;

        //std::cout << "far plane dist: " << farPlaneDist << std::endl;
        chunkSizeX=  8;
        chunkSizeY=  8;
        chunkSizeZ=  8;
        voxelResolution=  ipnutVoxelResolution;
        useColor=  true;
        chisel::Vec4 truncation(truncationDistQuad, truncationDistLinear, truncationDistConst, truncationDistScale);

         chiselMap = chisel::ChiselPtr(new chisel::Chisel(Eigen::Vector3i(chunkSizeX, chunkSizeY, chunkSizeZ), voxelResolution, useColor,cameraModel));
         chiselMap->SetCompactRatio(compact_ratio);
         std::cout<<"chunkManagers.capacity: "<<chiselMap->chunkManagers.capacity()<<std::endl;
         //chunkManagers

         projectionIntegrator.SetCentroids(chiselMap->GetChunkManager(0).GetCentroids());
         projectionIntegrator.SetTruncator(chisel::TruncatorPtr(new chisel::QuadraticTruncator(truncation(0), truncation(1), truncation(2), truncation(3))));
         projectionIntegrator.SetWeighter(chisel::WeighterPtr(new chisel::ConstantWeighter(weight)));
         projectionIntegrator.SetCarvingDist(carvingDist);
         projectionIntegrator.SetCarvingEnabled(useCarving);
         //chiselMap->setThreshold(MultiViewGeometry::g_para.velocity_threshold, MultiViewGeometry::g_para.total_threshold);
        
         cameraModel.SetIntrinsics(fx,fy,cx,cy);
         cameraModel.SetNearPlane(nearPlaneDist);
         cameraModel.SetFarPlane(farPlaneDist);
         cameraModel.SetWidth(width);
         cameraModel.SetHeight(height);
    }


    ~MobileFusion()
    {
        //delete tsdf_visualization_buffer;
        //delete currentObservationBuffer;
        
        if(use_gui)
        glDeleteBuffers(1, &vbo);
        //glDeleteBuffers(1, &vbo_data_transfer);
        //glDeleteBuffers(1, &unstable_vbo);
        //glDeleteBuffers(1, &feedback_vbo);
        //glDeleteBuffers(1, &vbo_point_cloud);
    }

    void DrawCube(float *vertex_list, GLint *index_list)
    {
        int i,j;

        glBegin(GL_LINES);
        for(i=0; i<12; ++i) // 12 条线段

        {
            for(j=0; j<2; ++j) // 每条线段 2个顶点

            {
//                Eigen::Vector4f vertex(vertex_list[index_list[i*2 + j] * 3],
//                        vertex_list[index_list[i*2 + j] * 3 + 1],
//                        vertex_list[index_list[i*2 + j] * 3 + 2],
//                        1);
//                vertex = t * vertex;
                glVertex3fv(&vertex_list[index_list[i*2 + j] * 3]);
            }
        }
        glEnd();
    }
    void GetColor(double v, double vmin, double vmax, int &r, int &g, int &b)
    {
        double dv;

        if (v < vmin)
           v = vmin;
        if (v > vmax)
           v = vmax;
        dv = vmax - vmin;

        r = 0;
        g = 0;
        b = 0;
        if (v < (vmin + 0.25 * dv)) {
           r = 0;
           g = (4 * (v - vmin) / dv) * 255;
        } else if (v < (vmin + 0.5 * dv)) {
           r = 0;
           b = (1 + 4 * (vmin + 0.25 * dv - v) / dv) * 255;
        } else if (v < (vmin + 0.75 * dv)) {
           r = (4 * (v - vmin - 0.5 * dv) / dv) * 255;
           b = 0;
        } else {
           g = (1 + 4 * (vmin + 0.75 * dv - v) / dv) * 255;
           b = 0;
        }

    }


inline int MobileShow(pangolin::OpenGlMatrix mvp,
                   const float threshold,
                   //const bool drawUnstable,
                   const bool drawNormals,
                   const bool drawColors,
                   const bool drawSemantic,
                   const bool drawInstance)
    {
        std::shared_ptr<Shader> program = drawVoxelHashingStyle;
        program->Bind();                    // set this program as current program
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        program->setUniform(Uniform("MVP", mvp));
        program->setUniform(Uniform("pose", pose));
        program->setUniform(Uniform("threshold", threshold));
        int colortype = 0;
        if(drawInstance)
        colortype = 4;
        else if(drawSemantic)
        colortype = 3;
        else if(drawNormals)
        colortype = 2;
        else if(drawColors)
        colortype = 1;
        else colortype = 0;//draw phong shading
        program->setUniform(Uniform("colorType", colortype));
        


        if(program == drawPhongLighting)
        {
            program->setUniform(Uniform("view_matrix", pose));
            program->setUniform(Uniform("proj_matrix", mvp));
            Eigen::Vector3f Lightla(0.2f, 0.2f, 0.2f);
            Eigen::Vector3f Lightld(1.0f, 1.0f, 1.0f);
            Eigen::Vector3f Lightls(1.0f, 1.0f, 1.0f);
            Eigen::Vector3f Lightldir(0.0, 0.0, 1.0f);
            Eigen::Vector3f fma(0.26f, 0.26f, 0.26f);
            Eigen::Vector3f fmd(0.35f, 0.35f, 0.35f);
            Eigen::Vector3f fms(0.30f, 0.30f, 0.30f);
            float fss = 16.0f;
            Eigen::Vector3f bma(0.85f, 0.85f, 0.85f);
            Eigen::Vector3f bmd(0.85f, 0.85f, 0.85f);
            Eigen::Vector3f bms(0.60f, 0.60f, 0.60f);
            float bss = 16.0f;

            Eigen::Matrix4f user_view_matrix = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f user_light_matrix = Eigen::Matrix4f::Identity();
            Eigen::Vector4f user_rot_center = Eigen::Vector4f(0, 0, 0, 1);
            program->setUniform(Uniform("Lightla", Lightla));
            program->setUniform(Uniform("Lightld", Lightld));
            program->setUniform(Uniform("Lightls", Lightls));
            program->setUniform(Uniform("Lightldir", Lightldir));
            program->setUniform(Uniform("fma", fma));
            program->setUniform(Uniform("fmd", fmd));
            program->setUniform(Uniform("fms", fms));
            program->setUniform(Uniform("bma", bma));
            program->setUniform(Uniform("bmd", bmd));
            program->setUniform(Uniform("bms", bms));
            program->setUniform(Uniform("bss", bss));
            program->setUniform(Uniform("fss", fss));
            program->setUniform(Uniform("user_view_matrix", user_view_matrix));
            program->setUniform(Uniform("user_light_matrix", user_light_matrix));
            program->setUniform(Uniform("user_rot_center", user_rot_center));
        }

        if(program == drawVoxelHashingStyle)
        {
            float s_materialShininess = 16.0f;
            Eigen::Vector4f s_materialAmbient   = Eigen::Vector4f(0.75f, 0.65f, 0.5f, 1.0f);
            Eigen::Vector4f s_materialDiffuse   = Eigen::Vector4f(1.0f, 0.9f, 0.7f, 1.0f);
            Eigen::Vector4f s_materialSpecular  = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
            Eigen::Vector4f s_lightAmbient 	    = Eigen::Vector4f(0.4f, 0.4f, 0.4f, 1.0f);
            Eigen::Vector4f s_lightDiffuse 		= Eigen::Vector4f(0.6f, 0.52944f, 0.4566f, 0.6f);
            Eigen::Vector4f s_lightSpecular 	= Eigen::Vector4f(0.3f, 0.3f, 0.3f, 1.0f);
            Eigen::Vector3f lightDir 	= Eigen::Vector3f(0.0f, -1.0f, 2.0f);

            program->setUniform(Uniform("materialShininess", s_materialShininess));
            program->setUniform(Uniform("materialAmbient", s_materialAmbient));
            program->setUniform(Uniform("materialDiffuse", s_materialDiffuse));
            program->setUniform(Uniform("materialSpecular", s_materialSpecular));
            program->setUniform(Uniform("lightAmbient", s_lightAmbient));
            program->setUniform(Uniform("lightDiffuse", s_lightDiffuse));
            program->setUniform(Uniform("lightSpecular", s_lightSpecular));
            program->setUniform(Uniform("lightDir", lightDir));
        }
        //This is for the point shader
        //setup a uniform:

    //    GLuint loc = glGetUniformLocation(program->programId(), "camera_array");
    //    glUniformMatrix4fv(loc, 26, false, camera_array_matrices, 0);




        if(global_vertex_data_updated)
        {
            if(global_show_lock.try_lock())
            {
                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                glBufferSubData(GL_ARRAY_BUFFER,0,(amount_tsdf_vertice_num) * byte_of_each_point,&tsdf_visualization_buffer[0]);

                glBindBuffer(GL_ARRAY_BUFFER, 0);         
                global_vertex_data_updated = 0;
                global_show_lock.unlock();
            }
            // begin draw signed distance fields
        }


        glBindBuffer(GL_ARRAY_BUFFER, vbo);


        /*
        glVertexAttribPointer(index,size,type,normalized,stride,pointer)
        
        index: index
        size: every vpo's size
        type: the data type
        normalize: if data is normailized,true
        stride: offset 
        pointer: the first vertex offset
        
         */ 

        //glVertexAttribPointer(0, 3, GL_HALF_FLOAT, GL_FALSE, byte_of_each_point, 0);
        // position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, byte_of_each_point, 0);//vertex
        glEnableVertexAttribArray(0);

        //glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 12 * sizeof(float), reinterpret_cast<GLvoid*>(sizeof(double) * 3));
        // color(color, semantic, instance)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, byte_of_each_point, reinterpret_cast<GLvoid*>(12));
        glEnableVertexAttribArray(1);
        /*
        // semantic        
        glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE,byte_of_each_point, reinterpret_cast<GLvoid*>(16));
        glEnableVertexAttribArray(2);
        // instance
        glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE,byte_of_each_point, reinterpret_cast<GLvoid*>(20));
        glEnableVertexAttribArray(3);
        */
        // normal
        glVertexAttribPointer(2, 3, GL_HALF_FLOAT, GL_TRUE, byte_of_each_point,reinterpret_cast< GLvoid*>(24));
        glEnableVertexAttribArray(2);
        /*
        glVertexAttribPointer(5, 1, GL_INT, GL_FALSE, byte_of_each_point,reinterpret_cast<GLvoid*>(byte_of_each_point));
        glEnableVertexAttribArray(5);   
        */
        glDrawArrays(GL_TRIANGLES,0,amount_tsdf_vertice_num );
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glDisableVertexAttribArray(2);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        program->Unbind();


        float vertex_list[24] =
        {
            -0.5f, -0.5f, -0.5f,
            0.5f, -0.5f, -0.5f,
            -0.5f, 0.5f, -0.5f,
            0.5f, 0.5f, -0.5f,
            -0.5f, -0.5f, 0.5f,
            0.5f, -0.5f, 0.5f,
            -0.5f, 0.5f, 0.5f,
            0.5f, 0.5f, 0.5f,
        };
         GLint index_list[24] =
        {
            0, 1,
            2, 3,
            4, 5,
            6, 7,
            0, 2,
            1, 3,
            4, 6,
            5, 7,
            0, 4,
            1, 5,
            7, 3,
            2, 6
        };
#if 0
        DrawCube(searchArea,index_list);
        std::vector<float> corners = chiselMap->candidateCubes;
        int cornersNum = corners.size() / 24;
        float *corner_pointer = corners.data();
        for(int i = 0; i < cornersNum; i++)
        {
            DrawCube(&corner_pointer[i*24],index_list);
        }
#endif

        return 0;
    }

    //void generateMesh();
    //only read frame
    void saveSubmap(int submapID, const std::string &filepath="./")
    {

        if( submapID >= chiselMap->chunkManagers.size() || submapID < 0 )
        {
            //std::cout<<"kill submap: Wrong ID!"<<std::endl;
            return;
        }
        std::string filename = filepath+"Map/"+std::to_string(cameraID)+"/";
        if(g_para.final_integration == 1)
        chiselMap->saveSubmap(submapID,filename, true);
        else chiselMap->saveSubmap(submapID,filename, false);
    }
    void saveFrames(int submapID, const std::string &filepath="./")
    {
        if( submapID >= chiselMap->chunkManagers.size() || submapID < 0 )
        {
            //std::cout<<"kill submap: Wrong ID!"<<std::endl;
            return;
        }
        gcSLAM.saveFrames(submapID,filepath);
        std::cout<<"save frames done!"<<std::endl;
    }

    /* void killSubmap(int submapID,const std::string &filepath="./")
    {
     simplifySubmap(submapID);
     saveSubmap(submapID,filepath);   
    }*/
    void integrateTwoChunkMap(int submapID1, int submapID2,chisel::ChunkMap &chunks1, chisel::ChunkMap &chunks2)
    {
        chisel::Transform relativePose;
        if(submapID1 ==  submapID2)
            return;
        PoseSE3d pose = gcSLAM.submapPosesRelativeChanges[submapID2].inverse()  * gcSLAM.submapPosesRelativeChanges[submapID1]  ;
        relativePose = pose.matrix().cast<float>();
        chiselMap->IntegrateSubmaps(chunks1,relativePose,chunks2);
    }
    void simplifySubmap(int submapID)
    {
        
        if( submapID >= chiselMap->chunkManagers.size() || submapID < 0 )
        {
            //std::cout<<"kill submap: Wrong ID!"<<std::endl;
        }
        chiselMap->simplifySubmapSemantic(submapID);
        //std::cout<<"simplify mesh done!"<<std::endl;
    }


    float GetVoxelResolution(){return voxelResolution;}
    
    tool::Timer ftimer;
    float searchArea[24];
    //std::vector<std::set<int>> activeSubmapIDs;
    unsigned char * colorImageData=nullptr;
private:


    int chunkSizeX;
    int chunkSizeY;
    int chunkSizeZ;
    float voxelResolution;
    bool useColor;
};


#endif // MOBILEFUSION_H
