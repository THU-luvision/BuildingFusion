

#include<cmath>
#include "MobileFusion.h"
#include "MapMaintain.hpp"
#include "../RoomDetection/PatchDetection.h"
#include "../RoomDetection/DrawImage.h"

#define DEBUG_MODE 0
#define VERTEX_HASHING 1


// for range 1 only.
static constexpr double PI = 3.14159265359;

static int max_instance = 0;

#if FOR_REBUTTAL
std::map<int, float> t_tsdf_fusion;
std::map<int, float> t_segmentation;
std::map<int, float> t_mesh;
std::map<int, float> t_room_detection;
std::map<int, float> m_gcfusion;
std::ofstream ofs_memory("./memory_keyframe.txt");
#endif

RGBAPixel ComputeColorForFlowPixel(float fx, float fy) {
    int NCOLORS = 55;
    size_t RY = 15;
    size_t YG = 6;
    size_t GC = 4;
    size_t CB = 11;
    size_t BM = 13;
    size_t MR = 6;
    RGBAPixelList ColorWheel(55);
    size_t k = 0;
    for (size_t i = 0; i < RY; ++i)
      ColorWheel[k++] = RGBAPixel(255,	255*i/RY,	0);

    for (size_t i = 0; i < YG; ++i)
      ColorWheel[k++] = RGBAPixel(255-255*i/YG, 255, 0);

    for (size_t i = 0; i < GC; ++i)
      ColorWheel[k++] = RGBAPixel(0,	255, 255*i/GC);

    for (size_t i = 0; i < CB; ++i)
      ColorWheel[k++] = RGBAPixel(0, 255-255*i/CB, 255);

    for (size_t i = 0; i < BM; ++i)
      ColorWheel[k++] = RGBAPixel(255*i/BM, 0,	255);

    for (size_t i = 0; i < MR; ++i)
      ColorWheel[k++] = RGBAPixel(255,	0, 255-255*i/MR);

  float rad = sqrt(fx * fx + fy * fy);
  float a = atan2(-fy, -fx) / PI;
  float fk = (a + 1.0) / 2.0 * (NCOLORS-1);
  size_t k0 = static_cast<size_t>(fk);
  size_t k1 = (k0 + 1) % NCOLORS;
  float f = fk - k0;
  //f = 0; // uncomment to see original color wheel

  RGBAPixel result;

  for (size_t channel = 0; channel < 3; ++channel) {
    float col0 = ColorWheel[k0][channel] / 255.0;
    float col1 = ColorWheel[k1][channel] / 255.0;
    float col = (1 - f) * col0 + f * col1;
    if (rad <= 1)
        col = 1 - rad * (1 - col); // increase saturation with radius
    else
        col *= .75; // out of range
    result[channel] = (int)(255.0 * col);
  }

  return result;
}

// for range 1 only.

torch::Tensor weighted_mean(torch::Tensor embedding, torch::Tensor weight)
{
    torch::Tensor expanded_weight = weight.view({-1,1}).expand({-1,embedding.sizes()[1]});
    return (embedding * expanded_weight).sum(0)  / expanded_weight.sum(0);

}

torch::Tensor gaussian(torch::Tensor d, torch::Tensor bw)
{
    return torch::exp(-0.5*(d*d/bw/bw)) / (bw * sqrt(2*3.14));
}

torch::Tensor cross_modal_gaussian(torch::Tensor t1,torch::Tensor t2,torch::Tensor b1, torch::Tensor b2)
{
    torch::Tensor d1 = torch::norm(t1,2,1) * b1;


    torch::Tensor d2 = torch::norm(t2,2,1) * b2;

//    std::cout << "d1: " << d1[0] << std::endl;
//    std::cout << "d2: " << d2[0] << std::endl;
    torch::Tensor p = torch::exp(-d1*d1 -d2*d2);
//    std::cout << "p: " << p[0] << std::endl;
    return p;
}

torch::Tensor InstanceSegmentation(Tensors outlist, torch::Tensor pose_device, RGBAPixelList &instance_centers)
{
    int embedding_length = 32;
    pose_device = pose_device.cuda();
    instance_centers.clear();
    torch::Tensor pred_semantic = outlist[0].argmax(1);
    torch::Tensor pred_embedding_device = outlist[1];
    torch::Tensor exsiting_offsets_device = outlist[2];
    torch::Tensor pred_displacements= outlist[3];
    torch::Tensor pred_bw_device = outlist[4];
    torch::Tensor occupancy = outlist[5];
    torch::Tensor pose_embedding_device = pose_device - pred_displacements;
//    std::cout << "1" << std::endl;
    pred_embedding_device = at::cat({pred_embedding_device, pose_embedding_device}, 1);
//    std::cout << pred_embedding_device.sizes() << std::endl;
    int total_points_num = pose_device.sizes()[0];
    torch::Tensor pred_instance = torch::zeros({total_points_num,1},torch::kInt64).cuda();
    torch::Tensor background_mask = (pred_semantic > 1).view({-1,1});
    torch::Tensor instance_flag = (pred_semantic < 2).view({-1,1});
    exsiting_offsets_device.masked_fill_(instance_flag.view({-1,1}),0);

    pred_instance.masked_fill_((pred_semantic == 0).view({-1,1}),0);
    pred_instance.masked_fill_((pred_semantic == 1).view({-1,1}),1);

    float bw = 1.2;
    int instance_count = 1;
    int expected_instance_num = 50;
    int minimum_instance_size = 50;
    instance_centers.push_back(Eigen::Vector3i(0,0,0));
    instance_centers.push_back(Eigen::Vector3i(0,0,0));

    auto embedding_shape = pred_embedding_device.sizes();
    float max_offsets = 1;
    int max_iteration_times = 30;
    int iteration_times = 0;
    while( (instance_count <  expected_instance_num) && (background_mask.sum(0).item<int>() > 100) && max_offsets > 0.3 && iteration_times < max_iteration_times)
    {
        iteration_times++;
        torch::Tensor keypoint = at::argmax(exsiting_offsets_device);
        max_offsets = exsiting_offsets_device[keypoint][0].item<float>();
        torch::Tensor pre_centroid = pred_embedding_device.narrow(0,keypoint.item<int>(),1);
        torch::Tensor b = pred_bw_device.narrow(0,keypoint.item<int>(),1) * bw;
        torch::Tensor embedding_distance = (pred_embedding_device - pre_centroid);
        torch::Tensor prob = cross_modal_gaussian(embedding_distance.narrow(1,0,embedding_length),embedding_distance.narrow(1,embedding_length,3),b[0][0],b[0][1]);
        prob = prob.view({-1,1});

        /*
        std::cout << "pred_semantic: "  << std::endl << pred_semantic[keypoint] << std::endl;
        std::cout << "exsiting_offsets_device: "  << std::endl << exsiting_offsets_device[keypoint][0] << std::endl;
        std::cout << "instance_flag: "  << std::endl << instance_flag[keypoint][0] << std::endl;
        std::cout << "background_mask: "  << std::endl << background_mask[keypoint][0] << std::endl;
        */
        instance_flag = (prob > 0.5) * background_mask;

        //std::cout << "prob: "  << std::endl << prob[keypoint][0] << std::endl;
        torch::Tensor local_embedding = torch::masked_select(pred_embedding_device,instance_flag);
        local_embedding = torch::reshape(local_embedding,{instance_flag.sum(0).item<int>(),35});

        embedding_distance = (local_embedding - pre_centroid);
        prob = cross_modal_gaussian(embedding_distance.narrow(1,0,embedding_length),embedding_distance.narrow(1,embedding_length,3),b[0][0],b[0][1]);
        /*
        std::cout << "prob: "  << std::endl << prob << std::endl;
        std::cout << "instance_flag: "  << std::endl << instance_flag[keypoint][0] << std::endl;
        std::cout << "background_mask: "  << std::endl << background_mask[keypoint][0] << std::endl;*/
        torch::Tensor centroid = at::mean(local_embedding,0);
        pre_centroid = centroid;
        for(int k = 0; k < 5; k++)
        {
            embedding_distance = (pred_embedding_device - pre_centroid);
            b = torch::reshape(torch::masked_select(pred_bw_device,instance_flag),{-1,2});
//            std::cout << "instance_flag: " << instance_flag.sum(0) << std::endl;
            b = at::mean(b,0);
//            std::cout << "b: " << b[0] << "  " << b[1] << std::endl;
            prob = cross_modal_gaussian(embedding_distance.narrow(1,0,embedding_length),embedding_distance.narrow(1,embedding_length,3), b[0], b[1]);
            prob = prob.view({-1,1});
            instance_flag = (prob > 0.5) * background_mask;

            local_embedding = torch::masked_select(pred_embedding_device,instance_flag.expand(embedding_shape));

            local_embedding = torch::reshape(local_embedding,{instance_flag.sum(0).item<int>(),35});
            centroid = at::mean(local_embedding,0);
            torch::Tensor mean_shift_vector = pre_centroid - centroid;
//            std::cout << "probability: " << prob[keypoint][0] << std::endl;
            pre_centroid = centroid;

        }

//        std::cout << instance_flag[keypoint.item<int>()][0] << std::endl;
        background_mask.masked_fill_(instance_flag.view({-1,1}),0);
        exsiting_offsets_device.masked_fill_(instance_flag.view({-1,1}),0);
        if(instance_flag.sum(0).item<int>() > minimum_instance_size )
        {
            instance_count += 1;
            pred_instance.masked_fill_(instance_flag.view({-1,1}),instance_count);
            torch::Tensor instance_pose_device = torch::masked_select(pose_device,instance_flag.expand(pose_device.sizes()));
            instance_pose_device = torch::reshape(instance_pose_device,{instance_flag.sum(0).item<int>(),3});
            torch::Tensor instance_mean_pose = at::mean(instance_pose_device,0);
            float x = ((int)(instance_mean_pose[0].item<float>() * 100) % 200) / 200.0;
            float y = ((int)(instance_mean_pose[1].item<float>() * 100) % 200) / 200.0;
            RGBAPixel instance_color = ComputeColorForFlowPixel(x,y);
            instance_centers.push_back(instance_color);
//            std::cout << instance_count << std::endl << instance_mean_pose << std::endl;
        }
        else
        {
            pred_instance.masked_fill_(instance_flag.view({-1,1}),-1);
        }
        /*
        std::cout << "instance sum: " << instance_flag.sum(0).item<int>() << std::endl;
        std::cout << "background_mask sum: " << background_mask.sum(0).item<int>()  << std::endl;
        std::cout << "instance num: " << instance_count << std::endl;
        std::cout << "keypoint: " << keypoint.item<int>()  << std::endl;
        std::cout << std::endl << std::endl << std::endl << std::endl;
        */
    }
    if(instance_centers.size() > max_instance)
    max_instance = instance_centers.size();
    else instance_centers.resize(max_instance);
    return pred_instance;
}



void GetMapDynamics(const std::vector<Frame> &frame_list,
                    std::vector<int> &keyframesToUpdate,const std::set<int> &frameIndex,
                    int corrKeyFrameIndex,int currentSubmapID)
{
    std::cout<<"get map dynamics..."<<std::endl;
    keyframesToUpdate.clear();
    std::vector<int> keyframeIDList;
    std::vector<float> keyframeCostList;
    keyframeIDList.clear();
    keyframeCostList.clear();
    for(auto i = frameIndex.begin(); i != frameIndex.end() ; ++i)
    {
        int index = *i;
        if(index < corrKeyFrameIndex)
        {
        if(frame_list[index].is_keyframe && frame_list[index].tracking_success )
        {
            /* previous current pose */
            Eigen::Matrix4f prePose = frame_list[index].pose_sophus[1].matrix().cast<float>();
            Eigen::Matrix4f curPose = frame_list[index].pose_sophus[0].matrix().cast<float>();

            float cost = GetPoseDifference(prePose, curPose);
            keyframeIDList.push_back(index);
            keyframeCostList.push_back(cost);
//            std::cout << "frames: " << i << " " << cost << std::endl;
        }
        }
        else  break;
    }


    // only consider dynamic map when keyframe number is larger than movingAveregeLength
    // Deintegrate 10 keyframes at each time slot
    int movingAverageLength = 5;

    SelectLargestNValues(movingAverageLength,
                      keyframeIDList,
                      keyframeCostList,
                      keyframesToUpdate);
#if 1
#if MobileCPU
    SelectLargestNValues(movingAverageLength,
                      keyframeIDList,
                      keyframeCostList,
                      keyframesToUpdate);
    SelectLargestNValues(movingAverageLength,
                      keyframeIDList,
                      keyframeCostList,
                      keyframesToUpdate);
#endif
    movingAverageLength = 2;
    SelectLargestNValues(movingAverageLength,
                      keyframeIDList,
                      keyframeCostList,
                      keyframesToUpdate);
    SelectLargestNValues(movingAverageLength,
                      keyframeIDList,
                      keyframeCostList,
                      keyframesToUpdate);
    if(keyframesToUpdate.size() < 5)
    {
        movingAverageLength = 1;
        SelectLargestNValues(movingAverageLength,
                          keyframeIDList,
                          keyframeCostList,
                          keyframesToUpdate);
        SelectLargestNValues(movingAverageLength,
                          keyframeIDList,
                          keyframeCostList,
                          keyframesToUpdate);
        SelectLargestNValues(movingAverageLength,
                          keyframeIDList,
                          keyframeCostList,
                          keyframesToUpdate);

    }
#endif
//    for(int i = 0; i < keyframesToUpdate.size();i++)
//    {
//        std::cout << "reintegrating keyframe: " << i << " " << keyframesToUpdate[i] << std::endl;
//    }
}


 void MobileFusion::IntegrateFrame(Frame & frame_ref)
{
    int totalPixelNum =  cameraModel.GetWidth() * cameraModel.GetHeight();
    std::cout << "integrate frame: "<< frame_ref.frame_index<<"..."<<std::endl;
    chisel::Transform lastPose;
    lastPose = frame_ref.pose_sophus[3].matrix().cast<float>();
    if(frame_ref.getRefinedDepth().empty())
    {
        return;
    }
    float * depthImageData = (float *)frame_ref.getRefinedDepth().data;
    unsigned char *colorImageData;
    if(frame_ref.getRgb().empty())
    {
        colorImageData = NULL;
    }
    else
    {
        cv::Mat &rgb = frame_ref.getRgb();
        colorImageData = new unsigned char[totalPixelNum * 4];
        for(int j = 0; j < totalPixelNum ; j++)
        {
            colorImageData[j*4 + 0] = rgb.at<unsigned char>(j*3+0);
            colorImageData[j*4 + 1] = rgb.at<unsigned char>(j*3+1);
            colorImageData[j*4 + 2] = rgb.at<unsigned char>(j*3+2);
            colorImageData[j*4 + 3] = 1;
        }
    }

    if(frame_ref.tracking_success && frame_ref.origin_index == gcSLAM.submapOrigin[frame_ref.submapID])
    {

        chiselMap->IntegrateDepthScanColorGlobal(projectionIntegrator,
                                           depthImageData,
                                           colorImageData,
                                           lastPose,
                                           cameraModel);
    }

    free(colorImageData);
}

int MobileFusion::tsdfFusion(std::vector<Frame> &frame_list,
                              int CorrKeyframeIndex,
                              const std::vector<MultiViewGeometry::KeyFrameDatabase> &kflist,const std::set<int> &frameIndex,
                              int integrateKeyframeID,int currentSubMapID)
{

//    printf("begin refine keyframes %d %d\r\n",CorrKeyframeIndex,frame_list.size());
//    if(frame_list[integrateKeyframeID].submapID != frame_list.back().submapID) return 0;
    Frame &frame_ref = frame_list[CorrKeyframeIndex];
    //if(kflist[integrateKeyframeID].keyFrameIndex == gcSLAM.submapOrigin[currentSubMapID])
    //return 0;
/*if(currentSubMapID == -1)
{
             Eigen::Matrix4f cur_pos = frame_ref.pose_sophus[0].matrix().cast<float>();
       chiselMap->UpdateMeshesDebug(cameraModel,1,Eigen::Transform<float, 3, Eigen::Affine>(cur_pos));
          tsdf_vertice_num = chiselMap->GetFullMeshes(tsdf_visualization_buffer,1);

         std::cout << "valid vertex num: " << tsdf_vertice_num << std::endl;
                  vertex_data_updated =1;
    return 1; 
    }
*/



    if(CorrKeyframeIndex == 0)
    {
        return 0;
    }
    std::vector<int> correspondent_submaps;
    if(frame_ref.is_keyframe )
    {


        ftimer.Tick("tsdf fusion");

         // get dynamic info
        std::vector<int> keyframesToUpdate;
         GetMapDynamics(frame_list,
                        keyframesToUpdate,frameIndex,
                        CorrKeyframeIndex,currentSubMapID);
        

         // create look up tables for kflist
         std::vector<int> frameIndexToKeyframeDB(CorrKeyframeIndex + 1);
         for(int i = 0; i < CorrKeyframeIndex; i++)
         {
             frameIndexToKeyframeDB[i] = -1;
         }
         for(int i = 0; i < kflist.size();i++)
         {
             if(kflist[i].keyFrameIndex <= CorrKeyframeIndex)
             frameIndexToKeyframeDB[kflist[i].keyFrameIndex] = i; 
         }



#if 1

         std::cout<< "CHISEL::Reintegrate keyframe size: "<< keyframesToUpdate.size()<<std::endl;
         for(int i =0 ; i < keyframesToUpdate.size(); i++)
         {
             int frame_index = keyframesToUpdate[i];
             TICK("CHISEL::Reintegration::Deintegration");
             ReIntegrateKeyframe(frame_list,kflist[frameIndexToKeyframeDB[frame_index]],0);
             ReIntegrateKeyframe(frame_list,kflist[frameIndexToKeyframeDB[frame_index]],1);
             TOCK("CHISEL::Reintegration::Deintegration");
            std::cout << "finish deintegrate frame: " << frame_index  << " time: "
                  << Stopwatch::getInstance().getTiming("CHISEL::Reintegration::Deintegration") << "ms" << std::endl;
         }


#endif

         //begin update

         int keyframeID =  kflist[integrateKeyframeID].keyFrameIndex;
         if(integrateKeyframeID >= 0)
         {
            int keyframeID =  kflist[integrateKeyframeID].keyFrameIndex;
            std::cout<<"integrate keyframe "<<keyframeID<<"..."<<std::endl;
            Frame &kf = frame_list[keyframeID];
            if(kf.submapID != currentSubMapID )
            return 0;

         //    OptimizeKeyframeVoxelDomain(frame_list,kflist[integrateKeyframeID]);
             if(kf.tracking_success && kf.origin_index == gcSLAM.submapOrigin[kf.submapID])
             {
                //std::cout<<"Start integrate KeyFrame"<<std::endl;
                TICK("CHISEL::IntegrateKeyFrame::PureIntegrateKeyFrame");
                ReIntegrateKeyframe(frame_list,kflist[integrateKeyframeID],1);
                TOCK("CHISEL::IntegrateKeyFrame::PureIntegrateKeyFrame");
             }
             else
             {
                 std::cout<<"tracking_success/origin_index: "<<kf.tracking_success<<"/"<<kf.origin_index<<std::endl;
             }
             
         }

#if SUBMAP_REINTEGRATION
        /*
        if(reactivated_submaps.size() != add_vertex_num)
        {

            add_vertex_num = reactivated_submaps.size();
            real_buffer.resize(GLOBAL_MODLE_VERTEX_NUM + add_vertex_num * ADD_VERTEX_NUM_EACH_TIME);
            std::cout<<"change real buffer size: "<<real_buffer.size()<<std::endl;
            rebind_buffer();
        }*/
        for(auto iter = reactivated_submaps.begin(); iter != reactivated_submaps.end(); ++iter)
        {
            if(std::fabs(CorrKeyframeIndex - iter->second)  < interval_threshold) 
                correspondent_submaps.push_back(iter->first);
        }       
        correspondent_submaps.push_back(mainSubmapID);
        if(referenceSubmapID >= 0)
        correspondent_submaps.push_back(referenceSubmapID);     
        if(correspondent_submaps.size() > 2)
        {
            std::cout<<"We need to integrate multiple submaps."<<std::endl;
            //ftimer.Tick("Submap Reintegration");
            ReIntegrateKeyframeInMultiSubmap(gcSLAM.globalFrameList, keyframeID, correspondent_submaps);
            //ftimer.Tock("Submap Reintegration");
            //ftimer.LogAll();

        }
        
#endif


        //std::cout << "finish to get full meshes: " << tsdf_vertice_num << std::endl;
        //std::cout<<"start update meshes"<<std::endl;
        //chiselMap->UpdateMeshesDebug(cameraModel,currentSubMapID);
        ftimer.Tock("tsdf fusion");
#if FOR_REBUTTAL
        t_tsdf_fusion[frame_ref.frame_index] = ftimer.Elapsed("tsdf fusion");
#endif
    }

    std::cout << "begin to update meshes"  << std::endl;

    ftimer.Tick("meshing");
#if SUBMAP_REINTEGRATION
    if(correspondent_submaps.size() > 2)
        chiselMap->UpdateMeshes(cameraModel,correspondent_submaps);
    else 
        chiselMap->UpdateMeshes(cameraModel,mainSubmapID,referenceSubmapID);
#else
    chiselMap->UpdateMeshes(cameraModel,mainSubmapID,referenceSubmapID);
#endif
    ftimer.Tock("meshing");
#if FOR_REBUTTAL
    t_mesh[frame_ref.frame_index] = ftimer.Elapsed("meshing");
#endif
    //chiselMap -> UpdateMeshes(cameraModel,currentSubMapID);
    
    TOCK("CHISEL_MESHING::UpdateMeshes");

    global_vertex_data_need_to_update = 1;

    return 1;
}
/**/

int MobileFusion::GetFullMeshes()
{   
    if(!global_vertex_data_need_to_update) return 0;
    std::cout << "begin to get full meshes"  << std::endl;

    TICK("CHISEL_MESHING::GetFullMeshes");
    //tsdf_vertice_num = chiselMap->GetFullMeshes(tsdf_visualization_buffer,currentSubMapID);//current Submap Mesh
    //tsdf_vertice_num = chiselMap->GetFullMeshes(tsdf_visualization_buffer);//global Mesh
    //tsdf_vertice_num = chiselMap->GetFullMeshes(chiselMap->GetFullMeshes(),tsdf_visualization_buffer);//Global Mesh, but in a ply model
    //PoseSE3dList poses(gcSLAM.submapPosesRelativeChanges.size());
        // TODO : Extract vertices to Tensor
    TICK("CHISEL_MESHING::UNet_GetAllVertices");
    std::vector<long> un_simplified_label;
    int num_vertices;
    float scale = 20;
    float full_scale = 1024;
    torch::Tensor coords = torch::empty({GLOBAL_MODLE_VERTEX_NUM ,3}, torch::dtype(torch::kFloat32));
    torch::Tensor features = torch::empty({GLOBAL_MODLE_VERTEX_NUM ,3}, torch::dtype(torch::kFloat32));
    //std::cout<<"Tensor is ready!"<<std::endl;

    std::set<int> room_submap;
    //std::unique_lock <std::mutex> semantic_lock(chisel::semantic_mesh_generating_mutex);
    if(current_room_id != -1)             
    {
        room_submap = std::set<int>(room_to_submap[current_room_id].begin(), room_to_submap[current_room_id].end());
        room_submap.insert(mainSubmapID);
    }
    //std::cout<<"begin to get segmentation input: "<<current_room_id<< std::endl;

    


    if(current_room_id == -1)
    num_vertices = chiselMap->GetSegmentationInput(coords.data<float>(), 
        features.data<float>(), unlabeled_submap, gcSLAM.submapPosesRelativeChanges,gcSLAM.cameraStartPose );
    else 
    {
        num_vertices = chiselMap->GetSegmentationInput(coords.data<float>(), 
        features.data<float>(), room_submap, gcSLAM.submapPosesRelativeChanges,gcSLAM.cameraStartPose );
    }
    //std::cout<<"hash_aggregated_map"<<hash_aggregated_map.size()<<std::endl;

    //std::cout<<"vertex number to segmantic:"<<num_vertices<<std::endl;

    TOCK("CHISEL_MESHING::UNet_GetAllVertices");

    if (num_vertices == 0)
        return 1;

    TICK("CHISEL_MESHING::UNet_Preprocessing");

    coords.resize_({num_vertices, 3});
    features.resize_({num_vertices, 3});

    coords *= scale;
    auto m = std::get<0>(coords.min(0));
    coords -= m;
    coords.clamp_max(full_scale);
    TOCK("CHISEL_MESHING::UNet_Preprocessing");

    TICK("CHISEL_MESHING::UNet_Forward");
    ftimer.Tick("segmentation");
    Tensors outlist = unet->forward(coords, features);    // CUDA float  (num_vertices, 20)
    // TODO: instance here
    std::cout<<"Finish forward propogation."<<std::endl;
    auto semantic_score = outlist[0];
    RGBAPixelList instance_list;
    auto pred_instance = InstanceSegmentation(outlist,coords/scale,instance_list);
    pred_instance = pred_instance.cpu();
    std::cout<<"Finish instance prediction."<<std::endl;

    auto predict = semantic_score.argmax(1).cpu();             // long (num_vertices,)
    
    assert(predict.is_cuda() == false);
    ftimer.Tock("segmentation");
    TOCK("CHISEL_MESHING::UNet_Forward");

    TICK("CHISEL_MESHING::Back_Generating_label");

//            cnpy::npy_save("./coord_"+std::string("0")+".npy",coords.data<float>(),{(unsigned long)num_vertices, 3},"w");
//            cnpy::npy_save("./features_"+std::string("0")+".npy",features.data<float>(),{(unsigned long)num_vertices, 3},"w");
//            cnpy::npy_save("./label_"+std::string("0")+".npy",predict.data<long>(),{(unsigned long)num_vertices},"w");

    long *tmp_label_ptr = NULL;
    long *tmp_instance_ptr = NULL;
    tmp_instance_ptr=pred_instance.data<long>();
    tmp_label_ptr=predict.data<long>();
    //std::cout<<"Start to update semantic label."<<std::endl;

    if(current_room_id == -1)
    chiselMap->UpdateSemanticMeshWithLabel(tmp_label_ptr, tmp_instance_ptr, unlabeled_submap);
    else 
    chiselMap->UpdateSemanticMeshWithLabel(tmp_label_ptr, tmp_instance_ptr, room_submap);

    
#if FOR_REBUTTAL
    t_segmentation[gcSLAM.globalFrameList.back().frame_index] = ftimer.Elapsed("segmentation");
    int last_frame_index = gcSLAM.globalFrameList.back().frame_index;
    float last_recorded_memory = getOccupiedMemory();
    m_gcfusion[last_frame_index] = last_recorded_memory;
    ofs_memory<<last_frame_index<<" "<<last_recorded_memory<<std::endl;
#endif
    //std::cout<<"Finish updating semantic label."<<std::endl;
    TOCK("CHISEL_MESHING::Back_Generating_label");
    //patch_image = visualization::PatchImage(planes);
    global_show_lock.lock();
    TICK("CHISEL_MESHING::Get_All_Semantic_Meshes");
    amount_tsdf_vertice_num = chiselMap->GetAllSemanticMeshes(tsdf_visualization_buffer,(int *)instance_list.data(),
        gcSLAM.submapPosesRelativeChanges,gcSLAM.cameraStartPose);
    TOCK("CHISEL_MESHING::Get_All_Semantic_Meshes");
    global_vertex_data_need_to_update = 0;
    global_vertex_data_updated = 1;
    global_show_lock.unlock();
    /*
    if(current_room_id == -1)
    amount_tsdf_vertice_num = chiselMap->GetRoomSemanticMeshes(tsdf_visualization_buffer,(int *)instance_list.data(),
        unlabeled_submap, gcSLAM.submapPosesRelativeChanges,gcSLAM.cameraStartPose);
    else 
    amount_tsdf_vertice_num = chiselMap->GetRoomSemanticMeshes(tsdf_visualization_buffer,(int *)instance_list.data(),
        room_submap, gcSLAM.submapPosesRelativeChanges,gcSLAM.cameraStartPose);
    std::cout<<"Get all mesh: "<<amount_tsdf_vertice_num<<std::endl;
    */
    


    //amount_tsdf_vertice_num= chiselMap->GetFullMeshes(tsdf_visualization_buffer ,mainSubmapID,referenceSubmapID, gcSLAM.submapPosesRelativeChanges,gcSLAM.cameraStartPose /* gcSLAM.submapPoses*/);

    std::cout << "valid vertex num(all): " << amount_tsdf_vertice_num << std::endl;  


    
    TOCK("CHISEL_MESHING::GetFullMeshes");

         // this should be implemented at each keyframe update firstly.
         // update the given vertices based on the updated camera pose
         // now we try to recreate a tsdf map, based on the given vertices
    std::cout<<"Get full meshes done!"<<std::endl;
    //ftimer.Tock("GetSemanticFullMeshes");
    return 1;
}



int MobileFusion::UpdateSegmentationGlobal()
{   

    std::vector<long> un_simplified_label;
    int num_vertices;
    float scale = 20;
    float full_scale = 1024;
    torch::Tensor coords = torch::empty({GLOBAL_MODLE_VERTEX_NUM ,3}, torch::dtype(torch::kFloat32));
    torch::Tensor features = torch::empty({GLOBAL_MODLE_VERTEX_NUM ,3}, torch::dtype(torch::kFloat32));
    //std::cout<<"Tensor is ready!"<<std::endl;
    //std::cout<<"begin to get segmentation input: "<<current_room_id<< std::endl;
    num_vertices = chiselMap->GetSegmentationInputGlobal(coords.data<float>(), 
        features.data<float>());
    //std::cout<<"hash_aggregated_map"<<hash_aggregated_map.size()<<std::endl;

    //std::cout<<"vertex number to segmantic:"<<num_vertices<<std::endl;

    TOCK("CHISEL_MESHING::UNet_GetAllVertices");

    if (num_vertices == 0)
        return 1;

    TICK("CHISEL_MESHING::UNet_Preprocessing");

    coords.resize_({num_vertices, 3});
    features.resize_({num_vertices, 3});

    coords *= scale;
    auto m = std::get<0>(coords.min(0));
    coords -= m;
    coords.clamp_max(full_scale);
    TOCK("CHISEL_MESHING::UNet_Preprocessing");

    TICK("CHISEL_MESHING::UNet_Forward");
    Tensors outlist = unet->forward(coords, features);    // CUDA float  (num_vertices, 20)
    // TODO: instance here

    auto semantic_score = outlist[0];
    RGBAPixelList instance_list;
    auto pred_instance = InstanceSegmentation(outlist,coords/scale,instance_list);
    pred_instance = pred_instance.cpu();


    auto predict = semantic_score.argmax(1).cpu();             // long (num_vertices,)
    
    assert(predict.is_cuda() == false);

    TOCK("CHISEL_MESHING::UNet_Forward");

    TICK("CHISEL_MESHING::Back_Generating_label");

//            cnpy::npy_save("./coord_"+std::string("0")+".npy",coords.data<float>(),{(unsigned long)num_vertices, 3},"w");
//            cnpy::npy_save("./features_"+std::string("0")+".npy",features.data<float>(),{(unsigned long)num_vertices, 3},"w");
//            cnpy::npy_save("./label_"+std::string("0")+".npy",predict.data<long>(),{(unsigned long)num_vertices},"w");

    long *tmp_label_ptr = NULL;
    long *tmp_instance_ptr = NULL;
    tmp_instance_ptr=pred_instance.data<long>();
    tmp_label_ptr=predict.data<long>();
    //std::cout<<"Start to update semantic label."<<std::endl;

    chiselMap->UpdateSemanticMeshWithLabelGlobal(tmp_label_ptr, tmp_instance_ptr);
    return 1;
}


void MobileFusion::GetFullMeshes(std::vector<unsigned char > &buffer)
{
    
    buffer.resize(amount_tsdf_vertice_num* byte_of_each_point);
    memcpy(&buffer[0],tsdf_visualization_buffer,amount_tsdf_vertice_num*byte_of_each_point);
}

size_t MobileFusion::GetFullPCD(std::vector<unsigned char > &buffer)
{
    return chiselMap->GetFullPCD(buffer, gcSLAM.submapPosesRelativeChanges,gcSLAM.cameraStartPose);
}
