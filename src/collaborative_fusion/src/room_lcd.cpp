#include <torch/torch.h>
#include "./RoomLCD/utils.h"
#include <open_chisel/io/PLY.h>
#include <open_chisel/io/LabelColor.h>
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <random>
#include "./GCSLAM/ICPRegistration.h"
#include "./Tools/TickTock.h"


chisel::PcdPtr labelMask(const chisel::PcdPtr &origin_pcd)
{
    chisel::PcdPtr result_pcd(new chisel::PointCloud());
    *result_pcd = *origin_pcd;
    /*
    std::map<long, std::vector<int>> label_points;
    for(int i = 0; i != origin_pcd->vertices.size(); ++i)
    {
        int semantic_label = get_semantic_label(origin_pcd->labels[i]);
        int instance_label = get_instance_label(origin_pcd->labels[i]);
        if((semantic_label == CHAIR  )&& instance_label != -1)
        {
            label_points[origin_pcd->labels[i]].push_back(i);
        }
    }

    for(auto iter = label_points.begin(); iter != label_points.end(); ++iter)
    {
        auto &points = iter->second;

            for(int i = 0; i != points.size(); ++i)
            {
                result_pcd->vertices.push_back(origin_pcd->vertices[points[i]]);
                result_pcd->normals.push_back(origin_pcd->normals[points[i]]);
                result_pcd->labels.push_back(origin_pcd->labels[points[i]]);
            }
    }*/
    return result_pcd;
}
std::pair<long, int> mode(std::vector<long> &group)
{
    std::sort(group.begin(), group.end()); 
  
    //finding max frequency  
    int max_count = 1, count = 1;
    long res = group[0]; 
    for (int i = 1; i < group.size(); i++) { 
        if (group[i] == group[i - 1]) 
            count++; 
        else { 
            if (count > max_count) { 
                max_count = count; 
                res = group[i - 1]; 
            } 
            count = 1; 
        } 
    } 
  
    // when the last element is most frequent 
    if (count > max_count) 
    { 
        max_count = count; 
        res = group[group.size() - 1]; 
    } 

    return std::make_pair(res, max_count);
    

}
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
void useICPToAddPair(std::vector<RoomCorrespondence> &tmp_room_correspondences, Mat4dList &transformations, 
    std::vector<chisel::PcdPtr> &old_pcds, chisel::PcdPtr &pcd2)
{
  transformations.resize(tmp_room_correspondences.size(), Eigen::Matrix4d::Identity());
  std::cout<<"use Icp to add correspondences"<<std::endl;
  for(int i = 0; i != tmp_room_correspondences.size(); ++i)
  {
    transformations[i] = ICPTransformation(tmp_room_correspondences[i].instance_center_correspondences).cast<double>();
    int source_camera_id = tmp_room_correspondences[i].source_camera_id;
    int target_camera_id = tmp_room_correspondences[i].target_camera_id;
    std::vector<pair<Eigen::Vector3f, Eigen::Vector3f>> point_pairs_3d;
    //int source_room_id = tmp_room_correspondences[i].source_id;
    //int target_room_id = tmp_room_correspondences[i].target_id;
    //std::cout<<"id: "<<tmp_room_correspondences[i].source_id<<std::endl;
    //std::cout<<"old_pcd: "<<old_pcds[0]->normals.size()<<std::endl;
    auto ref_pcd = labelMask(old_pcds[tmp_room_correspondences[i].source_id]);
    auto new_pcd = labelMask(pcd2);
    std::vector<Eigen::Vector2i> correspondence_set;
    int inlier_count = icp_registration::open3d_icp_registration(ref_pcd, new_pcd, 
        transformations[i], point_pairs_3d, correspondence_set, 0.06);
    
    float score = computeScoreByInstance(ref_pcd, new_pcd, correspondence_set, 500);
    //chisel::SaveSemanticPointCloudPLYASCII("./ref_pcd.ply", ref_pcd);
    //chisel::SaveSemanticPointCloudPLYASCII("./new_pcd.ply", new_pcd); 
    
    tmp_room_correspondences[i].instance_center_correspondences =std::move(point_pairs_3d);
    float inlier_ratio = (inlier_count + 0.0) / std::min(ref_pcd->vertices.size(), new_pcd->vertices.size());
    std::cout<<"id: "<<tmp_room_correspondences[i].source_id<<std::endl;
    std::cout<<"ratio: "<<inlier_ratio<<std::endl;
    std::cout<<"score: "<<score<<std::endl;
    //std::cout<<"extract "<<tmp_room_correspondences[i].instance_center_correspondences.size()<<" pairs of points."<<std::endl;
  }
}
int main(int argc, char *argv[])
{
    SceneDatabase scene_database;
    scene_database.Estimator.Initialize(0.2, 2000);
    MultiViewGeometry::g_para.room_lcd_n_nearest = 10;
    std::vector<Eigen::Matrix4f> transformations;
    std::vector<chisel::PcdPtr> pcds;
    chisel::PcdPtr pcd3 (new chisel::PointCloud());
    MultiViewGeometry::g_para.room_lcd_min_inlier = 3;
    MultiViewGeometry::g_para.room_lcd_min_ratio = 0.2;
    tool::Timer timer;
    while(true)
    {
        string path;
        std::cin>>path;
        //std::cout<<path<<std::endl;
        chisel::PcdPtr pcd(new chisel::PointCloud());
        chisel::ReadFPCDASCII(path,pcd);

        auto obj = get_graph(*pcd);
        //std::vector<RoomCorrespondence> room_correspondence;
        std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>> graph_matching;
        std::vector<std::vector<int>> inlier_matches;
        if(scene_database.get_size())
        {
            scene_database.query_with_matches(obj,graph_matching, inlier_matches);
            scene_database.insert(obj);
            if(graph_matching.size() > 0)
            {
                chisel::PcdPtr pcd_source(new chisel::PointCloud());
                chisel::PcdPtr pcd_target(new chisel::PointCloud());
                for(int i = 0; i != graph_matching[0].size(); ++i)
                {
                    pcd_source->vertices.push_back(graph_matching[0][i].first);
                    pcd_target->vertices.push_back(graph_matching[0][i].second);
                }
                
                chisel::SavePointCloudPLYASCII("./source_instance_center.ply", pcd_source);
                chisel::SavePointCloudPLYASCII("./target_instance_center.ply", pcd_target);
                cnpy::npy_save("./inlier_matches.npy", &inlier_matches[0][0],{inlier_matches[0].size()}, "w");
            }
            /*
            for(int i = 0; i < room_correspondence.size(); ++i)
            {
                std::cout<<room_correspondence[i].;
            }*/
            //std::cout<<"correspondence: "<<room_correspondence.size()<<std::endl;

            // if(room_correspondence.size() == 1)
            // {
            //     Eigen::Matrix4f initial_transformation = ICPTransformation(room_correspondence[0].instance_center_correspondences);
            //     //std::cout<<"Transform: "<<initial_transformation<<std::endl;;
            //     pcd->Transform(initial_transformation.inverse());
            //     chisel::SavePointCloudPLYASCII("after_transform.ply", pcd);
            //     pcd->Transform(initial_transformation); 
            // }
            // if(room_correspondence.size() > 0)
            // {
            //     int j = 0;
            //     for(int i = 0; i != room_correspondence[j].instance_center_correspondences.size(); ++i)
            //     {
            //         std::cout<<i <<room_correspondence[j].instance_center_correspondences[i].first.transpose()<<" "<<
            //             room_correspondence[j].instance_center_correspondences[i].second.transpose()<<std::endl;
                    
            //         Eigen::Vector3f first = room_correspondence[j].instance_center_correspondences[i].first;
            //         Eigen::Vector3f second = room_correspondence[j].instance_center_correspondences[i].second;

            //         Eigen::Vector3f point_tmp;
            //         for(int x = -4; x <5; ++x)
            //         {
            //             point_tmp(0) = first(0) + x*0.005;
            //             for(int y = -4; y <5; ++y)
            //             {
            //                 point_tmp(1) = first(1) + y*0.005;
            //                 for(int z = -4; z <5; ++z)
            //                 {
            //                     point_tmp(2) = first(2) + z*0.005;
            //                     pcd3->vertices.push_back(point_tmp);
            //                     pcd3->colors.push_back(Eigen::Vector3f(chisel::label_colors[3* i], chisel::label_colors[3 * i + 1], chisel::label_colors[3*i + 2] ));
            //                 }
            //             }
            //         }  
            //         for(int x = -4; x <5; ++x)
            //         {
            //             point_tmp(0) = second(0) + x*0.005;
            //             for(int y = -4; y <5; ++y)
            //             {
            //                 point_tmp(1) = second(1) + y*0.005;
            //                 for(int z = -4; z <5; ++z)
            //                 {
            //                     point_tmp(2) = second(2) + z*0.005;
            //                     pcd3->vertices.push_back(point_tmp);
            //                     pcd3->colors.push_back(Eigen::Vector3f(chisel::label_colors[3* i], chisel::label_colors[3 * i + 1], chisel::label_colors[3*i + 2] ));
            //                 }
            //             }
            //         }
            //         float step_x = (second(0) - first(0))/1000;
            //         float step_y = (second(1) - first(1))/1000;
            //         float step_z = (second(2) - first(2))/1000;
            //         for(int c = 0; c != 1000; ++c)
            //         {
            //             pcd3->vertices.push_back(first + Eigen::Vector3f(step_x, step_y, step_z) * c);
            //             pcd3->colors.push_back(Eigen::Vector3f(1,0,0));
            //         } 
            //     }
            //    chisel::SavePointCloudPLYASCII("correspondences.ply", pcd3);
            //}
            // Mat4dList transformations;
            // timer.Tick("computing score");
            // useICPToAddPair(room_correspondence, transformations, pcds, pcd);
            // timer.Tock("computing score");
            timer.LogAll();
        }
        else scene_database.insert(obj);
        pcds.push_back(pcd);
    
        //chisel::SaveSemanticPointCloudPLYASCII(path+".ply", pcd);          
    }
    /*
    if (argc != 5) {
        std::cerr << "Wrong Arguments." << std::endl;
    }
    // ./loop database_scene_list.txt query_scene_list.txt
    std::string path1 = argv[1];
    std::string path2 = argv[2];

    chisel::PcdPtr pcd1 (new chisel::PointCloud());
    chisel::PcdPtr pcd2 (new chisel::PointCloud());
    chisel::PcdPtr pcd3 (new chisel::PointCloud());

    chisel::PcdPtr pcd_icp (new chisel::PointCloud());
    std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > point_pairs_3d;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    //Eigen::Matrix4d T1 = Eigen::Matrix4d::Zero();
    //T = T1;
    chisel::ReadFPCDASCII(path1, pcd1);
    chisel::ReadFPCDASCII(path2, pcd2);

    //SaveSemanticPointCloudPLYASCII("./dense_room.ply", pcd0);
    tool::Timer timer;
    //std::cout<<"point_corres: "<<point_pairs_3d.size()<<std::endl<<T<<std::endl;

    Eigen::Matrix3f rot;
    rot << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    rot = rot.inverse();
    std::cout<<rot<<std::endl;


      

    auto obj1 = get_graph(*pcd1);
    auto obj2 = get_graph(*pcd2);
    auto pcd4 = obj1.visualize_to_ply();
    auto pcd5 = obj2.visualize_to_ply();
    SceneDatabase scene_database;
    std::vector<RoomCorrespondence> room_correspondence;
    scene_database.insert(obj1);
    scene_database.query(obj2, room_correspondence);
    std::cout<<"room_correspondence.size = "<<room_correspondence.size()<<std::endl;
    

    for(int j = 0; j != room_correspondence.size(); ++j)
    for(int i = 0; i != room_correspondence[j].instance_center_correspondences.size(); ++i)
    {
        std::cout<<i <<room_correspondence[j].instance_center_correspondences[i].first.transpose()<<" "<<
            room_correspondence[j].instance_center_correspondences[i].second.transpose()<<std::endl;
        
        Eigen::Vector3f first = room_correspondence[j].instance_center_correspondences[i].first;
        Eigen::Vector3f second = room_correspondence[j].instance_center_correspondences[i].second;

        Eigen::Vector3f point_tmp;
        for(int x = -4; x <5; ++x)
        {
            point_tmp(0) = first(0) + x*0.005;
            for(int y = -4; y <5; ++y)
            {
                point_tmp(1) = first(1) + y*0.005;
                for(int z = -4; z <5; ++z)
                {
                    point_tmp(2) = first(2) + z*0.005;
                    pcd3->vertices.push_back(point_tmp);
                    pcd3->colors.push_back(Eigen::Vector3f(chisel::label_colors[3* i], chisel::label_colors[3 * i + 1], chisel::label_colors[3*i + 2] ));
                }
            }
        }  
        for(int x = -4; x <5; ++x)
        {
            point_tmp(0) = second(0) + 10  + x*0.005;
            for(int y = -4; y <5; ++y)
            {
                point_tmp(1) = second(1) + y*0.005;
                for(int z = -4; z <5; ++z)
                {
                    point_tmp(2) = second(2) + z*0.005;
                    pcd3->vertices.push_back(point_tmp);
                    pcd3->colors.push_back(Eigen::Vector3f(chisel::label_colors[3* i], chisel::label_colors[3 * i + 1], chisel::label_colors[3*i + 2] ));
                }
            }
        }
        float step_x = (second(0) +10- first(0))/1000;
        float step_y = (second(1) - first(1))/1000;
        float step_z = (second(2) - first(2))/1000;
        for(int c = 0; c != 1000; ++c)
        {
            pcd3->vertices.push_back(first + Eigen::Vector3f(step_x, step_y, step_z) * c);
            pcd3->colors.push_back(Eigen::Vector3f(1,0,0));
        } 
    }

    Mat4dList transformations;
    useICPToAddPair(room_correspondence, transformations,pcd1, pcd2);
    room_correspondence[0].Rotate(rot);
    point_pairs_3d = room_correspondence[0].instance_center_correspondences;
    transformations[0] = ICPTransformation(point_pairs_3d).cast<double>().inverse();
    for(int i = 0; i != point_pairs_3d.size(); ++i)
    {
        
        Eigen::Vector3f first = point_pairs_3d[i].first;
        Eigen::Vector3f second = point_pairs_3d[i].second;

        Eigen::Vector3f point_tmp;
        float step_x = (second(0) +10- first(0))/1000;
        float step_y = (second(1) - first(1))/1000;
        float step_z = (second(2) - first(2))/1000;
        for(int c = 0; c != 1000; ++c)
        {
            pcd_icp->vertices.push_back(first + Eigen::Vector3f(step_x, step_y, step_z) * c);
            pcd_icp->colors.push_back(Eigen::Vector3f(1,0,0));
        } 
    }

    for(int i = 0; i != pcd1->vertices.size(); ++i)
    {
        //pcd2->vertices[i](0) += 10;
        pcd1->vertices[i] = rot * pcd1->vertices[i];
        pcd1->normals[i] = rot * pcd1->normals[i];
    }

    for(int i = 0; i != pcd2->vertices.size(); ++i)
    {
        //pcd2->vertices[i](0) += 10;
        pcd2->vertices[i] = rot * pcd2->vertices[i];
        pcd2->normals[i] = rot * pcd2->normals[i];
    }
    for(int i = 0; i != pcd2->vertices.size(); ++i)
    {
        //pcd2->vertices[i](0) += 10;
        pcd2->vertices[i] = transformations[0].block<3,3>(0,0).cast<float>() * pcd2->vertices[i] + transformations[0].block<3,1>(0,3).cast<float>();
    }

    std::cout << transformations[0]<<std::endl;
    pcd4->colors.resize(pcd4->vertices.size(), Eigen::Vector3f(0,0,0));
    pcd5->colors.resize(pcd5->vertices.size(), Eigen::Vector3f(1,1,1));
    chisel::SavePointCloudPLYASCII("obj1.ply", pcd4);
    chisel::SavePointCloudPLYASCII("obj2.ply", pcd5);
    chisel::SaveSemanticPointCloudPLYASCII("sparse_room_0.ply", pcd1);
    chisel::SaveSemanticPointCloudPLYASCII("sparse_room_1.ply", pcd2);       
  
    chisel::SavePointCloudPLYASCII("correspondences.ply", pcd3);
    chisel::SavePointCloudPLYASCII("correspondences_icp.ply",pcd_icp);
    */
 
    return 0;
}