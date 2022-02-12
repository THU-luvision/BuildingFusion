#ifndef ROOM_HANDLER_H
#define ROOM_HANDLER_H

#include "../GCFusion/RoomOptimization.h"
#include "../RoomLCD/GRANSAC.hpp"
#include "../RoomLCD/PlaneFittingModel.hpp"
#include <open_chisel/mesh/PointCloud.h>
#include <open_chisel/ForLabel.h>
namespace Server
{
    static GRANSAC::RANSAC<PlaneFittingModel, 8> Estimator;
    class RoomInfo
    {
        public:
        int camera_id = -1;
        int room_id = -1;  
        int position_in_database = -1;
        chisel::PcdPtr model;
        std::vector<int> contained_submaps;
        Eigen::Vector3f floor_normal;
        float room_area = 0;
        void ComputeFloorNormal(const std::vector<Point3fList> &submap_floor_normals, const std::vector<PoseSE3dList> &submap_relative_changes)
        {
            if(camera_id == -1) return;
            floor_normal.setZero();
            auto &c_normals = submap_floor_normals[camera_id];
            auto &c_poses = submap_relative_changes[camera_id];
            int count = 0;
            for(int i = 0; i != contained_submaps.size(); ++i)
            {
                int submap_id = contained_submaps[i];
                Eigen::Matrix3f relative_R = c_poses[submap_id].rotationMatrix().cast<float>();

                if(submap_id >= c_normals.size()) return;
                if(c_normals[submap_id][0] != 999.0)
                {
                    floor_normal += (relative_R * c_normals[i]);
                    count += 1;
                }
            }
            floor_normal /= count;
            //here we don't need to transform with R
            floor_normal.normalize();
            //Another method: use the floor of room to recompute the floor normal
            std::cout<<"Room Normal: "<<floor_normal<<std::endl;
        }
        void ComputeFloorNormal()
        {
            if(camera_id == -1) return;
            floor_normal << 999.0, 0, 0;
            Eigen::Vector3f floor ;
            floor<<0,0,1.0;
            Point3fList floor_points;
            for(int i = 0; i != model->labels.size(); ++i)
            {
                if(get_semantic_label(model->labels[i]) == 1 && model->normals[i].transpose() * floor > 0.5)
                {
                    floor_points.push_back(model->vertices[i]);
                }
            }
            
            if(floor_points.size() >= 20)
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
                auto normal = std::get<0>(result);
                floor_normal[0] = normal[0];
                floor_normal[1] = - normal[2];
                floor_normal[2] = normal[1];
                if(floor_normal[1] > 0)
                floor_normal = - floor_normal;
            }
            //Another method: use the floor of room to recompute the floor normal
            std::cout<<"Room Normal: "<<floor_normal<<std::endl;
        }
    };


}
#endif