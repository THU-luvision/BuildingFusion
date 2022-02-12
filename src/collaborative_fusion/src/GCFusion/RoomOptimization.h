#ifndef ROOM_OPTIMIZATION_H
#define ROOM_OPTIMIZATION_H
#include "../GCSLAM/MultiViewGeometry.h"
using namespace MultiViewGeometry;
struct RoomCorrespondence
{
    // RoomCorrespondence(int _source_id, int _target_id, 
    //                 std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> 
    //                 _instance_center_correspondences) {
    //     source_id = _source_id;
    //     target_id = _target_id;
    //     instance_center_correspondences = _instance_center_correspondences;
    // }
    int source_id;//room_id
    int target_id;
    int source_camera_id;//camera_id
    int target_camera_id;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> instance_center_correspondences;
    Eigen::Vector3d sum_p_ref;		// sum(p_ref)
    Eigen::Vector3d sum_p_new;		// sum(p_new)
    Eigen::Matrix3d sum_p_ref_new;  // sum(p_ref * p_new^T)
    Eigen::Matrix3d sum_p_ref_ref;  // sum(p_ref * p_ref^T)
    Eigen::Matrix3d sum_p_new_new;  // sum(p_new * p_new^T)
    Eigen::Matrix3d sum_p_new_ref;  // sum(p_new * p_ref^T)
    // to calculate reprojection error
    float sum_feature_p_ref_T_p_ref, sum_feature_p_new_T_p_new;
    float sum_icp_p_ref_T_p_ref, sum_icp_p_new_T_p_new;
    bool use_icp = false;
    double sum_weight;
    int sparse_feature_cnt;
    float icp_inlier_ratio = 0;
    float score_by_instance = 0;
    //Eigen::Matrix4d transformation;
    //preintegration

    void Rotate(const Eigen::Matrix3f &R)
    {
        for(int i = 0; i < instance_center_correspondences.size(); ++i)
        {
            instance_center_correspondences[i].first = R * instance_center_correspondences[i].first;
            instance_center_correspondences[i].second = R * instance_center_correspondences[i].second;
        }
    }

    void reset()
    {
        sum_p_ref.setZero();
        sum_p_new.setZero();
        sum_p_ref_new.setZero();
        sum_p_ref_ref.setZero();
        sum_p_new_new.setZero();
        sum_p_new_ref.setZero();
        sum_weight = 0;

        sum_feature_p_ref_T_p_ref = 0;
        sum_feature_p_new_T_p_new = 0;
        sum_icp_p_ref_T_p_ref = 0;
        sum_icp_p_new_T_p_new = 0;    

        sparse_feature_cnt = 0;
        instance_center_correspondences.clear();    
    }
    void preIntegrate(const std::vector<PoseSE3dList> &room_poses)
    {

        int use_weight = 0;
        sum_p_ref.setZero();
        sum_p_new.setZero();
        sum_p_ref_new.setZero();
        sum_p_ref_ref.setZero();
        sum_p_new_new.setZero();
        sum_p_new_ref.setZero();
        sum_weight = 0;

        sum_feature_p_ref_T_p_ref = 0;
        sum_feature_p_new_T_p_new = 0;
        sum_icp_p_ref_T_p_ref = 0;
        sum_icp_p_new_T_p_new = 0;
        PoseSE3d room_pose_ref = room_poses[source_camera_id][source_id];
        PoseSE3d room_pose_new = room_poses[target_camera_id][target_id];
        
        Eigen::Matrix3d R_ref = room_pose_ref.rotationMatrix();
        Eigen::Vector3d t_ref = room_pose_ref.translation();
        Eigen::Matrix3d R_new = room_pose_new.rotationMatrix();
        Eigen::Vector3d t_new = room_pose_new.translation();        

        sparse_feature_cnt = instance_center_correspondences.size();
        std::cout<<room_pose_ref.matrix()<<std::endl<<std::endl;
        std::cout<<room_pose_new.matrix()<<std::endl<<std::endl;
        std::cout<<"sparse_feature: "<<sparse_feature_cnt<<std::endl;
        for (int i = 0; i < sparse_feature_cnt; i++)
        {
            float depth =(R_ref * instance_center_correspondences[i].first.cast<double>() + t_ref)[2];
            Eigen::Vector3d first = R_ref * instance_center_correspondences[i].first.cast<double>()+ t_ref;
            Eigen::Vector3d second =R_new * instance_center_correspondences[i].second.cast<double>() + t_new;
            double weight = 1;

            sum_weight += weight;
            sum_p_ref +=  first * weight;
            sum_p_new += second * weight;
            sum_p_ref_new += first * second.transpose() * weight;
            sum_p_ref_ref += first * first.transpose() * weight;
            sum_p_new_new += second * second.transpose() * weight;
            sum_p_new_ref += second* first.transpose() * weight;

            sum_feature_p_ref_T_p_ref = first.transpose() * first;
            sum_feature_p_new_T_p_new = second.transpose() * second;
        }
        std::cout<<"sum_weight: "<<sum_weight<<std::endl;
        //sum_p_ref_feature = sum_p_ref ;
        //sum_p_new_feature = sum_p_new ;
    }
};
float reprojection_error_3Dto3D_room_submap(std::vector<RoomCorrespondence> &room_clist,
    std::vector<FrameCorrespondence> &fCList, 
    PoseSE3dList &room_poses, PoseSE3dList &submap_poses_delta,
    const std::vector<int> &submap_to_room,
    const std::vector<int> &submap_to_position);
float reprojection_error_3Dto3D_room_submap(const FrameCorrespondence &fC, 
    const PoseSE3dList &room_poses,
    const PoseSE3dList & submap_poses_delta,
    const std::vector<int> &submap_to_room,
    const std::vector<int> &submap_to_position);
float reprojection_error_3Dto3D_room(const RoomCorrespondence &rC, const PoseSE3dList & room_poses);
float reprojection_error_3Dto3D_room(const RoomCorrespondence &rC, const Sophus::SE3d & relative_post_from_ref_to_new);
void ComputeJacobianInfoRoom(RoomCorrespondence &fC,PoseSE3dList &room_poses,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj);
void ComputeJacobianInfoRoomSubmap(FrameCorrespondence &fC,
    PoseSE3dList &room_poses,
    PoseSE3dList &submap_poses_delta,
    const std::vector<int> submap_to_room,
    const std::vector<int> submap_to_position,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj);
void CombiningOptimization(std::vector<RoomCorrespondence > &room_correspondence, 
    std::vector<FrameCorrespondence> &submap_correspondences, PoseSE3dList & submapPosesRelativeChanges, 
    PoseSE3dList &submapPosesFinal, const std::vector<std::vector<int>> &room_to_submap, const std::vector<int> &submap_to_room);

#endif