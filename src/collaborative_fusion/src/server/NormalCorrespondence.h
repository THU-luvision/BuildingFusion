#ifndef NORMAL_CORRESPONDENCE_H
#define NORMAL_CORRESPONDENCE_H
#include "./ServerFrame.h"
namespace Server
{
static Eigen::Vector3d Gravity;

class NormalCorrespondence
{
    public:

    //in fact, we assume that each correspondence only have a pair of normal.
    Eigen::Vector3d sum_p_ref_feature_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_feature_s;		// sum(p_new)
    Eigen::Vector3d sum_p_ref_icp_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_icp_s;		// sum(p_new)

    Eigen::Vector3d sum_p_ref_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_s;		// sum(p_new)
    Eigen::Matrix3d sum_p_ref_new_s;  // sum(p_ref * p_new^T)
    Eigen::Matrix3d sum_p_ref_ref_s;  // sum(p_ref * p_ref^T)
    Eigen::Matrix3d sum_p_new_new_s;  // sum(p_new * p_new^T)
    Eigen::Matrix3d sum_p_new_ref_s;  // sum(p_new * p_ref^T)
    // to calculate reprojection error
    float sum_feature_p_ref_T_p_ref_s, sum_feature_p_new_T_p_new_s;
    float sum_icp_p_ref_T_p_ref_s, sum_icp_p_new_T_p_new_s;
    double sum_weight_s;    
    NormalCorrespondence(bool _type, int _ref_camera_id, int _ref_submap_id, 
        int _new_camera_id, int _new_submap_id, const Eigen::Vector3f & _ref_normal, const Eigen::Vector3f & _new_normal )
    {
        type = _type;
        if(type == 0)
        {
            ref_submap_id = _ref_submap_id;
            new_submap_id = _new_submap_id;
        }
        else if(type == 1)
        {
            ref_room_id = _ref_submap_id;
            new_room_id = _new_submap_id;
        }

        ref_camera_id = _ref_camera_id;
        new_camera_id = _new_camera_id;
        ref_normal = _ref_normal.cast<double>();
        new_normal = _new_normal.cast<double>();
        if(_ref_normal(0) != 999.0  && _new_normal(0) != 999.0)
        {
            is_valid = true;
        }
    }
    void preIntegrate(const std::vector<PoseSE3dList> &submap_poses_relative_changes, const std::vector<PoseSE3dList> &room_poses)
    {
        sum_p_ref_s.setZero();
        sum_p_new_s.setZero();
        sum_p_ref_new_s.setZero();
        sum_p_ref_ref_s.setZero();
        sum_p_new_new_s.setZero();
        sum_p_new_ref_s.setZero();
        sum_weight_s = 0;
        double location_weight = 1;
        int use_weight = 0;
        /*std::cout<<"R_ref:"<<R_ref<<std::endl;
        std::cout<<"t_ref:"<<t_ref<<std::endl;
        std::cout<<"R_new:"<<R_new<<std::endl;
        std::cout<<"t_new:"<<t_new<<std::endl;*/
        sum_feature_p_ref_T_p_ref_s = 0;
        sum_feature_p_new_T_p_new_s = 0;
        sum_icp_p_ref_T_p_ref_s = 0;
        sum_icp_p_new_T_p_new_s = 0;
        
        Eigen::Vector3d s_ref_normal;
            
        Eigen::Vector3d s_new_normal; 

        if(type == 0)
        {
            s_ref_normal = submap_poses_relative_changes[ref_camera_id][ref_submap_id].rotationMatrix() * ref_normal;
            s_new_normal = submap_poses_relative_changes[new_camera_id][new_submap_id].rotationMatrix() * new_normal;
        }
        else
        {
            s_ref_normal = room_poses[ref_camera_id][ref_room_id].rotationMatrix() * ref_normal;
            s_new_normal = room_poses[new_camera_id][new_room_id].rotationMatrix() * new_normal;
        }
        float depth = ref_normal(1);

        double weight = 1 / (depth * depth) *  MultiViewGeometry::g_para.normal_weight;
        if(type == 1) weight *= 2;
        sum_weight_s += weight;
        sum_p_ref_s += s_ref_normal * weight;
        sum_p_new_s += s_new_normal * weight;
        sum_p_ref_new_s += s_ref_normal * s_new_normal.transpose() * weight;
        sum_p_ref_ref_s += s_ref_normal * s_ref_normal.transpose() * weight;
        sum_p_new_new_s += s_new_normal * s_new_normal.transpose() * weight;
        sum_p_new_ref_s += s_new_normal * s_ref_normal.transpose() * weight;

        sum_feature_p_ref_T_p_ref_s = s_ref_normal.transpose() * s_ref_normal;
        sum_feature_p_new_T_p_new_s = s_new_normal.transpose() * s_new_normal;

#if 0
        if (rtst_error > 0)
        {
            sum_weight /= rtst_error;
            sum_p_ref /= rtst_error;
            sum_p_new /= rtst_error;
            sum_p_ref_new /= rtst_error;
            sum_p_ref_ref /= rtst_error;
            sum_p_new_new /= rtst_error;
            sum_p_new_ref /= rtst_error;

        }
#endif
        sum_p_ref_feature_s = sum_p_ref_s ;
        sum_p_new_feature_s = sum_p_new_s ;
    }
    // reprojection_error need to be redefined.
    float normal_error(const PoseSE3dList &camera_start_poses, 
        const std::vector<PoseSE3dList> &tmp_submap_poses, const std::vector<PoseSE3dList> &room_poses)
    {
        /*
        Eigen::Matrix3d c_R_ref = camera_start_poses[ref_camera_id].rotationMatrix();
        Eigen::Matrix3d c_R_new = camera_start_poses[new_camera_id].rotationMatrix();

        Eigen::Matrix3d R_ref ;
        Eigen::Matrix3d R_new ;
        if(type == 0)
        {
            R_ref = tmp_submap_poses[ref_submap_id].rotationMatrix();
            R_new = tmp_submap_poses[new_submap_id].rotationMatrix();
        }
        else if(type == 1)
        {
            R_ref = room_poses[ref_room_id].rotationMatrix();
            R_new = room_poses[new_room_id].rotationMatrix();            
        }
        return (c_R_ref * R_ref *ref_normal - c_R_new * R_new * new_normal).norm();
        */
    }

    Eigen::Vector3d ref_normal;
    Eigen::Vector3d new_normal;
    int ref_submap_id = -1;
    int new_submap_id = -1;    
    int ref_camera_id;
    int new_camera_id;
    int ref_room_id = -1;
    int new_room_id = -1;
    bool type = 0;// 0 for submap, 1 for room.
    bool is_valid = false;

};

class NormalGravityCorr
{
    public: 
    Eigen::Vector3d sum_p_ref_feature_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_feature_s;		// sum(p_new)
    Eigen::Vector3d sum_p_ref_icp_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_icp_s;		// sum(p_new)

    Eigen::Vector3d sum_p_ref_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_s;		// sum(p_new)
    Eigen::Matrix3d sum_p_ref_new_s;  // sum(p_ref * p_new^T)
    Eigen::Matrix3d sum_p_ref_ref_s;  // sum(p_ref * p_ref^T)
    Eigen::Matrix3d sum_p_new_new_s;  // sum(p_new * p_new^T)
    Eigen::Matrix3d sum_p_new_ref_s;  // sum(p_new * p_ref^T)
    // to calculate reprojection error
    float sum_feature_p_ref_T_p_ref_s, sum_feature_p_new_T_p_new_s;
    float sum_icp_p_ref_T_p_ref_s, sum_icp_p_new_T_p_new_s;
    double sum_weight_s;    
    NormalGravityCorr(bool _type, int _camera_id, int _submap_id, 
        const Eigen::Vector3f & _normal )
    {
        type = _type;
        if(type == 0)
        {
            submap_id = _submap_id;

        }
        else if(type == 1)
        {
            room_id = _submap_id;
        }

        camera_id = _camera_id;

        normal = _normal.cast<double>();

        if(_normal(0) != 999.0)
        {
            is_valid = true;
        }
    }
    void preIntegrate(const std::vector<PoseSE3dList> &submap_poses_relative_changes, const std::vector<PoseSE3dList> &room_poses)
    {
        sum_p_ref_s.setZero();
        sum_p_new_s.setZero();
        sum_p_ref_new_s.setZero();
        sum_p_ref_ref_s.setZero();
        sum_p_new_new_s.setZero();
        sum_p_new_ref_s.setZero();
        sum_weight_s = 0;
        double location_weight = 1;
        int use_weight = 0;
        /*std::cout<<"R_ref:"<<R_ref<<std::endl;
        std::cout<<"t_ref:"<<t_ref<<std::endl;
        std::cout<<"R_new:"<<R_new<<std::endl;
        std::cout<<"t_new:"<<t_new<<std::endl;*/
        sum_feature_p_ref_T_p_ref_s = 0;
        sum_feature_p_new_T_p_new_s = 0;
        sum_icp_p_ref_T_p_ref_s = 0;
        sum_icp_p_new_T_p_new_s = 0;
        
        Eigen::Vector3d s_normal;
            


        if(type == 0)
        {
            s_normal = submap_poses_relative_changes[camera_id][submap_id].rotationMatrix() * normal;

        }
        else
        {
            s_normal = room_poses[camera_id][room_id].rotationMatrix() * normal;

        }
        float depth = s_normal(1);

        double weight = 1 / (depth * depth) * MultiViewGeometry::g_para.normal_weight ;
        sum_weight_s += weight;
        sum_p_ref_s += s_normal * weight;
        sum_p_new_s += Gravity * weight;
        sum_p_ref_new_s += s_normal * Gravity.transpose() * weight;
        sum_p_ref_ref_s += s_normal * s_normal.transpose() * weight;
        sum_p_new_new_s += Gravity * Gravity.transpose() * weight;
        sum_p_new_ref_s += Gravity * s_normal.transpose() * weight;

        sum_feature_p_ref_T_p_ref_s = s_normal.transpose() * s_normal;
        sum_feature_p_new_T_p_new_s = Gravity.transpose() * Gravity;

#if 0
        if (rtst_error > 0)
        {
            sum_weight /= rtst_error;
            sum_p_ref /= rtst_error;
            sum_p_new /= rtst_error;
            sum_p_ref_new /= rtst_error;
            sum_p_ref_ref /= rtst_error;
            sum_p_new_new /= rtst_error;
            sum_p_new_ref /= rtst_error;

        }
#endif
        sum_p_ref_feature_s = sum_p_ref_s ;
        sum_p_new_feature_s = sum_p_new_s ;
    }
    // reprojection_error need to be redefined.

    Eigen::Vector3d normal;

    int submap_id = -1; 
    int camera_id;
    int room_id = -1;
    bool type = 0;// 0 for submap, 1 for room.
    bool is_valid = false;
};

};
#endif