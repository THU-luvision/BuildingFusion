#ifndef SERVER_CORRESPONDENCE_H
#define SERVER_CORRESPONDENCE_H

#include "ServerFrame.h"
#include "../GCSLAM/MultiViewGeometry.h"

namespace Server 
{
    class Correspondence
    {
        public:
        Correspondence(Server::ServerFrame &f_r, Server::ServerFrame &f_n):frame_ref(f_r),frame_new(f_n)
        {
            matches.clear();
            data_pairs_3d.clear();
            sameSubmap = (f_r.submapID == f_n.submapID);
            sameCamera = (f_r.cameraID == f_n.cameraID);
            sum_p_ref_s.setZero();
            sum_p_new_s.setZero();
            sum_p_ref_new_s.setZero();
            sum_p_ref_ref_s.setZero();
            sum_p_new_new_s.setZero();
            sum_p_new_ref_s.setZero();
            sum_weight_s = 0;      

            sum_p_ref_c.setZero();
            sum_p_new_c.setZero();
            sum_p_ref_new_c.setZero();
            sum_p_ref_ref_c.setZero();
            sum_p_new_new_c.setZero();
            sum_p_new_ref_c.setZero();
            sum_weight_c = 0;    

            sum_p_ref.setZero();
            sum_p_new.setZero();
            sum_p_ref_new.setZero();
            sum_p_ref_ref.setZero();
            sum_p_new_new.setZero();
            sum_p_new_ref.setZero();
            sum_weight = 0;    

            sum_feature_p_ref_T_p_ref_s = 0;
            sum_feature_p_new_T_p_new_s = 0;
            sum_icp_p_ref_T_p_ref_s = 0;
            sum_icp_p_new_T_p_new_s = 0;

            sum_feature_p_ref_T_p_ref_c = 0;
            sum_feature_p_new_T_p_new_c = 0;
            sum_icp_p_ref_T_p_ref_c = 0;
            sum_icp_p_new_T_p_new_c = 0;

            sum_feature_p_ref_T_p_ref = 0;
            sum_feature_p_new_T_p_new = 0;
            sum_icp_p_ref_T_p_ref = 0;
            sum_icp_p_new_T_p_new = 0;

        }
        void reset()
        {
            dense_feature_cnt = 0;
            sparse_feature_cnt = 0;
            sum_weight_s = 0;
            matches.clear();
            data_pairs_3d.clear();
            weight_per_feature.clear();
            weight_per_pair.clear();
            sum_p_ref_s.setZero();
            sum_p_new_s.setZero();
            sum_p_ref_new_s.setZero();
            sum_p_ref_ref_s.setZero();
            sum_p_new_new_s.setZero();
            sum_p_new_ref_s.setZero();

            sum_p_ref_c.setZero();
            sum_p_new_c.setZero();
            sum_p_ref_new_c.setZero();
            sum_p_ref_ref_c.setZero();
            sum_p_new_new_c.setZero();
            sum_p_new_ref_c.setZero();
        }
        ~Correspondence()
        {
            reset();
        }
    float calculate_average_disparity(MultiViewGeometry::CameraPara camera_para)
    {
      float average_disparity = 0;
      if(data_pairs_3d.size() > 0)
      {
        for(int k = 0; k < data_pairs_3d.size(); k++)
        {
          Eigen::Vector3d p0 = data_pairs_3d[k].first.cast<double>();
          Eigen::Vector3d p1 = data_pairs_3d[k].second.cast<double>();

          float ref_x = p0(0)/p0(2) * camera_para.c_fx + camera_para.c_cx;
          float ref_y = p0(1)/p0(2) * camera_para.c_fy + camera_para.c_cy;

          float new_x = p1(0)/p1(2) * camera_para.c_fx + camera_para.c_cx;
          float new_y = p1(1)/p1(2) * camera_para.c_fy + camera_para.c_cy;
          average_disparity += sqrt((ref_x - new_x) * (ref_x - new_x) + (ref_y - new_y)*(ref_y - new_y));
        }
        average_disparity /= (data_pairs_3d.size() + 1);
        average_disparity /= fmax(camera_para.c_fx,camera_para.c_fy);
      }
      else 
      {
        for (int i = 0; i < matches.size(); i++)
        {
          Eigen::Vector3d p0 = frame_ref.getLocalPoints()[matches[i].queryIdx];
          Eigen::Vector3d p1 = frame_new.getLocalPoints()[matches[i].trainIdx];

          float ref_x = p0(0)/p0(2) * camera_para.c_fx + camera_para.c_cx;
          float ref_y = p0(1)/p0(2) * camera_para.c_fy + camera_para.c_cy;

          float new_x = p1(0)/p1(2) * camera_para.c_fx + camera_para.c_cx;
          float new_y = p1(1)/p1(2) * camera_para.c_fy + camera_para.c_cy;

          /*
          So this is the rejection error.
          */
          average_disparity += sqrt((ref_x - new_x) * (ref_x - new_x) + (ref_y - new_y)*(ref_y - new_y));
        }
        average_disparity /= (matches.size() + 1);
        average_disparity /= fmax(camera_para.c_fx,camera_para.c_fy);
      }
      return average_disparity;
    }

    void printSum()
    {
        std::cout<<"frame ref:"<<frame_ref.frame_index<<"\n"<<frame_ref.pose_sophus[2].matrix()<<std::endl;
        std::cout<<"frame new:"<<frame_new.frame_index<<"\n"<<frame_new.pose_sophus[2].matrix()<<std::endl;
       
    }
    void preIntegrateICPInterSubmap(float icp_weight)
        {

            int pairMatchNum = data_pairs_3d.size();
            dense_feature_cnt = data_pairs_3d.size();

            /* reset the sum */
            sum_weight_s = 0;
            sum_p_ref_s.setZero();
            sum_p_new_s.setZero();
            sum_p_ref_new_s.setZero();
            sum_p_ref_ref_s.setZero();
            sum_p_new_new_s.setZero();
            sum_p_new_ref_s.setZero();

            Eigen::Matrix3d R_ref = frame_ref.pose_sophus[2].rotationMatrix();
            Eigen::Vector3d t_ref = frame_ref.pose_sophus[2].translation();
            Eigen::Matrix3d R_new = frame_new.pose_sophus[2].rotationMatrix();
            Eigen::Vector3d t_new = frame_new.pose_sophus[2].translation();    
            for (int i = 0; i < pairMatchNum; i++)
            {

                Eigen::Vector3d p0 = R_ref *data_pairs_3d[i].first.cast<double>() + t_ref;
                Eigen::Vector3d p1 = R_new *data_pairs_3d[i].second.cast<double>() + t_new;

                float depth = p0[2];
                double weight = 1 / (depth ) * icp_weight / pairMatchNum * Truncation_ICP_POINTS ;// * weight_per_pair[i];
                sum_weight_s += weight;
                sum_p_ref_s += p0 * weight;
                sum_p_new_s += p1 * weight;
                sum_p_ref_new_s += p0 * p1.transpose() * weight;
                sum_p_ref_ref_s += p0 * p0.transpose() * weight;
                sum_p_new_new_s += p1 * p1.transpose() * weight;
                sum_p_new_ref_s += p1 * p0.transpose() * weight;


                sum_icp_p_ref_T_p_ref_s = p0.transpose() * p0;
                sum_icp_p_new_T_p_new_s = p1.transpose() * p1;
            }
            sum_p_ref_icp_s = sum_p_ref_s - sum_p_ref_feature_s;
            sum_p_new_icp_s = sum_p_new_s - sum_p_new_feature_s;

        }
    void preIntegrateICPInterCamera(float icp_weight)
        {

            int pairMatchNum = data_pairs_3d.size();
            dense_feature_cnt = data_pairs_3d.size();

            /* reset the sum */
            sum_weight_c = 0;
            sum_p_ref_c.setZero();
            sum_p_new_c.setZero();
            sum_p_ref_new_c.setZero();
            sum_p_ref_ref_c.setZero();
            sum_p_new_new_c.setZero();
            sum_p_new_ref_c.setZero();

            Eigen::Matrix3d R_ref = frame_ref.pose_sophus[3].rotationMatrix();
            Eigen::Vector3d t_ref = frame_ref.pose_sophus[3].translation();
            Eigen::Matrix3d R_new = frame_new.pose_sophus[3].rotationMatrix();
            Eigen::Vector3d t_new = frame_new.pose_sophus[3].translation();    
            for (int i = 0; i < pairMatchNum; i++)
            {

                Eigen::Vector3d p0 = R_ref *data_pairs_3d[i].first.cast<double>() + t_ref;
                Eigen::Vector3d p1 = R_new *data_pairs_3d[i].second.cast<double>() + t_new;

                float depth = p0[2];
                double weight = 1 / (depth ) * icp_weight / pairMatchNum * Truncation_ICP_POINTS ;// * weight_per_pair[i];
                sum_weight_c += weight;
                sum_p_ref_c += p0 * weight;
                sum_p_new_c += p1 * weight;
                sum_p_ref_new_c += p0 * p1.transpose() * weight;
                sum_p_ref_ref_c += p0 * p0.transpose() * weight;
                sum_p_new_new_c += p1 * p1.transpose() * weight;
                sum_p_new_ref_c += p1 * p0.transpose() * weight;


                sum_icp_p_ref_T_p_ref_c = p0.transpose() * p0;
                sum_icp_p_new_T_p_new_c = p1.transpose() * p1;
            }
            sum_p_ref_icp_c = sum_p_ref_c - sum_p_ref_feature_c;
            sum_p_new_icp_c = sum_p_new_c - sum_p_new_feature_c;

        }
        void preIntegrateInterCamera()
        {
                sum_p_ref_c.setZero();
                sum_p_new_c.setZero();
                sum_p_ref_new_c.setZero();
                sum_p_ref_ref_c.setZero();
                sum_p_new_new_c.setZero();
                sum_p_new_ref_c.setZero();
                sum_weight_c = 0;
                double location_weight = 1;
                int use_weight = 0;

                Eigen::Matrix3d R_ref = frame_ref.pose_sophus[3].rotationMatrix();
                Eigen::Vector3d t_ref = frame_ref.pose_sophus[3].translation();
                Eigen::Matrix3d R_new = frame_new.pose_sophus[3].rotationMatrix();
                Eigen::Vector3d t_new = frame_new.pose_sophus[3].translation();
                sum_feature_p_ref_T_p_ref_c = 0;
                sum_feature_p_new_T_p_new_c = 0;
                sum_icp_p_ref_T_p_ref_c = 0;
                sum_icp_p_new_T_p_new_c = 0;
                Point3dList local_points_ref(frame_ref.getLocalPoints());
                Point3dList local_points_new(frame_new.getLocalPoints());
                for(int i = 0;i!=local_points_ref.size();++i)
                {
                    local_points_ref[i] = R_ref* local_points_ref[i] + t_ref;
                }
                for(int i = 0;i!=local_points_new.size();++i)
                {
                    local_points_new[i] = R_new * local_points_new[i] + t_new;
                }
                if (weight_per_feature.size() == matches.size())
                {
                    use_weight = 1;
                }
                sparse_feature_cnt = matches.size();

                for (int i = 0; i < matches.size(); i++)
                {
                    float depth = frame_ref.getLocalPoints()[matches[i].queryIdx][2];

                    double weight = 1 / (depth * depth);
                    if (use_weight)
                    {
                        weight = weight * weight_per_feature[i];
                    }
                    if(connection)
                    weight *= 5;
                    sum_weight_c += weight;
                    sum_p_ref_c += local_points_ref[matches[i].queryIdx] * weight;
                    sum_p_new_c += local_points_new[matches[i].trainIdx] * weight;
                    sum_p_ref_new_c += local_points_ref[matches[i].queryIdx] * local_points_new[matches[i].trainIdx].transpose() * weight;
                    sum_p_ref_ref_c += local_points_ref[matches[i].queryIdx] * local_points_ref[matches[i].queryIdx].transpose() * weight;
                    sum_p_new_new_c += local_points_new[matches[i].trainIdx] * local_points_new[matches[i].trainIdx].transpose() * weight;
                    sum_p_new_ref_c += local_points_new[matches[i].trainIdx] * local_points_ref[matches[i].queryIdx].transpose() * weight;

                    sum_feature_p_ref_T_p_ref_c = local_points_ref[matches[i].queryIdx].transpose() * local_points_ref[matches[i].queryIdx];
                    sum_feature_p_new_T_p_new_c = local_points_new[matches[i].trainIdx].transpose() * local_points_new[matches[i].trainIdx];
                }
                sum_p_ref_feature_c = sum_p_ref_c ;
                sum_p_new_feature_c = sum_p_new_c ;            
        }
        void preIntegrate()
        {
            double location_weight = 1;
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
            if (weight_per_feature.size() == matches.size())
            {
                use_weight = 1;
            }
            sparse_feature_cnt = matches.size();
            for (int i = 0; i < matches.size(); i++)
            {
                float depth = frame_ref.getLocalPoints()[matches[i].queryIdx][2];

                double weight = 1 / (depth * depth);
                if (use_weight)
                {
                    weight = weight * weight_per_feature[i];
                }
                sum_weight += weight;
                sum_p_ref += frame_ref.getLocalPoints()[matches[i].queryIdx] * weight;
                sum_p_new += frame_new.getLocalPoints()[matches[i].trainIdx] * weight;
                sum_p_ref_new += frame_ref.getLocalPoints()[matches[i].queryIdx] * frame_new.getLocalPoints()[matches[i].trainIdx].transpose() * weight;
                sum_p_ref_ref += frame_ref.getLocalPoints()[matches[i].queryIdx] * frame_ref.getLocalPoints()[matches[i].queryIdx].transpose() * weight;
                sum_p_new_new += frame_new.getLocalPoints()[matches[i].trainIdx] * frame_new.getLocalPoints()[matches[i].trainIdx].transpose() * weight;
                sum_p_new_ref += frame_new.getLocalPoints()[matches[i].trainIdx] * frame_ref.getLocalPoints()[matches[i].queryIdx].transpose() * weight;

                sum_feature_p_ref_T_p_ref = frame_ref.getLocalPoints()[matches[i].queryIdx].transpose() * frame_ref.getLocalPoints()[matches[i].queryIdx];
                sum_feature_p_new_T_p_new = frame_new.getLocalPoints()[matches[i].trainIdx].transpose() * frame_new.getLocalPoints()[matches[i].trainIdx];
            }
            sum_p_ref_feature = sum_p_ref ;
            sum_p_new_feature = sum_p_new ;
        }  

        void preIntegrateICP(float icp_weight)
        {

            int pairMatchNum = data_pairs_3d.size();
            dense_feature_cnt = data_pairs_3d.size();

/* reset the sum */
            sum_weight = 0;
            sum_p_ref.setZero();
            sum_p_new.setZero();
            sum_p_ref_new.setZero();
            sum_p_ref_ref.setZero();
            sum_p_new_new.setZero();
            sum_p_new_ref.setZero();
            for (int i = 0; i < pairMatchNum; i++)
            {

                Eigen::Vector3d p0 = data_pairs_3d[i].first.cast<double>();
                Eigen::Vector3d p1 = data_pairs_3d[i].second.cast<double>();

                float depth = p0[2];
                double weight = 1 / (depth ) * icp_weight / pairMatchNum * Truncation_ICP_POINTS ;// * weight_per_pair[i];

                sum_weight += weight;
                sum_p_ref += p0 * weight;
                sum_p_new += p1 * weight;
                sum_p_ref_new += p0 * p1.transpose() * weight;
                sum_p_ref_ref += p0 * p0.transpose() * weight;
                sum_p_new_new += p1 * p1.transpose() * weight;
                sum_p_new_ref += p1 * p0.transpose() * weight;


                sum_icp_p_ref_T_p_ref = p0.transpose() * p0;
                sum_icp_p_new_T_p_new = p1.transpose() * p1;
            }
            sum_p_ref_icp = sum_p_ref ;
            sum_p_new_icp = sum_p_new ;
                    
        }
        void preIntegrateInterSubmap()
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

                Eigen::Matrix3d R_ref = frame_ref.pose_sophus[2].rotationMatrix();
                Eigen::Vector3d t_ref = frame_ref.pose_sophus[2].translation();
                Eigen::Matrix3d R_new = frame_new.pose_sophus[2].rotationMatrix();
                Eigen::Vector3d t_new = frame_new.pose_sophus[2].translation();
                sum_feature_p_ref_T_p_ref_s = 0;
                sum_feature_p_new_T_p_new_s = 0;
                sum_icp_p_ref_T_p_ref_s = 0;
                sum_icp_p_new_T_p_new_s = 0;
                Point3dList local_points_ref(frame_ref.getLocalPoints());
                Point3dList local_points_new(frame_new.getLocalPoints());
                for(int i = 0;i!=local_points_ref.size();++i)
                {
                    local_points_ref[i] = R_ref* local_points_ref[i] + t_ref;
                }
                for(int i = 0;i!=local_points_new.size();++i)
                {
                    local_points_new[i] = R_new * local_points_new[i] + t_new;
                }
                if (weight_per_feature.size() == matches.size())
                {
                    use_weight = 1;
                }
                sparse_feature_cnt = matches.size();

                for (int i = 0; i < matches.size(); i++)
                {
                    float depth = frame_ref.getLocalPoints()[matches[i].queryIdx][2];

                    double weight = 1 / (depth * depth);
                    if (use_weight)
                    {
                        weight = weight * weight_per_feature[i];
                    }
                    sum_weight_s += weight;
                    sum_p_ref_s += local_points_ref[matches[i].queryIdx] * weight;
                    sum_p_new_s += local_points_new[matches[i].trainIdx] * weight;
                    sum_p_ref_new_s += local_points_ref[matches[i].queryIdx] * local_points_new[matches[i].trainIdx].transpose() * weight;
                    sum_p_ref_ref_s += local_points_ref[matches[i].queryIdx] * local_points_ref[matches[i].queryIdx].transpose() * weight;
                    sum_p_new_new_s += local_points_new[matches[i].trainIdx] * local_points_new[matches[i].trainIdx].transpose() * weight;
                    sum_p_new_ref_s += local_points_new[matches[i].trainIdx] * local_points_ref[matches[i].queryIdx].transpose() * weight;

                    sum_feature_p_ref_T_p_ref_s = local_points_ref[matches[i].queryIdx].transpose() * local_points_ref[matches[i].queryIdx];
                    sum_feature_p_new_T_p_new_s = local_points_new[matches[i].trainIdx].transpose() * local_points_new[matches[i].trainIdx];
                }
                sum_p_ref_feature_s = sum_p_ref_s ;
                sum_p_new_feature_s = sum_p_new_s ;
        }
        void setICP(bool _icp)
        {
            use_icp = _icp;
        }
        void setIsValid(bool _is_valid)
        {
            if(connection) return;
            is_valid = _is_valid;
        }
        void setConnection(bool _connection)
        {
            connection = _connection;
            if(connection) 
            is_valid = true;
        }
        Server::ServerFrame & frame_ref;
        Server::ServerFrame & frame_new;

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


        Eigen::Vector3d sum_p_ref_feature_c;		// sum(p_ref)
        Eigen::Vector3d sum_p_new_feature_c;		// sum(p_new)
        Eigen::Vector3d sum_p_ref_icp_c;		// sum(p_ref)
        Eigen::Vector3d sum_p_new_icp_c;		// sum(p_new)

        Eigen::Vector3d sum_p_ref_c;		// sum(p_ref)
        Eigen::Vector3d sum_p_new_c;		// sum(p_new)
        Eigen::Matrix3d sum_p_ref_new_c;  // sum(p_ref * p_new^T)
        Eigen::Matrix3d sum_p_ref_ref_c;  // sum(p_ref * p_ref^T)
        Eigen::Matrix3d sum_p_new_new_c;  // sum(p_new * p_new^T)
        Eigen::Matrix3d sum_p_new_ref_c;  // sum(p_new * p_ref^T)

        Eigen::Vector3d sum_p_ref_feature;		// sum(p_ref)
        Eigen::Vector3d sum_p_new_feature;		// sum(p_new)
        Eigen::Vector3d sum_p_ref_icp;		// sum(p_ref)
        Eigen::Vector3d sum_p_new_icp;		// sum(p_new)

        Eigen::Vector3d sum_p_ref;		// sum(p_ref)
        Eigen::Vector3d sum_p_new;		// sum(p_new)
        Eigen::Matrix3d sum_p_ref_new;  // sum(p_ref * p_new^T)
        Eigen::Matrix3d sum_p_ref_ref;  // sum(p_ref * p_ref^T)
        Eigen::Matrix3d sum_p_new_new;  // sum(p_new * p_new^T)
        Eigen::Matrix3d sum_p_new_ref;  // sum(p_new * p_ref^T)        

        // to calculate reprojection error
        std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > data_pairs_3d;
        float sum_feature_p_ref_T_p_ref_s, sum_feature_p_new_T_p_new_s;
        float sum_icp_p_ref_T_p_ref_s, sum_icp_p_new_T_p_new_s;

        float sum_feature_p_ref_T_p_ref_c, sum_feature_p_new_T_p_new_c;
        float sum_icp_p_ref_T_p_ref_c, sum_icp_p_new_T_p_new_c;

        float sum_feature_p_ref_T_p_ref, sum_feature_p_new_T_p_new;
        float sum_icp_p_ref_T_p_ref, sum_icp_p_new_T_p_new;

        bool use_icp = false;
        double sum_weight_s;
        double sum_weight_c;
        double sum_weight;
        bool allowance_icp = false;
        std::vector< cv::DMatch > matches;	// query for ref, train for new
        std::vector<float> weight_per_feature; //weight per feature
        std::vector<float> weight_per_pair;
        int sparse_feature_cnt=0;
        int dense_feature_cnt =0;
        int plane_corr_cnt;
        bool sameSubmap = false;
        bool sameCamera = false;    
        bool is_valid=true;
        bool connection = false;
    };
};
#endif