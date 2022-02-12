#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "ServerSLAM.h"
#include "Correspondence.h"
#include "../GCSLAM/MultiViewGeometry.h"
#include "NormalCorrespondence.h"
namespace Server 
{
  bool FrameMatchingTwoViewRGB(Correspondence &fCorr,
                               MultiViewGeometry::CameraPara para,
                               MILD::SparseMatcher &frame_new_matcher,
                               PoseSE3d &relative_pose_from_ref_to_new, float &average_disparity, float &scale_change_ratio, bool &update_keyframe_from_dense_matching, int local_track_cnt,bool use_initial_guess = 0,bool debug_mode = false);
  float optimizeSubmapCamera(Server::ServerSLAM &server_slam,int camera_id);
  
  bool optimizeRoomAndSubmapCamera(Server::ServerSLAM &server_slam,int camera_id);
  //To flatten the floor, we try to align the floor normal.
  float combineOptimization(Server::ServerSLAM &server_slam,int camera_id, bool from_frame = true, bool with_normal = false);

  float reprojection_error_3Dto3D(const Correspondence &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new);
  float reprojection_error_3Dto3D_submap_camera(const Correspondence &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new);
  float reprojection_error_3Dto3D_submap_camera(std::vector<Server::Correspondence> &fCList,Server::ServerSLAM &server_slam);
  float reprojection_error_3Dto3D_submap_camera(std::vector<Correspondence> &fCList,Server::ServerSLAM &server_slam);// calculate jacobian matrix of 3d point based on pose
  void ComputeJacobianInfoRoomSubmap(Server::Correspondence &fC,
    ServerSLAM &server_slam,
    std::map<int, std::vector<int> > & room_to_pos,
    std::map<int, std::vector<int> > & submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList &submap_poses_delta,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj);
  void ComputeJacobianInfoRoom(RoomCorrespondence &fC, 
    ServerSLAM &server_slam,
    std::map<int, std::vector<int>> &room_to_pos, 
    PoseSE3dList &room_poses,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj);
void ComputeJacobianInfoSubmapNormal(NormalCorrespondence &SC,
    ServerSLAM &server_slam,
    std::map<int, std::vector<int> > & room_to_pos,
    std::map<int, std::vector<int> > & submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList &submap_poses_delta,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj);
  float reprojection_error_3Dto3D_room(RoomCorrespondence &rC, const Sophus::SE3d & relative_pose_from_ref_to_new);
  float reprojection_error_3Dto3D_room(RoomCorrespondence &rC,     
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    PoseSE3dList & room_poses);
  float reprojection_error_3Dto3D_room_submap(Server::Correspondence &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new);
  float reprojection_error_3Dto3D_room_submap(Server::Correspondence &fC, 
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList & submap_poses_delta);
  float reprojection_error_3Dto3D_room_submap(std::vector<RoomCorrespondence> &room_clist,
    std::vector<Server::Correspondence> &fCList, 
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses, PoseSE3dList &submap_poses_delta);

  float reprojection_error_normal(Server::NormalCorrespondence &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new);
  float reprojection_error_normal(Server::NormalCorrespondence &fC, 
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList & submap_poses_delta);
  float reprojection_error_normal(std::vector<NormalCorrespondence> &room_clist,
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses, PoseSE3dList &submap_poses_delta);

  float reprojection_error_normal_gravity(Server::NormalGravityCorr &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new);
  float reprojection_error_normal_gravity(Server::NormalGravityCorr &fC, 
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList & submap_poses_delta);
  float reprojection_error_normal_gravity(std::vector<NormalGravityCorr> &room_clist,
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses, PoseSE3dList &submap_poses_delta);
    
  inline __m128 CrossProduct(__m128 a, __m128 b)
  {
    return _mm_sub_ps(
      _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 0, 2, 1)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 1, 0, 2))),
      _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 1, 0, 2)), _mm_shuffle_ps(b, b, _MM_SHUFFLE(3, 0, 2, 1)))
    );
  }



  inline void ComputeThreeMaxima(std::vector<std::vector<int>> &histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;

        for (int i = 0; i<L; i++)
        {
            const int s = histo[i].size();
            if (s>max1)
            {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            }
            else if (s>max2)
            {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            }
            else if (s>max3)
            {
                max3 = s;
                ind3 = i;
            }
        }

        if (max2<0.1f*(float)max1)
        {
            ind2 = -1;
            ind3 = -1;
        }
        else if (max3<0.1f*(float)max1)
        {
            ind3 = -1;
        }
    }
  inline void RefineByRotation(ServerFrame & frame_ref, ServerFrame & frame_new, std::vector< cv::DMatch > &init_matches)
    {
        // refine results by orientation

        int angle_interval = 20;
        int histo_length = round(360.0 / angle_interval);
    std::vector<std::vector<int>> rotHist(histo_length);
        const float factor = 1.0f / angle_interval;
        for (int i = 0; i<histo_length; i++)
            rotHist[i].reserve(500);

        for (int i = 0; i < init_matches.size(); i++)
        {
            float rot = frame_ref.getKeypoints()[init_matches[i].queryIdx].angle - frame_new.getKeypoints()[init_matches[i].trainIdx].angle;
            if (rot<0.0)
                rot += 360.0f;
            int bin = round(rot*factor);
            if (bin == histo_length)
                bin = 0;
            //std::cout<<"rot: "<<rot<<" bin: "<<bin<<std::endl;
            assert(bin >= 0 && bin<histo_length);
            rotHist[bin].push_back(i);
        }
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist, histo_length, ind1, ind2, ind3);
    std::vector< cv::DMatch > ransac_inlier_matches;

        for (int i = 0; i<histo_length; i++)
        {
      if (fmin(abs(i - ind1), abs(histo_length - (i - ind1))) < 2 || i == ind2 || i == ind3)
            {
                for (size_t j = 0, jend = rotHist[i].size(); j<jend; j++)
                {
                    ransac_inlier_matches.push_back(init_matches[rotHist[i][j]]);
                }
            }
        }
        init_matches = ransac_inlier_matches;
    }

  // Generate vertex map in camera coordinates
  inline cv::Mat gen_vertex_map(cv::Mat &_depth_image, const MultiViewGeometry::CameraPara &_camera)
  {
      int width = _depth_image.cols;
      int height = _depth_image.rows;
      float fx = _camera.c_fx;
      float fy = _camera.c_fy;
      float cx = _camera.c_cx;
      float cy = _camera.c_cy;

      cv::Mat vertex_image(height, width, CV_32FC3);
      for (int r_idx = 0; r_idx < height; ++r_idx) {
          for (int c_idx = 0; c_idx < width; ++c_idx) {
              float depth = _depth_image.at<unsigned short>(r_idx, c_idx)/ _camera.depth_scale;
              vertex_image.at<cv::Vec3f>(r_idx, c_idx)[0] = (c_idx-cx)*depth/fx;
              vertex_image.at<cv::Vec3f>(r_idx, c_idx)[1] = (r_idx-cy)*depth/fy;
              vertex_image.at<cv::Vec3f>(r_idx, c_idx)[2] = depth;
          }
      }

      return vertex_image;
  }

  inline cv::Mat gen_normal_map(cv::Mat _vertex_image)
  {
      const int ex = 5;
      const int ey = 5;
      int width = _vertex_image.cols;
      int height = _vertex_image.rows;

      cv::Mat normal_image(height, width, CV_32FC3);
      for (int r_idx = 0; r_idx < height; ++r_idx) {
          for (int c_idx = 0; c_idx < width; ++c_idx) {
              cv::Vec3f point_cv = _vertex_image.at<cv::Vec3f>(r_idx, c_idx);
              Eigen::Vector3f point(point_cv[0], point_cv[1], point_cv[2]);

              normal_image.at<cv::Vec3f>(r_idx, c_idx)[0] = 0;
              normal_image.at<cv::Vec3f>(r_idx, c_idx)[1] = 0;
              normal_image.at<cv::Vec3f>(r_idx, c_idx)[2] = 0;
              if (point.norm() > 0  && r_idx > 0 && r_idx + 1 < height && c_idx > 0 && c_idx + 1 <  width)
              {
                  int counter = 0;
                  Eigen::Vector3f center(0, 0, 0);
                  std::vector<Eigen::Vector3f> valid_p;
                  for (int yi = r_idx-ey/2; yi <= r_idx+ey/2; ++yi) {
                      for (int xi = c_idx-ex/2; xi <= c_idx+ex/2; ++xi) {
                          if (xi >= 0 && xi < width && yi >= 0 && yi < height) {
                              cv::Vec3f p_cv = _vertex_image.at<cv::Vec3f>(yi, xi);
                              if (cv::norm(p_cv) > 0) {
                                  Eigen::Vector3f p(p_cv[0], p_cv[1], p_cv[2]);
                                  center += p;
                                  valid_p.push_back(p);
                                  ++counter;
                              }
                          }
                      }
                  }
                  center /= (float)counter;

                  Eigen::MatrixXf A(3, counter);
                  for (int idx = 0; idx < counter; ++idx) {
                      A.col(idx) = valid_p[idx] - (Eigen::Vector3f)center;
                  }

                  Eigen::Matrix3f cov = A*A.transpose();

                  Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(cov.cast<double>(), true);

                  const Eigen::Vector3d &eigen_value = eigen_solver.eigenvalues().real();
                  int min_idx = 0;
                  eigen_value.minCoeff(&min_idx);
                  Eigen::Vector3d min_eigen_vector_d = eigen_solver.eigenvectors().real().col(min_idx);
                  Eigen::Vector3f min_eigen_vector = min_eigen_vector_d.cast<float>();

                  // Use eigen std::vector facing camera as normal
                  if (min_eigen_vector.dot(point) < 0) {
                      normal_image.at<cv::Vec3f>(r_idx, c_idx)[0] = min_eigen_vector(0);
                      normal_image.at<cv::Vec3f>(r_idx, c_idx)[1] = min_eigen_vector(1);
                      normal_image.at<cv::Vec3f>(r_idx, c_idx)[2] = min_eigen_vector(2);
                  }
                  else {
                      normal_image.at<cv::Vec3f>(r_idx, c_idx)[0] = -min_eigen_vector(0);
                      normal_image.at<cv::Vec3f>(r_idx, c_idx)[1] = -min_eigen_vector(1);
                      normal_image.at<cv::Vec3f>(r_idx, c_idx)[2] = -min_eigen_vector(2);
                  }
                  for (int cnt = 0; cnt < 3; cnt++)
                  {
                      if (fabs(normal_image.at<cv::Vec3f>(r_idx, c_idx)[cnt]) > 1e8)
                      {
                          normal_image.at<cv::Vec3f>(r_idx, c_idx)[0] = 0;
                          normal_image.at<cv::Vec3f>(r_idx, c_idx)[1] = 0;
                          normal_image.at<cv::Vec3f>(r_idx, c_idx)[2] = 0;
                      }
                  }
              }
          }
      }

      return normal_image;
  }

  float initGraphHuberNorm(std::vector<Correspondence> &fCList, std::vector<ServerFrame> &F);
  Eigen::Matrix3d skewMatrixProduct(Eigen::Vector3d t1, Eigen::Vector3d t2);
  Eigen::Matrix3d getSkewSymmetricMatrix(Eigen::Vector3d t);
  template <class NumType>
  void addBlockToTriplets(std::vector<Eigen::Triplet<NumType>> &coeff, Eigen::MatrixXd b,
    int start_x,int start_y);

  void ConstructCorrespondence(ServerFrame &frame_ref, ServerFrame &frame_new, 
    const PoseSE3d &relative_transform_from_ref_to_new_, Correspondence &key_frame_corr);  
};
#endif