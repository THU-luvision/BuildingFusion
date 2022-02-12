#include "Correspondence.h"
#include "ServerOptimizer.h"
#include "../GCSLAM/ORBSLAM/ORBextractor.h"
#include <xmmintrin.h>
#include "../GCSLAM/ICPRegistration.h"

#include <smmintrin.h>
#include <time.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/eigen.hpp>
#include <Eigen/Sparse>

;

#define REPROJECTION_TH 0.02

namespace Server
{
typedef double SPARSE_MATRIX_NUM_TYPE;

  //Triplet: The first dimension and second dimension is for 2d position in JTJ and JTr
  template <class NumType>
  void addBlockToTriplets(std::vector<Eigen::Triplet<NumType>> &coeff, Eigen::MatrixXd b,
    int start_x,int start_y)
  {
    int rows = b.rows();
    int cols = b.cols();
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < cols; j++)
      {
        coeff.push_back(Eigen::Triplet<NumType>(start_x+i,start_y+j,b(i,j)));
      }
    }


  }

   void optimize_3d_to_3d_huber_filter(ServerFrame &frame_ref,
                                             ServerFrame& frame_new,
                                             std::vector< cv::DMatch > &ransac_inlier_matches,
                                             PoseSE3d &relative_pose_from_ref_to_new,
                                             std::vector<float> &weight_per_point,
                                             float outlier_threshold = 0.015,
                                             int max_iter_num = 4,
                                             float huber_threshold = 0.008)
    {

    weight_per_point.clear();
    clock_t  start,end;
        int valid_3d_cnt = ransac_inlier_matches.size();

    Eigen::MatrixXf Je(6, 1), JTJ(6, 6);
    Eigen::Vector3f err;
    Eigen::VectorXf delta(6);
    Point3dList p_ref, p_new;
        p_ref.clear();
        p_new.clear();
        p_ref.reserve(valid_3d_cnt);
        p_new.reserve(valid_3d_cnt);



    std::vector<__m128> ref_points_vectorized(valid_3d_cnt);
    std::vector<__m128> new_points_vectorized(valid_3d_cnt);
    std::vector<float> depth(valid_3d_cnt);
        for (size_t i = 0; i < valid_3d_cnt; i++)
        {

            Eigen::Vector3d pt_ref(frame_ref.getLocalPoints()[ransac_inlier_matches[i].queryIdx]);
            Eigen::Vector3d pt_new(frame_new.getLocalPoints()[ransac_inlier_matches[i].trainIdx]);
            p_ref.push_back(pt_ref);
            p_new.push_back(pt_new);

          ref_points_vectorized[i] = _mm_setr_ps(pt_ref.x(),pt_ref.y(),pt_ref.z(),1);
          new_points_vectorized[i] = _mm_setr_ps(pt_new.x(),pt_new.y(),pt_new.z(),0);
        depth[i] = pt_ref.z();
        }
//		float init_error = reprojection_error_3Dto3D(p_ref, p_new, relative_pose_from_ref_to_new, 1);
        for (int iter_cnt = 0; iter_cnt < max_iter_num; iter_cnt++)
        {

            Je.setZero();
            JTJ.setZero();


            Eigen::MatrixXd R_ref = relative_pose_from_ref_to_new.rotationMatrix();
            Eigen::Vector3d t_ref = relative_pose_from_ref_to_new.translation();
            __m128 T_ref[3];

            T_ref[0] = _mm_setr_ps(R_ref(0,0),R_ref(0,1),R_ref(0,2),t_ref(0));
            T_ref[1] = _mm_setr_ps(R_ref(1,0),R_ref(1,1),R_ref(1,2),t_ref(1));
            T_ref[2] = _mm_setr_ps(R_ref(2,0),R_ref(2,1),R_ref(2,2),t_ref(2));

            Eigen::MatrixXf J_i_sse(3,6);

            J_i_sse.setZero();
            J_i_sse(0, 0) = 1;
            J_i_sse(1, 1) = 1;
            J_i_sse(2, 2) = 1;
            __m128 res, reprojection_error_vec;
            res[3] = 0;
            reprojection_error_vec[3] = 0;
            for (int i = 0; i < valid_3d_cnt; i++)
            {

                res = _mm_add_ps(_mm_dp_ps(T_ref[1], ref_points_vectorized[i], 0xf2),
                    _mm_dp_ps(T_ref[0], ref_points_vectorized[i], 0xf1));
                res = _mm_add_ps(res, _mm_dp_ps(T_ref[2], ref_points_vectorized[i], 0xf4));
                reprojection_error_vec = _mm_sub_ps(res, new_points_vectorized[i]);

                float error_sse = sqrt(_mm_cvtss_f32(_mm_dp_ps(res, res, 0x71))) / depth[i];

                float weight_huber = 1;
                if (error_sse > huber_threshold)
                {
                  weight_huber = huber_threshold / error_sse;
                }
                float weight = weight_huber / (depth[i]);


                const __m128 scalar = _mm_set1_ps(weight);
                reprojection_error_vec = _mm_mul_ps(reprojection_error_vec, scalar);
                __m128 cross_value = CrossProduct(res,reprojection_error_vec);
                J_i_sse(0, 4) = res[2];
                J_i_sse(0, 5) = -res[1];
                J_i_sse(1, 3) = -res[2];
                J_i_sse(1, 5) = res[0];
                J_i_sse(2, 3) = res[1];
                J_i_sse(2, 4) = -res[0];
                Je(0,0) += reprojection_error_vec[0];
                Je(1,0) += reprojection_error_vec[1];
                Je(2,0) += reprojection_error_vec[2];
                Je(3,0) += cross_value[0];
                Je(4,0) += cross_value[1];
                Je(5,0) += cross_value[2];

                JTJ += J_i_sse.transpose() * J_i_sse * weight;

            }
            delta = JTJ.inverse() * Je;
            Eigen::VectorXd delta_double = delta.cast<double>();


            relative_pose_from_ref_to_new = Sophus::SE3d::exp(delta_double).inverse() * relative_pose_from_ref_to_new;
        }
        std::vector< cv::DMatch > matches_refined;
        matches_refined.reserve(valid_3d_cnt);
        for (int i = 0; i < p_ref.size(); i++)
        {
            Eigen::Vector3d reprojection_error = applyPose(relative_pose_from_ref_to_new, p_ref[i]) - (p_new[i]);



            if (reprojection_error.norm() / p_ref[i].z() < outlier_threshold)
            {

                float weight_huber = 1;
                float error = reprojection_error.norm() / p_ref[i].z();
                if (error > huber_threshold)
                {
                  weight_huber = huber_threshold / error;
                }
                matches_refined.push_back(ransac_inlier_matches[i]);
                weight_per_point.push_back(weight_huber);
            }
        }
        ransac_inlier_matches = matches_refined;
    }

  float reprojection_error_3Dto3D(const Correspondence &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new)
  {
  Eigen::MatrixXd R_ref = relative_pose_from_ref_to_new.rotationMatrix();
  Eigen::Vector3d t_ref = relative_pose_from_ref_to_new.translation();

  // pre-integration method for norm-2 distance
  float total_error = 0; 

  if (fC.sum_weight > 0)
  {
      total_error = fC.sum_p_ref_ref(0,0) + fC.sum_p_ref_ref(1,1) + fC.sum_p_ref_ref(2,2) +
          fC.sum_p_new_new(0,0) + fC.sum_p_new_new(1,1) + fC.sum_p_new_new(2,2) +
          fC.sum_weight * t_ref.transpose() * t_ref
      - 2 * (float)(t_ref.transpose() * fC.sum_p_new) + 2 * (float)(t_ref.transpose() * R_ref * fC.sum_p_ref)
      - 2 * R_ref.cwiseProduct(fC.sum_p_new_ref).sum();

      if(total_error < 0)
      {
      std::cout << "total error: " << total_error << std::endl;

      }
      else
      {
      total_error = sqrt(total_error)  / fC.sum_weight;
      }
  }
  //std::cout << "KeyFrame Index Submap: "<<fC.frame_new.frame_index <<" "<<fC.frame_ref.frame_index <<" "<<total_error<< std::endl;
  return total_error;
  }
  void estimateRigid3DTransformation(ServerFrame &frame_ref,
      ServerFrame &frame_new,
      std::vector< cv::DMatch > &init_matches,
      Eigen::Matrix3d &R, Eigen::Vector3d &t,
      float reprojection_error_threshold,
      int max_iter_num)
    {



      std::vector<cv::DMatch> matches_before_filtering = init_matches;
      // random 100 times test
      int N_predict_inliers = init_matches.size();
      int N_total = matches_before_filtering.size();
      Eigen::Vector3d ref_points[4], mean_ref_points;
      Eigen::Vector3d new_points[4], mean_new_points;
      int candidate_seed;
      Eigen::Matrix3d H, UT, V;
      Eigen::Matrix3d temp_R;
      Eigen::Vector3d temp_t;
      int count;
      int best_results = 0;
      float reprojection_error_threshold_square = reprojection_error_threshold *reprojection_error_threshold;
          //	[R t] * [x,1]  3x4 * 4xN
      std::vector<__m128> ref_points_vectorized(N_total);
      std::vector<__m128> new_points_vectorized(N_total);
      std::vector<float> depth_square(N_total);
      for (size_t i = 0; i < N_total; i++)
      {
        Eigen::Vector3d ref_point = frame_ref.getLocalPoints()[matches_before_filtering[i].queryIdx];
        Eigen::Vector3d new_point = frame_new.getLocalPoints()[matches_before_filtering[i].trainIdx];
        ref_points_vectorized[i] = _mm_setr_ps((float)ref_point[0],(float)ref_point[1],(float)ref_point[2],1);
        new_points_vectorized[i] = _mm_setr_ps((float)new_point[0],(float)new_point[1],(float)new_point[2],1);
        depth_square[i] = ref_point[2] * ref_point[2];
      }
      Eigen::MatrixXf rigid_transform(4, 3);
      for (int cnt = 0; cnt < max_iter_num; cnt++)
      {
        H.setZero();
        mean_ref_points.setZero();
        mean_new_points.setZero();
        for (int i = 0; i < 4; i++)
        {
          candidate_seed = rand() % N_predict_inliers;
          ref_points[i] = frame_ref.getLocalPoints()[init_matches[candidate_seed].queryIdx];
          new_points[i] = frame_new.getLocalPoints()[init_matches[candidate_seed].trainIdx];
          mean_ref_points += ref_points[i];
          mean_new_points += new_points[i];
        }
        mean_ref_points /= 4;
        mean_new_points /= 4;
        for (int i = 0; i < 4; i++)
        {
          H += (ref_points[i] - mean_ref_points) * (new_points[i] - mean_new_points).transpose();
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        UT = svd.matrixU().transpose();
        V = svd.matrixV();
        temp_R = V * UT;
        if (temp_R.determinant() < 0)
        {
          V(0, 2) = -V(0, 2);
          V(1, 2) = -V(1, 2);
          V(2, 2) = -V(2, 2);
          temp_R = V * UT;
        }
        temp_t = mean_new_points - temp_R * mean_ref_points;
        Eigen::MatrixXd transformation(3,4);

        unsigned char mask0 = 0xf1;
        unsigned char mask1 = 0xf2;
        unsigned char mask2 = 0xf4;
        unsigned char mask3 = 0xf8;
        __m128 transform_vectorized[4];
        for (int i = 0; i < 3; i++)
        {
          transform_vectorized[i] = _mm_setr_ps( (float)temp_R(i, 0), (float)temp_R(i, 1), (float)temp_R(i, 2), (float)temp_t(i));
        }
        count = 0;
        for (size_t i = 0; i < N_total; i++)
        {
          __m128 res;
          res = _mm_add_ps(_mm_dp_ps(transform_vectorized[1], ref_points_vectorized[i], 0xf2),
              _mm_dp_ps(transform_vectorized[0], ref_points_vectorized[i], 0xf1));
          res = _mm_add_ps(res, _mm_dp_ps(transform_vectorized[2], ref_points_vectorized[i], 0xf4));
          res = _mm_sub_ps(res, new_points_vectorized[i]);
          count += (_mm_cvtss_f32(_mm_dp_ps(res, res, 0x71)) / depth_square[i] < reprojection_error_threshold_square);
        }
        if (count > best_results)
        {
          best_results = count;
          R = temp_R;
          t = temp_t;
        }

      }
    }



  float ransac3D3D(ServerFrame &frame_ref, ServerFrame &frame_new, std::vector< cv::DMatch > &init_matches, std::vector< cv::DMatch > &matches_before_filtering,
      float reprojectionThreshold, int max_iter_num, Correspondence &fCorr,
      PoseSE3d &relative_pose_from_ref_to_new, MultiViewGeometry::CameraPara para)
    {

      if (init_matches.size() < minimum_3d_correspondence)
      {
        return 1e7;
      }
      clock_t start, end;
      clock_t start_total, end_total;

      start_total = clock();
      start = clock();
      Eigen::Matrix3d direct_R;
      Eigen::Vector3d direct_t;
      estimateRigid3DTransformation(frame_ref, frame_new, init_matches, direct_R, direct_t, reprojectionThreshold * 2, max_iter_num);
      end = clock();
      float time_ransac = end - start;
      start = clock();
      std::vector< cv::DMatch > ransac_inlier_matches;
      std::vector< cv::DMatch > ransac_2d_inlier_matches;
      ransac_inlier_matches.clear();
      ransac_inlier_matches.reserve(matches_before_filtering.size());
      int previous_query_index = -1;
      for (size_t i = 0; i < matches_before_filtering.size(); i++)
      {
        Eigen::Vector3d ref_point = frame_ref.getLocalPoints()[matches_before_filtering[i].queryIdx];
        Eigen::Vector3d new_point = frame_new.getLocalPoints()[matches_before_filtering[i].trainIdx];

        Eigen::Vector3d estimate_new_local_points = direct_R * ref_point + direct_t;
        Eigen::Vector3d normalized_predict_points = estimate_new_local_points / estimate_new_local_points(2);
        cv::KeyPoint predict_new_2D = frame_ref.getKeypoints()[matches_before_filtering[i].queryIdx];
        cv::KeyPoint new_2D = frame_new.getKeypoints()[matches_before_filtering[i].trainIdx];
        predict_new_2D.pt.x = normalized_predict_points(0) * para.c_fx + para.c_cx;
        predict_new_2D.pt.y = normalized_predict_points(1) * para.c_fy + para.c_cy;
        float delta_x = predict_new_2D.pt.x - new_2D.pt.x;
        float delta_y = predict_new_2D.pt.y - new_2D.pt.y;
        float average_reprojection_error_2d = sqrt(delta_x * delta_x + delta_y * delta_y);
        float reprojection_error = (new_point - direct_R * ref_point - direct_t).norm() / ref_point(2);
        if (average_reprojection_error_2d < 2)
        {
          ransac_2d_inlier_matches.push_back(matches_before_filtering[i]);
        }
        if (reprojection_error < reprojectionThreshold * 2)
        {

          // only add the first match
          if (previous_query_index != matches_before_filtering[i].queryIdx)
          {
            previous_query_index = matches_before_filtering[i].queryIdx;
            ransac_inlier_matches.push_back(matches_before_filtering[i]);
          }
        }
      }

      if (ransac_inlier_matches.size() < minimum_3d_correspondence)
      {
        return 1e7;
      }
      end = clock();
      float time_select_init_inliers = end - start;


      relative_pose_from_ref_to_new = Sophus::SE3d(direct_R,direct_t);

      start = clock();
      //refine match
      std::vector<float> weight_per_feature;
      optimize_3d_to_3d_huber_filter(frame_ref, frame_new,
                                     ransac_inlier_matches,
                                     relative_pose_from_ref_to_new,
                                     weight_per_feature,
                                     reprojectionThreshold, 6,0.005);
      end = clock();
      float time_nonlinear_opt = end - start;


#if 1
      start = clock();
      init_matches.clear();
      weight_per_feature.clear();
      weight_per_feature.reserve(matches_before_filtering.size());
      init_matches.reserve(matches_before_filtering.size());
      Eigen::MatrixXd H = relative_pose_from_ref_to_new.matrix3x4();
      Eigen::MatrixXd invH = relative_pose_from_ref_to_new.matrix().inverse().block<3, 4>(0, 0);
      float average_reprojection_error_2D = 0;
      for (size_t i = 0; i < matches_before_filtering.size(); i++)
      {
        Eigen::Vector3d ref_point = frame_ref.getLocalPoints()[matches_before_filtering[i].queryIdx];
        Eigen::Vector3d new_point = frame_new.getLocalPoints()[matches_before_filtering[i].trainIdx];
        cv::KeyPoint predict_new_2D = frame_ref.getKeypoints()[matches_before_filtering[i].queryIdx];
        cv::KeyPoint predict_ref_2D = frame_new.getKeypoints()[matches_before_filtering[i].trainIdx];
        cv::KeyPoint ref_2D = frame_ref.getKeypoints()[matches_before_filtering[i].queryIdx];
        cv::KeyPoint new_2D = frame_new.getKeypoints()[matches_before_filtering[i].trainIdx];
        Eigen::Vector4d homo_ref_points,homo_new_points;

        homo_ref_points << ref_point(0), ref_point(1), ref_point(2), 1;
        homo_new_points << new_point(0), new_point(1), new_point(2), 1;
        Eigen::Vector3d estimate_new_local_points = H * homo_ref_points;
        Eigen::Vector3d estimate_ref_local_points = invH * homo_new_points;


        predict_new_2D.pt.x = estimate_new_local_points(0) / estimate_new_local_points(2) * para.c_fx + para.c_cx;
        predict_new_2D.pt.y = estimate_new_local_points(1) / estimate_new_local_points(2)* para.c_fy + para.c_cy;
        predict_ref_2D.pt.x = estimate_ref_local_points(0) / estimate_ref_local_points(2) * para.c_fx + para.c_cx;
        predict_ref_2D.pt.y = estimate_ref_local_points(1) / estimate_ref_local_points(2) * para.c_fy + para.c_cy;
        float delta_x = predict_new_2D.pt.x - new_2D.pt.x;
        float delta_y = predict_new_2D.pt.y - new_2D.pt.y;
        float average_reprojection_error_2d_ref = sqrt(delta_x * delta_x + delta_y * delta_y);
        delta_x = predict_ref_2D.pt.x - ref_2D.pt.x;
        delta_y = predict_ref_2D.pt.y - ref_2D.pt.y;
        float average_reprojection_error_2d_new = sqrt(delta_x * delta_x + delta_y * delta_y);
        float reprojection_error = (new_point - estimate_new_local_points).norm() / ref_point(2);
        double reprojection_error_2d = average_reprojection_error_2d_ref;// max(average_reprojection_error_2d_ref, average_reprojection_error_2d_new);

        if (reprojection_error < reprojectionThreshold  && reprojection_error_2d < MultiViewGeometry::g_para.reprojection_error_2d_threshold)
        {
          average_reprojection_error_2D += reprojection_error_2d;



          float huber_threshold = 0.008;
          float weight_huber = 1;
          if (reprojection_error > huber_threshold)
          {
            weight_huber = huber_threshold / reprojection_error;
          }

          init_matches.push_back(matches_before_filtering[i]);
          weight_per_feature.push_back(weight_huber);
        }
      }

      ransac_inlier_matches = init_matches;
#endif
      end = clock();
      float time_select_final_inlier = end - start;

#if 0
      start = clock();
      optimize_3d_to_3d_huber_filter(frame_ref, frame_new,
                                     ransac_inlier_matches,
                                     relative_pose_from_ref_to_new,
                                     weight_lp,
                                     reprojectionThreshold, 6,0.005);
      end = clock();
#endif

      float time_last_non_linear = end - start;



//      float reprojection_error = reprojection_error_3Dto3D(frame_ref, frame_new, ransac_inlier_matches, (relative_pose_from_ref_to_new), 0);
      init_matches = ransac_inlier_matches;
      // make sure there is no outliers
  //		RefineByRotation(frame_ref, frame_new, init_matches);
  //		outlierFiltering(frame_ref, frame_new, init_matches, 5,0.01);
  //		outlierFiltering(frame_ref, frame_new, init_matches, 5,0.01);
  //    std::cout << "reprojection error after lp optimzation: " << reprojection_error << std::endl;


      end_total = clock();
      float time_total = end_total - start_total ;
      if (init_matches.size() > minimum_3d_correspondence)
      {
        fCorr.matches = init_matches;
        fCorr.weight_per_feature = weight_per_feature;
        fCorr.preIntegrate();
        float reprojection_error = reprojection_error_3Dto3D(fCorr, relative_pose_from_ref_to_new);

        return reprojection_error;
      }
      return 1e6;
    }



  void outlierFiltering(ServerFrame &frame_ref, ServerFrame &frame_new, std::vector< cv::DMatch > &init_matches)
    {
        int candidate_num = 8;
        float distance_threshold = 0.015;
        int N = init_matches.size();
        std::vector< cv::DMatch > filtered_matches;
        filtered_matches.reserve(N);
        for (size_t i = 0; i < N; i++)
        {
            Eigen::Vector3d ref_point = frame_ref.getLocalPoints()[init_matches[i].queryIdx];
            Eigen::Vector3d new_point = frame_new.getLocalPoints()[init_matches[i].trainIdx];

            int distance_preserve_flag = 0;
            for (size_t j = 0; j < candidate_num; j++)
            {
                int rand_choice = rand() % N;
                Eigen::Vector3d ref_point_p = frame_ref.getLocalPoints()[init_matches[rand_choice].queryIdx];
                Eigen::Vector3d new_point_p = frame_new.getLocalPoints()[init_matches[rand_choice].trainIdx];
                double d1 = (ref_point_p - ref_point).norm();
                double d2 = (new_point_p - new_point).norm();
                if (fabs(d1 - d2) / ref_point(2) < distance_threshold)
                {
                    distance_preserve_flag = 1;
                    break;
                }
            }
            if (distance_preserve_flag)
            {
                filtered_matches.push_back(init_matches[i]);
            }
        }
        init_matches = filtered_matches;
    }


  bool FrameMatchingTwoViewRGB(Correspondence &fCorr,
                               MultiViewGeometry::CameraPara camera_para,
                               MILD::SparseMatcher &frame_new_matcher,
                               PoseSE3d &relative_pose_from_ref_to_new,
                               float &average_disparity,
                               float &scale_change_ratio,
                               bool &update_keyframe_from_dense_matching,int local_track_cnt,
                               bool use_initial_guess,bool debug_mode)
  {

    float time_feature_matching,time_ransac,time_filter,time_refine,time_rotation_filter ;
    update_keyframe_from_dense_matching = 0;

    float reprojection_error_feature_based;
    float reprojection_error_dense_based;

    PoseSE3d init_guess_relative_pose_from_ref_to_new = relative_pose_from_ref_to_new;
    ServerFrame &frame_ref = fCorr.frame_ref;
    ServerFrame &frame_new = fCorr.frame_new;
    //std::cout << "************frame registration: "<< frame_ref.submapID<<"/"<< frame_ref.frame_index << " vs "<<frame_new.submapID<<"/"<< frame_new.frame_index << "************" << std::endl;

    bool matching_success = 0;
    bool dense_success = 0, sparse_success = 0;



    std::vector<std::vector<cv::DMatch>> matches;
    std::vector<cv::DMatch > init_matches;
    init_matches.clear();
    matches.clear();
    clock_t start, end;
    clock_t start_total, end_total;
    double duration;
    start_total = clock();

    // feature matching based on hamming distance
    frame_new_matcher.search_8(frame_ref.getDescriptor(), init_matches, MultiViewGeometry::g_para.hamming_distance_threshold);
    int matched_feature_pairs = init_matches.size();
    int rotation_inliers = 0;
      //std::cout<<"init_matches' size(before filtering):" <<init_matches.size()<<std::endl;

#if 1
{

    RefineByRotation(frame_ref, frame_new, init_matches);
    rotation_inliers = init_matches.size();
}
#endif
    // use ransac to remove outliers
    int candidate_num = 8;
    float min_distance_threshold = 0.015;
    int inliers_num_first, inliers_num_second;			// make sure 90% are inliers
    inliers_num_first = matched_feature_pairs;
    std::vector< cv::DMatch > matches_before_filtering = init_matches;
 //   RefineByRotation(frame_ref, frame_new, init_matches);
 //   RefineByRotation(frame_ref, frame_new, init_matches);
 //   RefineByRotation(frame_ref, frame_new, init_matches);
 //   RefineByRotation(frame_ref, frame_new, init_matches);
 //   RefineByRotation(frame_ref, frame_new, init_matches);

    outlierFiltering(frame_ref, frame_new, init_matches);
    outlierFiltering(frame_ref, frame_new, init_matches);
    outlierFiltering(frame_ref, frame_new, init_matches);
    outlierFiltering(frame_ref, frame_new, init_matches);
    outlierFiltering(frame_ref, frame_new, init_matches);
    




    int ransac_input_num = init_matches.size();

    double reprojection_error;


    reprojection_error = ransac3D3D(frame_ref,
                                    frame_new,
                                    init_matches,
                                    matches_before_filtering,
                                    MultiViewGeometry::g_para.reprojection_error_3d_threshold,
                                    MultiViewGeometry::g_para.ransac_maximum_iterations,
                                    fCorr,
                                    relative_pose_from_ref_to_new,
                                    camera_para);

    start = clock();
    /********************** fine search **********************/
    {
      // refine binary feature search results
      Eigen::MatrixXd H = relative_pose_from_ref_to_new.matrix3x4();
      std::vector< cv::DMatch > predict_matches;
      std::vector<cv::KeyPoint> predict_ref_points;

      predict_ref_points.resize(frame_ref.getLocalPoints().size());
      for (int i = 0; i < frame_ref.getLocalPoints().size(); i++)
      {
        Eigen::Vector4d homo_points;
        homo_points << frame_ref.getLocalPoints()[i](0), frame_ref.getLocalPoints()[i](1), frame_ref.getLocalPoints()[i](2), 1;
        Eigen::Vector3d predict_points = H*homo_points;
        predict_points = predict_points / predict_points(2);
        cv::KeyPoint predict_ref = frame_ref.getKeypoints()[i];
        predict_ref.pt.x = predict_points(0) * camera_para.c_fx + camera_para.c_cx;
        predict_ref.pt.y = predict_points(1) * camera_para.c_fy + camera_para.c_cy;
        predict_ref_points[i] = predict_ref;
        cv::DMatch m;
        m.queryIdx = i;
        m.trainIdx = i;
        if (i % 20 == 0)
        {
          predict_matches.push_back(m);

        }
      }
#if 0

#endif
      init_matches.clear();
      frame_new_matcher.search_8_with_range(frame_ref.getDescriptor(), init_matches, frame_new.getKeypoints(), predict_ref_points, 30,
                                              MultiViewGeometry::g_para.hamming_distance_threshold * 1.5);
      //RefineByRotation(frame_ref, frame_new, init_matches);
      std::vector< cv::DMatch > complete_matches = init_matches;
      //std::cout<<"init_matches' size(before filtering):" <<init_matches.size()<<std::endl;
      outlierFiltering(frame_ref, frame_new, init_matches);
      outlierFiltering(frame_ref, frame_new, init_matches);
      outlierFiltering(frame_ref, frame_new, init_matches);
      outlierFiltering(frame_ref, frame_new, init_matches);
      //std::cout<<"init_matches' size(before ransac):" <<init_matches.size()<<std::endl;
      reprojection_error = ransac3D3D(frame_ref, frame_new, init_matches, complete_matches, MultiViewGeometry::g_para.reprojection_error_3d_threshold,
        MultiViewGeometry::g_para.ransac_maximum_iterations, fCorr, relative_pose_from_ref_to_new,camera_para);
      //std::cout<<"init_matches' size(after ransac):" <<init_matches.size()<<std::endl;
    }

    end = clock();
    time_refine = end - start ;

    start = clock();


    std::vector<float> feature_weight_lp = fCorr.weight_per_feature;

    reprojection_error_feature_based = reprojection_error;
    float scale_increase = 0, scale_decrease = 0;
    for (int i = 0; i < init_matches.size(); i++)
    {
      cv::KeyPoint predict_new_2D = frame_ref.getKeypoints()[init_matches[i].queryIdx];
      cv::KeyPoint predict_ref_2D = frame_new.getKeypoints()[init_matches[i].trainIdx];
      if (predict_new_2D.octave >  predict_ref_2D.octave)
      {
        scale_increase++;
      }
      if (predict_new_2D.octave < predict_ref_2D.octave)
      {
        scale_decrease++;
      }
    }
    scale_change_ratio = fmax(scale_decrease, scale_increase) / (init_matches.size()+1);

    if(reprojection_error < REPROJECTION_TH)
    {
      sparse_success = 1;
    }

    if(reprojection_error < REPROJECTION_TH)
    {
      //fCorr.preIntegrateInterSubmap();
      average_disparity = fCorr.calculate_average_disparity(camera_para);
//        fCorr.clear_memory();
      if(average_disparity >0 && average_disparity <= MultiViewGeometry::g_para.max_average_disparity)
      matching_success = 1;
    }


    end = clock();
    float time_finishing = end - start;

    if(!fCorr.sameCamera)
    {
        std::cout<<"matching success?........................"<<matching_success<<std::endl;
    }
#if 1
  if(!matching_success &&(fCorr.allowance_icp == true) && fCorr.frame_new.frame_index - fCorr.frame_ref.frame_index < 3 )
  {
    // dense match  
     Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
#if 0
     icp_registration::depth_registration_ICP(fCorr.frame_ref.getDepth(),fCorr.frame_new.getDepth(),T ,  fCorr.data_pairs_3d, MultiViewGeometry::g_para.max_feature_num,true);
#elif 0
     std::cout<<"coloredICP registration: "<<fCorr.frame_ref.frame_index<<" "<<fCorr.frame_new.frame_index<<std::endl;
     icp_registration::colored_icp_registration(fCorr.frame_ref.getRgb(),fCorr.frame_new.getRgb(),fCorr.frame_ref.getDepth(),fCorr.frame_new.getDepth(),T,fCorr.data_pairs_3d,MultiViewGeometry::g_para.max_feature_num,true);
#else 
     std::cout<<"dense_tracking: "<<fCorr.frame_ref.frame_index<<" "<<fCorr.frame_new.frame_index<<std::endl;
     matching_success = icp_registration::dense_tracking(fCorr.frame_ref.getRgb(),fCorr.frame_new.getRgb(),fCorr.frame_ref.getDepth(),fCorr.frame_new.getDepth(),T,fCorr.data_pairs_3d);

#endif
     fCorr.setICP(true);
     relative_pose_from_ref_to_new = PoseSE3d(T);
     average_disparity = fCorr.calculate_average_disparity(camera_para);
     if(average_disparity > 0 && average_disparity < MultiViewGeometry::g_para.max_average_disparity &&matching_success)
     {
     matching_success = true;
     std::cout<<"average disparity: "<<average_disparity<<std::endl;
     }
    else
    {
      std::cout<<average_disparity<<" "<<MultiViewGeometry::g_para.max_average_disparity<<std::endl;
    }
  }
#endif
    return matching_success;
  }

  Eigen::Matrix3d skewMatrixProduct(Eigen::Vector3d t1, Eigen::Vector3d t2)
  {
      Eigen::Matrix3d M;
      M(0, 0) = -t1(1)*t2(1) - t1(2)*t2(2); M(0, 1) = t1(1)*t2(0); M(0, 2) = t1(2)*t2(0);
      M(1, 0) = t1(0)*t2(1);	 M(1, 1) = -t1(2)*t2(2) - t1(0)*t2(0); M(1, 2) = t1(2)*t2(1);
      M(2, 0) = t1(0)*t2(2);   M(2, 1) = t1(1)*t2(2); M(2, 2) = -t1(1)*t2(1) - t1(0)*t2(0);
      return M;
  }

  Eigen::Matrix3d getSkewSymmetricMatrix(Eigen::Vector3d t)
  {
      Eigen::Matrix3d t_hat;
      t_hat << 0, -t(2), t(1),
          t(2), 0, -t(0),
          -t(1), t(0), 0;
      return t_hat;
  }




  float reprojection_error_3Dto3D_submap_camera(const Correspondence &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new)
  {
    Eigen::MatrixXd R_ref = relative_pose_from_ref_to_new.rotationMatrix();
    Eigen::Vector3d t_ref = relative_pose_from_ref_to_new.translation();
    // pre-integration method for norm-2 distance
    float total_error = 0;

    if (fC.sum_weight_s > 0)
    {
      total_error = fC.sum_p_ref_ref_s(0,0) + fC.sum_p_ref_ref_s(1,1) + fC.sum_p_ref_ref_s(2,2) +
          fC.sum_p_new_new_s(0,0) + fC.sum_p_new_new_s(1,1) + fC.sum_p_new_new_s(2,2) +
          fC.sum_weight_s * t_ref.transpose() * t_ref
        - 2 * (float)(t_ref.transpose() * fC.sum_p_new_s) + 2 * (float)(t_ref.transpose() * R_ref * fC.sum_p_ref_s)
        - 2 * R_ref.cwiseProduct(fC.sum_p_new_ref_s).sum();

      if(total_error < 0)
      {
        std::cout << "total error: " << total_error << std::endl;

      }
      else
      {
        total_error = sqrt(total_error)  / fC.sum_weight_s;
      }
    }
    return total_error;
  }
  float reprojection_error_3Dto3D_submap_camera(const Correspondence &fC,ServerSLAM &server_slam)
  {
      int rcID = fC.frame_ref.cameraID;
      int ncID = fC.frame_new.cameraID;
      int rsID = fC.frame_ref.submapID;
      int nsID = fC.frame_new.submapID; 

      float error = reprojection_error_3Dto3D_submap_camera(fC,(server_slam.cameraStartPoses[ncID]*server_slam.submapPosesFinal[ncID][nsID]).inverse()*(server_slam.cameraStartPoses[rcID] * server_slam.submapPosesFinal[rcID][rsID]));
      //std::cout<<"ref submap: "<<rsID<<"/" <<fC.frame_ref.frame_index <<" new submap: "<<nsID<<"/"<<fC.frame_new.frame_index<<" error:"<<error <<std::endl;  
      return error;
  }
  float reprojection_error_3Dto3D_submap_camera(std::vector<Correspondence> &fCList,ServerSLAM &server_slam)
  {
    float average_reprojection_error = 0;
    float count = 0;
    for (int i = 0; i < fCList.size(); i++)
    {
      average_reprojection_error += reprojection_error_3Dto3D_submap_camera(fCList[i],server_slam) * fCList[i].sum_weight_s;
      count += fCList[i].sum_weight_s;
    }
    return average_reprojection_error / count;
  }
void ComputeJacobianInfoSubmapCamera(Server::Correspondence &fC,Server::ServerSLAM &server_slam,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj)
  {
    int valid_3d_cnt = fC.sparse_feature_cnt + fC.dense_feature_cnt ;
    // construct the four matrix based on pre-integrated points
    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();
    if (valid_3d_cnt < minimum_3d_correspondence)
    {
      return;
    }
    //prepare data
    int refID = fC.frame_ref.submapID;
    int newID = fC.frame_new.submapID;
    int refCameraID = fC.frame_ref.cameraID;
    int newCameraID = fC.frame_new.cameraID;
    Eigen::Matrix3d R_ref = (server_slam.cameraStartPoses[refCameraID]*server_slam.submapPosesFinal[refCameraID][refID]).rotationMatrix();
    Eigen::Vector3d t_ref = (server_slam.cameraStartPoses[refCameraID]*server_slam.submapPosesFinal[refCameraID][refID]).translation();
    Eigen::Matrix3d R_new = (server_slam.cameraStartPoses[newCameraID]*server_slam.submapPosesFinal[newCameraID][newID]).rotationMatrix();
    Eigen::Vector3d t_new = (server_slam.cameraStartPoses[newCameraID]*server_slam.submapPosesFinal[newCameraID][newID]).translation();
    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri, rjWrj;
    riWrj = R_ref * fC.sum_p_ref_new_s * R_new.transpose();
    riWri = R_ref * fC.sum_p_ref_ref_s * R_ref.transpose();
    rjWrj = R_new * fC.sum_p_new_new_s * R_new.transpose();

    Eigen::Vector3d R_ref_sum_p_ref = R_ref * fC.sum_p_ref_s;
    Eigen::Vector3d R_new_sum_p_new = R_new * fC.sum_p_new_s;
    Eigen::Vector3d residual = R_ref_sum_p_ref + fC.sum_weight_s * (t_ref - t_new) - R_new_sum_p_new;
    //calculating JTr, see ProblemFormulation.pdf
    Pre_JiTr.block<3, 1>(0, 0) = residual;
    Pre_JiTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_ref_sum_p_ref.cross(t_ref - t_new) + t_ref.cross(residual);

    Pre_JjTr.block<3, 1>(0, 0) = residual;
    Pre_JjTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_new_sum_p_new.cross(t_ref - t_new) + t_new.cross(residual);
    Pre_JjTr = -Pre_JjTr;

    //calculating JTJ
    Pre_JiTJi.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_s;
    Pre_JiTJi.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight_s * t_ref);
    Pre_JiTJi.block<3, 3>(3, 0) = -Pre_JiTJi.block<3, 3>(0, 3);
    Pre_JiTJi(3, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(3, 4) = -riWri(1, 0);					Pre_JiTJi(3, 5) = -riWri(2, 0);
    Pre_JiTJi(4, 3) = -riWri(0, 1);					Pre_JiTJi(4, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(4, 5) = -riWri(2, 1);
    Pre_JiTJi(5, 3) = -riWri(0, 2);					Pre_JiTJi(5, 4) = -riWri(1, 2);					Pre_JiTJi(5, 5) = riWri(0, 0) + riWri(1, 1);
    Pre_JiTJi.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_ref_sum_p_ref) - skewMatrixProduct(R_ref_sum_p_ref, t_ref) - fC.sum_weight_s * 1 * skewMatrixProduct(t_ref, t_ref);

    Pre_JjTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_s;
    Pre_JjTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight_s * t_new);
    Pre_JjTJj.block<3, 3>(3, 0) = -Pre_JjTJj.block<3, 3>(0, 3);
    Pre_JjTJj(3, 3) = rjWrj(2, 2) + rjWrj(1, 1);	Pre_JjTJj(3, 4) = -rjWrj(1, 0);					Pre_JjTJj(3, 5) = -rjWrj(2, 0);
    Pre_JjTJj(4, 3) = -rjWrj(0, 1);					Pre_JjTJj(4, 4) = rjWrj(0, 0) + rjWrj(2, 2);	Pre_JjTJj(4, 5) = -rjWrj(2, 1);
    Pre_JjTJj(5, 3) = -rjWrj(0, 2);					Pre_JjTJj(5, 4) = -rjWrj(1, 2);					Pre_JjTJj(5, 5) = rjWrj(0, 0) + rjWrj(1, 1);
    Pre_JjTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_new, R_new_sum_p_new) - skewMatrixProduct(R_new_sum_p_new, t_new) - fC.sum_weight_s * 1 * skewMatrixProduct(t_new, t_new);


    Pre_JiTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_s;
    Pre_JiTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight_s * t_new);
    Pre_JiTJj.block<3, 3>(3, 0) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight_s * t_ref).transpose();
    Pre_JiTJj(3, 3) = riWrj(2, 2) + riWrj(1, 1);	Pre_JiTJj(3, 4) = -riWrj(1, 0);	Pre_JiTJj(3, 5) = -riWrj(2, 0);
    Pre_JiTJj(4, 3) = -riWrj(0, 1);	Pre_JiTJj(4, 4) = riWrj(0, 0) + riWrj(2, 2);	Pre_JiTJj(4, 5) = -riWrj(2, 1);
    Pre_JiTJj(5, 3) = -riWrj(0, 2);	Pre_JiTJj(5, 4) = -riWrj(1, 2);		Pre_JiTJj(5, 5) = riWrj(0, 0) + riWrj(1, 1);
    Pre_JiTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_new_sum_p_new) - skewMatrixProduct(R_ref_sum_p_ref, t_new) - fC.sum_weight_s * 1 * skewMatrixProduct(t_ref, t_new);
    Pre_JiTJj = -Pre_JiTJj;
    Pre_JjTJi = Pre_JiTJj.transpose();

  #if 0
    std::cout << "precomputing jacobian matrics: " << fC.frame_new.frame_index << " " << fC.frame_ref.frame_index << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;
    std::cout << "JjTr:" << Pre_JjTr.transpose() << std::endl;
    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    std::cout << "JiTJj: " << std::endl << Pre_JiTJj << std::endl;
    std::cout << "JjTJi: " << std::endl << Pre_JjTJi << std::endl;
    std::cout << "JjTJj: " << std::endl << Pre_JjTJj << std::endl;
  #endif
  }
bool optimizeSubmapCameraRobust(ServerSLAM &server_slam, float robust_u, int cameraID)
  {

    std::vector<int> keyframe_candidate_fcorrs;
    int node_id = server_slam.node_id[cameraID];
    std::vector<Server::Correspondence> &fCList = server_slam.camera_nodes[node_id].correspondences;
    std::cout<<"optimizeSubmapCamera!  Correspondence: "<<fCList.size()<<std::endl;
    
    //std::vector<int> vsubmapIDs;
    std::map<int,std::vector<int>> lookTable;
    std::vector<pair<int,int>> inverseLookTable;
    int gsubmapID = 0;//globalSubmapID
    for(auto i =server_slam.camera_nodes[node_id].camera.begin() ;i!= server_slam.camera_nodes[node_id].camera.end();++i)
    {
        for(int j = 0;j!=server_slam.submapPoses[*i].size();++j)
        {
            lookTable[*i].push_back(gsubmapID);
            inverseLookTable.push_back(make_pair(*i,j));
            gsubmapID+=1;
        }
    }

    /*for (int i = 0; i < keyframes.size(); i++)
    {
      std::cout << i << " " << keyframes[i] << std::endl;
    }*/
    if (inverseLookTable.size() < 3 )
    {
      std::cout << "no need to optimize!" << std::endl;
      return -1;
    }
    int latest_keyframe_index =server_slam.latest_keyframe_index;
    int latest_keyframe_camera = server_slam.latest_keyframe_camera;
    for (int i = 0; i < fCList.size(); i++)
    {
        ServerFrame &frame_ref = fCList[i].frame_ref;
        ServerFrame &frame_new = fCList[i].frame_new;        
    if(frame_new.submapID >= server_slam.submapPoses[frame_new.cameraID].size() || frame_ref.submapID >= server_slam.submapPoses[frame_ref.cameraID].size())
        continue;
    else  keyframe_candidate_fcorrs.push_back(i);
    }
    


    std::vector<float> weight_per_pair(keyframe_candidate_fcorrs.size());
    // will be replaced by conjugate gradient descent.
    int optNum = inverseLookTable.size()- 1;
    Eigen::MatrixXd J, err;
    Eigen::MatrixXd delta(6 * optNum, 1), JTe(6 * optNum, 1);
    Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> JTJ(6 * optNum, 6 * optNum);

    int valid_observation_cnt = 0;
    //double prev_err = 10000;

    clock_t start, end;

    // the solver is only built at the first iteration
    Eigen::SimplicialLDLT	<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> > SimplicialLDLTSolver;

    std::vector<Eigen::Triplet<SPARSE_MATRIX_NUM_TYPE>> coeff;
    coeff.reserve(6 * 6 * 4 * fCList.size());
    Eigen::MatrixXd JiTJi_pre(6, 6), JiTJj_pre(6, 6), JjTJi_pre(6, 6), JjTJj_pre(6, 6), JiTe_pre(6, 1), JjTe_pre(6, 1);

    clock_t start_opt, end_opt;
    double time_opt;
    start_opt = clock();

  /*  PoseSE3dList frame_poses;
    for(int i = 0; i < F.size(); i++)
    {
        frame_poses.push_back(F[i].pose_sophus[0]);
    }
  */
    std::vector<Server::Correspondence> optimized_fc;
    for (int i = 0; i < keyframe_candidate_fcorrs.size(); i++)
    {
        optimized_fc.push_back(fCList[keyframe_candidate_fcorrs[i]]);
    }

    float init_error = reprojection_error_3Dto3D_submap_camera(optimized_fc,server_slam);
    float init_total_error = init_error;//reprojection_error_3Dto3D_submap(fCList, keyframe_candidate_fcorrs,submapPoses);
    float final_error=1.0;
    std::cout << "init/final error: " << init_error <<std::endl;
    for (int iter = 0; iter<3; iter++)
    {
      JTe.setZero();
      JTJ.setZero();
      err.setZero();
      coeff.clear();
      valid_observation_cnt = 0;

      double time_framePair;
      double time_generatingJacobian;
      double time_buildSolver;
      start = clock();
      float robust_weight;


      for (int i = 0; i < keyframe_candidate_fcorrs.size(); i++)
      {
        ServerFrame &frame_ref = fCList[keyframe_candidate_fcorrs[i]].frame_ref;
        ServerFrame &frame_new = fCList[keyframe_candidate_fcorrs[i]].frame_new;

        int frame_ref_pos = lookTable[frame_ref.cameraID][frame_ref.submapID];
        int frame_new_pos = lookTable[frame_new.cameraID][frame_new.submapID];
        //std::cout<<"pos: "<<frame_ref_pos<<" "<<frame_new_pos<<std::endl;

        if (frame_ref_pos < 0 || frame_new_pos < 0)
        {
          continue;
        }



        robust_weight = 1.0f;


        clock_t start_jacobian, end_jacobian;
        start_jacobian = clock();
        ComputeJacobianInfoSubmapCamera(fCList[keyframe_candidate_fcorrs[i]],server_slam,
          JiTe_pre,
          JjTe_pre,
          JiTJi_pre,
          JiTJj_pre,
          JjTJi_pre,
          JjTJj_pre);

        JiTe_pre *= robust_weight;
        JjTe_pre *= robust_weight;
        JiTJi_pre *= robust_weight;
        JiTJj_pre *= robust_weight;
        JjTJj_pre *= robust_weight;
        JjTJi_pre *= robust_weight;

        if(frame_ref_pos < frame_new_pos)
        {
        if (frame_ref_pos == 0)
        {
          addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
          JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
        }
        else
        {
          addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
          addBlockToTriplets(coeff, JiTJj_pre, (frame_ref_pos - 1) * 6, (frame_new_pos - 1) * 6);
          addBlockToTriplets(coeff, JjTJi_pre, (frame_new_pos - 1) * 6, (frame_ref_pos - 1) * 6);
          addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
          JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
          JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
        }
        }
        else
        {
        if (frame_new_pos == 0)
        {
          addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
          JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
        }
        else
        {
          addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
          addBlockToTriplets(coeff, JiTJj_pre, (frame_ref_pos - 1) * 6, (frame_new_pos - 1) * 6);
          addBlockToTriplets(coeff, JjTJi_pre, (frame_new_pos - 1) * 6, (frame_ref_pos - 1) * 6);
          addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
          JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
          JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
        }
        }
      }

      end = clock();
      time_framePair = (double)(end - start) / CLOCKS_PER_SEC * 1000;
      start = clock();
      JTJ.setFromTriplets(coeff.begin(), coeff.end());
      end = clock();
      time_generatingJacobian = (double)(end - start) / CLOCKS_PER_SEC * 1000;
      start = clock();
      SimplicialLDLTSolver.compute(JTJ);
      end = clock();
      time_buildSolver = (double)(end - start) / CLOCKS_PER_SEC * 1000;


      // update the pose of each frame
      start = clock();
      delta = SimplicialLDLTSolver.solve(JTe);
      end = clock();
      double time_svd = (double)(end - start) / CLOCKS_PER_SEC * 1000;
      std::cout<<"inverseLookTable size: " <<inverseLookTable.size()<<std::endl;
      for (int i = 1; i < inverseLookTable.size(); i++)
      {

          Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (i - 1), 0);
          //std::cout<<"delta_i: "<<delta_i<<std::endl;
          if(isnan(delta_i(0)))
          {
            std::cout << "nan detected in pose update! " << std::endl;
            continue;
          }
          //std::cout<<i<<" "<<delta_i<<std::endl<<std::endl;
          int cameraID = inverseLookTable[i].first;
          int submapID = inverseLookTable[i].second;
          /* refine all the keyframes' poses in this submap*/
          //if(submapID == 0) continue;
          auto tmp_global_pose = server_slam.cameraStartPoses[cameraID] * server_slam.submapPosesFinal[cameraID][submapID];
          tmp_global_pose =  Sophus::SE3d::exp(delta_i).inverse() * tmp_global_pose;

          server_slam.submapPosesFinal[cameraID][submapID] = 
            server_slam.cameraStartPoses[cameraID].inverse() * tmp_global_pose;
          
          server_slam.submapPosesRelativeChanges[cameraID][submapID] = server_slam.submapPosesFinal[cameraID][submapID] * server_slam.submapPoses[cameraID][submapID].inverse();
          //server_slam.submapPosesFinal[cameraID][submapID] =  * server_slam.submapPosesRelativeChanges[cameraID][submapID];
      }

      double time_calculate_reprojection_error;
      double refined_error;

    }

    end_opt = clock();

    time_opt = (double )(end_opt - start_opt) / CLOCKS_PER_SEC * 1000;

    final_error = reprojection_error_3Dto3D_submap_camera(optimized_fc,server_slam);
    float final_total_error = final_error;//reprojection_error_3Dto3D_submap(fCList,keyframe_candidate_fcorrs,submapPoses);
    std::cout << "init/final error " << init_error << "/" << final_error
         << "       " << init_total_error << "/" << final_total_error << std::endl;
    double last_final_error = server_slam.camera_nodes[node_id].last_final_error;
    // if((final_error - init_error) / init_error > /*0.5*/0.1 || (final_error - last_final_error) / last_final_error > 0.5)
    if((final_error - init_error) / init_error > /*0.5*/0.1 )
    {
        // remove all outliers

        for (int i = 0; i < keyframe_candidate_fcorrs.size(); i++)
        {
            ServerFrame &frame_ref = fCList[keyframe_candidate_fcorrs[i]].frame_ref;
            ServerFrame &frame_new = fCList[keyframe_candidate_fcorrs[i]].frame_new;

            if(frame_ref.frame_index == latest_keyframe_index && frame_ref.cameraID == latest_keyframe_camera || frame_new.frame_index == latest_keyframe_index && frame_new.cameraID == latest_keyframe_camera)
            {
                if( fCList[keyframe_candidate_fcorrs[i]].matches.size() > 0)
                {
                    std::cout << "try to remove: "<< " " << frame_ref.frame_index << " " << frame_new.frame_index << std::endl;
                    fCList[keyframe_candidate_fcorrs[i]].setIsValid(false);
                    fCList[keyframe_candidate_fcorrs[i]].reset();
                }
            }
        }
        /*for(int i = 0; i < F.size(); i++)
        {
            F[i].pose_sophus[0] = frame_poses[i];
        }*/
        return false;
    }
    else 
    {
      server_slam.camera_nodes[node_id].last_final_error = final_error;
      return true;
    }
  }
float optimizeSubmapCamera(ServerSLAM &server_slam,int cameraID)
{
    float robust_u = 0.5;
    bool need_optimization = true;
    std::cout<<"begin to optimize collibration"<<std::endl;
    while(need_optimization)
    {
    need_optimization =  !optimizeSubmapCameraRobust(server_slam,robust_u,cameraID);
    }
    std::cout<<"finish optimizing collibration"<<std::endl;
}



float reprojection_error_3Dto3D_room_submap(std::vector<RoomCorrespondence> &room_clist,
    std::vector<Server::Correspondence> &fCList, 
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses, PoseSE3dList &submap_poses_delta)
{
    float average_reprojection_error = 0;
    float count = 0;
    for (int i = 0; i < room_clist.size(); i++)
    {
        average_reprojection_error += 
          reprojection_error_3Dto3D_room(room_clist[i],server_slam, room_to_pos, room_poses) * room_clist[i].sum_weight;
        count += room_clist[i].sum_weight;
    }    
    for (int i = 0; i < fCList.size(); i++)
    {
        average_reprojection_error += 
            reprojection_error_3Dto3D_room_submap(fCList[i], server_slam, room_to_pos, 
            submap_to_pos, room_poses, submap_poses_delta) * fCList[i].sum_weight_c;
        count += fCList[i].sum_weight_c;
    }
    //std::cout<<count<<" average_reprojection_error: "<<average_reprojection_error/count<<std::endl;
    return average_reprojection_error / count;
}
float reprojection_error_normal(std::vector<NormalCorrespondence> &normal_clist,
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses, PoseSE3dList &submap_poses_delta)
{
    float average_reprojection_error = 0;
    float count = 0;
    for (int i = 0; i < normal_clist.size(); i++)
    {
      if(normal_clist[i].is_valid == false) continue;
        average_reprojection_error += 
          reprojection_error_normal(normal_clist[i], server_slam, room_to_pos, 
            submap_to_pos, room_poses, submap_poses_delta) * normal_clist[i].sum_weight_s;
        count += normal_clist[i].sum_weight_s;
    }    
    if(count == 0) return 0;
    //std::cout<<count<<" average_reprojection_error: "<<average_reprojection_error/count<<std::endl;
    return average_reprojection_error / count;
}
float reprojection_error_normal_gravity(std::vector<NormalGravityCorr> &normal_clist,
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses, PoseSE3dList &submap_poses_delta)
{
    float average_reprojection_error = 0;
    float count = 0;
    for (int i = 0; i < normal_clist.size(); i++)
    {
      //std::cout<<i<<" "<<count<<std::endl;
      if(normal_clist[i].is_valid == false) continue;
        average_reprojection_error += 
          reprojection_error_normal_gravity(normal_clist[i], server_slam, room_to_pos, 
            submap_to_pos, room_poses, submap_poses_delta) * normal_clist[i].sum_weight_s;
        count += normal_clist[i].sum_weight_s;
    }    
    if(count == 0) return 0;
    //std::cout<<count<<" average_reprojection_error: "<<average_reprojection_error/count<<std::endl;
    return average_reprojection_error / count;
}
float reprojection_error_3Dto3D_room_submap( Server::Correspondence &fC, 
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList & submap_poses_delta)
{
    PoseSE3d ref_pose, new_pose;
    ServerFrame &frame_ref = fC.frame_ref;
    ServerFrame &frame_new = fC.frame_new;

    int ref_submap_id = frame_ref.submapID;
    int new_submap_id = frame_new.submapID;
    
    int source_camera_id = frame_ref.cameraID;
    int target_camera_id = frame_new.cameraID;

    int submap_pos_ref = submap_to_pos[source_camera_id][ref_submap_id];
    int submap_pos_new = submap_to_pos[target_camera_id][new_submap_id];

    if(submap_pos_ref != -1)
    {
      ref_pose = (server_slam.cameraStartPoses[source_camera_id] * submap_poses_delta[submap_pos_ref]);
    }
    else
    {
      int room_id = server_slam.submap_to_room_for_each_camera[source_camera_id][ref_submap_id];
      int room_pos = room_to_pos[source_camera_id][room_id];
      ref_pose = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos]);
    }

    if(submap_pos_new != -1)
    {
      new_pose = (server_slam.cameraStartPoses[target_camera_id] * submap_poses_delta[submap_pos_new]);
    }
    else
    {
      int room_id = server_slam.submap_to_room_for_each_camera[target_camera_id][new_submap_id];
      int room_pos = room_to_pos[target_camera_id][room_id];
      new_pose = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos]);   
    }    
    //std::cout<< (new_pose.inverse()*ref_pose).matrix()<<std::endl;
    return reprojection_error_3Dto3D_room_submap(fC, new_pose.inverse()*ref_pose);

}
float reprojection_error_normal( Server::NormalCorrespondence &fC, 
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList & submap_poses_delta)
{
    PoseSE3d ref_pose, new_pose;
    if(fC.is_valid == 0) return 0;
    if(fC.type == 0)
    {
      int ref_submap_id = fC.ref_submap_id;
      int new_submap_id = fC.new_submap_id;
      
      int source_camera_id = fC.ref_camera_id;
      int target_camera_id = fC.new_camera_id;

      int submap_pos_ref = submap_to_pos[source_camera_id][ref_submap_id];
      int submap_pos_new = submap_to_pos[target_camera_id][new_submap_id];

      if(submap_pos_ref != -1)
      {
        ref_pose = (server_slam.cameraStartPoses[source_camera_id] * submap_poses_delta[submap_pos_ref]);
      }
      else
      {
        int room_id = server_slam.submap_to_room_for_each_camera[source_camera_id][ref_submap_id];
        int room_pos = room_to_pos[source_camera_id][room_id];
        ref_pose = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos]);
      }

      if(submap_pos_new != -1)
      {
        new_pose = (server_slam.cameraStartPoses[target_camera_id] * submap_poses_delta[submap_pos_new]);
      }
      else
      {
        int room_id = server_slam.submap_to_room_for_each_camera[target_camera_id][new_submap_id];
        int room_pos = room_to_pos[target_camera_id][room_id];
        new_pose = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos]);   
      }    
      //std::cout<< (new_pose.inverse()*ref_pose).matrix()<<std::endl;
      return reprojection_error_normal(fC, new_pose.inverse()*ref_pose);
    }
    else 
    {
      int ref_room_id = fC.ref_room_id;
      int new_room_id = fC.new_room_id;
      int source_camera_id = fC.ref_camera_id;
      int target_camera_id = fC.new_camera_id;
      int room_pos_ref = room_to_pos[source_camera_id][ref_room_id];
      int room_pos_new = room_to_pos[target_camera_id][new_room_id];

      PoseSE3d relative_pose = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos_new]).inverse() * 
        (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos_ref]);      
      return reprojection_error_normal(fC, new_pose.inverse()*ref_pose);
    }

}

float reprojection_error_normal_gravity( Server::NormalGravityCorr &fC, 
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    std::map<int, std::vector<int>> &submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList & submap_poses_delta)
{
    PoseSE3d pose;
    if(fC.is_valid == 0) return 0;
    if(fC.type == 0)
    {
      int submap_id = fC.submap_id;

      
      int camera_id = fC.camera_id;


      int submap_pos = submap_to_pos[camera_id][submap_id];


      if(submap_pos != -1)
      {
        pose = (server_slam.cameraStartPoses[camera_id] * submap_poses_delta[submap_pos]);
      }
      else
      {
        int room_id = server_slam.submap_to_room_for_each_camera[camera_id][submap_id];
        int room_pos = room_to_pos[camera_id][room_id];
        pose = (server_slam.cameraStartPoses[camera_id] * room_poses[room_pos]);
      }

      //std::cout<< (new_pose.inverse()*ref_pose).matrix()<<std::endl;
      return reprojection_error_normal_gravity(fC, pose);
    }
    else 
    {
      int room_id = fC.room_id;

      int camera_id = fC.camera_id;
      int room_pos = room_to_pos[camera_id][room_id];


      PoseSE3d relative_pose = (server_slam.cameraStartPoses[camera_id] * room_poses[room_pos]);
      return reprojection_error_normal_gravity(fC, relative_pose);
    }
}
  float reprojection_error_3Dto3D_room_submap( Server::Correspondence &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new)
  {
    Eigen::MatrixXd R_ref = relative_pose_from_ref_to_new.rotationMatrix();
    Eigen::Vector3d t_ref = relative_pose_from_ref_to_new.translation();
    // pre-integration method for norm-2 distance
    float total_error = 0;

    if (fC.sum_weight_c > 0)
    {               
      total_error = fC.sum_p_ref_ref_c(0,0) + fC.sum_p_ref_ref_c(1,1) + fC.sum_p_ref_ref_c(2,2) +
          fC.sum_p_new_new_c(0,0) + fC.sum_p_new_new_c(1,1) + fC.sum_p_new_new_c(2,2) +
          fC.sum_weight_c * t_ref.transpose() * t_ref
        - 2 * (float)(t_ref.transpose() * fC.sum_p_new_c) + 2 * (float)(t_ref.transpose() * R_ref * fC.sum_p_ref_c)
        - 2 * R_ref.cwiseProduct(fC.sum_p_new_ref_c).sum(); 

      if(total_error < 0)
      {
        std::cout << "total error: " << total_error << std::endl;

      }
      else
      {
        total_error = sqrt(total_error)/ fC.sum_weight_c;
      }
    }
    //std::cout<<fC.sum_weight_c<<" "<<total_error<<std::endl;
    return total_error;
  }
float reprojection_error_3Dto3D_room(RoomCorrespondence &rC,     
    ServerSLAM &server_slam, 
    std::map<int, std::vector<int>> &room_to_pos,
    PoseSE3dList & room_poses)
{
    //prepare data
    int ref_room_id = rC.source_id;
    int new_room_id = rC.target_id;
    int source_camera_id = rC.source_camera_id;
    int target_camera_id = rC.target_camera_id;
    int room_pos_ref = room_to_pos[source_camera_id][ref_room_id];
    int room_pos_new = room_to_pos[target_camera_id][new_room_id];

    PoseSE3d relative_pose = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos_new]).inverse() * 
      (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos_ref]);
    return reprojection_error_3Dto3D_room(rC, relative_pose);
}
float reprojection_error_3Dto3D_room(RoomCorrespondence &rC, const Sophus::SE3d & relative_pose_from_ref_to_new)
{
    Eigen::MatrixXd R_ref = relative_pose_from_ref_to_new.rotationMatrix();
    Eigen::Vector3d t_ref = relative_pose_from_ref_to_new.translation();
    // pre-integration method for norm-2 distance
    float total_error = 0;

    if (rC.sum_weight > 0)
    {
      total_error = 
          rC.sum_p_ref_ref(0,0) + rC.sum_p_ref_ref(1,1) + rC.sum_p_ref_ref(2,2) +
          rC.sum_p_new_new(0,0) + rC.sum_p_new_new(1,1) + rC.sum_p_new_new(2,2) +
          rC.sum_weight * t_ref.transpose() * t_ref
        - 2 * (float)(t_ref.transpose() * rC.sum_p_new) + 2 * (float)(t_ref.transpose() * R_ref * rC.sum_p_ref)
        - 2 * R_ref.cwiseProduct(rC.sum_p_new_ref).sum();

      if(total_error < 0)
      {
        std::cout << "total error: " << total_error << std::endl;

      }
      else
      {
        total_error = sqrt(total_error) / rC.sum_weight ;
      }
    //std::cout<<rC.sum_weight<<" "<<total_error<<std::endl;
    }
    return total_error;    
}
//Wrong reprojection_error
float reprojection_error_normal(NormalCorrespondence &rC, const Sophus::SE3d & relative_pose_from_ref_to_new)
{
    Eigen::MatrixXd R_ref = relative_pose_from_ref_to_new.rotationMatrix();
    float total_error = 0;

    if (rC.sum_weight_s > 0)
    {
      total_error = (R_ref * rC.ref_normal - rC.new_normal).norm();

      if(total_error < 0)
      {
        std::cout << "total error: " << total_error << std::endl;
      }
      /*
      else
      {
        total_error = total_error  / rC.sum_weight;
      }*/
    //std::cout<<rC.sum_weight<<" "<<total_error<<std::endl;
    }
    return total_error;    
}

float reprojection_error_normal_gravity(NormalGravityCorr &rC, const Sophus::SE3d & relative_pose_from_ref_to_new)
{
    Eigen::MatrixXd R = relative_pose_from_ref_to_new.rotationMatrix();
    float total_error = 0;

    if (rC.sum_weight_s > 0)
    {
      total_error = (R * rC.sum_p_ref_s - rC.sum_p_new_s).norm();
      //std::cout<<(R * rC.sum_p_ref_s).transpose()  <<" "<<rC.sum_p_new_s.transpose()<<std::endl;
      if(total_error < 0)
      {
        std::cout << "total error: " << total_error << std::endl;
      }
      /*
      else
      {
        total_error = total_error  / rC.sum_weight;
      }*/
    //std::cout<<rC.sum_weight_s<<" "<<total_error<<std::endl;
    }
    return total_error / rC.sum_weight_s /50;    
}


void ComputeJacobianInfoRoom(RoomCorrespondence &fC, 
    ServerSLAM &server_slam,
    std::map<int, std::vector<int>> &room_to_pos, 
    PoseSE3dList &room_poses,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj)
  {
    int valid_3d_cnt = fC.instance_center_correspondences.size();
    // construct the four matrix based on pre-integrated points
    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();
    if (valid_3d_cnt < minimum_3d_correspondence)
    {
      return;
    }
    //prepare data
    int ref_room_id = fC.source_id;
    int new_room_id = fC.target_id;
    int source_camera_id = fC.source_camera_id;
    int target_camera_id = fC.target_camera_id;
    int room_pos_ref = room_to_pos[source_camera_id][ref_room_id];
    int room_pos_new = room_to_pos[target_camera_id][new_room_id];

    Eigen::Matrix3d R_ref = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos_ref]).rotationMatrix();
    Eigen::Vector3d t_ref = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos_ref]).translation();
    Eigen::Matrix3d R_new = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos_new]).rotationMatrix();
    Eigen::Vector3d t_new = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos_new]).translation();
    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri, rjWrj;
    riWrj = R_ref * fC.sum_p_ref_new * R_new.transpose();
    riWri = R_ref * fC.sum_p_ref_ref * R_ref.transpose();
    rjWrj = R_new * fC.sum_p_new_new * R_new.transpose();

    Eigen::Vector3d R_ref_sum_p_ref = R_ref * fC.sum_p_ref;
    Eigen::Vector3d R_new_sum_p_new = R_new * fC.sum_p_new;
    Eigen::Vector3d residual = R_ref_sum_p_ref + fC.sum_weight * (t_ref - t_new) - R_new_sum_p_new;
    //calculating JTr, see ProblemFormulation.pdf
    Pre_JiTr.block<3, 1>(0, 0) = residual;
    Pre_JiTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_ref_sum_p_ref.cross(t_ref - t_new) + t_ref.cross(residual);

    Pre_JjTr.block<3, 1>(0, 0) = residual;
    Pre_JjTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_new_sum_p_new.cross(t_ref - t_new) + t_new.cross(residual);
    Pre_JjTr = -Pre_JjTr;

    //calculating JTJ
    Pre_JiTJi.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight;
    Pre_JiTJi.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight * t_ref);
    Pre_JiTJi.block<3, 3>(3, 0) = -Pre_JiTJi.block<3, 3>(0, 3);
    Pre_JiTJi(3, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(3, 4) = -riWri(1, 0);					Pre_JiTJi(3, 5) = -riWri(2, 0);
    Pre_JiTJi(4, 3) = -riWri(0, 1);					Pre_JiTJi(4, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(4, 5) = -riWri(2, 1);
    Pre_JiTJi(5, 3) = -riWri(0, 2);					Pre_JiTJi(5, 4) = -riWri(1, 2);					Pre_JiTJi(5, 5) = riWri(0, 0) + riWri(1, 1);
    Pre_JiTJi.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_ref_sum_p_ref) - skewMatrixProduct(R_ref_sum_p_ref, t_ref) - fC.sum_weight * 1 * skewMatrixProduct(t_ref, t_ref);

    Pre_JjTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight;
    Pre_JjTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight * t_new);
    Pre_JjTJj.block<3, 3>(3, 0) = -Pre_JjTJj.block<3, 3>(0, 3);
    Pre_JjTJj(3, 3) = rjWrj(2, 2) + rjWrj(1, 1);	Pre_JjTJj(3, 4) = -rjWrj(1, 0);					Pre_JjTJj(3, 5) = -rjWrj(2, 0);
    Pre_JjTJj(4, 3) = -rjWrj(0, 1);					Pre_JjTJj(4, 4) = rjWrj(0, 0) + rjWrj(2, 2);	Pre_JjTJj(4, 5) = -rjWrj(2, 1);
    Pre_JjTJj(5, 3) = -rjWrj(0, 2);					Pre_JjTJj(5, 4) = -rjWrj(1, 2);					Pre_JjTJj(5, 5) = rjWrj(0, 0) + rjWrj(1, 1);
    Pre_JjTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_new, R_new_sum_p_new) - skewMatrixProduct(R_new_sum_p_new, t_new) - fC.sum_weight * 1 * skewMatrixProduct(t_new, t_new);


    Pre_JiTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight;
    Pre_JiTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight * t_new);
    Pre_JiTJj.block<3, 3>(3, 0) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight * t_ref).transpose();
    Pre_JiTJj(3, 3) = riWrj(2, 2) + riWrj(1, 1);	Pre_JiTJj(3, 4) = -riWrj(1, 0);	Pre_JiTJj(3, 5) = -riWrj(2, 0);
    Pre_JiTJj(4, 3) = -riWrj(0, 1);	Pre_JiTJj(4, 4) = riWrj(0, 0) + riWrj(2, 2);	Pre_JiTJj(4, 5) = -riWrj(2, 1);
    Pre_JiTJj(5, 3) = -riWrj(0, 2);	Pre_JiTJj(5, 4) = -riWrj(1, 2);		Pre_JiTJj(5, 5) = riWrj(0, 0) + riWrj(1, 1);
    Pre_JiTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_new_sum_p_new) - skewMatrixProduct(R_ref_sum_p_ref, t_new) - fC.sum_weight * 1 * skewMatrixProduct(t_ref, t_new);
    Pre_JiTJj = -Pre_JiTJj;
    Pre_JjTJi = Pre_JiTJj.transpose();

  #if 0
    std::cout << "precomputing jacobian matrics: " << fC.source_id << " " << fC.target_id << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;
    std::cout << "JjTr:" << Pre_JjTr.transpose() << std::endl;
    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    std::cout << "JiTJj: " << std::endl << Pre_JiTJj << std::endl;
    std::cout << "JjTJi: " << std::endl << Pre_JjTJi << std::endl;
    std::cout << "JjTJj: " << std::endl << Pre_JjTJj << std::endl;
  #endif
}
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
    Eigen::MatrixXd &Pre_JjTJj)
  {
    int valid_3d_cnt = fC.sparse_feature_cnt +  fC.dense_feature_cnt;
    // construct the four matrix based on pre-integrated points
    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();
    if (valid_3d_cnt < minimum_3d_correspondence)
    {
      return;
    }
    //prepare data
    ServerFrame &frame_ref = fC.frame_ref;
    ServerFrame &frame_new = fC.frame_new;

    int ref_submap_id = frame_ref.submapID;
    int new_submap_id = frame_new.submapID;
    
    int source_camera_id = frame_ref.cameraID;
    int target_camera_id = frame_new.cameraID;
    //std::cout<<ref_submap_id<<" "<<new_submap_id<<" "<<source_camera_id<<" "<<target_camera_id<<std::endl;
    int submap_pos_ref = submap_to_pos[source_camera_id][ref_submap_id];
    int submap_pos_new = submap_to_pos[target_camera_id][new_submap_id];
    //std::cout<<submap_pos_ref<<" "<<submap_pos_new<<std::endl;//-1 0
    Eigen::Matrix3d R_ref ;
    Eigen::Vector3d t_ref ;
    Eigen::Matrix3d R_new ;
    Eigen::Vector3d t_new ;
    if(submap_pos_ref != -1)
    {
      R_ref = (server_slam.cameraStartPoses[source_camera_id] * submap_poses_delta[submap_pos_ref]).rotationMatrix();
      t_ref = (server_slam.cameraStartPoses[source_camera_id] * submap_poses_delta[submap_pos_ref]).translation();
    }
    else
    {
      int room_id = server_slam.submap_to_room_for_each_camera[source_camera_id][ref_submap_id];
      int room_pos = room_to_pos[source_camera_id][room_id];
      R_ref = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos]).rotationMatrix();
      t_ref = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos]).translation();
    }
    //std::cout<<submap_pos_ref<<" "<<submap_pos_new<<std::endl;
    if(submap_pos_new != -1)
    {
      R_new = (server_slam.cameraStartPoses[target_camera_id] * submap_poses_delta[submap_pos_new]).rotationMatrix();
      t_new = (server_slam.cameraStartPoses[target_camera_id] * submap_poses_delta[submap_pos_new]).translation();
    }
    else
    {
      int room_id = server_slam.submap_to_room_for_each_camera[target_camera_id][new_submap_id];
      int room_pos = room_to_pos[target_camera_id][room_id];
      R_new = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos]).rotationMatrix();
      t_new = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos]).translation();      
    }
    /*
    std::cout<<R_ref<<std::endl<<t_ref<<std::endl;
    std::cout<<std::endl;
    std::cout<<R_new<<std::endl<<t_new<<std::endl;
    */
    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri, rjWrj;
    riWrj = R_ref * fC.sum_p_ref_new_c * R_new.transpose();
    riWri = R_ref * fC.sum_p_ref_ref_c * R_ref.transpose();
    rjWrj = R_new * fC.sum_p_new_new_c * R_new.transpose();

    Eigen::Vector3d R_ref_sum_p_ref = R_ref * fC.sum_p_ref_c;
    Eigen::Vector3d R_new_sum_p_new = R_new * fC.sum_p_new_c;
    Eigen::Vector3d residual = R_ref_sum_p_ref + fC.sum_weight_c * (t_ref - t_new) - R_new_sum_p_new;
    //calculating JTr, see ProblemFormulation.pdf
    Pre_JiTr.block<3, 1>(0, 0) = residual;
    Pre_JiTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_ref_sum_p_ref.cross(t_ref - t_new) + t_ref.cross(residual);

    Pre_JjTr.block<3, 1>(0, 0) = residual;
    Pre_JjTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_new_sum_p_new.cross(t_ref - t_new) + t_new.cross(residual);
    Pre_JjTr = -Pre_JjTr;

    //calculating JTJ
    Pre_JiTJi.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_c;
    Pre_JiTJi.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight_c * t_ref);
    Pre_JiTJi.block<3, 3>(3, 0) = -Pre_JiTJi.block<3, 3>(0, 3);
    Pre_JiTJi(3, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(3, 4) = -riWri(1, 0);					Pre_JiTJi(3, 5) = -riWri(2, 0);
    Pre_JiTJi(4, 3) = -riWri(0, 1);					Pre_JiTJi(4, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(4, 5) = -riWri(2, 1);
    Pre_JiTJi(5, 3) = -riWri(0, 2);					Pre_JiTJi(5, 4) = -riWri(1, 2);					Pre_JiTJi(5, 5) = riWri(0, 0) + riWri(1, 1);
    Pre_JiTJi.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_ref_sum_p_ref) - skewMatrixProduct(R_ref_sum_p_ref, t_ref) - fC.sum_weight_c * 1 * skewMatrixProduct(t_ref, t_ref);

    Pre_JjTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_c;
    Pre_JjTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight_c * t_new);
    Pre_JjTJj.block<3, 3>(3, 0) = -Pre_JjTJj.block<3, 3>(0, 3);
    Pre_JjTJj(3, 3) = rjWrj(2, 2) + rjWrj(1, 1);	Pre_JjTJj(3, 4) = -rjWrj(1, 0);					Pre_JjTJj(3, 5) = -rjWrj(2, 0);
    Pre_JjTJj(4, 3) = -rjWrj(0, 1);					Pre_JjTJj(4, 4) = rjWrj(0, 0) + rjWrj(2, 2);	Pre_JjTJj(4, 5) = -rjWrj(2, 1);
    Pre_JjTJj(5, 3) = -rjWrj(0, 2);					Pre_JjTJj(5, 4) = -rjWrj(1, 2);					Pre_JjTJj(5, 5) = rjWrj(0, 0) + rjWrj(1, 1);
    Pre_JjTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_new, R_new_sum_p_new) - skewMatrixProduct(R_new_sum_p_new, t_new) - fC.sum_weight_c * 1 * skewMatrixProduct(t_new, t_new);


    Pre_JiTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_c;
    Pre_JiTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight_c * t_new);
    Pre_JiTJj.block<3, 3>(3, 0) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight_c * t_ref).transpose();
    Pre_JiTJj(3, 3) = riWrj(2, 2) + riWrj(1, 1);	Pre_JiTJj(3, 4) = -riWrj(1, 0);	Pre_JiTJj(3, 5) = -riWrj(2, 0);
    Pre_JiTJj(4, 3) = -riWrj(0, 1);	Pre_JiTJj(4, 4) = riWrj(0, 0) + riWrj(2, 2);	Pre_JiTJj(4, 5) = -riWrj(2, 1);
    Pre_JiTJj(5, 3) = -riWrj(0, 2);	Pre_JiTJj(5, 4) = -riWrj(1, 2);		Pre_JiTJj(5, 5) = riWrj(0, 0) + riWrj(1, 1);
    Pre_JiTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_new_sum_p_new) - skewMatrixProduct(R_ref_sum_p_ref, t_new) - fC.sum_weight_c * 1 * skewMatrixProduct(t_ref, t_new);
    Pre_JiTJj = -Pre_JiTJj;
    Pre_JjTJi = Pre_JiTJj.transpose();

#if 0
    //std::cout << "precomputing jacobian matrics: " << fC.source_id << " " << fC.target_id << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;
    std::cout << "JjTr:" << Pre_JjTr.transpose() << std::endl;
    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    std::cout << "JiTJj: " << std::endl << Pre_JiTJj << std::endl;
    std::cout << "JjTJi: " << std::endl << Pre_JjTJi << std::endl;
    std::cout << "JjTJj: " << std::endl << Pre_JjTJj << std::endl;
#endif
}

void ComputeJacobianInfoRoomNormal(NormalGravityCorr &SC,
    ServerSLAM &server_slam,
    std::map<int, std::vector<int> > & room_to_pos,
    PoseSE3dList &room_poses,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JiTJi)
{
    Pre_JiTr.setZero();
    Pre_JiTJi.setZero();

    //prepare data

    int room_id = SC.room_id;
    
    int camera_id = SC.camera_id;
    //std::cout<<ref_submap_id<<" "<<new_submap_id<<" "<<source_camera_id<<" "<<target_camera_id<<std::endl;
    int room_pos = room_to_pos[camera_id][room_id];
    //std::cout<<submap_pos_ref<<" "<<submap_pos_new<<std::endl;//-1 0
    Eigen::Matrix3d R ;
    Eigen::Vector3d t ;
    R = (server_slam.cameraStartPoses[camera_id] * room_poses[room_pos]).rotationMatrix();
    t = (server_slam.cameraStartPoses[camera_id] * room_poses[room_pos]).translation();

    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri;
    riWrj = R * SC.sum_p_ref_new_s ;
    riWri = R * SC.sum_p_ref_ref_s * R.transpose();

    Pre_JiTr.block<3, 1>(0, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1));

    Pre_JiTJi(0, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(0, 4) = -riWri(1, 0);					Pre_JiTJi(0, 5) = -riWri(2, 0);
    Pre_JiTJi(1, 3) = -riWri(0, 1);					Pre_JiTJi(1, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(1, 5) = -riWri(2, 1);
    Pre_JiTJi(2, 3) = -riWri(0, 2);					Pre_JiTJi(2, 4) = -riWri(1, 2);					Pre_JiTJi(2, 5) = riWri(0, 0) + riWri(1, 1);
}
void ComputeJacobianInfoSubmapNormal(NormalGravityCorr &SC,
    ServerSLAM &server_slam,
    std::map<int, std::vector<int> > & room_to_pos,
    std::map<int, std::vector<int> > & submap_to_pos,
    PoseSE3dList &room_poses,
    PoseSE3dList &submap_poses_delta,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JiTJi)
{
    Pre_JiTr.setZero();
    Pre_JiTJi.setZero();

    //prepare data

    int submap_id = SC.submap_id;
    
    int camera_id = SC.camera_id;
    //std::cout<<ref_submap_id<<" "<<new_submap_id<<" "<<source_camera_id<<" "<<target_camera_id<<std::endl;
    int submap_pos = submap_to_pos[camera_id][submap_id];
    //std::cout<<submap_pos_ref<<" "<<submap_pos_new<<std::endl;//-1 0
    Eigen::Matrix3d R ;
    Eigen::Vector3d t ;
    if(submap_pos != -1)
    {
      R = (server_slam.cameraStartPoses[camera_id] * submap_poses_delta[submap_pos]).rotationMatrix();
      t = (server_slam.cameraStartPoses[camera_id] * submap_poses_delta[submap_pos]).translation();
    }
    else
    {
      int room_id = server_slam.submap_to_room_for_each_camera[camera_id][submap_id];
      int room_pos = room_to_pos[camera_id][room_id];
      R = (server_slam.cameraStartPoses[camera_id] * room_poses[room_pos]).rotationMatrix();
      t = (server_slam.cameraStartPoses[camera_id] * room_poses[room_pos]).translation();
    }
    Eigen::Matrix3d Eye3x3;
    
    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri;
    riWrj = R * SC.sum_p_ref_new_s ;
    riWri = R * SC.sum_p_ref_ref_s * R.transpose();

    Pre_JiTr.block<3, 1>(0, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1));

    Pre_JiTJi(0, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(0, 4) = -riWri(1, 0);					Pre_JiTJi(0, 5) = -riWri(2, 0);
    Pre_JiTJi(1, 3) = -riWri(0, 1);					Pre_JiTJi(1, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(1, 5) = -riWri(2, 1);
    Pre_JiTJi(2, 3) = -riWri(0, 2);					Pre_JiTJi(2, 4) = -riWri(1, 2);					Pre_JiTJi(2, 5) = riWri(0, 0) + riWri(1, 1);
    
    //naive way
    //Eigen::Vector3d residual = R * SC.normal - Gravity;

        
    //Pre_JiTr.block<3, 1>(3, 0) = -MultiViewGeometry::getSkewSymmetricMatrix( R_ref * SC.ref_normal).transpose() * residual;

    //Pre_JjTr.block<3, 1>(0, 0) = residual;
    /*
    Pre_JiTr.block<3, 1>(0, 0) = MultiViewGeometry::getSkewSymmetricMatrix( R * SC.normal).transpose() * residual;

    Pre_JiTJi.block<3,3>(0, 3) = MultiViewGeometry::getSkewSymmetricMatrix( R * SC.normal).transpose() 
        * MultiViewGeometry::getSkewSymmetricMatrix( R * SC.normal);  
    */
    #if 0
    //std::cout<<residual.transpose()<<std::endl;
    std::cout << "precomputing jacobian matrics: " << SC.submap_id  << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;

    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    #endif
}
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
    Eigen::MatrixXd &Pre_JjTJj)
{

    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();

    //prepare data

    int ref_submap_id = SC.ref_submap_id;
    int new_submap_id = SC.new_submap_id;
    
    int source_camera_id = SC.ref_camera_id;
    int target_camera_id = SC.new_camera_id;
    //std::cout<<ref_submap_id<<" "<<new_submap_id<<" "<<source_camera_id<<" "<<target_camera_id<<std::endl;
    int submap_pos_ref = submap_to_pos[source_camera_id][ref_submap_id];
    int submap_pos_new = submap_to_pos[target_camera_id][new_submap_id];
    //std::cout<<submap_pos_ref<<" "<<submap_pos_new<<std::endl;//-1 0
    Eigen::Matrix3d R_ref ;
    Eigen::Vector3d t_ref ;
    Eigen::Matrix3d R_new ;
    Eigen::Vector3d t_new ;
    if(submap_pos_ref != -1)
    {
      R_ref = (server_slam.cameraStartPoses[source_camera_id] * submap_poses_delta[submap_pos_ref]).rotationMatrix();
      t_ref = (server_slam.cameraStartPoses[source_camera_id] * submap_poses_delta[submap_pos_ref]).translation();
    }
    else
    {
      int room_id = server_slam.submap_to_room_for_each_camera[source_camera_id][ref_submap_id];
      int room_pos = room_to_pos[source_camera_id][room_id];
      R_ref = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos]).rotationMatrix();
      t_ref = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos]).translation();
    }
    //std::cout<<submap_pos_ref<<" "<<submap_pos_new<<std::endl;
    if(submap_pos_new != -1)
    {
      R_new = (server_slam.cameraStartPoses[target_camera_id] * submap_poses_delta[submap_pos_new]).rotationMatrix();
      t_new = (server_slam.cameraStartPoses[target_camera_id] * submap_poses_delta[submap_pos_new]).translation();
    }
    else
    {
      int room_id = server_slam.submap_to_room_for_each_camera[target_camera_id][new_submap_id];
      int room_pos = room_to_pos[target_camera_id][room_id];
      R_new = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos]).rotationMatrix();
      t_new = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos]).translation();      
    }

    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri, rjWrj;
    riWrj = R_ref * SC.sum_p_ref_new_s * R_new.transpose();
    riWri = R_ref * SC.sum_p_ref_ref_s * R_ref.transpose();
    rjWrj = R_new * SC.sum_p_new_new_s * R_new.transpose();


    //Here we only care about the rotation
    //calculating JTr, see ProblemFormulation.pdf
    //Pre_JiTr.block<3, 1>(0, 0) = residual;


    Pre_JiTr.block<3, 1>(0, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1));

    Pre_JjTr.block<3, 1>(0, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1));
    Pre_JjTr = -Pre_JjTr;

    //calculating JTJ
    //Pre_JiTJi.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JiTJi(0, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(0, 4) = -riWri(1, 0);					Pre_JiTJi(0, 5) = -riWri(2, 0);
    Pre_JiTJi(1, 3) = -riWri(0, 1);					Pre_JiTJi(1, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(1, 5) = -riWri(2, 1);
    Pre_JiTJi(2, 3) = -riWri(0, 2);					Pre_JiTJi(2, 4) = -riWri(1, 2);					Pre_JiTJi(2, 5) = riWri(0, 0) + riWri(1, 1);
    //Pre_JiTJi.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_ref_sum_p_ref) - skewMatrixProduct(R_ref_sum_p_ref, t_ref) - fC.sum_weight_s * 1 * skewMatrixProduct(t_ref, t_ref);
    //Pre_JjTJj.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JjTJj(0, 3) = rjWrj(2, 2) + rjWrj(1, 1);	Pre_JjTJj(0, 4) = -rjWrj(1, 0);					Pre_JjTJj(0, 5) = -rjWrj(2, 0);
    Pre_JjTJj(1, 3) = -rjWrj(0, 1);					Pre_JjTJj(1, 4) = rjWrj(0, 0) + rjWrj(2, 2);	Pre_JjTJj(1, 5) = -rjWrj(2, 1);
    Pre_JjTJj(2, 3) = -rjWrj(0, 2);					Pre_JjTJj(2, 4) = -rjWrj(1, 2);					Pre_JjTJj(2, 5) = rjWrj(0, 0) + rjWrj(1, 1);
    //Pre_JjTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_new, R_new_sum_p_new) - skewMatrixProduct(R_new_sum_p_new, t_new) - fC.sum_weight_s * 1 * skewMatrixProduct(t_new, t_new);

    //Pre_JiTJj.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JiTJj(0, 3) = riWrj(2, 2) + riWrj(1, 1);	Pre_JiTJj(0, 4) = -riWrj(1, 0);	Pre_JiTJj(0, 5) = -riWrj(2, 0);
    Pre_JiTJj(1, 3) = -riWrj(0, 1);	Pre_JiTJj(1, 4) = riWrj(0, 0) + riWrj(2, 2);	Pre_JiTJj(1, 5) = -riWrj(2, 1);
    Pre_JiTJj(2, 3) = -riWrj(0, 2);	Pre_JiTJj(2, 4) = -riWrj(1, 2);		Pre_JiTJj(2, 5) = riWrj(0, 0) + riWrj(1, 1);
    //Pre_JiTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_new_sum_p_new) - skewMatrixProduct(R_ref_sum_p_ref, t_new) - fC.sum_weight_s * 1 * skewMatrixProduct(t_ref, t_new);
    Pre_JiTJj = -Pre_JiTJj;
    Pre_JjTJi.block<3,3>(0,3) = Pre_JiTJj.block<3,3>(0,3).transpose();
    
    #if 0
    std::cout << "precomputing jacobian matrics: " << SC.ref_submap_id << " " << SC.new_submap_id << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;
    std::cout << "JjTr:" << Pre_JjTr.transpose() << std::endl;
    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    std::cout << "JiTJj: " << std::endl << Pre_JiTJj << std::endl;
    std::cout << "JjTJi: " << std::endl << Pre_JjTJi << std::endl;
    std::cout << "JjTJj: " << std::endl << Pre_JjTJj << std::endl;
    #endif
}

void ComputeJacobianInfoRoomNormal(NormalCorrespondence &SC,
    ServerSLAM &server_slam,
    std::map<int, std::vector<int>> &room_to_pos, 
    PoseSE3dList &room_poses,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj)
{

    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();

    //prepare data

    int ref_room_id = SC.ref_room_id;
    int new_room_id = SC.new_room_id;
    int source_camera_id = SC.ref_camera_id;
    int target_camera_id = SC.new_camera_id;
    int room_pos_ref = room_to_pos[source_camera_id][ref_room_id];
    int room_pos_new = room_to_pos[target_camera_id][new_room_id];

    Eigen::Matrix3d R_ref = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos_ref]).rotationMatrix();
    Eigen::Vector3d t_ref = (server_slam.cameraStartPoses[source_camera_id] * room_poses[room_pos_ref]).translation();
    Eigen::Matrix3d R_new = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos_new]).rotationMatrix();
    Eigen::Vector3d t_new = (server_slam.cameraStartPoses[target_camera_id] * room_poses[room_pos_new]).translation();

    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri, rjWrj;
    riWrj = R_ref * SC.sum_p_ref_new_s * R_new.transpose();
    riWri = R_ref * SC.sum_p_ref_ref_s * R_ref.transpose();
    rjWrj = R_new * SC.sum_p_new_new_s * R_new.transpose();

    //Here we only care about the rotation
    //calculating JTr, see ProblemFormulation.pdf
    //Pre_JiTr.block<3, 1>(0, 0) = residual;


    Pre_JiTr.block<3, 1>(0, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1));

    Pre_JjTr.block<3, 1>(0, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1));
    Pre_JjTr = -Pre_JjTr;

    //calculating JTJ
    //Pre_JiTJi.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JiTJi(0, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(0, 4) = -riWri(1, 0);					Pre_JiTJi(0, 5) = -riWri(2, 0);
    Pre_JiTJi(1, 3) = -riWri(0, 1);					Pre_JiTJi(1, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(1, 5) = -riWri(2, 1);
    Pre_JiTJi(2, 3) = -riWri(0, 2);					Pre_JiTJi(2, 4) = -riWri(1, 2);					Pre_JiTJi(2, 5) = riWri(0, 0) + riWri(1, 1);
    //Pre_JiTJi.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_ref_sum_p_ref) - skewMatrixProduct(R_ref_sum_p_ref, t_ref) - fC.sum_weight_s * 1 * skewMatrixProduct(t_ref, t_ref);
    //Pre_JjTJj.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JjTJj(0, 3) = rjWrj(2, 2) + rjWrj(1, 1);	Pre_JjTJj(0, 4) = -rjWrj(1, 0);					Pre_JjTJj(0, 5) = -rjWrj(2, 0);
    Pre_JjTJj(1, 3) = -rjWrj(0, 1);					Pre_JjTJj(1, 4) = rjWrj(0, 0) + rjWrj(2, 2);	Pre_JjTJj(1, 5) = -rjWrj(2, 1);
    Pre_JjTJj(2, 3) = -rjWrj(0, 2);					Pre_JjTJj(2, 4) = -rjWrj(1, 2);					Pre_JjTJj(2, 5) = rjWrj(0, 0) + rjWrj(1, 1);
    //Pre_JjTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_new, R_new_sum_p_new) - skewMatrixProduct(R_new_sum_p_new, t_new) - fC.sum_weight_s * 1 * skewMatrixProduct(t_new, t_new);

    //Pre_JiTJj.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JiTJj(0, 3) = riWrj(2, 2) + riWrj(1, 1);	Pre_JiTJj(0, 4) = -riWrj(1, 0);	Pre_JiTJj(0, 5) = -riWrj(2, 0);
    Pre_JiTJj(1, 3) = -riWrj(0, 1);	Pre_JiTJj(1, 4) = riWrj(0, 0) + riWrj(2, 2);	Pre_JiTJj(1, 5) = -riWrj(2, 1);
    Pre_JiTJj(2, 3) = -riWrj(0, 2);	Pre_JiTJj(2, 4) = -riWrj(1, 2);		Pre_JiTJj(2, 5) = riWrj(0, 0) + riWrj(1, 1);
    //Pre_JiTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_new_sum_p_new) - skewMatrixProduct(R_ref_sum_p_ref, t_new) - fC.sum_weight_s * 1 * skewMatrixProduct(t_ref, t_new);
    Pre_JiTJj = -Pre_JiTJj;
    Pre_JjTJi.block<3,3>(0,3) = Pre_JiTJj.block<3,3>(0,3).transpose();
    
    #if 0
    std::cout << "precomputing jacobian matrics: " << SC.ref_submap_id << " " << SC.new_submap_id << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;
    std::cout << "JjTr:" << Pre_JjTr.transpose() << std::endl;
    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    std::cout << "JiTJj: " << std::endl << Pre_JiTJj << std::endl;
    std::cout << "JjTJi: " << std::endl << Pre_JjTJi << std::endl;
    std::cout << "JjTJj: " << std::endl << Pre_JjTJj << std::endl;
    #endif
}
//combining optimization in server
bool optimizeRoomAndSubmapCamera(Server::ServerSLAM &server_slam,int camera_id, bool from_frame)
{
    
    
    int node_id = server_slam.node_id[camera_id];
    auto &submap_correspondences = server_slam.camera_nodes[node_id].correspondences;
    auto &room_correspondences = server_slam.camera_nodes[node_id].room_correspondences;
    std::cout<<"Start to optimize room and submap for all camera..."<<std::endl;
    std::vector<Correspondence> correspondences_we_need;//the correspondences which do not belong into the same room.
    //first, extract the variable need to be optimize.
    int roomOptNum = 0;
    int submapOptNum = 0;
    //int latest_keyframe_camera = server_slam.latest_keyframe_camera;
    int latest_keyframe_index = server_slam.latest_keyframe_index;
    int latest_room_index = server_slam.latest_room_index;
    for(int i = 0; i != submap_correspondences.size(); ++i)
    {
      if(!submap_correspondences[i].sameCamera)
       correspondences_we_need.push_back(submap_correspondences[i]);
      else
      {
        int camera_id = submap_correspondences[i].frame_ref.cameraID;
        int submap_id_ref = submap_correspondences[i].frame_ref.submapID;
        int submap_id_new = submap_correspondences[i].frame_new.submapID;
        auto &submap_to_room = server_slam.submap_to_room_for_each_camera[camera_id];
        if(submap_to_room[submap_id_ref] == -1 || submap_to_room[submap_id_new] == -1 
          ||submap_to_room[submap_id_ref] != submap_to_room[submap_id_new])
          {
            if(submap_correspondences[i].use_icp)
            {
            std::cout<<"ref/room: "<<submap_id_ref<<"/"<<submap_to_room[submap_id_ref]
            <<" new/room: "<<submap_id_new<<"/"<<submap_to_room[submap_id_new]<<" is_dense: "<<submap_correspondences[i].use_icp<<std::endl;
            }
            correspondences_we_need.push_back(submap_correspondences[i]);
          }
      }
    }
    for(int i = 0; i != room_correspondences.size(); ++i)
    {
      room_correspondences[i].preIntegrate(server_slam.room_poses);
    }
    for(int i = 0; i < correspondences_we_need.size(); ++i)
    {
      if(!correspondences_we_need[i].is_valid)
        continue;
      if(correspondences_we_need[i].use_icp)
        correspondences_we_need[i].preIntegrateICPInterCamera(MultiViewGeometry::g_para.icp_weight);
      else
        correspondences_we_need[i].preIntegrateInterCamera();
    }
    int equation_num = correspondences_we_need.size() + room_correspondences.size();

    std::map<int,std::vector<int>>  room_to_pos;
    std::vector<pair<int, int>> pos_to_room;
    std::map<int, std::vector<int>> submap_to_pos;
    std::vector<pair<int, int>> pos_to_submap;
    for(auto i =server_slam.camera_nodes[node_id].camera.begin() ;i!= server_slam.camera_nodes[node_id].camera.end();++i)
    { 
        auto &rooms = server_slam.rooms_for_each_camera[*i];
        room_to_pos[*i].resize(rooms.size(),-1);
        for(int j = 0; j < rooms.size(); ++j) 
        {
          //*i camera_id, j room_id
          room_to_pos[*i][j] = roomOptNum;
          pos_to_room.push_back(std::make_pair(*i, j));
          roomOptNum += 1;
        }
    }

    for(auto i = server_slam.camera_nodes[node_id].camera.begin(); i!= server_slam.camera_nodes[node_id].camera.end();++i)
    {
      auto &submap_to_room = server_slam.submap_to_room_for_each_camera[*i];
      std::cout<<"camera: "<<*i<<std::endl;
      submap_to_pos[*i].resize(submap_to_room.size(), -1);
      for(int j = 0; j != submap_to_room.size(); ++j)
      {
        if(submap_to_room[j] == -1)
        {
          std::cout<<j<<" ";
          submap_to_pos[*i][j] = submapOptNum;
          pos_to_submap.push_back(std::make_pair(*i,j));
          submapOptNum +=1;
        }
      }
      std::cout<<std::endl;
    }


    int optNum = submapOptNum + roomOptNum;
    //std::cout<<optNum<<" "<<equation_num<<std::endl;
    if(optNum < 3) 
    {
        std::cout<<"optNum is less than 3, no need to optimize."<<std::endl;
        return true;
    }
    if(equation_num < 3)
    {
        std::cout<<"equation_num is less than 3, no need to optimize."<<std::endl;
        return true;      
    }
    // also use a submap poses list to restore the relative changes, and so the form can be consistent to the room poses.
    // which means we consider each submap(not clusttered into any room) as a individual room
    PoseSE3dList room_poses(roomOptNum,PoseSE3d());
    PoseSE3dList submap_poses_delta(submapOptNum, PoseSE3d());
    std::cout<<"variables wait to be optimized: "<<optNum<<std::endl;
    Eigen::MatrixXd J, err;
    Eigen::MatrixXd delta(6 * (optNum-1), 1), JTe(6 * (optNum-1), 1);
    Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> JTJ(6 * (optNum-1), 6 * (optNum-1));


    // the solver is only built at the first iteration
    Eigen::SimplicialLDLT	<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> > SimplicialLDLTSolver;
    std::vector<Eigen::Triplet<SPARSE_MATRIX_NUM_TYPE>> coeff;
    coeff.reserve(6 * 6 * 4 * equation_num);
    Eigen::MatrixXd JiTJi_pre(6, 6), JiTJj_pre(6, 6), JjTJi_pre(6, 6), JjTJj_pre(6, 6), JiTe_pre(6, 1), JjTe_pre(6, 1);


    clock_t start_opt, end_opt, start, end;
    double time_opt;
    start_opt = clock();

    float init_error =  reprojection_error_3Dto3D_room_submap(room_correspondences, correspondences_we_need, 
      server_slam, room_to_pos, submap_to_pos, room_poses, submap_poses_delta);
    float final_error=1.0;
    float last_final_error = 1.0;
    std::cout<<"init error: "<<init_error<<std::endl;
    for (int iter = 0; iter<3||iter < 10 && final_error > 0.0003; iter++)
    {
      JTe.setZero();
      JTJ.setZero();
      err.setZero();
      coeff.clear();

      double time_framePair;
      double time_generatingJacobian;
      double time_buildSolver;
      start = clock();
      float robust_weight;

        for(int i = 0; i < room_correspondences.size(); ++i)
        {
            //Do something similiar to the follows
            int source_camera_id = room_correspondences[i].source_camera_id;
            int target_camera_id = room_correspondences[i].target_camera_id;


            int source_room_id = room_correspondences[i].source_id;
            int target_room_id = room_correspondences[i].target_id;

            int room_ref_pos = room_to_pos[source_camera_id][source_room_id];
            int room_new_pos = room_to_pos[target_camera_id][target_room_id];

            if(room_ref_pos == room_new_pos) continue;
            if(room_ref_pos == -1 || room_new_pos == -1)
            {
                std::cout<<"Something wrong about the room correspondences."<<std::endl;
                continue;
            }

            robust_weight = 1;
            //std::cout<<"Compute JacobianInfo of Rooms..."<<std::endl;
            ComputeJacobianInfoRoom(room_correspondences[i],
                server_slam,
                room_to_pos,
                room_poses,
                JiTe_pre,
                JjTe_pre,
                JiTJi_pre,
                JiTJj_pre,
                JjTJi_pre,
                JjTJj_pre);

            JiTe_pre *= robust_weight;
            JjTe_pre *= robust_weight;
            JiTJi_pre *= robust_weight;
            JiTJj_pre *= robust_weight;
            JjTJj_pre *= robust_weight;
            JjTJi_pre *= robust_weight;
            
            if(room_ref_pos < room_new_pos)
            {
                if (room_ref_pos == 0)
                {
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos - 1) * 6);
                JTe.block<6, 1>((room_new_pos - 1) * 6, 0) += JjTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (room_ref_pos - 1) * 6, (room_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (room_new_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((room_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            else
            {
                if (room_new_pos == 0)
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos - 1) * 6, (room_ref_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos - 1) * 6, 0) += JiTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (room_ref_pos - 1) * 6, (room_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (room_new_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((room_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            
        }

        for (int i = 0; i < correspondences_we_need.size(); i++)
        {
            ServerFrame &frame_ref = correspondences_we_need[i].frame_ref;
            ServerFrame &frame_new = correspondences_we_need[i].frame_new;
            int source_camera_id = frame_ref.cameraID;
            int target_camera_id = frame_new.cameraID;

            int source_submap_id = frame_ref.submapID;
            int target_submap_id = frame_new.submapID;

            int frame_ref_pos = submap_to_pos[source_camera_id][source_submap_id];
            int frame_new_pos = submap_to_pos[target_camera_id][target_submap_id];

            if(frame_ref_pos == -1)
            {
                frame_ref_pos = 
                  room_to_pos[source_camera_id][server_slam.submap_to_room_for_each_camera[source_camera_id][source_submap_id]];
            }
            else 
            frame_ref_pos += roomOptNum;

            if(frame_new_pos == -1)
            {
                frame_new_pos = 
                  room_to_pos[target_camera_id][server_slam.submap_to_room_for_each_camera[target_camera_id][target_submap_id]];
            }
            else 
            frame_new_pos += roomOptNum;

            if (frame_ref_pos < 0 || frame_new_pos < 0)
            {
                continue;
            }
            robust_weight = 1.0f;


            clock_t start_jacobian, end_jacobian;
            start_jacobian = clock();
            //std::cout<<"Compute JacobianInfo of submaps..."<<std::endl;
            ComputeJacobianInfoRoomSubmap(correspondences_we_need[i],
                server_slam,
                room_to_pos,
                submap_to_pos,
                room_poses, 
                submap_poses_delta,
                JiTe_pre,
                JjTe_pre,
                JiTJi_pre,
                JiTJj_pre,
                JjTJi_pre,
                JjTJj_pre);
            //std::cout<<"pos: "<<frame_ref_pos<<" "<<frame_new_pos<<std::endl;
            JiTe_pre *= robust_weight;
            JjTe_pre *= robust_weight;
            JiTJi_pre *= robust_weight;
            JiTJj_pre *= robust_weight;
            JjTJj_pre *= robust_weight;
            JjTJi_pre *= robust_weight;

            if(frame_ref_pos < frame_new_pos)
            {
                if (frame_ref_pos == 0)
                {
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (frame_ref_pos - 1) * 6, (frame_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (frame_new_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            else
            {
                if (frame_new_pos == 0)
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (frame_ref_pos - 1) * 6, (frame_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (frame_new_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
        }

            

            end = clock();
            time_framePair = (double)(end - start) / CLOCKS_PER_SEC * 1000;
            start = clock();
            JTJ.setFromTriplets(coeff.begin(), coeff.end());
            end = clock();
            time_generatingJacobian = (double)(end - start) / CLOCKS_PER_SEC * 1000;
            start = clock();
            SimplicialLDLTSolver.compute(JTJ);
            end = clock();
            time_buildSolver = (double)(end - start) / CLOCKS_PER_SEC * 1000;


            // update the pose of each frame
            start = clock();
            delta = SimplicialLDLTSolver.solve(JTe);
            end = clock();
            double time_svd = (double)(end - start) / CLOCKS_PER_SEC * 1000;
            for (int i = 1; i < room_poses.size(); i++)
            {
                int camera_id = pos_to_room[i].first;
                Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (i - 1), 0);
                if(isnan(delta_i(0)))
                {
                std::cout << "nan detected in pose update! " << std::endl;
                continue;
                }
                //std::cout<<i<<" r "<<delta_i<<std::endl<<std::endl;
                /* refine all the keyframes' poses in this submap*/
                //if(submapID == 0) continue;
                auto tmp_global_pose = server_slam.cameraStartPoses[camera_id] * room_poses[i];
                tmp_global_pose =  Sophus::SE3d::exp(delta_i).inverse() 
                * tmp_global_pose;

                room_poses[i] = server_slam.cameraStartPoses[camera_id].inverse() * tmp_global_pose;

            }

            for(int i = (room_poses.size() == 0); i != submap_poses_delta.size(); ++i)
            {
                int position = room_poses.size() + i;
                int camera_id = pos_to_submap[i].first;
                //int submap_id = position_to_submap[position];
                Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (position - 1), 0);
                if(isnan(delta_i(0)))
                {
                std::cout << "nan detected in pose update! " << std::endl;
                continue;
                }
                //std::cout<<i<<" s "<<delta_i<<std::endl<<std::endl;
                /* refine all the keyframes' poses in this submap*/
                //if(submapID == 0) continue;
                auto tmp_global_pose = server_slam.cameraStartPoses[camera_id] * submap_poses_delta[i];
                tmp_global_pose =  Sophus::SE3d::exp(delta_i).inverse() 
                * tmp_global_pose;

                submap_poses_delta[i] = server_slam.cameraStartPoses[camera_id].inverse() * tmp_global_pose;
                // this is used for update all the submap poses
                
            }
            double time_calculate_reprojection_error;
            double refined_error;
            last_final_error = final_error;
            final_error = reprojection_error_3Dto3D_room_submap(room_correspondences, correspondences_we_need, 
      server_slam, room_to_pos, submap_to_pos, room_poses, submap_poses_delta);
            if(std::fabs(final_error - last_final_error)< 0.000001) break;
            
     }
    std::cout<<"after: "<<std::endl;
    for(int i = 0; i != room_poses.size(); ++i)
    {
        std::cout<<room_poses[i].matrix()<<std::endl<<std::endl;
    }
    // this is used for update all the submap poses


    //update all the keyframe poses
    end_opt = clock();

    time_opt = (double )(end_opt - start_opt) / CLOCKS_PER_SEC * 1000;

    std::cout << "init/final error " << init_error << "/" << final_error << std::endl;


    double node_last_final_error = server_slam.camera_nodes[node_id].last_final_error;
    std::cout<<"node_last_final_error/final error: "<<node_last_final_error<<"/"<<final_error<<std::endl;
    if((final_error - init_error) / init_error > 0.2  ||final_error > 0.005 
      &&(final_error -node_last_final_error) / node_last_final_error > 0.8)
    // if((final_error - init_error) / init_error > 0.1  ||final_error > 0.01
    //   &&(final_error -node_last_final_error) / node_last_final_error > 0.8)
    {
      //final_error > 0.003 &&(final_error -node_last_final_error) / node_last_final_error > 0.8
      //(final_error -node_last_final_error) / node_last_final_error > 0.5
        // remove all outliers 
        std::cout<<"Error is too large. Try to remove the correspondence."<<std::endl;
        if(from_frame )
        {
          for (int i = 0; i < submap_correspondences.size(); i++)
          {
              ServerFrame &frame_ref = submap_correspondences[i].frame_ref;
              ServerFrame &frame_new = submap_correspondences[i].frame_new;

              if( frame_new.frame_index == server_slam.keyframes[camera_id].rbegin()->second.frame_index && frame_new.cameraID == camera_id)
              {
                  if( submap_correspondences[i].matches.size() > 0 || submap_correspondences[i].data_pairs_3d.size() > 0)
                  {
                      std::cout << "try to remove: "<< " " << frame_ref.frame_index << " " << frame_new.frame_index << std::endl;
                      if(submap_correspondences[i].connection == false)
                      {
                        submap_correspondences[i].setIsValid(false);
                        submap_correspondences[i].reset();
                      }
                  }
              }
          }
          return false;
        }
        /*
        else
        {
          for (int i = 0; i < room_correspondences.size(); ++i)
          {
            int source_room_id = room_correspondences[i].source_id;
            int target_room_id = room_correspondences[i].target_id;
            int source_camera_id = room_correspondences[i].source_camera_id;
            int target_camera_id = room_correspondences[i].target_camera_id;

            if(source_room_id == latest_room_index  && source_camera_id == camera_id || 
              target_room_id == latest_room_index && target_camera_id == camera_id)
            {
              if(room_correspondences[i].instance_center_correspondences.size()>0)
              {
                std::cout << "try to remove: "<< " " << source_room_id << " " << target_room_id << std::endl;
                room_correspondences[i].reset();
              }
            }

          }
        }
        */

    }
    auto & submapPosesRelativeChanges = server_slam.submapPosesRelativeChanges;
    auto & submapPosesFinal = server_slam.submapPosesFinal;
    for(int i = 0; i != room_poses.size(); ++i)
    {
          int camera_id = pos_to_room[i].first;
          int room_id = pos_to_room[i].second;
          auto &room_info = server_slam.rooms_for_each_camera[camera_id][room_id];

        server_slam.room_poses[camera_id][room_id] = room_poses[i] * server_slam.room_poses[camera_id][room_id];
        for(auto j = room_info.contained_submaps.begin();
            j!=room_info.contained_submaps.end(); ++j)
        {
            submapPosesRelativeChanges[camera_id][*j] = 
                room_poses[i] * submapPosesRelativeChanges[camera_id][*j];
            //std::cout<<submapPosesFinal[camera_id][*j].matrix()<<std::endl<<std::endl;
            submapPosesFinal[camera_id][*j] = 
                room_poses[i] * submapPosesFinal[camera_id][*j];
            //std::cout<<submapPosesFinal[camera_id][*j].matrix()<<std::endl<<std::endl;

            //std::cout<<server_slam.submapPosesFinal[camera_id][*j].matrix()<<std::endl<<std::endl;
        }
    }

    for(int i = 0; i != submap_poses_delta.size(); ++i)
    {
        //remember to switch position with submap_id 
        int camera_id = pos_to_submap[i].first;
        int submap_id = pos_to_submap[i].second;

        if(submap_id < 0)
        {
            std::cout<<"something wrong with the table position_to_submap."<<std::endl;
            continue;
        }
        submapPosesRelativeChanges[camera_id][submap_id] = 
            submap_poses_delta[i] * submapPosesRelativeChanges[camera_id][submap_id];
        submapPosesFinal[camera_id][submap_id] = 
            submap_poses_delta[i] * submapPosesFinal[camera_id][submap_id];
    }
    server_slam.camera_nodes[node_id].last_final_error = final_error;
    return true;
}

bool optimizeRoomAndSubmapCameraWithNormal(Server::ServerSLAM &server_slam,int camera_id, bool from_frame)
{
    
    
    int node_id = server_slam.node_id[camera_id];
    auto &submap_correspondences = server_slam.camera_nodes[node_id].correspondences;
    auto &room_correspondences = server_slam.camera_nodes[node_id].room_correspondences;
    //std::vector<NormalCorrespondence> normal_correspondences;
    std::vector<NormalGravityCorr> normal_correspondences;
    std::cout<<"Start to optimize room and submap for all camera..."<<std::endl;
    std::vector<Correspondence> correspondences_we_need;//the correspondences which do not belong into the same room.
    //first, extract the variable need to be optimize.
    int roomOptNum = 0;
    int submapOptNum = 0;
    //int latest_keyframe_camera = server_slam.latest_keyframe_camera;
    //int latest_keyframe_index = server_slam.latest_keyframe_index;
    //int latest_room_index = server_slam.latest_room_index;
    for(int i = 0; i != submap_correspondences.size(); ++i)
    {
      if(!submap_correspondences[i].is_valid)
      continue;
      if(!submap_correspondences[i].sameCamera )
       correspondences_we_need.push_back(submap_correspondences[i]);
      else
      {
        int camera_id = submap_correspondences[i].frame_ref.cameraID;
        int submap_id_ref = submap_correspondences[i].frame_ref.submapID;
        int submap_id_new = submap_correspondences[i].frame_new.submapID;
        if(submap_id_ref < server_slam.submapPosesFinal[camera_id].size() && submap_id_new < server_slam.submapPosesFinal[camera_id].size())
        {
          auto &submap_to_room = server_slam.submap_to_room_for_each_camera[camera_id];
          if(submap_to_room[submap_id_ref] == -1 || submap_to_room[submap_id_new] == -1 
            ||submap_to_room[submap_id_ref] != submap_to_room[submap_id_new])
            {
              if(submap_correspondences[i].use_icp)
              {
              std::cout<<"ref/room: "<<submap_id_ref<<"/"<<submap_to_room[submap_id_ref]
              <<" new/room: "<<submap_id_new<<"/"<<submap_to_room[submap_id_new]<<" is_dense: "<<submap_correspondences[i].use_icp<<std::endl;
              }
              correspondences_we_need.push_back(submap_correspondences[i]);
            }
        }
      }
    }

    for(int i = 0; i != room_correspondences.size(); ++i)
    {
      room_correspondences[i].preIntegrate(server_slam.room_poses);      
      int source_camera_id = room_correspondences[i].source_camera_id;
      int target_camera_id = room_correspondences[i].target_camera_id;
      int source_room_id = room_correspondences[i].source_id;
      int target_room_id = room_correspondences[i].target_id;
/*
      normal_correspondences.emplace_back(1, source_camera_id, source_room_id, target_camera_id, target_room_id, 
        server_slam.rooms_for_each_camera[source_camera_id][source_room_id].floor_normal, 
        server_slam.rooms_for_each_camera[target_camera_id][target_room_id].floor_normal);
*/
    }
    //std::cout<<server_slam.submap_floor_normals[0].size()<<" "<<server_slam.submapPoses[0].size()<<std::endl;
    for(int i = 0; i < correspondences_we_need.size(); ++i)
    {
      if(correspondences_we_need[i].use_icp)
        correspondences_we_need[i].preIntegrateICPInterCamera(MultiViewGeometry::g_para.icp_weight);
      else
        correspondences_we_need[i].preIntegrateInterCamera();
      //std::cout<<correspondences_we_need[i].sum_weight_c<<std::endl;
      int source_camera_id = correspondences_we_need[i].frame_ref.cameraID;
      int target_camera_id = correspondences_we_need[i].frame_new.cameraID;
      int ref_submap_id =  correspondences_we_need[i].frame_ref.submapID;
      int new_submap_id = correspondences_we_need[i].frame_new.submapID;
      //Here source camera id is always equal to target camera id.
/*      
      normal_correspondences.emplace_back(0, source_camera_id, ref_submap_id, target_camera_id, new_submap_id, 
        server_slam.submap_floor_normals[source_camera_id][ref_submap_id], server_slam.submap_floor_normals[target_camera_id][new_submap_id]);
*/
    }
/*
    for(int i = 0; i != normal_correspondences.size(); ++i)
    {
      std::cout<<normal_correspondences[i].is_valid<<" ref/new normal: ("<<normal_correspondences[i].ref_normal[0]<<", "
        <<normal_correspondences[i].ref_normal[1]<<", "<<normal_correspondences[i].ref_normal[2]<<")/("<<normal_correspondences[i].new_normal[0]<<", "
        <<normal_correspondences[i].new_normal[1]<<", "<<normal_correspondences[i].new_normal[2]<<")"<<std::endl;
      normal_correspondences[i].preIntegrate(server_slam.submapPosesRelativeChanges);
    }
*/



    std::map<int,std::vector<int>>  room_to_pos;
    std::vector<pair<int, int>> pos_to_room;
    std::map<int, std::vector<int>> submap_to_pos;
    std::vector<pair<int, int>> pos_to_submap;
    bool set_gravity = false;
    for(auto i =server_slam.camera_nodes[node_id].camera.begin() ;i!= server_slam.camera_nodes[node_id].camera.end();++i)
    { 
        auto &rooms = server_slam.rooms_for_each_camera[*i];
        room_to_pos[*i].resize(rooms.size(),-1);
        for(int j = 0; j < rooms.size(); ++j) 
        {
          //*i camera_id, j room_id
          room_to_pos[*i][j] = roomOptNum;
          pos_to_room.push_back(std::make_pair(*i, j));
          roomOptNum += 1;
          if(!set_gravity)
          {
            if(rooms[j].floor_normal[0] != 999.0)
            {
              Gravity =rooms[j].floor_normal.cast<double>();
              set_gravity = true;
            }
          }
          normal_correspondences.emplace_back(1, *i, j, rooms[j].floor_normal);
        }
    }

    for(auto i = server_slam.camera_nodes[node_id].camera.begin(); i!= server_slam.camera_nodes[node_id].camera.end();++i)
    {
      auto &submap_to_room = server_slam.submap_to_room_for_each_camera[*i];
      //std::cout<<"camera: "<<*i<<std::endl;
      submap_to_pos[*i].resize(submap_to_room.size(), -1);
      for(int j = 0; j != submap_to_room.size(); ++j)
      {
        if(submap_to_room[j] == -1)
        {
          //std::cout<<j<<" ";
          submap_to_pos[*i][j] = submapOptNum;
          pos_to_submap.push_back(std::make_pair(*i,j));
          if(!set_gravity)
          {
            if(server_slam.submap_floor_normals[*i][j](0) != 999.0)
            {
              Gravity =server_slam.submap_floor_normals[*i][j].cast<double>();
              set_gravity = true;
            }
          }
          normal_correspondences.emplace_back(0, *i, j, server_slam.submap_floor_normals[*i][j]);

          submapOptNum +=1;
        }
      }
      //std::cout<<std::endl;
    }
    for(int i = 0; i != normal_correspondences.size(); ++i)
    {
      /*
      std::cout<<"Normal: "<<normal_correspondences[i].normal(0)<<", "<<normal_correspondences[i].normal(1)
        <<", "<<normal_correspondences[i].normal(2)<<std::endl;
      std::cout<<"Gravity: "<<Gravity.transpose()<<std::endl;
      */
      normal_correspondences[i].preIntegrate(server_slam.submapPosesRelativeChanges, server_slam.room_poses);
    }
    
    int without_normal_equation_num = correspondences_we_need.size() + room_correspondences.size();
    int equation_num = correspondences_we_need.size() + room_correspondences.size() + normal_correspondences.size();
    int optNum = submapOptNum + roomOptNum;
    std::cout<<optNum<<" "<<equation_num<<std::endl;

    if(optNum < 3) 
    {
        std::cout<<"optNum is less than 3, no need to optimize."<<std::endl;
        return true;
    }
    if(without_normal_equation_num< 3)
    {
        std::cout<<"equation_num is less than 3, no need to optimize."<<std::endl;
        return true;      
    }

    // also use a submap poses list to restore the relative changes, and so the form can be consistent to the room poses.
    // which means we consider each submap(not clusttered into any room) as a individual room
    PoseSE3dList room_poses(roomOptNum,PoseSE3d());
    PoseSE3dList submap_poses_delta(submapOptNum, PoseSE3d());
    std::cout<<"variables wait to be optimized: "<<optNum<<std::endl;
    Eigen::MatrixXd J, err;
    Eigen::MatrixXd delta(6 * (optNum - 1), 1), JTe(6 * (optNum - 1), 1);
    Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> JTJ(6 * (optNum - 1) , 6 * (optNum - 1));


    // the solver is only built at the first iteration
    Eigen::SimplicialLDLT	<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> > SimplicialLDLTSolver;

    //Eigen::SparseLU<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE>, COLAMDOrdering<int> > SparseLuSolver;
    //Eigen::SparseQR<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE>, COLAMDOrdering<int> > SparseQRSolver;
    std::vector<Eigen::Triplet<SPARSE_MATRIX_NUM_TYPE>> coeff;
    coeff.reserve(6 * 6 * 4 * without_normal_equation_num + 3 * 6 * 4 * normal_correspondences.size());
    Eigen::MatrixXd JiTJi_pre(6, 6), JiTJj_pre(6, 6), JjTJi_pre(6, 6), JjTJj_pre(6, 6), JiTe_pre(6, 1), JjTe_pre(6, 1);
    Eigen::MatrixXd JiTJi_pre_n(3, 6),  JiTJj_pre_n(3, 6), JjTJi_pre_n(3, 6), JjTJj_pre_n(3, 6), JiTe_pre_n(3, 1), JjTe_pre_n(3, 1);

    clock_t start_opt, end_opt, start, end;
    double time_opt;
    start_opt = clock();

    float init_error =  reprojection_error_3Dto3D_room_submap(room_correspondences, correspondences_we_need, 
      server_slam, room_to_pos, submap_to_pos, room_poses, submap_poses_delta);
    float final_error=1.0;
    float last_final_error = 1.0;
    float normal_init_error = 0, normal_final_error = 0;
    float weight_normal = 0.3;
    normal_init_error = reprojection_error_normal_gravity(normal_correspondences, server_slam,
      room_to_pos, submap_to_pos, room_poses, submap_poses_delta);
    float combine_init_error =  (1 - weight_normal) * init_error + weight_normal * normal_init_error; 
    float combine_final_error = 1;



    std::cout<<"init error: "<<init_error<<std::endl;

    for (int iter = 0; iter<3||iter < 10 && combine_final_error > 0.0003; iter++)
    {
      JTe.setZero();
      JTJ.setZero();
      err.setZero();
      coeff.clear();

      double time_framePair;
      double time_generatingJacobian;
      double time_buildSolver;
      start = clock();
      float robust_weight;
#if 1
        for(int i = 0; i < room_correspondences.size(); ++i)
        {
            //Do something similiar to the follows
            int source_camera_id = room_correspondences[i].source_camera_id;
            int target_camera_id = room_correspondences[i].target_camera_id;


            int source_room_id = room_correspondences[i].source_id;
            int target_room_id = room_correspondences[i].target_id;

            int room_ref_pos = room_to_pos[source_camera_id][source_room_id];
            int room_new_pos = room_to_pos[target_camera_id][target_room_id];

            if(room_ref_pos == room_new_pos) continue;
            if(room_ref_pos == -1 || room_new_pos == -1)
            {
                std::cout<<"Something wrong about the room correspondences."<<std::endl;
                continue;
            }

            robust_weight = 1;
            //std::cout<<"Compute JacobianInfo of Rooms..."<<std::endl;
            ComputeJacobianInfoRoom(room_correspondences[i],
                server_slam,
                room_to_pos,
                room_poses,
                JiTe_pre,
                JjTe_pre,
                JiTJi_pre,
                JiTJj_pre,
                JjTJi_pre,
                JjTJj_pre);

            JiTe_pre *= robust_weight;
            JjTe_pre *= robust_weight;
            JiTJi_pre *= robust_weight;
            JiTJj_pre *= robust_weight;
            JjTJj_pre *= robust_weight;
            JjTJi_pre *= robust_weight;
            
            if(room_ref_pos < room_new_pos)
            {
                if (room_ref_pos == 0)
                {
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos- 1 ) * 6);
                JTe.block<6, 1>((room_new_pos - 1 ) * 6, 0) += JjTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (room_ref_pos - 1) * 6, (room_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (room_new_pos- 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((room_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            else
            {
                if (room_new_pos == 0)
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos - 1) * 6, (room_ref_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos- 1) * 6, 0) += JiTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos- 1 ) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (room_ref_pos - 1) * 6, (room_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (room_new_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((room_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            
        }

        for (int i = 0; i < correspondences_we_need.size(); i++)
        {
            ServerFrame &frame_ref = correspondences_we_need[i].frame_ref;
            ServerFrame &frame_new = correspondences_we_need[i].frame_new;
            int source_camera_id = frame_ref.cameraID;
            int target_camera_id = frame_new.cameraID;

            int source_submap_id = frame_ref.submapID;
            int target_submap_id = frame_new.submapID;

            int frame_ref_pos = submap_to_pos[source_camera_id][source_submap_id];
            int frame_new_pos = submap_to_pos[target_camera_id][target_submap_id];

            if(frame_ref_pos == -1)
            {
                frame_ref_pos = 
                  room_to_pos[source_camera_id][server_slam.submap_to_room_for_each_camera[source_camera_id][source_submap_id]];
            }
            else 
            frame_ref_pos += roomOptNum;

            if(frame_new_pos == -1)
            {
                frame_new_pos = 
                  room_to_pos[target_camera_id][server_slam.submap_to_room_for_each_camera[target_camera_id][target_submap_id]];
            }
            else 
            frame_new_pos += roomOptNum;

            if (frame_ref_pos < 0 || frame_new_pos < 0)
            {
                continue;
            }
            robust_weight = 1.0f;


            clock_t start_jacobian, end_jacobian;
            start_jacobian = clock();
            //std::cout<<"Compute JacobianInfo of submaps..."<<std::endl;
            ComputeJacobianInfoRoomSubmap(correspondences_we_need[i],
                server_slam,
                room_to_pos,
                submap_to_pos,
                room_poses, 
                submap_poses_delta,
                JiTe_pre,
                JjTe_pre,
                JiTJi_pre,
                JiTJj_pre,
                JjTJi_pre,
                JjTJj_pre);
            //std::cout<<"pos: "<<frame_ref_pos<<" "<<frame_new_pos<<std::endl;
            JiTe_pre *= robust_weight;
            JjTe_pre *= robust_weight;
            JiTJi_pre *= robust_weight;
            JiTJj_pre *= robust_weight;
            JjTJj_pre *= robust_weight;
            JjTJi_pre *= robust_weight;

            if(frame_ref_pos < frame_new_pos)
            {
                if (frame_ref_pos == 0)
                {
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (frame_ref_pos - 1) * 6, (frame_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (frame_new_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            else
            {
                if (frame_new_pos == 0)
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (frame_ref_pos - 1) * 6, (frame_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (frame_new_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
        }
#endif
#if 0
        for (int i = 0; i < normal_correspondences.size(); i++)
        {
            //std::cout<<"Ref: "<<normal_correspondences[i].ref_submap_id<<normal_correspondences[i].ref_normal<<std::endl;
            //std::cout<<"New: "<<normal_correspondences[i].new_submap_id<<normal_correspondences[i].new_normal<<std::endl; 
             
            if(!normal_correspondences[i].is_valid)
            continue;        
            int ref_pos, new_pos;
            if(normal_correspondences[i].type == 0)
            {

              int source_camera_id = normal_correspondences[i].ref_camera_id;
              int target_camera_id = normal_correspondences[i].new_camera_id;

              int source_submap_id = normal_correspondences[i].ref_submap_id;
              int target_submap_id = normal_correspondences[i].new_submap_id;

              int frame_ref_pos = submap_to_pos[source_camera_id][source_submap_id];
              int frame_new_pos = submap_to_pos[target_camera_id][target_submap_id];

              if(frame_ref_pos == -1)
              {
                  frame_ref_pos = 
                    room_to_pos[source_camera_id][server_slam.submap_to_room_for_each_camera[source_camera_id][source_submap_id]];
              }
              else 
              frame_ref_pos += roomOptNum;

              if(frame_new_pos == -1)
              {
                  frame_new_pos = 
                    room_to_pos[target_camera_id][server_slam.submap_to_room_for_each_camera[target_camera_id][target_submap_id]];
              }
              else 
              frame_new_pos += roomOptNum;

              if (frame_ref_pos < 0 || frame_new_pos < 0)
              {
                  continue;
              }
              robust_weight = 1.0f;


              clock_t start_jacobian, end_jacobian;
              start_jacobian = clock();
              //std::cout<<"Compute JacobianInfo of submaps..."<<std::endl;
              ComputeJacobianInfoSubmapNormal(normal_correspondences[i],
                  server_slam,
                  room_to_pos,
                  submap_to_pos,
                  room_poses, 
                  submap_poses_delta,
                  JiTe_pre_n,
                  JjTe_pre_n,
                  JiTJi_pre_n,
                  JiTJj_pre_n,
                  JjTJi_pre_n,
                  JjTJj_pre_n);
              ref_pos = frame_ref_pos;
              new_pos = frame_new_pos;
            }
            else
            {
              int source_camera_id = room_correspondences[i].source_camera_id;
              int target_camera_id = room_correspondences[i].target_camera_id;


              int source_room_id = room_correspondences[i].source_id;
              int target_room_id = room_correspondences[i].target_id;

              int room_ref_pos = room_to_pos[source_camera_id][source_room_id];
              int room_new_pos = room_to_pos[target_camera_id][target_room_id];

              if(room_ref_pos == room_new_pos) continue;
              if(room_ref_pos == -1 || room_new_pos == -1)
              {
                  std::cout<<"Something wrong about the room correspondences."<<std::endl;
                  continue;
              }

              robust_weight = 1;
              //std::cout<<"Compute JacobianInfo of Rooms..."<<std::endl;
              ComputeJacobianInfoRoomNormal(normal_correspondences[i],
                  server_slam,
                  room_to_pos,
                  room_poses, 
                  JiTe_pre_n,
                  JjTe_pre_n,
                  JiTJi_pre_n,
                  JiTJj_pre_n,
                  JjTJi_pre_n,
                  JjTJj_pre_n);            
              //std::cout<<"pos: "<<frame_ref_pos<<" "<<frame_new_pos<<std::endl;
              JiTe_pre_n *= robust_weight;
              JjTe_pre_n *= robust_weight;
              JiTJi_pre_n *= robust_weight;
              JiTJj_pre_n *= robust_weight;
              JjTJj_pre_n *= robust_weight;
              JjTJi_pre_n *= robust_weight;
              ref_pos = room_ref_pos;
              new_pos = room_new_pos;
            }

            if(ref_pos < new_pos)
            {
                if (ref_pos == 0)
                {
                addBlockToTriplets(coeff, JjTJj_pre_n, (new_pos - 1) * 3 + 6 * (optNum-1), (new_pos - 1) * 6);
                JTe.block<3, 1>((new_pos - 1) * 3 + 6 * (optNum-1), 0) += JjTe_pre_n;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre_n, (ref_pos - 1) * 3 + 6 * (optNum-1), (ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre_n, (ref_pos - 1) * 3 + 6 * (optNum-1), (new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre_n, (new_pos - 1) * 3 + 6 * (optNum-1), (ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre_n, (new_pos - 1) * 3 + 6 * (optNum-1), (new_pos - 1) * 6);
                JTe.block<3, 1>((ref_pos - 1) * 3 + 6 * (optNum-1), 0) += JiTe_pre_n;
                JTe.block<3, 1>((new_pos - 1) * 3 + 6 * (optNum-1), 0) += JjTe_pre_n;
                }
            }
            else
            {
                if (new_pos == 0)
                {
                addBlockToTriplets(coeff, JiTJi_pre_n, (ref_pos - 1) * 3 + 6 * (optNum-1), (ref_pos - 1) * 6);
                JTe.block<3, 1>((ref_pos - 1) * 3 + 6 * (optNum-1), 0) += JiTe_pre_n;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre_n, (ref_pos - 1) * 3 + 6 * (optNum-1), (ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre_n, (ref_pos - 1) * 3 + 6 * (optNum-1), (new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre_n, (new_pos - 1) * 3 + 6 * (optNum-1), (ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre_n, (new_pos - 1) * 3 + 6 * (optNum-1), (new_pos - 1) * 6);
                JTe.block<3, 1>((ref_pos - 1) * 3 + 6 * (optNum-1), 0) += JiTe_pre_n;
                JTe.block<3, 1>((new_pos - 1) * 3 + 6 * (optNum-1), 0) += JjTe_pre_n;
                }
            }
        }
#elif 1
        for (int i = 0; i < normal_correspondences.size(); i++)
        {
            //std::cout<<"Ref: "<<normal_correspondences[i].ref_submap_id<<normal_correspondences[i].ref_normal<<std::endl;
            //std::cout<<"New: "<<normal_correspondences[i].new_submap_id<<normal_correspondences[i].new_normal<<std::endl; 
             
            if(!normal_correspondences[i].is_valid)
            continue;        
            int pos;
            if(normal_correspondences[i].type == 0)
            {

              int camera_id = normal_correspondences[i].camera_id;
              

              int submap_id = normal_correspondences[i].submap_id;


              int frame_pos = submap_to_pos[camera_id][submap_id];


              if(frame_pos == -1)
              {
                  frame_pos = 
                    room_to_pos[camera_id][server_slam.submap_to_room_for_each_camera[camera_id][submap_id]];
              }
              else 
              frame_pos += roomOptNum;

              if (frame_pos < 0)
              {
                  continue;
              }
              robust_weight = 1.0f;


              clock_t start_jacobian, end_jacobian;
              start_jacobian = clock();
              //std::cout<<"Compute JacobianInfo of submaps..."<<std::endl;
              ComputeJacobianInfoSubmapNormal(normal_correspondences[i],
                  server_slam,
                  room_to_pos,
                  submap_to_pos,
                  room_poses, 
                  submap_poses_delta,
                  JiTe_pre_n,
                  JiTJi_pre_n);
              pos = frame_pos;

            }
            else
            {
              int camera_id = normal_correspondences[i].camera_id;


              int room_id = normal_correspondences[i].room_id;

              int room_pos = room_to_pos[camera_id][room_id];

              if(room_pos == -1 )
              {
                  std::cout<<"Something wrong about the room correspondences."<<std::endl;
                  continue;
              }

              robust_weight = 1;
              //std::cout<<"Compute JacobianInfo of Rooms..."<<std::endl;
              ComputeJacobianInfoRoomNormal(normal_correspondences[i],
                  server_slam,
                  room_to_pos,
                  room_poses, 
                  JiTe_pre_n,
                  JiTJi_pre_n);            
              //std::cout<<"pos: "<<frame_ref_pos<<" "<<frame_new_pos<<std::endl;
              JiTe_pre_n *= robust_weight;
              JiTJi_pre_n *= robust_weight;
              pos = room_pos;
            }


            if(pos > 0)
            {
              //addBlockToTriplets(coeff, JiTJi_pre_n, (pos- 1 ) * 3 + 6 * (optNum - 1), (pos- 1) * 6);
              //JTe.block<3, 1>((pos- 1) * 3 + 6 * (optNum - 1), 0) += JiTe_pre_n;
              //3 * 6
              addBlockToTriplets(coeff, JiTJi_pre_n, (pos- 1 ) * 6 + 3, (pos- 1) * 6);
              JTe.block<3, 1>((pos- 1) * 6 + 3, 0) += JiTe_pre_n;
            }
  
        }
#endif
          end = clock();
          time_framePair = (double)(end - start) / CLOCKS_PER_SEC * 1000;
          start = clock();
          //std::cout<<"Set from triplets."<<std::endl;
          JTJ.setFromTriplets(coeff.begin(), coeff.end());
          //std::cout<<"Finish setting."<<std::endl;          
          //std::cout<<JTJ<<"\n\n"<<JTe<<std::endl;


          end = clock();
          time_generatingJacobian = (double)(end - start) / CLOCKS_PER_SEC * 1000;
          start = clock();
          //std::cout<<JTJ<<std::endl;
          
          SimplicialLDLTSolver.compute(JTJ);
          //SparseLuSolver.analyzePattern(JTJ);
          //SparseLuSolver.factorize(JTJ);
          //SparseQRSolver.compute(JTJ);
          end = clock();
          time_buildSolver = (double)(end - start) / CLOCKS_PER_SEC * 1000;

          //std::cout<<"Ready to solve the equation."<<std::endl;
          // update the pose of each frame
          start = clock();
          delta = SimplicialLDLTSolver.solve(JTe);
          //delta = SparseLuSolver.solve(JTe);
          //delta = SparseQRSolver.solve(JTe);
          end = clock();
          
          /*
          Eigen::JacobiSVD<MatrixXd> svd(JTJ, ComputeThinU | ComputeThinV);
          delta = svd.solve(JTe);
          */
          double time_svd = (double)(end - start) / CLOCKS_PER_SEC * 1000;
          for (int i = 1; i < room_poses.size(); i++)
          {
              int camera_id = pos_to_room[i].first;
              Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (i - 1), 0);
              //std::cout<<i<<" r "<<delta_i<<std::endl<<std::endl;
              if(isnan(delta_i(0)))
              {
                std::cout << "nan detected in pose update! " << std::endl;
                continue;
              }
              
              /* refine all the keyframes' poses in this submap*/
              //if(submapID == 0) continue;
              auto tmp_global_pose = server_slam.cameraStartPoses[camera_id] * room_poses[i];
              tmp_global_pose =  Sophus::SE3d::exp(delta_i).inverse() 
              * tmp_global_pose;

              room_poses[i] = server_slam.cameraStartPoses[camera_id].inverse() * tmp_global_pose;

          }

          for(int i = (room_poses.size() == 0); i != submap_poses_delta.size(); ++i)
          {
              int position = room_poses.size() + i;
              int camera_id = pos_to_submap[i].first;
              //int submap_id = position_to_submap[position];
              Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (position - 1), 0);
              //std::cout<<i<<" r "<<delta_i<<std::endl<<std::endl;
              if(isnan(delta_i(0)))
              {
              std::cout << "nan detected in pose update! " << std::endl;
              continue;
              }
              //std::cout<<i<<" s "<<delta_i<<std::endl<<std::endl;
              /* refine all the keyframes' poses in this submap*/
              //if(submapID == 0) continue;
              auto tmp_global_pose = server_slam.cameraStartPoses[camera_id] * submap_poses_delta[i];
              tmp_global_pose =  Sophus::SE3d::exp(delta_i).inverse() 
              * tmp_global_pose;

              submap_poses_delta[i] = server_slam.cameraStartPoses[camera_id].inverse() * tmp_global_pose;
              // this is used for update all the submap poses
              
          }
          double time_calculate_reprojection_error;
          double refined_error;
          last_final_error = combine_final_error;
          final_error = reprojection_error_3Dto3D_room_submap(room_correspondences, correspondences_we_need, 
            server_slam, room_to_pos, submap_to_pos, room_poses, submap_poses_delta);
          normal_final_error = reprojection_error_normal_gravity(normal_correspondences, 
            server_slam, room_to_pos, submap_to_pos, room_poses, submap_poses_delta);
          combine_final_error = (1 - weight_normal) * final_error + weight_normal * normal_final_error;
          if(std::fabs(combine_final_error - last_final_error)< 0.000001) break;
          
     }
/*     
    std::cout<<"after: "<<std::endl;
    for(int i = 0; i != room_poses.size(); ++i)
    {
        std::cout<<room_poses[i].matrix()<<std::endl<<std::endl;
    }
*/
    // this is used for update all the submap poses


    //update all the keyframe poses
    end_opt = clock();

    time_opt = (double )(end_opt - start_opt) / CLOCKS_PER_SEC * 1000;

    std::cout << "init/final error " << init_error << "/" << final_error << std::endl;
    std::cout << "normal init/final error " << normal_init_error << "/" << normal_final_error << std::endl;    
    std::cout << "combine init/final error " << combine_init_error << "/" << combine_final_error << std::endl;   
    double node_last_final_error = server_slam.camera_nodes[node_id].last_final_error;
    std::cout<<"node_last_final_error/final error: "<<node_last_final_error<<"/"<<combine_final_error<<std::endl;
    // We still need geometry check, however we should combine the normal error and correspondence error
#if 1
    if(!std::isnan( node_last_final_error ) && (combine_final_error - combine_init_error) /combine_init_error > 0.5  ||combine_final_error > 0.008
      &&(combine_final_error - node_last_final_error) / node_last_final_error > 1)
    {
      //final_error > 0.003 &&(final_error -node_last_final_error) / node_last_final_error > 0.8
      //(final_error -node_last_final_error) / node_last_final_error > 0.5
        // remove all outliers 
        std::cout<<"Error is too large. Try to remove the correspondence."<<std::endl;
        if(from_frame )
        {
          for (int i = 0; i < submap_correspondences.size(); i++)
          {
              ServerFrame &frame_ref = submap_correspondences[i].frame_ref;
              ServerFrame &frame_new = submap_correspondences[i].frame_new;

              if( frame_new.frame_index == server_slam.keyframes[camera_id].rbegin()->second.frame_index && frame_new.cameraID == camera_id)
              {
                  if( submap_correspondences[i].matches.size() > 0 || submap_correspondences[i].data_pairs_3d.size() > 0)
                  {
                      std::cout << "try to remove: "<< " " << frame_ref.frame_index << " " << frame_new.frame_index << std::endl;
                      if(submap_correspondences[i].connection == false)
                      {
                        submap_correspondences[i].setIsValid(false);
                        submap_correspondences[i].reset();
                      }
                  }
              }
          }
          return false;
        }

        /*
        else
        {
          for (int i = 0; i < room_correspondences.size(); ++i)
          {
            int source_room_id = room_correspondences[i].source_id;
            int target_room_id = room_correspondences[i].target_id;
            int source_camera_id = room_correspondences[i].source_camera_id;
            int target_camera_id = room_correspondences[i].target_camera_id;

            if(source_room_id == latest_room_index  && source_camera_id == camera_id || 
              target_room_id == latest_room_index && target_camera_id == camera_id)
            {
              if(room_correspondences[i].instance_center_correspondences.size()>0)
              {
                std::cout << "try to remove: "<< " " << source_room_id << " " << target_room_id << std::endl;
                room_correspondences[i].reset();
              }
            }

          }
        }
        */

    }
#endif
    auto & submapPosesRelativeChanges = server_slam.submapPosesRelativeChanges;
    auto & submapPosesFinal = server_slam.submapPosesFinal;
    for(int i = 0; i != room_poses.size(); ++i)
    {
          int camera_id = pos_to_room[i].first;
          int room_id = pos_to_room[i].second;
          auto &room_info = server_slam.rooms_for_each_camera[camera_id][room_id];

        server_slam.room_poses[camera_id][room_id] = room_poses[i] * server_slam.room_poses[camera_id][room_id];
        for(auto j = room_info.contained_submaps.begin();
            j!=room_info.contained_submaps.end(); ++j)
        {
            submapPosesRelativeChanges[camera_id][*j] = 
                room_poses[i] * submapPosesRelativeChanges[camera_id][*j];
            //std::cout<<submapPosesFinal[camera_id][*j].matrix()<<std::endl<<std::endl;
            submapPosesFinal[camera_id][*j] = 
                room_poses[i] * submapPosesFinal[camera_id][*j];
            //std::cout<<submapPosesFinal[camera_id][*j].matrix()<<std::endl<<std::endl;

            //std::cout<<server_slam.submapPosesFinal[camera_id][*j].matrix()<<std::endl<<std::endl;
        }
    }

    for(int i = 0; i != submap_poses_delta.size(); ++i)
    {
        //remember to switch position with submap_id 
        int camera_id = pos_to_submap[i].first;
        int submap_id = pos_to_submap[i].second;

        if(submap_id < 0)
        {
            std::cout<<"something wrong with the table position_to_submap."<<std::endl;
            continue;
        }
        submapPosesRelativeChanges[camera_id][submap_id] = 
            submap_poses_delta[i] * submapPosesRelativeChanges[camera_id][submap_id];
        submapPosesFinal[camera_id][submap_id] = 
            submap_poses_delta[i] * submapPosesFinal[camera_id][submap_id];
    }
    server_slam.camera_nodes[node_id].last_final_error =combine_final_error;
    return true;
}

  float combineOptimization(Server::ServerSLAM &server_slam,int camera_id, bool from_frame, bool with_normal)
  {
    //bool need_optimization = true;
    if(with_normal == false)
    {
      std::cout<<"begin to optimize room and submap"<<std::endl;
      /*
      while(need_optimization)
      {
        need_optimization =  !optimizeRoomAndSubmapCamera(server_slam,camera_id,from_frame);
      }*/
      optimizeRoomAndSubmapCamera(server_slam,camera_id,from_frame);
      std::cout<<"finish optimizing room and submap"<<std::endl;
    }
    else
    {
      std::cout<<"begin to optimize room and submap with normal"<<std::endl;
      //bool correct_corr = optimizeRoomAndSubmapCamera(server_slam,camera_id,from_frame);
      if(server_slam.submapPosesRelativeChanges[camera_id].size() > 5)
      optimizeRoomAndSubmapCameraWithNormal(server_slam,camera_id,from_frame);
      else
      std::cout<<"Find Error correspondences by geometry check."<<std::endl;
      std::cout<<"finish optimizing room and submap with normal"<<std::endl;      
    }
  }
  void ConstructCorrespondence(ServerFrame &frame_ref, ServerFrame &frame_new, 
    const PoseSE3d &relative_transform_from_ref_to_new, Correspondence &key_frame_corr)
  {
    key_frame_corr.use_icp = true;
    auto &local_points = frame_ref.getLocalPoints();
    key_frame_corr.data_pairs_3d.clear();
    Eigen::Matrix3d R = relative_transform_from_ref_to_new.rotationMatrix();
    Eigen::Vector3d t = relative_transform_from_ref_to_new.translation();
    for(int i = 0; i !=local_points.size(); ++i)
    {
          Eigen::Vector3f corr_point = (R * local_points[i] + t).cast<float>();
          key_frame_corr.data_pairs_3d.push_back(make_pair(local_points[i].cast<float>(), corr_point));
    }
  }
};