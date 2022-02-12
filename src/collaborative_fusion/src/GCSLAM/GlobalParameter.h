#ifndef GLOBAL_PARAMETER_H
#define GLOBAL_PARAMETER_H  
namespace MultiViewGeometry
{
  class GlobalParameters
  {
  public:
    float salient_score_threshold;
    float minimum_disparity;
    float ransac_maximum_iterations;
    float reprojection_error_3d_threshold;
    float reprojection_error_2d_threshold;


    int maximum_keyframe_match_num;
    int save_ply_files;
    int debug_mode;
    int max_feature_num;
    int use_icp_registration;
    float icp_weight = 0.3;
    int hamming_distance_threshold;
    int camera_num;
    float far_plane_distance;
    int max_average_disparity = 0.5;

    int closure_key_points;

    int maximum_frame_num=30000;
    int step_len;
    float plane_fitting_ransac_threshold = 0.05;
    float room_lcd_ransac_threshold = 0.2;
    //information: 0.2 8 0.4
    int room_lcd_min_inlier = 6;
    float room_lcd_min_ratio = 0.3;
    float normal_weight;
    float room_lcd_min_icp_ratio = 0.5;
    float room_lcd_min_score = 0.125;
    int room_lcd_n_nearest = 20;
    int final_integration = 0;
    int normal_optimization = 0;
  };
  extern GlobalParameters g_para;
};
#endif