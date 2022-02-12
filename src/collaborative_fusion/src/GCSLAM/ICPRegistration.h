
#ifndef ICP_REG
#define ICP_REG
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>
#include <open_chisel/mesh/PointCloud.h>
#include <Eigen/Dense>
#include <memory>
#define DENSE_TRACKING 1
#define OPEN_3D 1
#if OPEN_3D
#include <Open3D/Registration/ColoredICP.h>
#include <Open3D/Camera/PinholeCameraIntrinsic.h>
#include <Open3D/Geometry/PointCloud.h>
#include <Open3D/Registration/Registration.h>
#endif

#if DENSE_TRACKING
#include "DenseOdometry.h"
//#include <Odometry/Odometry.h>
#endif
namespace icp_registration
{
  
static double imageScaling;
static float min_inliers_ratio = 0.3;
#if OPEN_3D
static open3d::camera::PinholeCameraIntrinsic intrinsic;
#endif

#if DENSE_TRACKING
#endif

static int depth_scale;
void initPointProjector(int width, int height, double a,double b,double f1,double f2,double imageScaling,int _depth_scale);

#if OPEN_3D
std::shared_ptr<open3d::geometry::PointCloud> CreatePointCloudFromCvMat(
        const cv::Mat &rgb,const cv::Mat &depth, bool is_refined_depth = false) ;

int colored_icp_registration(cv::Mat & source_color, cv::Mat & target_color, cv::Mat & source_depth,cv::Mat & target_depth,
    Eigen::Matrix4d &T ,std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > &point_pairs_3d,int feature_num = 500, bool is_refined_depth = false);
int open3d_icp_registration(const chisel::PcdPtr &source_pcd, const chisel::PcdPtr &target_pcd, Eigen::Matrix4d &T, 
    std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > &point_pairs_3d, std::vector<Eigen::Vector2i> &correspondence_set, float max_distance = 0.03, int feature_num = 500);
#endif

#if DENSE_TRACKING
int dense_tracking(const cv::Mat & source_color, const cv::Mat & target_color,const cv::Mat & source_depth, const cv::Mat & target_depth,
    Eigen::Matrix4d &T ,std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > &point_pairs_3d) ;
#endif


}
#endif

