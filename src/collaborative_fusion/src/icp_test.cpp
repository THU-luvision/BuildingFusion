#include "./GCSLAM/ICPRegistration.h"

int main(int argc, char **argv)
{
    if(argc < 5) std::cout<<" usage: icp_test [ref_color_path] [ref_depth_path] [new_color_path] [new_depth_path]"<<std::endl;
    
    cv::Mat source_rgb = cv::imread(argv[1]);
    cv::Mat source_depth = cv::imread(argv[2],-1);
    cv::Mat target_rgb = cv::imread(argv[3]);
    cv::Mat target_depth = cv::imread(argv[4],-1);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > point_pairs_3d;
    icp_registration::dense_tracking(source_rgb, target_rgb, source_depth, target_depth, T, point_pairs_3d) ;
    std::cout<<T<<std::endl;

}