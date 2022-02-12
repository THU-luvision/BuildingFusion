
#include "ICPRegistration.h"

#include <fstream>
#include <utility>

namespace icp_registration
{
void initPointProjector(int width, int height, double fx,double fy,double cx,double cy,double _imageScaling,int _depth_scale)
{
#if OPEN_3D
  intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
#endif

#if DENSE_TRACKING
  fc_camera.SetPara(fx,fy,cx,cy,width,height,depth_scale);
#endif
}


#if OPEN_3D
std::shared_ptr<open3d::geometry::PointCloud> CreatePointCloudFromCvMat(
        const cv::Mat &rgb,const cv::Mat &depth, bool is_refined_depth) {
    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();
    auto pointcloud = std::make_shared<open3d::geometry::PointCloud>();
    Eigen::Matrix4d camera_pose = extrinsic.inverse();
    auto focal_length = intrinsic.GetFocalLength();
    auto principal_point = intrinsic.GetPrincipalPoint();
    double scale = 255.0;
    int num_valid_pixels = 0;//CountValidDepthPixels(image.depth_, 1);
    pointcloud->points_.resize(depth.rows*depth.cols);
    pointcloud->colors_.resize(depth.rows*depth.cols);
    int cnt = 0;
    //std::cout<<principal_point.first<<" "<<principal_point.second<<std::endl;
    for (int i = 0; i < depth.rows; i++) {

        unsigned char *pc = rgb.data;
        for (int j = 0; j < depth.cols; j++, pc += 3) {
            double  z;
            if(is_refined_depth)
            z = depth.at<float>(i, j) ;
            else
            z = (double)depth.at<unsigned short>(i, j) / depth_scale;

            //if(j%10 == 0) std::cout<<z<<std::endl;
            if (z > 0) {
                double x = (j - principal_point.first) * z / focal_length.first;
                double y =
                        (i - principal_point.second) * z / focal_length.second;
                Eigen::Vector4d point =
                        camera_pose * Eigen::Vector4d(x, y, z, 1.0);
                pointcloud->points_[cnt] = point.block<3, 1>(0, 0);
                pointcloud->colors_[cnt++] =
                        Eigen::Vector3d(rgb.at<cv::Vec3b>(i, j)[0], rgb.at<cv::Vec3b>(i, j)[1], rgb.at<cv::Vec3b>(i, j)[2]) /
                        scale;
            }
        }
    }
    pointcloud->points_.resize(cnt);
    pointcloud->colors_.resize(cnt);
    return pointcloud;
}

std::shared_ptr<open3d::geometry::PointCloud> chiselpcd_to_open3d(const chisel::PointCloud &cpcd)
{

    auto pointcloud = std::make_shared<open3d::geometry::PointCloud>();

    pointcloud->points_.resize(cpcd.vertices.size());
    pointcloud->colors_.resize(cpcd.colors.size());
    pointcloud->normals_.resize(cpcd.normals.size());

    for(int i = 0; i != cpcd.vertices.size(); ++i)
    {
      pointcloud->points_[i] = cpcd.vertices[i].cast<double>();
    }

    for(int i = 0; i !=cpcd.colors.size(); ++i)
    {
      pointcloud->colors_[i] = cpcd.colors[i].cast<double>();
    }

    for(int i = 0; i !=cpcd.normals.size(); ++i)
    {
      pointcloud->normals_[i] = cpcd.normals[i].cast<double>();
    }
    return pointcloud;  
}
int open3d_icp_registration(const chisel::PcdPtr &source_pcd, const chisel::PcdPtr &target_pcd, Eigen::Matrix4d &T, 
    std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > &point_pairs_3d, std::vector<Eigen::Vector2i> &correspondence_set, float max_distance, int feature_num)
{
    auto opcd_source = chiselpcd_to_open3d(*source_pcd);
    auto opcd_target = chiselpcd_to_open3d(*target_pcd);
    if(opcd_source->normals_.size() == 0)
    opcd_source->EstimateNormals();
    if(opcd_target->normals_.size() == 0)
    opcd_target->EstimateNormals();
    open3d::registration::TransformationEstimationPointToPlane icper;
    //open3d::registration::TransformationEstimationPointToPoint icper(false);
    auto result = open3d::registration::RegistrationICP(*opcd_source,*opcd_target,max_distance,T,icper);

    //WritePointCloudToPLY("source.ply",*source_pcd);
    //WritePointCloudToPLY("target.ply",*target_pcd);

#if 0
    open3d::visualization::Visualizer visualizer;
    source_pcd->Transform(result.transformation_);
    visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
    visualizer.AddGeometry(source_pcd);
    visualizer.AddGeometry(target_pcd);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
#endif
    correspondence_set = result.correspondence_set_;
    point_pairs_3d = std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> >();
    int step_len = result.correspondence_set_.size() / feature_num;
    if(step_len <1) step_len = 1;
    auto & cs = result.correspondence_set_;
    //std::cout<<"get icp data pair..."<<cs.size()<<" "<<opcd_source->points_.size()<<" "<<opcd_target->points_.size()<<std::endl;
    for(int i = 0;i<cs.size();i+=step_len)
    {
      /*
        Eigen::Vector3d point= source_pcd->points_[cs[i](0)];
        Eigen::Vector4d new_point =
                result.transformation_ *
                Eigen::Vector4d(point(0), point(1), point(2), 1.0);
      //point_pairs_3d.push_back(std::make_pair(source_pcd->points_[cs[i](0)].cast<float>(), (source_pcd->points_[cs[i](0)] * T).cast<float>()));
      
      std::cout<<"source: "<<source_pcd->points_[cs[i](0)]<<std::endl;
      std::cout<<"target: "<<target_pcd->points_[cs[i](1)]<<std::endl;
      std::cout<<"after transformation: "<<new_point<<std::endl;*/
      point_pairs_3d.push_back(std::make_pair(opcd_source->points_[cs[i](0)].cast<float>(), opcd_target->points_[cs[i](1)].cast<float>()));
    }    
    std::cout << result.transformation_<<std::endl;
    T = result.transformation_;
    std::cout << result.inlier_rmse_<<std::endl;
    //source_pcd->Transform(T);
    //auto merged_pcd = MergeTwoPointCloud(source_pcd,target_pcd);
    //WritePointCloudToPLY("after_transform_source.ply",*source_pcd);
    /*    
    if(result.inlier_rmse_ < 0.5)
      return 1;
    return 0;
    */
    return result.correspondence_set_.size();
}
std::shared_ptr<open3d::geometry::PointCloud> MergeTwoPointCloud(const std::shared_ptr<open3d::geometry::PointCloud> & a,
 const std::shared_ptr<open3d::geometry::PointCloud> &b)
 {
       auto pointcloud = std::make_shared<open3d::geometry::PointCloud>();
       pointcloud->points_ = a->points_;
       pointcloud->points_.insert(pointcloud->points_.end(), b->points_.begin(), b->points_.end());
       pointcloud->colors_ = a->colors_;
       pointcloud->colors_.insert(pointcloud->colors_.end(), b->colors_.begin(), b->colors_.end());
       for(int i = 0; i < a->colors_.size(); ++i)
       pointcloud->colors_[i](0) = 0;
       for(int j = 0; j < b->colors_.size();++j)
       {
         pointcloud->colors_[j+a->colors_.size()](0)=1;
       }
       return pointcloud;
 }
bool WritePointCloudToPLY(const std::string &fileName,const open3d::geometry::PointCloud &pcd)
{
        std::ofstream stream(fileName.c_str());
        if (!stream)
        {
            return false;
        }

        size_t numPoints = pcd.points_.size();
        stream << "ply" << std::endl;
        stream << "format ascii 1.0" << std::endl;
        stream << "element vertex " << numPoints << std::endl;
        stream << "property float x" << std::endl;
        stream << "property float y" << std::endl;
        stream << "property float z" << std::endl;
        if(pcd.HasNormals())
        {
            stream << "property float nx"<<std::endl;
            stream << "property float ny"<<std::endl;
            stream << "property float nz"<<std::endl;
        }
        if (pcd.HasColors())
        {
            stream << "property uchar red" << std::endl;
            stream << "property uchar green" << std::endl;
            stream << "property uchar blue" << std::endl;
        }
        stream << "element face 0" <<  std::endl;
        stream << "property list uchar int vertex_index" << std::endl;
        stream << "end_header" << std::endl;

        size_t vert_idx = 0;
        for (int i = 0;i!=numPoints;++i)
        {
            //Eigen::Vector3d vertd = R * vert.cast<double>();
            stream << pcd.points_[i](0) << " " << pcd.points_[i](1) << " " << pcd.points_[i](2);
            if (pcd.HasNormals())
            {
                stream << " " << pcd.normals_[i](0) << " " << pcd.normals_[i](1) << " " << pcd.normals_[i](2);
            }
            if (pcd.HasColors())
            {
                
                int r = static_cast<int>(pcd.colors_[i](0) * 255.0f);
                int g = static_cast<int>(pcd.colors_[i](1) * 255.0f);
                int b = static_cast<int>(pcd.colors_[i](2) * 255.0f);

                stream << " " << r << " " << g << " " << b;
            }

            stream << std::endl;
            vert_idx++;
        }
    return true;
}
int colored_icp_registration(cv::Mat & source_color, cv::Mat & target_color, cv::Mat & source_depth,cv::Mat & target_depth,Eigen::Matrix4d &T ,
  std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > &point_pairs_3d,int feature_num, bool is_refined_depth)
{
    std::cout<<"get icp data pair..."<<std::endl;
    auto source_pcd = icp_registration::CreatePointCloudFromCvMat(source_color,source_depth,is_refined_depth);
    auto target_pcd = icp_registration::CreatePointCloudFromCvMat(target_color,target_depth,is_refined_depth);
    source_pcd->EstimateNormals();
    target_pcd->EstimateNormals();
    double max_distance = 0.08;
    auto result = open3d::registration::RegistrationColoredICP(*source_pcd,*target_pcd,max_distance);
    //WritePointCloudToPLY("source.ply",*source_pcd);
    //WritePointCloudToPLY("target.ply",*target_pcd);

#if 0
    open3d::visualization::Visualizer visualizer;
    source_pcd->Transform(result.transformation_);
    visualizer.CreateVisualizerWindow("Open3D", 1600, 900);
    visualizer.AddGeometry(source_pcd);
    visualizer.AddGeometry(target_pcd);
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();
#endif

    point_pairs_3d = std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> >();
    int step_len = result.correspondence_set_.size() / feature_num;
    if(step_len <1) step_len = 1;
    auto & cs = result.correspondence_set_;
    std::cout<<"get icp data pair..."<<cs.size()<<" "<<source_pcd->points_.size()<<" "<<target_pcd->points_.size()<<std::endl;
    for(int i = 0;i<cs.size();i+=step_len)
    {
      /*
        Eigen::Vector3d point= source_pcd->points_[cs[i](0)];
        Eigen::Vector4d new_point =
                result.transformation_ *
                Eigen::Vector4d(point(0), point(1), point(2), 1.0);
      //point_pairs_3d.push_back(std::make_pair(source_pcd->points_[cs[i](0)].cast<float>(), (source_pcd->points_[cs[i](0)] * T).cast<float>()));
      
      std::cout<<"source: "<<source_pcd->points_[cs[i](0)]<<std::endl;
      std::cout<<"target: "<<target_pcd->points_[cs[i](1)]<<std::endl;
      std::cout<<"after transformation: "<<new_point<<std::endl;*/
      point_pairs_3d.push_back(std::make_pair(source_pcd->points_[cs[i](0)].cast<float>(), 
        target_pcd->points_[cs[i](1)].cast<float>()));


    }    
    std::cout << result.transformation_<<std::endl;
    T = result.transformation_;
    std::cout << result.inlier_rmse_<<std::endl;
    //source_pcd->Transform(T);
    //auto merged_pcd = MergeTwoPointCloud(source_pcd,target_pcd);
    //WritePointCloudToPLY("after_transform_source.ply",*source_pcd);
    if(result.inlier_rmse_ < 0.5)
    return 1;
    return 0;
}


#endif

#if DENSE_TRACKING
int dense_tracking(const cv::Mat & source_color, const cv::Mat & target_color,const cv::Mat & source_depth, const cv::Mat & target_depth,
    Eigen::Matrix4d &T ,std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f> > &point_pairs_3d)
{
  Eigen::Matrix4f T_f = T.cast<float>();
  bool success = DenseEstimation(source_color,target_color,source_depth,target_depth,T_f,point_pairs_3d,1);
  //std::cout<<"Result: "<<success<<std::endl;
  T = T_f.cast<double>();
  if(point_pairs_3d.size()< 100) return false;
  return success;
} 
#endif
}

