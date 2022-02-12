#ifndef DENSE_ODOMETRY_H
#define DENSE_ODOMETRY_H

#include "../Geometry/Geometry.h"
#include "../Tools/ImageProcessing.h"
#include <opencv2/opencv.hpp>
#include "OdometryPredefined.h"
enum CameraType{TUM_DATASET, OPEN3D_DATASET};
class PinholeCamera 
{
    public:

    PinholeCamera()
    {
        SetCameraType(OPEN3D_DATASET);
    }
    PinholeCamera(float _fx, float _fy, float _cx, float _cy, int _width, int _height, float _depth_scale, float *_distortion = nullptr)
    {
        fx = _fx;
        fy = _fy;
        cx = _cx;
        cy = _cy;
        width = _width;
        height = _height;
        depth_scale = _depth_scale;
        if(_distortion != nullptr)
        for(int i = 0; i <5;++i)
        distortion_para[i] = _distortion[i];
        else
        for(int i = 0; i <5;++i)
        distortion_para[i] = 0;

    }
    PinholeCamera GenerateNextPyramid() const
    {
        PinholeCamera next_camera(fx/2,fy/2,cx/2,cy/2,width/2,height/2,depth_scale);
        return next_camera;
    }
    Eigen::Matrix3f ToCameraMatrix() const
    {
        Eigen::Matrix3f intrinsic;
        intrinsic<<fx,0,cx,
                    0,fy,cy,
                    0, 0, 1;
        return intrinsic;
    }
    float GetFx() const {return fx;}
    float GetFy() const {return fy;}
    float GetCx() const {return cx;}
    float GetCy() const {return cy;}
    float GetWidth() const {return width;}
    float GetHeight() const {return height;}
    float GetDepthScale() const {return depth_scale;}
    void SetPara(float _fx, float _fy, float _cx, float _cy, int _width, int _height, float _depth_scale, float *_distortion = nullptr)
    {
        fx = _fx;
        fy = _fy;
        cx = _cx;
        cy = _cy;
        width = _width;
        height = _height;
        depth_scale = _depth_scale;
        if(_distortion != nullptr)
        for(int i = 0; i <5;++i)
        distortion_para[i] = _distortion[i];
    }
    void SetCameraType(const CameraType &type)
    {
        if(type == CameraType::TUM_DATASET)
        {
        fx = 517.3;
        fy = 516.5;
        cx = 318.6;
        cy = 255.3;
        depth_scale = 5000;
        width = 640;
        height = 480;
        distortion_para[0] = 0.2624;
        distortion_para[1] = -0.9531;
        distortion_para[2] = -0.0054;
        distortion_para[3] = 0.0026;
        distortion_para[4] = 1.1633;
        }
        else if (type == CameraType::OPEN3D_DATASET)
        {
        fx = 514.817;
        fy = 515.375;
        cx = 318.771;
        cy = 238.447;
        depth_scale = 1000;
        width = 640;
        height = 480;
        for(int i = 0; i <5;++i)
        distortion_para[i] = 0;
        }
        
    }
    protected:

    float fx;
    float fy;
    float cx;
    float cy;
    float depth_scale = 1000.0;
    float distortion_para[5];
    int width;
    int height;
};

static PinholeCamera fc_camera;
static int multi_scale_level = 3;
static std::vector<int> iter_count_per_level = {4,8,16};
static int feature_number = 1000;
std::vector<PinholeCamera > CreatePyramidCameras();
int InitializeRGBDDenseTracking(const cv::Mat &source_color, const cv::Mat &source_depth, 
    const cv::Mat &target_color, const cv::Mat &target_depth,
    cv::Mat &source_gray, cv::Mat &source_refined_depth, cv::Mat &target_gray, cv::Mat &target_refined_depth);

bool DenseEstimation(const cv::Mat &source_color, const cv::Mat &target_color, 
    const cv::Mat &source_depth, const cv::Mat &target_depth, 
    TransformationMatrix &transformation, PointCorrespondenceSet &correspondence_set,bool use_hybrid = false);

void AddElementToCorrespondenceMap(cv::Mat &wraping_map,cv::Mat &wraping_depth,
    int u_s, int v_s, int u_t, int v_t, float transformed_d_s);

void ComputeCorrespondencePixelWise(const cv::Mat &source, const cv::Mat &target, const PinholeCamera &fc_camera,
    const TransformationMatrix & relative_pose, PixelCorrespondenceSet &correspondences);

void TransformToMatXYZ(const cv::Mat &image, const PinholeCamera &fc_camera, ImageXYZ &imageXYZ);

void ComputeJacobianHybridTerm(
    int row,std::vector<Eigen::Vector6f> &J, std::vector<float> &residual, 
    const cv::Mat &source_color, const cv::Mat & source_depth, 
    const cv::Mat & target_color, const cv::Mat & target_depth, 
    const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
    const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
    const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, const TransformationMatrix & relative_pose, 
    const PixelCorrespondenceSet &correspondences);
void ComputeJacobianPhotoTerm(
    int row,std::vector<Eigen::Vector6f> &J, std::vector<float> &residual, 
    const cv::Mat &source_color, const cv::Mat & source_depth, 
    const cv::Mat & target_color, const cv::Mat & target_depth, 
    const cv::Mat &target_color_dx, const cv::Mat & target_color_dy, 
    const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, 
    const TransformationMatrix & relative_pose, 
    const PixelCorrespondenceSet &correspondences);

std::tuple<Eigen::Matrix6f,Eigen::Vector6f,float> ComputeJTJandJTrHybridTerm(
    const cv::Mat &source_color, const cv::Mat & source_depth, 
    const cv::Mat & target_color, const cv::Mat & target_depth, 
    const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
    const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
    const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, const TransformationMatrix & relative_pose, 
    const PixelCorrespondenceSet &correspondences);

std::tuple<Eigen::Matrix6f,Eigen::Vector6f,float> ComputeJTJandJTrPhotoTerm(
    const cv::Mat &source_color, const cv::Mat & source_depth, 
    const cv::Mat & target_color, const cv::Mat & target_depth, 
    const cv::Mat &target_color_dx, const cv::Mat & target_color_dy, 
    const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, const TransformationMatrix & relative_pose, 
    const PixelCorrespondenceSet &correspondences);
int ConvertDepthTo32FNaN(const cv::Mat &depth, cv::Mat &refined_depth, float depth_scale);

void ConvertColorToIntensity32F(const cv::Mat &color, cv::Mat &intensity, float scale);
/*
void NormalizeIntensity(const cv::Mat &intensity,cv::Mat &normalized_intensity, float scale)
{
normalized_intensity.create(intensity.rows, intensity.cols, CV_32FC1, cv::Scalar(0));
for(int i = 0; i < intensity.rows * intensity.cols; ++i)
    normalized_intensity.at<float>(i) = intensity.at<int>(i) /scale;        
}
*/
void DoSingleIteration(
    const cv::Mat &source_color, const cv::Mat & source_depth, 
    const cv::Mat & target_color, const cv::Mat & target_depth, 
    const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
    const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
    const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, TransformationMatrix & relative_pose, 
    PixelCorrespondenceSet &correspondences);
void DoSingleIteration(
    const cv::Mat &source_color, const cv::Mat & source_depth, 
    const cv::Mat & target_color, const cv::Mat & target_depth, 
    const cv::Mat &target_color_dx, const cv::Mat & target_color_dy,
    const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, TransformationMatrix & relative_pose, 
    PixelCorrespondenceSet &correspondences);
void NormalizeIntensity(cv::Mat &source_gray, cv::Mat & target_gray, const PixelCorrespondenceSet &correspondence);
size_t MultiScaleComputing(const cv::Mat& source_color, const cv::Mat& target_color, 
    const cv::Mat& source_depth, const cv::Mat& target_depth, TransformationMatrix &T, 
    PointCorrespondenceSet &correspondence_set, bool use_hybrid);


std::tuple<Eigen::Matrix6f,Eigen::Vector6f,float> ComputeJTJandJTr(
    const cv::Mat &source_color, const cv::Mat & source_depth, 
    const cv::Mat & target_color, const cv::Mat & target_depth, 
    const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
    const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
    const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, const TransformationMatrix & relative_pose, 
    const PixelCorrespondenceSet &correspondences);
#endif