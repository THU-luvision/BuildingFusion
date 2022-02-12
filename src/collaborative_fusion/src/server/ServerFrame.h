
#ifndef SERVER_FRAME_H
#define SERVER_FRAME_H

#include <string>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>

namespace Server 
{
typedef std::vector<Eigen::Vector3d , Eigen::aligned_allocator<Eigen::Vector3d> > Point3dList;
typedef Eigen::Vector3d Point3d;
typedef std::vector<Sophus::SE3d , Eigen::aligned_allocator<Eigen::Vector3d> > PoseSE3dList;
typedef Sophus::SE3d PoseSE3d;

class ServerFrame
{
    public:
    ServerFrame() = default;
/*
    ServerFrame(int _frame_index, int _submapID, int _cameraID,int _keyPointsNum, std::vector<cv::KeyPoint> _keypoints, cv::Mat _descriptor,Point3dList &_local_points, PoseSE3dList & _pose_sophus):
    frame_index(_frame_index),submapID(_submapID),cameraID(_cameraID),keyPointsNum(_keyPointsNum),keypoints(_keypoints),descriptor(_descriptor),local_points(_local_points),pose_sophus(_pose_sophus)
    {}
    ServerFrame( const ServerFrame &SF)
    {
        frame_index = SF.frame_index;
        submapID = SF.submapID;
        cameraID = SF.cameraID;
        keypoints = SF.keypoints;
        descriptor = SF.descriptor;
        local_points = SF.local_points;
        pose_sophus = SF.pose_sophus;
        keyPointsNum = SF.keyPointsNum;
    }
*/
    Point3dList & getLocalPoints()
    {
        return local_points;
    }
    std::vector<cv::KeyPoint > &getKeypoints()
    {
        return keypoints;
    }
    cv::Mat &getDescriptor()
    {
        return descriptor;
    }
    cv::Mat &getDepth()
    {
        return depth;
    }
    cv::Mat &getRgb()
    {
        return rgb;
    }

    void clear_frame()
    {
        keypoints.clear();
        descriptor.release();
        depth.release();
        rgb.release();
    }
    int frame_index;
    int submapID;
    int cameraID;
    bool tracking_success;//denote if tracking last keyframe successfully in client
    int keyPointsNum;
    std::vector<cv::KeyPoint > keypoints;
    cv::Mat descriptor;
    cv::Mat depth;
    cv::Mat rgb;
    Point3dList local_points;
    PoseSE3dList pose_sophus;
};
};
#endif