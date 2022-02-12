#include <vector>
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/Image.h>
#include "sensor_msgs/Imu.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <collaborative_fusion/FramePose.h>
#include <collaborative_fusion/FrameDescriptor.h>
#include <collaborative_fusion/FrameKeyPoints.h>
#include <collaborative_fusion/FrameLocalPoints.h>
#include <collaborative_fusion/Frame.h>
#include <collaborative_fusion/LoopClosureDetection.h>
#include <collaborative_fusion/UpdateFrame.h>
#include <collaborative_fusion/GlobalOptimization.h>
#include <collaborative_fusion/MeshVisualization.h>
#include <collaborative_fusion/RoomRegistration.h>
#include <collaborative_fusion/RoomModel.h>
#include <collaborative_fusion/RoomInfo.h>
#include "server/RoomHandler.h"
#include "server/ServerFrame.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "eigen_conversions/eigen_msg.h"
#include "GCSLAM/MultiViewGeometry.h"
#include "server/CompressedPointCloud.h"
namespace CommunicationAPI
{
typedef Eigen::Affine3d Transform;

sensor_msgs::ImagePtr inline toRosImage(cv::Mat &image );



void toRosDescriptor(cv::Mat &des,collaborative_fusion::FrameDescriptor &fd);

void inline toRosDepth(cv::Mat &depth, sensor_msgs::Image &msg);

void toCvDepth(sensor_msgs::Image &depth, cv::Mat &des);

void inline toRosRgb(cv::Mat &rgb, sensor_msgs::Image &msg);

void toCvRgb(sensor_msgs::Image &rgb, cv::Mat &crgb);

void toRosTransform(PoseSE3d &pose, geometry_msgs::Transform &t);

void toCvTransform(geometry_msgs::Transform &pose, PoseSE3d &ps);

void toRosKeyPoints(std::vector<cv::KeyPoint> &keypoints,collaborative_fusion::FrameKeyPoints &fk);

void toRosLocalPoints(Point3dList &local_points,collaborative_fusion::FrameLocalPoints &fl);

void toRosPose(PoseSE3dList &poses,collaborative_fusion::FramePose &fp);

void toRosFrame(Frame &f, collaborative_fusion::Frame &F);

void toCvDescriptor(collaborative_fusion::FrameDescriptor &fd,cv::Mat & des);

void toCvKeyPoints(collaborative_fusion::FrameKeyPoints &fk,std::vector<cv::KeyPoint> &keypoints);

void toCvLocalPoints(collaborative_fusion::FrameLocalPoints &fl, Point3dList &local_points);

void toCvPose(collaborative_fusion::FramePose &fp,PoseSE3dList &poses);

void toCvPoseList(std::vector<geometry_msgs::Transform> &g_poses, PoseSE3dList &poses);

void toRosPoseList(PoseSE3dList &poses, std::vector<geometry_msgs::Transform> &g_poses);

void toUpdateFrame(Frame &f,collaborative_fusion::UpdateFrame &uf);

void updateServerFrame(collaborative_fusion::UpdateFrame &uf,Server::ServerFrame &sf);

void toServerFrame(collaborative_fusion::Frame &cf, Server::ServerFrame & sf);

void toRosRoomModel(chisel::PointCloud &pcd, collaborative_fusion::RoomModel &model);
void toChiselPcd(collaborative_fusion::RoomModel &model, chisel::PointCloud &pcd);

void toRosRoomInfo(int camera_id, int room_id, 
  std::vector<int> &room_to_submap, chisel::PcdPtr &pcd_ptr, 
    collaborative_fusion::RoomInfo & room_info);
void toServerRoomInfo(collaborative_fusion::RoomInfo & cr, Server::RoomInfo &sr);
void toEigenVec3f(std::vector<geometry_msgs::Point32> & ros_vec, Point3fList & e_vec);
void toRosVec3f(Point3fList & e_vec, std::vector<geometry_msgs::Point32> & ros_vec);
void toRosPointCloud(chisel::PointCloud &pcd,std::vector<int> &compressed_colors, std::vector<int> &semantic_colors, 
  std::vector<collaborative_fusion::PointCloud> &ros_pcd);
void toServerPointCloud(collaborative_fusion::PointCloud &ros_pcd, Server::ServerPointCloud &s_pcd);
void toServerPointCloud(std::vector<collaborative_fusion::PointCloud> &ros_pcd, std::vector<Server::ServerPointCloud> &s_pcd);
void toRosPointCloud(std::vector<chisel::PointCloud> &pcds, std::vector<std::vector<int>> &compressed_colors, 
  std::vector<std::vector<int>> &semantic_colors, std::vector<collaborative_fusion::PointCloud> &ros_pcds);
};