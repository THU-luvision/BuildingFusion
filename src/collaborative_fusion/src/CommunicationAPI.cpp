#include "CommunicationAPI.h"

namespace CommunicationAPI
{
void inline toRosImage(cv::Mat &image, sensor_msgs::Image &msg )
{ 
  cv_bridge::CvImage(std_msgs::Header(),"mono8",image).toImageMsg(msg);
}

void inline toRosDepth(cv::Mat &depth, sensor_msgs::Image &msg)
{ 
  cv_bridge::CvImage(std_msgs::Header(),"mono16",depth).toImageMsg(msg);
}
void toRosDescriptor(cv::Mat &des,collaborative_fusion::FrameDescriptor &fdes)
{ 
  sensor_msgs::Image msg;
  toRosImage(des,fdes.descriptor);
}


void inline toRosRgb(cv::Mat &rgb, sensor_msgs::Image &msg)
{ 
  cv_bridge::CvImage(std_msgs::Header(),"bgr8",rgb).toImageMsg(msg);
}

void toCvRgb(sensor_msgs::Image &rgb, cv::Mat &crgb)
{ 
cv_bridge::CvImagePtr cv_ptr; 
cv_ptr = cv_bridge::toCvCopy(rgb, "bgr8");
cv_ptr->image.copyTo(crgb);
}

void toRosKeyPoints(std::vector<cv::KeyPoint> &keypoints,collaborative_fusion::FrameKeyPoints &frame_keypoints)
{ 
std::vector<collaborative_fusion::KeyPoint> ks;
for(int i = 0;i!=keypoints.size();++i)
{
collaborative_fusion::KeyPoint k;
k.x = keypoints[i].pt.x;
k.y = keypoints[i].pt.y;
k.size = keypoints[i].size;
k.angle = keypoints[i].angle;
k.response=keypoints[i].response;
k.octave = keypoints[i].octave;
k.class_id = keypoints[i].class_id;
ks.push_back(k);
}
frame_keypoints.keypoints = ks;
}

void toRosLocalPoints(Point3dList &local_points,collaborative_fusion::FrameLocalPoints &frame_local_points)
{ 
  std::vector<geometry_msgs::Point> points;
  for(int i = 0;i!=local_points.size();++i)
  { 
    geometry_msgs::Point p;
    tf::pointEigenToMsg(local_points[i],p);
    points.push_back(p);
  }
  frame_local_points.local_points = points;
}
void toRosTransform(PoseSE3d &pose, geometry_msgs::Transform &t)
{
  Transform _t;
  _t = pose.matrix();
  tf::transformEigenToMsg(_t,t);
}

void toCvTransform(geometry_msgs::Transform &pose, PoseSE3d &ps)
{
  Transform _t;
  tf::transformMsgToEigen(pose,_t);
  ps = PoseSE3d(_t);
}
void toRosPose(PoseSE3dList &poses, collaborative_fusion::FramePose &frame_pose)
{ 
  geometry_msgs::Transform transform0,transform1,transform2,transform3;
  Transform t0,t1,t2,t3;
  t0 = poses[0].matrix();
  t1 = poses[1].matrix();
  t2 = poses[2].matrix();
  t3 = poses[3].matrix();
  tf::transformEigenToMsg(t0,transform0);
  tf::transformEigenToMsg(t1,transform1);
  tf::transformEigenToMsg(t2,transform2);
  tf::transformEigenToMsg(t3,transform3);
  frame_pose.pose0 = transform0;
  frame_pose.pose1 = transform1;
  frame_pose.pose2 = transform2;
  frame_pose.pose3 = transform3;
}

void toRosPoseList(PoseSE3dList &poses, std::vector<geometry_msgs::Transform> &g_poses)
{ 
  
  for(int i = 0;i!=poses.size(); ++i)
  {
    geometry_msgs::Transform gt;
    Transform t;
    t = poses[i].matrix();
    tf::transformEigenToMsg(t,gt);
    g_poses.push_back(gt);
  }
}
void toRosFrame(Frame &f, collaborative_fusion::Frame &cf)
{
      collaborative_fusion::FrameDescriptor cfd;
      CommunicationAPI::toRosDescriptor(f.getDescriptor(),cfd); 
      collaborative_fusion::FrameKeyPoints cfk;
      CommunicationAPI::toRosKeyPoints(f.getKeypoints(),cfk);
      collaborative_fusion::FrameLocalPoints cfl;
      CommunicationAPI::toRosLocalPoints(f.getLocalPoints(),cfl);
      collaborative_fusion::FramePose cfp;
      CommunicationAPI::toRosPose(f.pose_sophus,cfp);
      sensor_msgs::Image depth;
      CommunicationAPI::toRosDepth(f.getDepth(),depth);
      sensor_msgs::Image rgb;
      CommunicationAPI::toRosRgb(f.getRgb(),rgb);
      cf.frame_index = f.frame_index;
      cf.submap_id = f.submapID;
      cf.camera_id = f.cameraID;
      cf.depth = depth;
      cf.rgb = rgb;
      cf.keyPointsNum = f.keyPointsNum;
      cf.descriptor = cfd;
      cf.keypoints = cfk;
      cf.local_points = cfl;
      cf.pose = cfp;
}

void toUpdateFrame(Frame &f,collaborative_fusion::UpdateFrame &uf)
{
      uf.frame_index = f.frame_index;
      uf.submap_id = f.submapID;
      uf.camera_id = f.cameraID;
      Transform relative_transform ;
      relative_transform = f.pose_sophus[2].matrix();
      tf::transformEigenToMsg(relative_transform,uf.relative_pose);
}
void updateServerFrame(collaborative_fusion::UpdateFrame &uf,Server::ServerFrame &sf)
{
  Transform relative_transform;
  tf::transformMsgToEigen(uf.relative_pose,relative_transform);
  sf.pose_sophus[2] =   PoseSE3d(relative_transform);
  //std::cout<<sf.submapID<<" "<<sf.frame_index<<std::endl;
  //std::cout<<sf.pose_sophus[2].matrix()<<std::endl;
}
void toCvDescriptor(collaborative_fusion::FrameDescriptor &fd,cv::Mat & des)
{
cv_bridge::CvImagePtr cv_ptr; 
cv_ptr = cv_bridge::toCvCopy(fd.descriptor, "mono8");
cv_ptr->image.copyTo(des);
}
void toCvDepth(sensor_msgs::Image &depth, cv::Mat &des)
{
cv_bridge::CvImagePtr cv_ptr; 
cv_ptr = cv_bridge::toCvCopy(depth, "mono16");
cv_ptr->image.copyTo(des);
}
void toCvKeyPoints(collaborative_fusion::FrameKeyPoints &fk,std::vector<cv::KeyPoint> &keypoints)
{
  std::vector<collaborative_fusion::KeyPoint> cf_keypoints = fk.keypoints;
  keypoints.clear();
  for(int i = 0;i!=cf_keypoints.size();++i)
  {
    cv::KeyPoint keypoint(cf_keypoints[i].x,cf_keypoints[i].y,cf_keypoints[i].size,cf_keypoints[i].angle,cf_keypoints[i].response,cf_keypoints[i].octave,cf_keypoints[i].class_id);
    keypoints.push_back(keypoint);
  }
}

void toCvLocalPoints(collaborative_fusion::FrameLocalPoints &fl, Point3dList &local_points)
{
  local_points.clear();
  std::vector<geometry_msgs::Point> points = fl.local_points;
  for(int i = 0;i!=points.size();++i)
  {
    Point3d p;
    tf::pointMsgToEigen(points[i],p);
    local_points.push_back(p);
  }
}
void toCvPose(collaborative_fusion::FramePose &fp,PoseSE3dList &poses)
{
  geometry_msgs::Transform transform0,transform1,transform2,transform3;
  transform0 = fp.pose0;
  transform1 = fp.pose1;
  transform2 = fp.pose2;
  transform3 = fp.pose3;

  Transform t0,t1,t2,t3;
  tf::transformMsgToEigen(transform0,t0);
  tf::transformMsgToEigen(transform1,t1);
  tf::transformMsgToEigen(transform2,t2);
  tf::transformMsgToEigen(transform3,t3);

  poses.clear();
  poses.emplace_back(t0);
  poses.emplace_back(t1);
  poses.emplace_back(t2);
  poses.emplace_back(t3);
}
void toCvPoseList(std::vector<geometry_msgs::Transform> &g_poses, PoseSE3dList &poses)
{
  poses.clear();
  for(int i = 0;i!=g_poses.size();++i)
  {
    Transform t;
    tf::transformMsgToEigen(g_poses[i], t);
    poses.emplace_back(t);
  }
}
void toServerFrame(collaborative_fusion::Frame &cf, Server::ServerFrame & sf)
{
  sf.frame_index = cf.frame_index;
  sf.cameraID = cf.camera_id;
  sf.submapID = cf.submap_id;
  sf.keyPointsNum = cf.keyPointsNum;
  toCvPose(cf.pose, sf.pose_sophus);
  toCvLocalPoints(cf.local_points, sf.local_points);
  toCvKeyPoints(cf.keypoints, sf.keypoints);
  toCvDescriptor(cf.descriptor, sf.descriptor); 
  toCvDepth(cf.depth, sf.depth);
  toCvRgb(cf.rgb, sf.rgb);
}

void toRosVec3f(Point3fList & e_vec, std::vector<geometry_msgs::Point32> & ros_vec)
{
  ros_vec.resize(e_vec.size());
  for(int i = 0; i != e_vec.size(); ++i)
  {
    ros_vec[i].x = e_vec[i](0);
    ros_vec[i].y = e_vec[i](1);
    ros_vec[i].z = e_vec[i](2);
  }
}
void toEigenVec3f(std::vector<geometry_msgs::Point32> & ros_vec, Point3fList & e_vec)
{
  e_vec.resize(ros_vec.size());
  for(int i = 0; i != e_vec.size(); ++i)
  {
    e_vec[i](0) = ros_vec[i].x;
    e_vec[i](1) = ros_vec[i].y;
    e_vec[i](2) = ros_vec[i].z;
  }  
}
void toRosRoomModel(chisel::PointCloud &pcd, collaborative_fusion::RoomModel &model)
{
  std::vector<geometry_msgs::Point32> &vertices = model.vertices;
  std::vector<geometry_msgs::Point32> &normals = model.normals;
  std::vector<long> &labels = model.labels;
  std::vector<geometry_msgs::Point32> &colors = model.colors;
  size_t size = pcd.vertices.size();
  vertices.resize(size);
  normals.resize(size);
  labels.resize(size);
  colors.resize(size);
  for(int i = 0; i !=pcd.vertices.size(); ++i)
  {
    vertices[i].x = pcd.vertices[i](0);
    vertices[i].y = pcd.vertices[i](1);
    vertices[i].z = pcd.vertices[i](2);

    normals[i].x = pcd.normals[i](0);
    normals[i].y = pcd.normals[i](1);
    normals[i].z = pcd.normals[i](2);

    colors[i].x = pcd.colors[i](0);
    colors[i].y = pcd.colors[i](1);
    colors[i].z = pcd.colors[i](2);
    
    labels[i] = pcd.labels[i];
  }
}
void toChiselPcd(collaborative_fusion::RoomModel &model, chisel::PointCloud &pcd)
{
  std::vector<geometry_msgs::Point32> &vertices = model.vertices;
  std::vector<geometry_msgs::Point32> &normals = model.normals;
  std::vector<geometry_msgs::Point32> &colors = model.colors;
  std::vector<long> &labels = model.labels;
  size_t size = vertices.size();
  pcd.vertices.resize(size);
  pcd.normals.resize(size);
  pcd.labels.resize(size);
  pcd.colors.resize(size);
  for(int i = 0; i !=pcd.vertices.size(); ++i)
  {
    pcd.vertices[i](0) = vertices[i].x ;
    pcd.vertices[i](1) = vertices[i].y ;
    pcd.vertices[i](2) = vertices[i].z ;

    pcd.normals[i](0) = normals[i].x ;
    pcd.normals[i](1) = normals[i].y ;
    pcd.normals[i](2) = normals[i].z ;

    pcd.colors[i](0) = colors[i].x;
    pcd.colors[i](1) = colors[i].y;
    pcd.colors[i](2) = colors[i].z;
     
    pcd.labels[i] = labels[i];
  }  
}
void toRosPointCloud(chisel::PointCloud &pcd,std::vector<int> &compressed_colors, std::vector<int> &semantic_colors, 
  collaborative_fusion::PointCloud &ros_pcd)
{
  std::vector<geometry_msgs::Point32> &vertices = ros_pcd.vertices;
  std::vector<geometry_msgs::Point32> &normals = ros_pcd.normals;
  ros_pcd.colors = compressed_colors;
  ros_pcd.semantic = semantic_colors;
  size_t size = pcd.vertices.size();

  vertices.resize(size);
  normals.resize(size);
  
  for(int i = 0; i !=pcd.vertices.size(); ++i)
  {
    vertices[i].x = pcd.vertices[i](0);
    vertices[i].y = pcd.vertices[i](1);
    vertices[i].z = pcd.vertices[i](2);

    normals[i].x = pcd.normals[i](0);
    normals[i].y = pcd.normals[i](1);
    normals[i].z = pcd.normals[i](2);
  }  
}
void toRosPointCloud(std::vector<chisel::PointCloud> &pcds, std::vector<std::vector<int>> &compressed_colors, 
  std::vector<std::vector<int>> &semantic_colors, std::vector<collaborative_fusion::PointCloud> &ros_pcds)
{
  ros_pcds.resize(pcds.size());
  for(int i = 0; i != pcds.size(); ++i)
  {
    toRosPointCloud(pcds[i], compressed_colors[i], semantic_colors[i], ros_pcds[i]);
  }
}
void toServerPointCloud(collaborative_fusion::PointCloud &ros_pcd, Server::ServerPointCloud &s_pcd)
{
  std::vector<geometry_msgs::Point32> &vertices = ros_pcd.vertices;
  std::vector<geometry_msgs::Point32> &normals = ros_pcd.normals;
  s_pcd.colors = ros_pcd.colors;
  s_pcd.semantic = ros_pcd.semantic;
  s_pcd.vertices.resize(vertices.size());
  s_pcd.normals.resize(normals.size());
  for(int i = 0; i < vertices.size(); ++i)
  {
    s_pcd.vertices[i](0) = vertices[i].x ;
    s_pcd.vertices[i](1) = vertices[i].y ;
    s_pcd.vertices[i](2) = vertices[i].z ;

    s_pcd.normals[i](0) = normals[i].x ;
    s_pcd.normals[i](1) = normals[i].y ;
    s_pcd.normals[i](2) = normals[i].z ;    
  }
}
void toServerPointCloud(std::vector<collaborative_fusion::PointCloud> &ros_pcds, std::vector<Server::ServerPointCloud> &s_pcds)
{
  s_pcds.resize(ros_pcds.size());
  for(int i = 0; i != ros_pcds.size(); ++i)
  {
    toServerPointCloud(ros_pcds[i], s_pcds[i]);
  }
}
void toRosRoomInfo(int camera_id, int room_id, 
  std::vector<int> &room_to_submap, chisel::PcdPtr &pcd_ptr, 
    collaborative_fusion::RoomInfo & room_info)
{
  room_info.camera_id = camera_id;
  room_info.room_id = room_id;
  room_info.room_to_submap = room_to_submap;
  toRosRoomModel(*pcd_ptr,room_info.room_model);
}
void toServerRoomInfo(collaborative_fusion::RoomInfo & cr, Server::RoomInfo &sr)
{
  sr.camera_id = cr.camera_id;
  sr.room_id = cr.room_id;
  sr.contained_submaps = cr.room_to_submap;
  sr.model = chisel::PcdPtr(new chisel::PointCloud());
  sr.room_area = cr.room_area;
  toChiselPcd(cr.room_model, *sr.model);
}

}