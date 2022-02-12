#ifndef FRAME_H
#define FRAME_H

#include <string>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <memory>
#include <fstream>
#include <sstream>
#include <sophus/se3.hpp>
#include <atomic>
#include <thread>


#include "../Geometry/Geometry.h"
typedef Eigen::Vector3i ChunkID;
typedef std::vector<ChunkID, Eigen::aligned_allocator<ChunkID> > ChunkIDList;
//typedef std::vector<Eigen::Vector3f , Eigen::aligned_allocator<Eigen::Vector3f> > Point3fList;

typedef std::vector<Sophus::SE3d , Eigen::aligned_allocator<Eigen::Vector3d> > PoseSE3dList;
typedef Sophus::SE3d PoseSE3d;



inline Eigen::Vector3d applyPose( const Sophus::SE3d &pose, const Eigen::Vector3d &point )
{
    return pose.so3() * point + pose.translation();
}


class Frame
{
public:
EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  // dense scene info


  int frame_index;
  size_t keyPointsNum;
  cv::Mat rgb;
  cv::Mat depth;
  cv::Mat refined_depth;
  cv::Mat normal_map;
  cv::Mat weight;
  cv::Mat colorValidFlag;
  // sparse feature info
  std::vector<cv::KeyPoint > keypoints;
  cv::Mat descriptor;
  Point3dList local_points;


// pose_sophus[0] for start submap poses * relative_pose_in submap, 
// pose_sophus[1] for previous pose of pose_sophus[0],
// [0], [1] are mainly used for chunk allocation(integration) 
// pose_sophus[2] for current the relative pose in submap,
// pose_sophus[3] for global pose(*)
  PoseSE3dList pose_sophus;
  // time stamp
  double time_stamp;

  int tracking_success;

  int is_keyframe;
  int is_fixed_frame;
  int origin_index;
  float depth_scale;
  std::vector<bool> isValid;
  std::vector<bool> isSaved;
  //bool lock = false;
  int submapID;
  int cameraID;//multi-camera, so we need cameraID
  //for tsdf fusion
  ChunkIDList validChunks;
  std::vector<void *> validChunksPtr;
  //std::vector<std::atomic_flag> locks;
  void setSubmapID(int _submapID)
  {
      submapID = _submapID;
  }
  void setCameraID(int _cameraID)
  {
    cameraID = _cameraID;
  }
  bool savePoints(const std::string &filename, const Point3dList &local_points)
  {
    if(local_points.size())
    {
    std::ofstream ofs(filename);
    ofs<<local_points.size();
    for(int i = 0;i!= local_points.size(); ++i)
    {
        //std::cout<<local_points[i]<<std::endl;
        ofs<<" "<<local_points[i](0)<<" "<<local_points[i](1) <<" "<<local_points[i](2);
    }
    ofs.close();
    }
    return true;
  }
  bool loadPoints(const std::string &filename, Point3dList &local_points)
  {
      local_points = Point3dList();
      std::ifstream ifs(filename);
      int size,i = 0;
      double x,y,z;
      if(ifs) ifs >> size;
      while(ifs&&i<size)
      {
          ifs>>x>>y>>z;
          local_points.push_back(Eigen::Vector3d(x,y,z));
          ++i;
      }
      ifs.close();
      return i == size;
  }
  int GetOccupiedMemorySize()
  {
//      printf("memory occupied: %d %d %d %d      %d %d %d %d     %d %d %d %d\r\n",
//             (rgb.datalimit - rgb.data),
//             (depth.datalimit - depth.data) ,
//             (refined_depth.datalimit - refined_depth.data) ,
//             (normal_map.datalimit - normal_map.data) ,

//             (weight.datalimit - weight.data) ,
//             (descriptor.datalimit - descriptor.data) ,
//             keypoints.size() * sizeof(cv::KeyPoint) ,
//             feature_tracked_flag.size() * sizeof(unsigned char) ,

//             local_points.size() * sizeof(Eigen::Vector3d) ,
//             validChunks.size() * sizeof(ChunkID) ,
//             pose_sophus.size() * sizeof(Sophus::SE3d),
//             validChunksPtr.size() * sizeof(void *));
      return ( (rgb.datalimit - rgb.data) +
               (depth.datalimit - depth.data) +
               (refined_depth.datalimit - refined_depth.data) +
               (normal_map.datalimit - normal_map.data) +
               (weight.datalimit - weight.data) +
               (descriptor.datalimit - descriptor.data) +
               keypoints.size() * sizeof(cv::KeyPoint) +
               local_points.size() * sizeof(Eigen::Vector3d) +
               validChunks.size() * sizeof(ChunkID) +
               pose_sophus.size() * sizeof(Sophus::SE3d)+
               validChunksPtr.size() * sizeof(void *) +
               (colorValidFlag.datalimit - colorValidFlag.data) + 4*6
                              );
  }

  // preserve feature/rgb/depth
  void clear_keyframe_memory()
  {
      //isValid[7] = false;
      //depth.release();
      isValid[6] = false;
      weight.release();

      normal_map.release();

  }
  // preserve local depth
  void clear_redudent_memoery()
  {
      isValid[0] = false;
      rgb.release();
      isValid[1] = false;
      colorValidFlag.release();
      isValid[7] = false;
      depth.release();
      isValid[6] = false;
      weight.release();
      normal_map.release();
      isValid[2] = false;
      keypoints.clear();
      std::vector<cv::KeyPoint > r_keypoints;
      keypoints.swap(r_keypoints);
      isValid[3] = false;
      descriptor.release();
      isValid[4] = false;
      local_points.clear();
      Point3dList r_local_points;
      local_points.swap(r_local_points);
  }

  // remove frames totally
  void clear_memory()
  {
    
    isValid[0] = false;
    rgb.release();
    isValid[1] = false;
    colorValidFlag.release();
    isValid[7] = false;
    depth.release();
    isValid[5] = false;
    refined_depth.release();
    isValid[6] = false;
    weight.release();
    normal_map.release();

    isValid[2] = false;
    keypoints.clear();
    std::vector<cv::KeyPoint > r_keypoints;
    keypoints.swap(r_keypoints);
    
    isValid[3] = false;
    descriptor.release();
    
    isValid[4] = false;
    local_points.clear();
    Point3dList r_local_points;
    local_points.swap(r_local_points);
    
  }

  Eigen::Vector3d localToGlobal(const Eigen::Vector3d &point)
  {
      return pose_sophus[0].so3() * point + pose_sophus[0].translation();
  }
  void saveRgb(const std::string &filepath = "./")
  {
       std::string filename;
       if(isSaved[0]) return;
      if(isValid[0] &&rgb.dataend -rgb.datastart>0)
      {
       // if(filepath == "./")
        filename = filepath+"frames/color/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index)+".png";
        //std::cout<<"save rgb: "<<filename<<std::endl;
       // else filename = filepath+"/"+std::to_string(frame_index)+".png";
      cv::imwrite(filename,rgb);
      isSaved[0] = true;
      }
  }
    void saveRefinedDepth(const std::string &filepath = "./")
    {
        std::string filename;
        if(isSaved[5]) return;
      if(isValid[5] &&refined_depth.dataend - refined_depth.datastart >0)
      {
          filename = filepath + "frames/refined_depth/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs<<"refined_depth"<<refined_depth;
    fs.release();
      isSaved[5] = true;
      }

    }
    void saveDepth(const std::string &filepath = "./")
    {
      std::string filename;
      if(isSaved[7]) return;
      if(isValid[7] &&depth.dataend -depth.datastart>0)
      {
        filename = filepath+"frames/depth/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index)+".png";
      cv::imwrite(filename,depth);
      isSaved[7] = true;
      }
    }
    void saveColorValidFlag(const std::string &filepath = "./")
    {
        std::string filename;
        if(isSaved[1]) return;
      if(isValid[1] &&colorValidFlag.dataend - colorValidFlag.datastart>0)
      {

      filename = filepath + "frames/colorValidFlag/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs<<"colorValidFlag"<<colorValidFlag;
    fs.release();
      isSaved[1] = true;
      }
    }
    void saveKeypoints(const std::string & filepath = "./")
    {
        std::string filename;
        if(isSaved[2]) return;
       if(isValid[2] &&keypoints.size() > 0)
      {

    filename = filepath + "frames/keypoints/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs<<"keypoints"<<keypoints;
    fs.release();
      isSaved[2] = true;
      }
    }
    void saveDescriptor(const std::string &filepath = "./")
    {
        std::string filename;
        if(isSaved[3]) return;
      if(isValid[3] &&descriptor.dataend - descriptor.datastart >0)
      {

          filename = filepath + "frames/descriptor/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs<<"descriptor"<<descriptor;
    fs.release();
      isSaved[3] = true;
      }
    }
    void saveLocalPoints(const std::string &filepath="./")
    {
        std::string filename;
        if(isSaved[4]) return;
      if(isValid[4] &&local_points.size() > 0)
      {

          filename = filepath + "frames/local_points/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index);
          savePoints(filename, local_points );
          isSaved[4] = true;
      }
    }
    void saveWeight(const std::string & filepath = "./")
    {
        std::string filename;
        if(isSaved[6]) return;
      if(isValid[6] &&weight.dataend - weight.datastart >0)
      {

    filename = filepath + "frames/weight/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs<<"weight"<<weight;
    fs.release();
      isSaved[6] = true;
      }
    }
    void saveAllK(const std::string &filepath = "./")
    {
      saveDepth(filepath);
      saveRgb(filepath);
      saveRefinedDepth(filepath);
      saveDescriptor(filepath);
      saveLocalPoints(filepath);
      //saveWeight(filepath);
      saveKeypoints(filepath);
      saveColorValidFlag(filepath);
    }
    void saveAllN(const std::string &filepath = "./")
    {
      //saveDepth(filepath);
      //saveRgb(filepath);
      saveRefinedDepth(filepath);
      //saveDescriptor(filepath);
      //saveLocalPoints(filepath);
      //saveWeight(filepath);
      //saveKeypoints(filepath);
      //saveColorValidFlag(filepath);
    }
/*


      if(isValid[0] &&rgb.dataend -rgb.datastart>0)
      {
      isValid[0] = false;
        filename = filepath+"frames/color/"+std::to_string(frame_index)+".png";
      cv::imwrite(filename,rgb);

      }
      if(isValid[5] &&refined_depth.dataend -refined_depth.datastart>0)
      {
      isValid[5] = false;
      filename = filepath+ "frames/refined_depth/"+std::to_string(frame_index) + ".png";
      cv::imwrite(filename,refined_depth);
      }

      if(isValid[2] &&keypoints.size() > 0)
      {
      isValid[2] = false;
    filename = filepath + "frames/keypoints/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs<<"keypoints"<<keypoints;
    fs.release();

      }
      if(isValid[3] &&descriptor.dataend - descriptor.datastart >0)
      {
      isValid[3] = false;
          filename = filepath + "frames/descriptor/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs<<"descriptor"<<descriptor;
    fs.release();

      }
      if(isValid[4] &&local_points.size() > 0)
      {
          isValid[4] = false;
          filename = filepath + "frames/local_points/"+std::to_string(frame_index);
          savePoints(filename, local_points );

      }
      if(isValid[6] &&weight.dataend - weight.datastart >0)
      {
      isValid[6] = false;
    filename = filepath + "frames/weight/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    fs<<"weight"<<weight;
    fs.release();

      }
*/
  void saveImages(const std::string &filepath = "./")
  {
      //while(lock);
      //lock = true;
      /*saveRgb(filepath);
      saveColorValidFlag(filepath);
      saveDescriptor(filepath);
      saveKeypoints(filepath);
      saveLocalPoints(filepath);
      saveRefinedDepth(filepath);
      saveWeight(filepath);
      */
      if(is_keyframe)
      saveAllK();
      //else
      //saveAllN();

      clear_memory();
      //lock = false;
      /*
      std::vector<std::thread> threads(7);
      threads[0] = std::thread(saveRgb,std::ref(filepath));
      threads[1] = std::thread(saveLocalPoints,std::ref(filepath));
      for(int i = 0,i!=7;++i)
      threads[i].join();
      clear_memory();
      */
  }
  void saveKeyFrame(const std::string &filepath = "./")
  {
      saveWeight(filepath);
      clear_keyframe_memory();
  }

  /*void loadImages(const std::string &filepath = "./")
  {
      if(isValid == true)
      return;
      std::string filename = filepath +"color/"+std::to_string(frame_index) + ".png";
      rgb = cv::imread(filename);
      filename = filepath + "refined_depth/"+std::to_string(frame_index) + ".png";
      refined_depth = cv::imread(filename);

      isValid = true;
  }*/
  cv::Mat & getRgb(const std::string &filepath= "./")
  {
      //while(lock);
      //lock = true;
      if(isValid[0])
        ;
      else if(isSaved[0])
      {

      std::string filename = filepath +"frames/color/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".png";
      //std::cout<<filename<<std::endl;
          rgb = cv::imread(filename);
          //cv::imshow("1",rgb);
          //cv::waitKey(0);
          isValid[0] = true;
      }
        //lock = false;
        return rgb;
  }
  cv::Mat & getDepth(const std::string &filepath= "./")
  {
      //while(lock);
      //lock = true;
      if(isValid[7])
        ;
      else if(isSaved[7])
      {

      std::string filename = filepath +"frames/depth/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".png";
      //std::cout<<filename<<std::endl;
          depth = cv::imread(filename);
          //cv::imshow("1",rgb);
          //cv::waitKey(0);
          isValid[7] = true;
      }
        //lock = false;
        return depth;
  }
  cv::Mat & getRefinedDepth(const std::string &filepath= "./")
  {
      if(isValid[5])
      ;//return descriptor;
      else if(isSaved[5])
      {
      std::string filename = filepath+"frames/refined_depth/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["refined_depth"]>>refined_depth;
          isValid[5] = true;
         // return descriptor;
      }
      return refined_depth;
  }
  cv::Mat & getColorValidFlag(const std::string &filepath= "./")
  {
      //while(lock);
      //lock = true;
      if(isValid[1])
      ;//return colorValidFlag;
      else if(isSaved[1])
      {
      std::string filename = filepath +"frames/colorValidFlag/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";

    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["colorValidFlag"]>>colorValidFlag;
    //fs.release();
          //colorValidFlag = cv::imread(filename);
          isValid[1] = true;

          //std::cout<<"colorValidFlag: "<<frame_index <<" "<< colorValidFlag.dataend - colorValidFlag.datastart<<std::endl;
         // return colorValidFlag;
      }
      //lock = false;
      return colorValidFlag;
  }
std::vector<cv::KeyPoint >& getKeypoints(const std::string &filepath= "./")
  {
      //while(lock);
      //lock = true;
      if(isValid[2])
      ;//return keypoints;
      else if(isSaved[2])
      {
      std::string filename = filepath+"frames/keypoints/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["keypoints"]>>keypoints;
          isValid[2] = true;
         // return keypoints;
      }
      //lock = false;
      return keypoints;
  }
  cv::Mat & getDescriptor(const std::string &filepath= "./")
  {
      //while(lock);
      //lock = true;
      if(isValid[3])
      ;//return descriptor;
      else if(isSaved[3])
      {
      std::string filename = filepath+"frames/descriptor/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["descriptor"]>>descriptor;
          isValid[3] = true;
         // return descriptor;
      }
    //lock = false;
    return descriptor;

  }
  Point3dList & getLocalPoints(const std::string &filepath= "./")
  {
      if(isValid[4])
      ;//return local_points;
      else if(isSaved[4])
      {
      std::string filename = filepath+"frames/local_points/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) ;
        if(loadPoints(filename,local_points))
        {
            isValid[4] = true;
          return local_points;
        }
        std::cout<<"Load local points wrong!"<<std::endl;
       // return local_points;
      }
      //lock = false;
      return local_points;
  }
  cv::Mat & getWeight(const std::string &filepath= "./")
  {
      if(isValid[6])
      ;//return weight;
      else if(isSaved[6])
      {
      std::string filename = filepath+"frames/weight/camera_"+std::to_string(cameraID)+"/"+std::to_string(frame_index) + ".xml";
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    fs["weight"]>>weight;

          isValid[6] = true;
          //return weight;
      }
    //lock = false;
    return weight;
  }
    Frame()
    {
        frame_index = 0;
        is_fixed_frame = 0;
        origin_index = 1e8;	// no origin by default
        isValid = std::vector<bool>(8,true);
        isSaved = std::vector<bool>(8,false);
        keypoints.clear();
        descriptor.release();
        rgb.release();
        depth.release();
        refined_depth.release();
        //locks = std::vector<std::atomic_flag>(7);
        local_points.clear();
        tracking_success = 0;
        is_keyframe = 0;
        pose_sophus.push_back(Sophus::SE3d());
        pose_sophus.push_back(Sophus::SE3d());
        pose_sophus.push_back(Sophus::SE3d());
        pose_sophus.push_back(Sophus::SE3d());
    }
};



#endif
