#ifndef OFFLINE_RECONSTRUCTION_H
#define OFFLINE_RECONSTRUCTION_H
#include <iostream>
#include <opencv/cv.h>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <time.h>
#include <list>
#include <omp.h>
#include <stdio.h>
#include <ctime>

#include <pthread.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "GCSLAM/frame.h"
#include "GCSLAM/MILD/loop_closure_detector.hpp"
#include "GCSLAM/MILD/BayesianFilter.hpp"
#include "GCSLAM/MultiViewGeometry.h"
#include "GCSLAM/GCSLAM.h"

#include "GCFusion/MapMaintain.hpp"
#include "GCFusion/MobileGUI.hpp"
#include "GCFusion/MobileFusion.h"


#include "BasicAPI.h"

#include "CHISEL/src/open_chisel/Chisel.h"
#include "CHISEL/src/open_chisel/ProjectionIntegrator.h"
#include "CHISEL/src/open_chisel/camera/PinholeCamera.h"
#include "CHISEL/src/open_chisel/Stopwatch.h"

#include "Tools/LogReader.h"
#include "Tools/LiveLogReader.h"
#include "Tools/RawLogReader.h"
#define MULTI_THREAD 1

void OfflineReconstruction(const std::string &base_path, const std::string &associate, 
    const Eigen::Matrix4f & start_pose, const std::string &final_generated_model);
void OfflineReconstruction(const std::string &base_path, const std::string &last_associate, const std::string &associate, 
   const std::string &next_associate, const Eigen::Matrix4f & start_pose, const std::string &final_generated_model);

#endif