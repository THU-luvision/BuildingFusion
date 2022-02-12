cmake_minimum_required(VERSION 2.8.2)
project(collaborative_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  eigen_conversions
  cv_bridge
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   KeyPoint.msg
   FramePose.msg
   FrameLocalPoints.msg
   FrameKeyPoints.msg
   FrameDescriptor.msg
   Frame.msg
   UpdateFrame.msg
   RoomModel.msg
   RoomInfo.msg
   PointCloud.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   LoopClosureDetection.srv
   GlobalOptimization.srv
   MeshVisualization.srv
   Registration.srv
   RoomRegistration.srv
   WithDraw.srv
   Termination.srv
#   Service1.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs   sensor_msgs   std_msgs #collaborative_fusion
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES collaborative_fusion
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/collaborative_fusion.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/collaborative_fusion_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_collaborative_fusion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
#set(CMAKE_BUILD_TYPE Release )#CACHE STRING "Choose the type of build." FORCE)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -w -g2 -ggdb")


# these are useless because we already use the find_package funtion.
#But if we remain this order, the make process will not succeed.



#set(OpenCV_DIR "/workspace/deps/opencv-3.3.0/build")
set(OPENNI2_LIBRARY "/workspace/deps/OpenNI-Linux-x64-2.2/Redist/libOpenNI2.so")
set(OPENNI2_INCLUDE_DIR "/workspace/deps/OpenNI-Linux-x64-2.2/Include")

set(TORCH_PATH "${PROJECT_SOURCE_DIR}/../../deps/libtorch")


#MESSAGE( STATUS "Remember to check OpenNI dir: " ${OPENNI2_LIBRARY})


set(GCFUSION_SHADER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Shaders" CACHE PATH "Where the shaders live")
message("${CMAKE_MODULE_PATH}")


find_package(OpenCV REQUIRED)
find_package(OpenGL)
find_package(GLUT)
find_package( PkgConfig )

find_package(OpenNI2 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin 0.1 REQUIRED)
find_package(Boost COMPONENTS system thread  filesystem REQUIRED)
find_package(Open3D HINTS /usr/local/lib/CMake)
list(APPEND Open3D_LIBRARIES dl)
message("--------------------------------------------------${TORCH_PATH}")
find_package(Torch REQUIRED 
             HINTS ${TORCH_PATH})




MESSAGE( STATUS "Remember to check Opencv dir: " ${OpenCV_DIR})
MESSAGE("torch dir: ${TORCH_INCLUDE_DIRS}")
MESSAGE("torch lib: ${TORCH_LIBRARIES}")
MESSAGE("pangolin ${Pangolin_LIBRARIES}")
MESSAGE("roscpp ${roscpp_INCLUDE_DIRS}")


#find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
#if(NOT REALSENSE2_FOUND)
#    SET(REALSENSE2_FOUND "realsense2")
#    message(WARN "Failed to find_library(realsense2)")
#endif()
#MESSAGE( STATUS "Realsense dir: " ${ADDITIONAL_INCLUDE_DIRS})
#MESSAGE( STATUS "Realsense lib: " ${REALSENSE2_FOUND})


file(GLOB helloworld_SRC
"src/BasicAPI.cpp"
"src/CommunicationAPI.cpp"
"src/offline_reconstruction.cpp"
"src/offline_reconstruction.h"
"src/server/*.h"
"src/server/*.cpp"
"src/*.h"
"src/Tools/*.h"
"src/Tools/*.cpp"
"src/GCSLAM/*.cpp"
"src/GCSLAM/*.h"

"src/GCSLAM/MILD/*.cpp"
"src/GCSLAM/MILD/*.h"
"src/GCSLAM/ORBSLAM/*.cpp"
"src/GCSLAM/ORBSLAM/*.h"

"src/Geometry/*.h"
"src/Geometry/*.cpp"
"src/GCFusion/*.cpp"
"src/GCFusion/*.h"
"src/Shaders/*.cpp"
"src/Shaders/*.h"
"src/RoomDetection/*.h"
"src/RoomDetection/*.cpp"
"src/RoomLCD/*.h"
"src/RoomLCD/*.cpp"
"src/RoomLCD/graphmatching/*.h"
"src/RoomLCD/graphmatching/*.cpp"
"src/IO/rply/*.h"
"src/IO/rply/*.c"
"src/IO/*.h"
"src/IO/*.cpp"
"ARDemo/SocketServer.h"
"ARDemo/SocketServer.cpp"
)



set(SOURCE_FILES
 ${helloworld_SRC})

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -w")

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

message("sophus DIR: ${SOPHUS_INCLUDE_DIR}")
message("open3D dir: ${OPEN3D_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS}  )
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories( ${SOPHUS_INCLUDE_DIR} )
include_directories( ${Boost_INCLUDE_DIR} )
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${OPEN3D_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src/CHISEL/src)


#include_directories(${FC_INCLUDE_DIR})
include_directories(${ADDITIONAL_INCLUDE_DIRS})
#add_definitions(-DWITH_REALSENSE)
#set(EXTRA_LIBS ${EXTRA_LIBS} ${REALSENSE2_FOUND})
#link_directories(${Open3D_LIBRARY_DIRS})
# set(CMAKE_CXX_FLAGS ${ADDITIONAL_CMAKE_CXX_FLAGS} "-O3 -msse2 -msse3  -msse4.1  -std=c++14 -DSHADER_DIR=${GCFUSION_SHADER_DIR}")
set(CMAKE_CXX_FLAGS ${ADDITIONAL_CMAKE_CXX_FLAGS} " -g -O0 -msse2 -msse3  -msse4.1  -std=c++14 -DSHADER_DIR=${GCFUSION_SHADER_DIR}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2 -ftree-vectorize -lboost_system -lpthread -lboost_thread -lboost_filesystem -mavx2")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_GLIBCXX_USE_CXX11_ABI=0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lcnpy -lz")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")



add_subdirectory(${PROJECT_SOURCE_DIR}/src/scn_cpp)
include_directories(${SCN_CPP_INCLUDE_PATH})
add_subdirectory(${PROJECT_SOURCE_DIR}/src/CHISEL/src)


#link_directories(${CHISEL_DIRS})


message("${CMAKE_CXX_FLAGS}")
add_executable(collaborative_fusion_client  src/main.cpp ${SOURCE_FILES})
add_dependencies(collaborative_fusion_client collaborative_fusion_generate_messages_cpp)
target_link_libraries(collaborative_fusion_client ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES}  
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )


add_executable(collaborative_fusion_server src/server_mild.cpp ${SOURCE_FILES})
add_dependencies(collaborative_fusion_server collaborative_fusion_generate_messages_cpp)


target_link_libraries(collaborative_fusion_server ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES} 
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )

add_executable(room_lcd_test src/room_lcd.cpp ${SOURCE_FILES})
add_dependencies(room_lcd_test collaborative_fusion_generate_messages_cpp)


target_link_libraries(room_lcd_test ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES} 
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )

add_executable(icp_test src/icp_test.cpp ${SOURCE_FILES})
add_dependencies(icp_test collaborative_fusion_generate_messages_cpp)


target_link_libraries(icp_test ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES} 
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )
add_executable(normal_optimization_test src/flatten_floor_test.cpp ${SOURCE_FILES})
add_dependencies(normal_optimization_test collaborative_fusion_generate_messages_cpp)


target_link_libraries(normal_optimization_test ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES} 
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )

add_executable(server_termination src/server_termination.cpp ${SOURCE_FILES})
add_dependencies(server_termination collaborative_fusion_generate_messages_cpp)


target_link_libraries(server_termination ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES} 
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )

add_executable(final_processing src/final_processing.cpp ${SOURCE_FILES})
add_dependencies(final_processing collaborative_fusion_generate_messages_cpp)


target_link_libraries(final_processing ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES} 
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )
add_executable(offline_optimization src/offline_optimization.cpp ${SOURCE_FILES})
#add_dependencies(final_processing collaborative_fusion_generate_messages_cpp)


target_link_libraries(offline_optimization ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES} 
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )

add_executable(optimization_comparison src/optimization_comparison.cpp )


target_link_libraries(optimization_comparison ${OpenCV_LIBS} 
                      ${Eigen_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      chisel
                      ${OPENNI2_LIBRARY}
                      ${OPENGL_LIBRARIES} 
                      ${GLUT_LIBRARY}
                      ${EXTRA_LIBS}
                      ${pthread} 
                      ${Boost_LIBRARIES} 
                      ${TORCH_LIBRARIES}
                      scn_cpp
                      ${catkin_LIBRARIES}
                      ${Open3D_LIBRARIES}
          )

