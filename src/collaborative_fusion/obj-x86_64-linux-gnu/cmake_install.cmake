# Install script for directory: /media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/ros/kinetic")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collaborative_fusion/msg" TYPE FILE FILES
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collaborative_fusion/srv" TYPE FILE FILES
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collaborative_fusion/cmake" TYPE FILE FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/catkin_generated/installspace/collaborative_fusion-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/devel/include/collaborative_fusion")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/devel/share/roseus/ros/collaborative_fusion")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/devel/share/common-lisp/ros/collaborative_fusion")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/devel/share/gennodejs/ros/collaborative_fusion")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python2" -m compileall "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/devel/lib/python2.7/dist-packages/collaborative_fusion")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/devel/lib/python2.7/dist-packages/collaborative_fusion")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/catkin_generated/installspace/collaborative_fusion.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collaborative_fusion/cmake" TYPE FILE FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/catkin_generated/installspace/collaborative_fusion-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collaborative_fusion/cmake" TYPE FILE FILES
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/catkin_generated/installspace/collaborative_fusionConfig.cmake"
    "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/catkin_generated/installspace/collaborative_fusionConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collaborative_fusion" TYPE FILE FILES "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/gtest/cmake_install.cmake")
  include("/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/src/scn_cpp/cmake_install.cmake")
  include("/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/src/CHISEL/src/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/obj-x86_64-linux-gnu/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
