# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "collaborative_fusion: 10 messages, 7 services")

set(MSG_I_FLAGS "-Icollaborative_fusion:/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/kinetic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(collaborative_fusion_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv" ""
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv" ""
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg" "collaborative_fusion/KeyPoint"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv" "geometry_msgs/Point32:geometry_msgs/Quaternion:geometry_msgs/Vector3:collaborative_fusion/UpdateFrame:geometry_msgs/Transform"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg" "collaborative_fusion/RoomModel:geometry_msgs/Point32"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv" "geometry_msgs/Point:sensor_msgs/Image:collaborative_fusion/FrameKeyPoints:collaborative_fusion/FrameDescriptor:collaborative_fusion/FrameLocalPoints:collaborative_fusion/FramePose:geometry_msgs/Transform:geometry_msgs/Vector3:collaborative_fusion/Frame:geometry_msgs/Quaternion:collaborative_fusion/KeyPoint:std_msgs/Header"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg" "geometry_msgs/Point32"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg" ""
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv" "geometry_msgs/Quaternion:geometry_msgs/Point32:geometry_msgs/Vector3:geometry_msgs/Transform:collaborative_fusion/RoomModel:collaborative_fusion/RoomInfo"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg" "geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/Vector3"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg" "geometry_msgs/Point:collaborative_fusion/FrameKeyPoints:collaborative_fusion/FrameDescriptor:collaborative_fusion/FrameLocalPoints:collaborative_fusion/FramePose:geometry_msgs/Transform:geometry_msgs/Vector3:sensor_msgs/Image:geometry_msgs/Quaternion:collaborative_fusion/KeyPoint:std_msgs/Header"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg" "geometry_msgs/Point32"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv" "geometry_msgs/Point32:collaborative_fusion/PointCloud"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv" ""
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg" "geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/Vector3"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg" "geometry_msgs/Point"
)

get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg" NAME_WE)
add_custom_target(_collaborative_fusion_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collaborative_fusion" "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg" "sensor_msgs/Image:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)

### Generating Services
_generate_srv_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_cpp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
)

### Generating Module File
_generate_module_cpp(collaborative_fusion
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(collaborative_fusion_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(collaborative_fusion_generate_messages collaborative_fusion_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_cpp _collaborative_fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collaborative_fusion_gencpp)
add_dependencies(collaborative_fusion_gencpp collaborative_fusion_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collaborative_fusion_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)

### Generating Services
_generate_srv_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_eus(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
)

### Generating Module File
_generate_module_eus(collaborative_fusion
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(collaborative_fusion_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(collaborative_fusion_generate_messages collaborative_fusion_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_eus _collaborative_fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collaborative_fusion_geneus)
add_dependencies(collaborative_fusion_geneus collaborative_fusion_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collaborative_fusion_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)

### Generating Services
_generate_srv_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_lisp(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
)

### Generating Module File
_generate_module_lisp(collaborative_fusion
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(collaborative_fusion_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(collaborative_fusion_generate_messages collaborative_fusion_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_lisp _collaborative_fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collaborative_fusion_genlisp)
add_dependencies(collaborative_fusion_genlisp collaborative_fusion_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collaborative_fusion_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)

### Generating Services
_generate_srv_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_nodejs(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
)

### Generating Module File
_generate_module_nodejs(collaborative_fusion
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(collaborative_fusion_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(collaborative_fusion_generate_messages collaborative_fusion_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_nodejs _collaborative_fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collaborative_fusion_gennodejs)
add_dependencies(collaborative_fusion_gennodejs collaborative_fusion_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collaborative_fusion_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  "${MSG_I_FLAGS}"
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_msg_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)

### Generating Services
_generate_srv_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/kinetic/share/sensor_msgs/cmake/../msg/Image.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point32.msg;/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)
_generate_srv_py(collaborative_fusion
  "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
)

### Generating Module File
_generate_module_py(collaborative_fusion
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(collaborative_fusion_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(collaborative_fusion_generate_messages collaborative_fusion_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Termination.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/WithDraw.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameKeyPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/GlobalOptimization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomInfo.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/LoopClosureDetection.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/PointCloud.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/KeyPoint.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/RoomRegistration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FramePose.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/Frame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/RoomModel.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/MeshVisualization.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/srv/Registration.srv" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/UpdateFrame.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameLocalPoints.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/zheng/Repository/CollaborativeFusion/src/collaborative_fusion/msg/FrameDescriptor.msg" NAME_WE)
add_dependencies(collaborative_fusion_generate_messages_py _collaborative_fusion_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collaborative_fusion_genpy)
add_dependencies(collaborative_fusion_genpy collaborative_fusion_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collaborative_fusion_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collaborative_fusion
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(collaborative_fusion_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(collaborative_fusion_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(collaborative_fusion_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collaborative_fusion
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(collaborative_fusion_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(collaborative_fusion_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(collaborative_fusion_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collaborative_fusion
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(collaborative_fusion_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(collaborative_fusion_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(collaborative_fusion_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collaborative_fusion
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(collaborative_fusion_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(collaborative_fusion_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(collaborative_fusion_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collaborative_fusion
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(collaborative_fusion_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(collaborative_fusion_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(collaborative_fusion_generate_messages_py std_msgs_generate_messages_py)
endif()
