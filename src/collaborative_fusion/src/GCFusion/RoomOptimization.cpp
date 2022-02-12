#include "RoomOptimization.h"

static int room_minimum_3d_correspondence = 5;
typedef double SPARSE_MATRIX_NUM_TYPE;

float reprojection_error_3Dto3D_room_submap(std::vector<RoomCorrespondence> &room_clist,
    std::vector<FrameCorrespondence> &fCList, 
    PoseSE3dList &room_poses, PoseSE3dList &submap_poses_delta,
    const std::vector<int> &submap_to_room,
    const std::vector<int> &submap_to_position)
{
    float average_reprojection_error = 0;
    float count = 0;
    for (int i = 0; i < room_clist.size(); i++)
    {
        average_reprojection_error += reprojection_error_3Dto3D_room(room_clist[i],room_poses) * room_clist[i].sum_weight;
        count += room_clist[i].sum_weight;
    }    
    for (int i = 0; i < fCList.size(); i++)
    {
        average_reprojection_error += 
            reprojection_error_3Dto3D_room_submap(fCList[i],room_poses, submap_poses_delta,
            submap_to_room, submap_to_position) * fCList[i].sum_weight_c;
        count += fCList[i].sum_weight_c;
    }
    std::cout<<count<<" average_reprojection_error: "<<average_reprojection_error<<std::endl;
    return average_reprojection_error / count;
}

float reprojection_error_3Dto3D_room_submap(const FrameCorrespondence &fC, 
    const PoseSE3dList &room_poses,
    const PoseSE3dList & submap_poses_delta,
    const std::vector<int> &submap_to_room,
    const std::vector<int> &submap_to_position)
{
    PoseSE3d ref_pose, new_pose;
    int refID = fC.frame_ref.submapID;
    int newID = fC.frame_new.submapID;
    if(submap_to_room[refID] == -1)
    {
        ref_pose = submap_poses_delta[submap_to_position[refID] - room_poses.size()];
    }
    else ref_pose = room_poses[submap_to_room[refID]];

    if(submap_to_room[newID] == -1)
    {
        new_pose = submap_poses_delta[submap_to_position[newID] - room_poses.size()];
    }
    else new_pose = room_poses[submap_to_room[newID]];
    
    return reprojection_error_3Dto3D_camera(fC, new_pose.inverse()*ref_pose);

}
float reprojection_error_3Dto3D_room(const RoomCorrespondence &rC, const PoseSE3dList & room_poses)
{
    PoseSE3d relative_pose = room_poses[rC.target_id].inverse() * room_poses[rC.source_id];
    return reprojection_error_3Dto3D_room(rC, relative_pose);
}
float reprojection_error_3Dto3D_room(const RoomCorrespondence &rC, const Sophus::SE3d & relative_pose_from_ref_to_new)
{
    Eigen::MatrixXd R_ref = relative_pose_from_ref_to_new.rotationMatrix();
    Eigen::Vector3d t_ref = relative_pose_from_ref_to_new.translation();
    // pre-integration method for norm-2 distance
    float total_error = 0;

    if (rC.sum_weight > 0)
    {
      total_error = rC.sum_p_ref_ref(0,0) + rC.sum_p_ref_ref(1,1) + rC.sum_p_ref_ref(2,2) +
          rC.sum_p_new_new(0,0) + rC.sum_p_new_new(1,1) + rC.sum_p_new_new(2,2) +
          rC.sum_weight * t_ref.transpose() * t_ref
        - 2 * (float)(t_ref.transpose() * rC.sum_p_new) + 2 * (float)(t_ref.transpose() * R_ref * rC.sum_p_ref)
        - 2 * R_ref.cwiseProduct(rC.sum_p_new_ref).sum();

      if(total_error < 0)
      {
        std::cout << "total error: " << total_error << std::endl;

      }
      else
      {
        total_error = sqrt(total_error)  / rC.sum_weight;
      }
    }
    return total_error;    
}
void ComputeJacobianInfoRoom(RoomCorrespondence &fC,PoseSE3dList &room_poses,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj)
  {
    int valid_3d_cnt = fC.instance_center_correspondences.size();
    // construct the four matrix based on pre-integrated points
    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();
    if (valid_3d_cnt < room_minimum_3d_correspondence)
    {
      return;
    }
    //prepare data
    int refID = fC.source_id;
    int newID = fC.target_id;
    Eigen::Matrix3d R_ref = room_poses[refID].rotationMatrix();
    Eigen::Vector3d t_ref = room_poses[refID].translation();
    Eigen::Matrix3d R_new = room_poses[newID].rotationMatrix();
    Eigen::Vector3d t_new = room_poses[newID].translation();
    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri, rjWrj;
    riWrj = R_ref * fC.sum_p_ref_new * R_new.transpose();
    riWri = R_ref * fC.sum_p_ref_ref * R_ref.transpose();
    rjWrj = R_new * fC.sum_p_new_new * R_new.transpose();

    Eigen::Vector3d R_ref_sum_p_ref = R_ref * fC.sum_p_ref;
    Eigen::Vector3d R_new_sum_p_new = R_new * fC.sum_p_new;
    Eigen::Vector3d residual = R_ref_sum_p_ref + fC.sum_weight * (t_ref - t_new) - R_new_sum_p_new;
    //calculating JTr, see ProblemFormulation.pdf
    Pre_JiTr.block<3, 1>(0, 0) = residual;
    Pre_JiTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_ref_sum_p_ref.cross(t_ref - t_new) + t_ref.cross(residual);

    Pre_JjTr.block<3, 1>(0, 0) = residual;
    Pre_JjTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_new_sum_p_new.cross(t_ref - t_new) + t_new.cross(residual);
    Pre_JjTr = -Pre_JjTr;

    //calculating JTJ
    Pre_JiTJi.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight;
    Pre_JiTJi.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight * t_ref);
    Pre_JiTJi.block<3, 3>(3, 0) = -Pre_JiTJi.block<3, 3>(0, 3);
    Pre_JiTJi(3, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(3, 4) = -riWri(1, 0);					Pre_JiTJi(3, 5) = -riWri(2, 0);
    Pre_JiTJi(4, 3) = -riWri(0, 1);					Pre_JiTJi(4, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(4, 5) = -riWri(2, 1);
    Pre_JiTJi(5, 3) = -riWri(0, 2);					Pre_JiTJi(5, 4) = -riWri(1, 2);					Pre_JiTJi(5, 5) = riWri(0, 0) + riWri(1, 1);
    Pre_JiTJi.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_ref_sum_p_ref) - skewMatrixProduct(R_ref_sum_p_ref, t_ref) - fC.sum_weight * 1 * skewMatrixProduct(t_ref, t_ref);

    Pre_JjTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight;
    Pre_JjTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight * t_new);
    Pre_JjTJj.block<3, 3>(3, 0) = -Pre_JjTJj.block<3, 3>(0, 3);
    Pre_JjTJj(3, 3) = rjWrj(2, 2) + rjWrj(1, 1);	Pre_JjTJj(3, 4) = -rjWrj(1, 0);					Pre_JjTJj(3, 5) = -rjWrj(2, 0);
    Pre_JjTJj(4, 3) = -rjWrj(0, 1);					Pre_JjTJj(4, 4) = rjWrj(0, 0) + rjWrj(2, 2);	Pre_JjTJj(4, 5) = -rjWrj(2, 1);
    Pre_JjTJj(5, 3) = -rjWrj(0, 2);					Pre_JjTJj(5, 4) = -rjWrj(1, 2);					Pre_JjTJj(5, 5) = rjWrj(0, 0) + rjWrj(1, 1);
    Pre_JjTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_new, R_new_sum_p_new) - skewMatrixProduct(R_new_sum_p_new, t_new) - fC.sum_weight * 1 * skewMatrixProduct(t_new, t_new);


    Pre_JiTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight;
    Pre_JiTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight * t_new);
    Pre_JiTJj.block<3, 3>(3, 0) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight * t_ref).transpose();
    Pre_JiTJj(3, 3) = riWrj(2, 2) + riWrj(1, 1);	Pre_JiTJj(3, 4) = -riWrj(1, 0);	Pre_JiTJj(3, 5) = -riWrj(2, 0);
    Pre_JiTJj(4, 3) = -riWrj(0, 1);	Pre_JiTJj(4, 4) = riWrj(0, 0) + riWrj(2, 2);	Pre_JiTJj(4, 5) = -riWrj(2, 1);
    Pre_JiTJj(5, 3) = -riWrj(0, 2);	Pre_JiTJj(5, 4) = -riWrj(1, 2);		Pre_JiTJj(5, 5) = riWrj(0, 0) + riWrj(1, 1);
    Pre_JiTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_new_sum_p_new) - skewMatrixProduct(R_ref_sum_p_ref, t_new) - fC.sum_weight * 1 * skewMatrixProduct(t_ref, t_new);
    Pre_JiTJj = -Pre_JiTJj;
    Pre_JjTJi = Pre_JiTJj.transpose();

  #if 0
    std::cout << "precomputing jacobian matrics: " << fC.source_id << " " << fC.target_id << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;
    std::cout << "JjTr:" << Pre_JjTr.transpose() << std::endl;
    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    std::cout << "JiTJj: " << std::endl << Pre_JiTJj << std::endl;
    std::cout << "JjTJi: " << std::endl << Pre_JjTJi << std::endl;
    std::cout << "JjTJj: " << std::endl << Pre_JjTJj << std::endl;
  #endif
  }
void ComputeJacobianInfoRoomSubmap(FrameCorrespondence &fC,
    PoseSE3dList &room_poses,
    PoseSE3dList &submap_poses_delta,
    const std::vector<int> submap_to_room,
    const std::vector<int> submap_to_position,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj)
  {
    int valid_3d_cnt = fC.sparse_feature_cnt +  fC.dense_feature_cnt;
    // construct the four matrix based on pre-integrated points
    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();
    if (valid_3d_cnt < minimum_3d_correspondence)
    {
      return;
    }
    //prepare data
    int refID = fC.frame_ref.submapID;
    int newID = fC.frame_new.submapID;

    Eigen::Matrix3d R_ref = room_poses[refID].rotationMatrix();
    Eigen::Vector3d t_ref = room_poses[refID].translation();
    Eigen::Matrix3d R_new = room_poses[newID].rotationMatrix();
    Eigen::Vector3d t_new = room_poses[newID].translation();

    if(submap_to_room[refID] != -1)
    {
        //use room pose
        int room_id = submap_to_room[refID];
        R_ref = room_poses[room_id].rotationMatrix();
        t_ref = room_poses[room_id].translation();
    }
    else
    {
        int submap_position = submap_to_position[refID] - room_poses.size();
        R_ref = submap_poses_delta[submap_position].rotationMatrix();
        t_ref = submap_poses_delta[submap_position].translation();
        //use submap pose delta
    }
    if(submap_to_room[newID] != -1)
    {
        //use room pose
        int room_id = submap_to_room[newID];
        R_new = room_poses[room_id].rotationMatrix();
        t_new = room_poses[room_id].translation();
    }
    else
    {
        //use submap pose delta
        int submap_position = submap_to_position[newID] - room_poses.size();
        R_new = submap_poses_delta[submap_position].rotationMatrix();
        t_new = submap_poses_delta[submap_position].translation();
    }

    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri, rjWrj;
    riWrj = R_ref * fC.sum_p_ref_new_c * R_new.transpose();
    riWri = R_ref * fC.sum_p_ref_ref_c * R_ref.transpose();
    rjWrj = R_new * fC.sum_p_new_new_c * R_new.transpose();

    Eigen::Vector3d R_ref_sum_p_ref = R_ref * fC.sum_p_ref_c;
    Eigen::Vector3d R_new_sum_p_new = R_new * fC.sum_p_new_c;
    Eigen::Vector3d residual = R_ref_sum_p_ref + fC.sum_weight_c * (t_ref - t_new) - R_new_sum_p_new;
    //calculating JTr, see ProblemFormulation.pdf
    Pre_JiTr.block<3, 1>(0, 0) = residual;
    Pre_JiTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_ref_sum_p_ref.cross(t_ref - t_new) + t_ref.cross(residual);

    Pre_JjTr.block<3, 1>(0, 0) = residual;
    Pre_JjTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1))
      + R_new_sum_p_new.cross(t_ref - t_new) + t_new.cross(residual);
    Pre_JjTr = -Pre_JjTr;

    //calculating JTJ
    Pre_JiTJi.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_c;
    Pre_JiTJi.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight_c * t_ref);
    Pre_JiTJi.block<3, 3>(3, 0) = -Pre_JiTJi.block<3, 3>(0, 3);
    Pre_JiTJi(3, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(3, 4) = -riWri(1, 0);					Pre_JiTJi(3, 5) = -riWri(2, 0);
    Pre_JiTJi(4, 3) = -riWri(0, 1);					Pre_JiTJi(4, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(4, 5) = -riWri(2, 1);
    Pre_JiTJi(5, 3) = -riWri(0, 2);					Pre_JiTJi(5, 4) = -riWri(1, 2);					Pre_JiTJi(5, 5) = riWri(0, 0) + riWri(1, 1);
    Pre_JiTJi.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_ref_sum_p_ref) - skewMatrixProduct(R_ref_sum_p_ref, t_ref) - fC.sum_weight_c * 1 * skewMatrixProduct(t_ref, t_ref);

    Pre_JjTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_c;
    Pre_JjTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight_c * t_new);
    Pre_JjTJj.block<3, 3>(3, 0) = -Pre_JjTJj.block<3, 3>(0, 3);
    Pre_JjTJj(3, 3) = rjWrj(2, 2) + rjWrj(1, 1);	Pre_JjTJj(3, 4) = -rjWrj(1, 0);					Pre_JjTJj(3, 5) = -rjWrj(2, 0);
    Pre_JjTJj(4, 3) = -rjWrj(0, 1);					Pre_JjTJj(4, 4) = rjWrj(0, 0) + rjWrj(2, 2);	Pre_JjTJj(4, 5) = -rjWrj(2, 1);
    Pre_JjTJj(5, 3) = -rjWrj(0, 2);					Pre_JjTJj(5, 4) = -rjWrj(1, 2);					Pre_JjTJj(5, 5) = rjWrj(0, 0) + rjWrj(1, 1);
    Pre_JjTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_new, R_new_sum_p_new) - skewMatrixProduct(R_new_sum_p_new, t_new) - fC.sum_weight_c * 1 * skewMatrixProduct(t_new, t_new);


    Pre_JiTJj.block<3, 3>(0, 0) = Eye3x3 *fC.sum_weight_c;
    Pre_JiTJj.block<3, 3>(0, 3) = -getSkewSymmetricMatrix(R_new_sum_p_new + fC.sum_weight_c * t_new);
    Pre_JiTJj.block<3, 3>(3, 0) = -getSkewSymmetricMatrix(R_ref_sum_p_ref + fC.sum_weight_c * t_ref).transpose();
    Pre_JiTJj(3, 3) = riWrj(2, 2) + riWrj(1, 1);	Pre_JiTJj(3, 4) = -riWrj(1, 0);	Pre_JiTJj(3, 5) = -riWrj(2, 0);
    Pre_JiTJj(4, 3) = -riWrj(0, 1);	Pre_JiTJj(4, 4) = riWrj(0, 0) + riWrj(2, 2);	Pre_JiTJj(4, 5) = -riWrj(2, 1);
    Pre_JiTJj(5, 3) = -riWrj(0, 2);	Pre_JiTJj(5, 4) = -riWrj(1, 2);		Pre_JiTJj(5, 5) = riWrj(0, 0) + riWrj(1, 1);
    Pre_JiTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_new_sum_p_new) - skewMatrixProduct(R_ref_sum_p_ref, t_new) - fC.sum_weight_c * 1 * skewMatrixProduct(t_ref, t_new);
    Pre_JiTJj = -Pre_JiTJj;
    Pre_JjTJi = Pre_JiTJj.transpose();

#if 0
    //std::cout << "precomputing jacobian matrics: " << fC.source_id << " " << fC.target_id << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;
    std::cout << "JjTr:" << Pre_JjTr.transpose() << std::endl;
    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    std::cout << "JiTJj: " << std::endl << Pre_JiTJj << std::endl;
    std::cout << "JjTJi: " << std::endl << Pre_JjTJi << std::endl;
    std::cout << "JjTJj: " << std::endl << Pre_JjTJj << std::endl;
#endif
  }
void CombiningOptimization(std::vector<RoomCorrespondence > &room_correspondences, 
    std::vector<FrameCorrespondence> &submap_correspondences, PoseSE3dList & submapPosesRelativeChanges, 
    PoseSE3dList &submapPosesFinal, const std::vector<std::vector<int>> &room_to_submap, const std::vector<int> &submap_to_room)
{
    //firstly, extract the submap_correspondece we need, which means we need to discard
    //the correspondences that are in the same room.
    PoseSE3dList room_poses(room_to_submap.size(), PoseSE3d());
    std::vector<FrameCorrespondence> submap_corr_candidate;
    for(int i = 0; i != submap_correspondences.size(); ++i)
    {
        int room_id_ref = submap_to_room[submap_correspondences[i].frame_ref.submapID];
        int room_id_new = submap_to_room[submap_correspondences[i].frame_new.submapID];
        std::cout<<"room id: "<<room_id_ref<<" "<<room_id_new<<std::endl;
        if(room_id_ref == -1 || room_id_new == -1 || room_id_ref != room_id_new)
        {
            submap_correspondences[i].preIntegrateInterCamera();
            submap_corr_candidate.push_back(submap_correspondences[i]);
        }
    }
    int optNum = room_poses.size();
    size_t equationNum = room_correspondences.size() + submap_corr_candidate.size();

    //Look table from Jaccobian to the submap variables.
    std::vector<int> submap_to_position(submap_to_room.size(), -1);
    
    std::vector<int> position_to_submap(room_poses.size(), -1);
    for(int i = 0; i != submap_to_room.size(); ++i)
    {
        if(submap_to_room[i] == -1)
        {
            submap_to_position[i] = optNum;
            position_to_submap.push_back(i);
            optNum += 1;
        }
    }
    if(optNum < 3) 
    {
        std:cout<<"optNum is less than 3, no need to optimize."<<std::endl;
        return;
    }
    // also use a submap poses list to restore the relative changes, and so the form can be consistent to the room poses.
    // which means we consider each submap(not clusttered into any room) as a individual room
    PoseSE3dList submap_poses_delta(position_to_submap.size() - room_poses.size(), PoseSE3d());
    std::cout<<"variables wait to be optimized: "<<optNum<<std::endl;

    Eigen::MatrixXd J, err;
    Eigen::MatrixXd delta(6 * (optNum-1), 1), JTe(6 * (optNum-1), 1);
    Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> JTJ(6 * (optNum-1), 6 * (optNum-1));


    // the solver is only built at the first iteration
    Eigen::SimplicialLDLT	<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> > SimplicialLDLTSolver;
    std::vector<Eigen::Triplet<SPARSE_MATRIX_NUM_TYPE>> coeff;
    coeff.reserve(6 * 6 * 4 * equationNum);
    Eigen::MatrixXd JiTJi_pre(6, 6), JiTJj_pre(6, 6), JjTJi_pre(6, 6), JjTJj_pre(6, 6), JiTe_pre(6, 1), JjTe_pre(6, 1);

    clock_t start_opt, end_opt, start, end;
    double time_opt;
    start_opt = clock();

    float init_error =  reprojection_error_3Dto3D_room_submap(room_correspondences, submap_corr_candidate,room_poses, 
        submap_poses_delta, submap_to_room, submap_to_position);
    float final_error=1.0;;
    for(int i = 0; i != room_poses.size(); ++i)
    {
        std::cout<<room_poses[i].matrix()<<std::endl<<std::endl;
    }
    for (int iter = 0; iter<3||iter < 10 && final_error > 0.0003; iter++)
    {
      JTe.setZero();
      JTJ.setZero();
      err.setZero();
      coeff.clear();

      double time_framePair;
      double time_generatingJacobian;
      double time_buildSolver;
      start = clock();
      float robust_weight;

        for(int i = 0; i < room_correspondences.size(); ++i)
        {
            //Do something similiar to the follows
            int room_ref_pos = room_correspondences[i].source_id;
            int room_new_pos = room_correspondences[i].target_id;
            if(room_ref_pos == room_new_pos) continue;
            if(room_ref_pos == -1 || room_new_pos == -1)
            {
                std::cout<<"Something wrong about the room correspondences."<<std::endl;
                continue;
            }

            robust_weight = 1;
            ComputeJacobianInfoRoom(room_correspondences[i],
                room_poses,
                JiTe_pre,
                JjTe_pre,
                JiTJi_pre,
                JiTJj_pre,
                JjTJi_pre,
                JjTJj_pre);

            JiTe_pre *= robust_weight;
            JjTe_pre *= robust_weight;
            JiTJi_pre *= robust_weight;
            JiTJj_pre *= robust_weight;
            JjTJj_pre *= robust_weight;
            JjTJi_pre *= robust_weight;
            
            if(room_ref_pos < room_new_pos)
            {
                if (room_ref_pos == 0)
                {
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos - 1) * 6);
                JTe.block<6, 1>((room_new_pos - 1) * 6, 0) += JjTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (room_ref_pos - 1) * 6, (room_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (room_new_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((room_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            else
            {
                if (room_new_pos == 0)
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos - 1) * 6, (room_ref_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos - 1) * 6, 0) += JiTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (room_ref_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (room_ref_pos - 1) * 6, (room_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (room_new_pos - 1) * 6, (room_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (room_new_pos - 1) * 6, (room_new_pos - 1) * 6);
                JTe.block<6, 1>((room_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((room_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            
        }

        for (int i = 0; i < submap_corr_candidate.size(); i++)
        {
            Frame &frame_ref = submap_corr_candidate[i].frame_ref;
            Frame &frame_new = submap_corr_candidate[i].frame_new;
            int frame_ref_pos = submap_to_position[frame_ref.submapID];
            int frame_new_pos = submap_to_position[frame_new.submapID];
            if(frame_ref_pos == -1)
            {
                frame_ref_pos = submap_to_room[frame_ref.submapID];
            }

            if(frame_new_pos == -1)
            {
                frame_new_pos = submap_to_room[frame_new.submapID];
            }
            if (frame_ref_pos < 0 || frame_new_pos < 0)
            {
                continue;
            }
            //fCList[keyframe_candidate_fcorrs[i]].preIntegrateWithHuberNorm();


            robust_weight = 1.0f;


            clock_t start_jacobian, end_jacobian;
            start_jacobian = clock();
            ComputeJacobianInfoRoomSubmap(submap_corr_candidate[i],
                room_poses, 
                submap_poses_delta,
                submap_to_room,
                submap_to_position,
                JiTe_pre,
                JjTe_pre,
                JiTJi_pre,
                JiTJj_pre,
                JjTJi_pre,
                JjTJj_pre);

            JiTe_pre *= robust_weight;
            JjTe_pre *= robust_weight;
            JiTJi_pre *= robust_weight;
            JiTJj_pre *= robust_weight;
            JjTJj_pre *= robust_weight;
            JjTJi_pre *= robust_weight;

            if(frame_ref_pos < frame_new_pos)
            {
                if (frame_ref_pos == 0)
                {
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (frame_ref_pos - 1) * 6, (frame_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (frame_new_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
            else
            {
                if (frame_new_pos == 0)
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                }
                else
                {
                addBlockToTriplets(coeff, JiTJi_pre, (frame_ref_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JiTJj_pre, (frame_ref_pos - 1) * 6, (frame_new_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJi_pre, (frame_new_pos - 1) * 6, (frame_ref_pos - 1) * 6);
                addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
                JTe.block<6, 1>((frame_ref_pos - 1) * 6, 0) += JiTe_pre;
                JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;
                }
            }
        }

            

            end = clock();
            time_framePair = (double)(end - start) / CLOCKS_PER_SEC * 1000;
            start = clock();
            JTJ.setFromTriplets(coeff.begin(), coeff.end());
            end = clock();
            time_generatingJacobian = (double)(end - start) / CLOCKS_PER_SEC * 1000;
            start = clock();
            SimplicialLDLTSolver.compute(JTJ);
            end = clock();
            time_buildSolver = (double)(end - start) / CLOCKS_PER_SEC * 1000;


            // update the pose of each frame
            start = clock();
            delta = SimplicialLDLTSolver.solve(JTe);
            end = clock();
            double time_svd = (double)(end - start) / CLOCKS_PER_SEC * 1000;
            for (int i = 1; i < room_poses.size(); i++)
            {
                Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (i - 1), 0);
                if(isnan(delta_i(0)))
                {
                std::cout << "nan detected in pose update! " << std::endl;
                continue;
                }
                std::cout<<i<<" r "<<delta_i<<std::endl<<std::endl;
                /* refine all the keyframes' poses in this submap*/
                //if(submapID == 0) continue;

                room_poses[i] =  Sophus::SE3d::exp(delta_i).inverse() 
                * room_poses[i];

            }
            for(int i = 0; i != submap_poses_delta.size(); ++i)
            {
                int position = room_poses.size() + i;
                //int submap_id = position_to_submap[position];
                Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (position - 1), 0);
                if(isnan(delta_i(0)))
                {
                std::cout << "nan detected in pose update! " << std::endl;
                continue;
                }
                std::cout<<i<<" s "<<delta_i<<std::endl<<std::endl;
                /* refine all the keyframes' poses in this submap*/
                //if(submapID == 0) continue;

                submap_poses_delta[i] =  Sophus::SE3d::exp(delta_i).inverse() 
                * submap_poses_delta[i];
                // this is used for update all the submap poses
                
            }
            double time_calculate_reprojection_error;
            double refined_error;
            
            final_error = reprojection_error_3Dto3D_room_submap(room_correspondences, submap_corr_candidate,room_poses, 
        submap_poses_delta, submap_to_room, submap_to_position);
     }
    std::cout<<"after: "<<std::endl;
    for(int i = 0; i != room_poses.size(); ++i)
    {
        std::cout<<room_poses[i].matrix()<<std::endl<<std::endl;
    }
    // this is used for update all the submap poses
    for(int i = 0; i != room_to_submap.size(); ++i)
    {
        for(auto j = room_to_submap[i].begin();
            j!=room_to_submap[i].end(); ++j)
        {
            submapPosesRelativeChanges[*j] = 
                room_poses[i] * submapPosesRelativeChanges[*j];
            submapPosesFinal[*j] = 
                room_poses[i] * submapPosesFinal[*j];

        }
    }

    for(int i = 0; i != submap_poses_delta.size(); ++i)
    {
        //remember to switch position with submap_id 
        int submap_id = position_to_submap[room_poses.size() + i];
        if(submap_id < 0)
        {
            std::cout<<"something wrong with the table position_to_submap."<<std::endl;
            continue;
        }
        submapPosesRelativeChanges[submap_id] = 
            submap_poses_delta[i] * submapPosesRelativeChanges[submap_id];
        submapPosesFinal[submap_id] = 
            submap_poses_delta[i] * submapPosesFinal[submap_id];
    }
    //update all the keyframe poses
    end_opt = clock();

    time_opt = (double )(end_opt - start_opt) / CLOCKS_PER_SEC * 1000;

    std::cout << "init/final error " << init_error << "/" << final_error << std::endl;
}