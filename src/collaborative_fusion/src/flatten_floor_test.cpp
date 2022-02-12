/*
This program is used to test the normal optimization, to flatten the ground of the model reconstructed by CollaborativeFusion.
We treat normal as a point, and use 
|Ri ni - Rj nj| as the cost function. An alternative is |(Ri ni)^T Rj nj - 1 |
*/


#include <open_chisel/io/PLY.h>
#include <open_chisel/io/LabelColor.h>
#include <open_chisel/ForLabel.h>
#include <iostream>
#include "./GCSLAM/MultiViewGeometry.h"
#include "./Tools/TickTock.h"
using namespace std;
using namespace MultiViewGeometry;
using namespace Eigen;
typedef double SPARSE_MATRIX_NUM_TYPE;
class SubmapCorrespondence
{
    public:

    //in fact, we assume that each correspondence only have a pair of normal.
    Eigen::Vector3d sum_p_ref_feature_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_feature_s;		// sum(p_new)
    Eigen::Vector3d sum_p_ref_icp_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_icp_s;		// sum(p_new)

    Eigen::Vector3d sum_p_ref_s;		// sum(p_ref)
    Eigen::Vector3d sum_p_new_s;		// sum(p_new)
    Eigen::Matrix3d sum_p_ref_new_s;  // sum(p_ref * p_new^T)
    Eigen::Matrix3d sum_p_ref_ref_s;  // sum(p_ref * p_ref^T)
    Eigen::Matrix3d sum_p_new_new_s;  // sum(p_new * p_new^T)
    Eigen::Matrix3d sum_p_new_ref_s;  // sum(p_new * p_ref^T)
    // to calculate reprojection error
    float sum_feature_p_ref_T_p_ref_s, sum_feature_p_new_T_p_new_s;
    float sum_icp_p_ref_T_p_ref_s, sum_icp_p_new_T_p_new_s;
    double sum_weight_s;    
    SubmapCorrespondence(int ref_id, int new_id, const Eigen::Vector3f & _ref_normal, const Eigen::Vector3f & _new_normal )
    {
        ref_submap_id = ref_id;
        new_submap_id = new_id;
        ref_normal = _ref_normal.cast<double>();
        new_normal = _new_normal.cast<double>();
    }
    void preIntegrateInterSubmap()
    {
        sum_p_ref_s.setZero();
        sum_p_new_s.setZero();
        sum_p_ref_new_s.setZero();
        sum_p_ref_ref_s.setZero();
        sum_p_new_new_s.setZero();
        sum_p_new_ref_s.setZero();
        sum_weight_s = 0;
        double location_weight = 1;
        int use_weight = 0;
        /*std::cout<<"R_ref:"<<R_ref<<std::endl;
        std::cout<<"t_ref:"<<t_ref<<std::endl;
        std::cout<<"R_new:"<<R_new<<std::endl;
        std::cout<<"t_new:"<<t_new<<std::endl;*/
        sum_feature_p_ref_T_p_ref_s = 0;
        sum_feature_p_new_T_p_new_s = 0;
        sum_icp_p_ref_T_p_ref_s = 0;
        sum_icp_p_new_T_p_new_s = 0;


        float depth = ref_normal(2);

        double weight = 1 / (depth * depth);
        sum_weight_s += weight;
        sum_p_ref_s += ref_normal * weight;
        sum_p_new_s += new_normal * weight;
        sum_p_ref_new_s += ref_normal * new_normal.transpose() * weight;
        sum_p_ref_ref_s += ref_normal * ref_normal.transpose() * weight;
        sum_p_new_new_s += new_normal * new_normal.transpose() * weight;
        sum_p_new_ref_s += new_normal * ref_normal.transpose() * weight;

        sum_feature_p_ref_T_p_ref_s = ref_normal.transpose() * ref_normal;
        sum_feature_p_new_T_p_new_s = new_normal.transpose() * new_normal;

#if 0
        if (rtst_error > 0)
        {
            sum_weight /= rtst_error;
            sum_p_ref /= rtst_error;
            sum_p_new /= rtst_error;
            sum_p_ref_new /= rtst_error;
            sum_p_ref_ref /= rtst_error;
            sum_p_new_new /= rtst_error;
            sum_p_new_ref /= rtst_error;

        }
#endif
        sum_p_ref_feature_s = sum_p_ref_s ;
        sum_p_new_feature_s = sum_p_new_s ;
    }
    float normal_error(const PoseSE3dList &submapPoses)
    {
        Eigen::Matrix3d R_ref = submapPoses[ref_submap_id].rotationMatrix();
        Eigen::Vector3d t_ref = submapPoses[ref_submap_id].translation();
        Eigen::Matrix3d R_new = submapPoses[new_submap_id].rotationMatrix();
        Eigen::Vector3d t_new = submapPoses[new_submap_id].translation();
        return (R_ref *ref_normal+ t_ref - R_new * new_normal - t_ref).norm();
    }
    Eigen::Vector3d ref_normal;
    Eigen::Vector3d new_normal;
    int ref_submap_id;
    int new_submap_id;
};


float normal_error(std::vector<SubmapCorrespondence> &correspondences, PoseSE3dList &submapPoses)
{
    float error = 0;
    for(int i = 0; i != correspondences.size(); ++i)
    {
        error += correspondences[i].normal_error(submapPoses);
    }
    return error;
}
void ComputeJacobianInfoSubmapNormal(SubmapCorrespondence &SC,PoseSE3dList &submapPoses,
    Eigen::MatrixXd &Pre_JiTr,
    Eigen::MatrixXd &Pre_JjTr,
    Eigen::MatrixXd &Pre_JiTJi,
    Eigen::MatrixXd &Pre_JiTJj,
    Eigen::MatrixXd &Pre_JjTJi,
    Eigen::MatrixXd &Pre_JjTJj)
{

    // construct the four matrix based on pre-integrated points
    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();

    //prepare data
    int refID = SC.ref_submap_id;
    int newID = SC.new_submap_id;
    Eigen::Matrix3d R_ref = submapPoses[refID].rotationMatrix();
    Eigen::Vector3d t_ref = submapPoses[refID].translation();
    Eigen::Matrix3d R_new = submapPoses[newID].rotationMatrix();
    Eigen::Vector3d t_new = submapPoses[newID].translation();
    Eigen::Matrix3d Eye3x3;

    Eye3x3.setIdentity();
    Eigen::Matrix3d riWrj, riWri, rjWrj;
    riWrj = R_ref * SC.sum_p_ref_new_s * R_new.transpose();
    riWri = R_ref * SC.sum_p_ref_ref_s * R_ref.transpose();
    rjWrj = R_new * SC.sum_p_new_new_s * R_new.transpose();

    Eigen::Vector3d R_ref_sum_p_ref = R_ref * SC.sum_p_ref_s;
    Eigen::Vector3d R_new_sum_p_new = R_new * SC.sum_p_new_s;

    //Here we only care about the rotation
    //calculating JTr, see ProblemFormulation.pdf
    //Pre_JiTr.block<3, 1>(0, 0) = residual;


    Pre_JiTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1));

    Pre_JjTr.block<3, 1>(3, 0) = Eigen::Vector3d(riWrj(2, 1) - riWrj(1, 2), -riWrj(2, 0) + riWrj(0, 2), riWrj(1, 0) - riWrj(0, 1));
    Pre_JjTr = -Pre_JjTr;

    //calculating JTJ
    //Pre_JiTJi.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JiTJi(3, 3) = riWri(2, 2) + riWri(1, 1);	Pre_JiTJi(3, 4) = -riWri(1, 0);					Pre_JiTJi(3, 5) = -riWri(2, 0);
    Pre_JiTJi(4, 3) = -riWri(0, 1);					Pre_JiTJi(4, 4) = riWri(0, 0) + riWri(2, 2);	Pre_JiTJi(4, 5) = -riWri(2, 1);
    Pre_JiTJi(5, 3) = -riWri(0, 2);					Pre_JiTJi(5, 4) = -riWri(1, 2);					Pre_JiTJi(5, 5) = riWri(0, 0) + riWri(1, 1);
    //Pre_JiTJi.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_ref_sum_p_ref) - skewMatrixProduct(R_ref_sum_p_ref, t_ref) - fC.sum_weight_s * 1 * skewMatrixProduct(t_ref, t_ref);
    //Pre_JjTJj.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JjTJj(3, 3) = rjWrj(2, 2) + rjWrj(1, 1);	Pre_JjTJj(3, 4) = -rjWrj(1, 0);					Pre_JjTJj(3, 5) = -rjWrj(2, 0);
    Pre_JjTJj(4, 3) = -rjWrj(0, 1);					Pre_JjTJj(4, 4) = rjWrj(0, 0) + rjWrj(2, 2);	Pre_JjTJj(4, 5) = -rjWrj(2, 1);
    Pre_JjTJj(5, 3) = -rjWrj(0, 2);					Pre_JjTJj(5, 4) = -rjWrj(1, 2);					Pre_JjTJj(5, 5) = rjWrj(0, 0) + rjWrj(1, 1);
    //Pre_JjTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_new, R_new_sum_p_new) - skewMatrixProduct(R_new_sum_p_new, t_new) - fC.sum_weight_s * 1 * skewMatrixProduct(t_new, t_new);

    //Pre_JiTJj.block<3, 3>(0, 0) = Eye3x3 *SC.sum_weight_s;
    Pre_JiTJj(3, 3) = riWrj(2, 2) + riWrj(1, 1);	Pre_JiTJj(3, 4) = -riWrj(1, 0);	Pre_JiTJj(3, 5) = -riWrj(2, 0);
    Pre_JiTJj(4, 3) = -riWrj(0, 1);	Pre_JiTJj(4, 4) = riWrj(0, 0) + riWrj(2, 2);	Pre_JiTJj(4, 5) = -riWrj(2, 1);
    Pre_JiTJj(5, 3) = -riWrj(0, 2);	Pre_JiTJj(5, 4) = -riWrj(1, 2);		Pre_JiTJj(5, 5) = riWrj(0, 0) + riWrj(1, 1);
    //Pre_JiTJj.block<3, 3>(3, 3) += -skewMatrixProduct(t_ref, R_new_sum_p_new) - skewMatrixProduct(R_ref_sum_p_ref, t_new) - fC.sum_weight_s * 1 * skewMatrixProduct(t_ref, t_new);
    Pre_JiTJj = -Pre_JiTJj;
    Pre_JjTJi = Pre_JiTJj.transpose();

    // using naive way:
    /*
    Eigen::Vector3d residual = R_ref * SC.ref_normal - R_new * SC.new_normal;

        
    //Pre_JiTr.block<3, 1>(3, 0) = -MultiViewGeometry::getSkewSymmetricMatrix( R_ref * SC.ref_normal).transpose() * residual;

    //Pre_JjTr.block<3, 1>(0, 0) = residual;
    Pre_JjTr.block<3, 1>(3, 0) = MultiViewGeometry::getSkewSymmetricMatrix( R_new * SC.new_normal).transpose() * residual;

    Pre_JjTJj.block<3,3>(3, 3) = MultiViewGeometry::getSkewSymmetricMatrix( R_new * SC.new_normal).transpose() 
        * MultiViewGeometry::getSkewSymmetricMatrix( R_new * SC.new_normal).transpose();
    */
    
    #if 1
    std::cout << "precomputing jacobian matrics: " << SC.ref_submap_id << " " << SC.new_submap_id << std::endl;
    std::cout << "JiTr:" << Pre_JiTr.transpose() << std::endl;
    std::cout << "JjTr:" << Pre_JjTr.transpose() << std::endl;
    std::cout << "JiTJi: " << std::endl << Pre_JiTJi << std::endl;
    std::cout << "JiTJj: " << std::endl << Pre_JiTJj << std::endl;
    std::cout << "JjTJi: " << std::endl << Pre_JjTJi << std::endl;
    std::cout << "JjTJj: " << std::endl << Pre_JjTJj << std::endl;
    #endif
}
  float optimizeSubmapRobust(std::vector<SubmapCorrespondence> &fCList,PoseSE3dList & submapPoses)
  {

    int optNum = submapPoses.size()- 1;
    Eigen::MatrixXd J, err;
    Eigen::MatrixXd delta(6 * optNum, 1), JTe(6 * optNum, 1);
    Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> JTJ(6 * optNum, 6 * optNum);

    int valid_observation_cnt = 0;
    double prev_err = 10000;

    clock_t start, end;

    Eigen::SimplicialLDLT	<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE> > SimplicialLDLTSolver;
    Eigen::SparseLU<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE>, COLAMDOrdering<int> > SparseLuSolver;
    Eigen::SparseQR<Eigen::SparseMatrix<SPARSE_MATRIX_NUM_TYPE>, COLAMDOrdering<int> > SparseQRSolver;
    std::vector<Eigen::Triplet<SPARSE_MATRIX_NUM_TYPE>> coeff;
    coeff.reserve(6 * 6 * 4 * fCList.size());
    Eigen::MatrixXd JiTJi_pre(6, 6), JiTJj_pre(6, 6), JjTJi_pre(6, 6), JjTJj_pre(6, 6), JiTe_pre(6, 1), JjTe_pre(6, 1);

    clock_t start_opt, end_opt;
    double time_opt;
    start_opt = clock();

    for (int iter = 0; iter<3; iter++)
    {
      JTe.setZero();
      JTJ.setZero();
      err.setZero();
      coeff.clear();
      valid_observation_cnt = 0;

      double time_framePair;
      double time_generatingJacobian;
      double time_buildSolver;
      start = clock();
      float robust_weight;

      Eigen::Matrix3d JTJ_simple;
      Eigen::Vector3d _JTr_simple;

      for (int i = 0; i < fCList.size(); i++)
      {
        int frame_ref_pos = fCList[i].ref_submap_id;
        int frame_new_pos = fCList[i].new_submap_id;

        if (frame_ref_pos < 0 || frame_new_pos < 0)
        {
          continue;
        }
        robust_weight = 1.0f;

        clock_t start_jacobian, end_jacobian;
        start_jacobian = clock();
        ComputeJacobianInfoSubmapNormal(fCList[i], submapPoses,
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
            std::cout<<"RIGHT!"<<std::endl;
          addBlockToTriplets(coeff, JjTJj_pre, (frame_new_pos - 1) * 6, (frame_new_pos - 1) * 6);
          JTe.block<6, 1>((frame_new_pos - 1) * 6, 0) += JjTe_pre;

          JTJ_simple = JiTJi_pre.block<3,3>(3,3);
          _JTr_simple = JTe.block<3,1>(3,0);
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
      JTJ.setFromTriplets(coeff.begin(), coeff.end());
#if 0
      
      Eigen::JacobiSVD<MatrixXd> svd(JTJ, ComputeThinU | ComputeThinV);
      delta = svd.solve(JTe);
#elif 0
      SparseLuSolver.compute(JTJ);
      delta = SparseLuSolver.solve(JTe);
#else
      SparseQRSolver.compute(JTJ);
      delta = SparseQRSolver.solve(JTe);    
#endif
      std::cout<<"delta: "<<delta<<std::endl;
    
      double time_svd = (double)(end - start) / CLOCKS_PER_SEC * 1000;
      for (int i = 1; i < submapPoses.size(); i++)
      {
          Eigen::Vector6d delta_i = delta.block<6,1>(6*(i-1), 0);
          if(isnan(delta_i(0)))
          {
            std::cout << "nan detected in pose update! " << std::endl;
            continue;
          }
          std::cout<<i<<" "<<delta_i<<std::endl<<std::endl;
          int submapID = i;

          submapPoses[submapID] =  Sophus::SE3d::exp(delta_i).inverse() * submapPoses[submapID];
#if 0
          if(submapID == 10)
          {

          std::cout<<submapID<<std::endl;
          std::cout<<"delta: "<<Sophus::SE3d::exp(delta_i).inverse().log().transpose()<<std::endl;
          std::cout<<"Relative Change: "<<submapPosesRelativeChanges[submapID].log().transpose()<<std::endl;
          std::cout<<"SubmapPoses: "<<submapPoses[submapID].log().transpose()<<std::endl;
          std::cout<<"Relative Change * submapPoses: "<<(submapPosesRelativeChanges[submapID] * submapPoses[submapID]).log().transpose()<<std::endl;
          std::cout<<"Final submapPoses: "<<submapPosesFinal[submapID].log().transpose()<<std::endl;          
          }
#endif
      }
    }
    return prev_err;
  }
int main()
{
    
    int submap_count = 3;
    int start_point = 0;
    std::cin>>submap_count;
    std::cin>>start_point;
    std::vector<chisel::PcdPtr> submap_pcds;
    std::vector<Eigen::Vector3f> normals;
    Eigen::Vector3f floor, last_normal;
    floor<<0,0,1;
    last_normal = floor;
    std::vector<SubmapCorrespondence> correspondences;
    PoseSE3dList submapPoses(submap_count);
    for(int i = 0; i != submap_count; ++i)
    {
        //read fpcd
        std::string path = "./for_normal_optimization/" + std::to_string(i + start_point)+".fpcd";
        std::cout<<"load "<<path<<std::endl;
        submap_pcds.emplace_back(new chisel::PointCloud());
        chisel::ReadFPCDASCII(path, submap_pcds[i]);
        //compute submap normal
        std::cout<<"compute normals"<<path<<std::endl;
        Eigen::Vector3f normal = Eigen::Vector3f::Zero();
        auto &tmp_pcd = *submap_pcds[i];
        int floor_count = 0;
        for(int p = 0; p != tmp_pcd.labels.size(); ++p)
        {
            //std::cout<<p<<std::endl;
            if(get_semantic_label(tmp_pcd.labels[p]) == 1 && tmp_pcd.normals[p].transpose() * floor >= 0.7 )
            {
                normal += tmp_pcd.normals[p];
                floor_count += 1;
            }
        }
        //std::cout<<floor_count<<std::endl;
        if(floor_count == 0)
        {
          normal = last_normal;
        }
        else 
        {
        normal /= floor_count; 
        //std::cout<<"normal: "<<normal<<std::endl;
        normal.normalize();
        std::cout<<"normal: "<<normal<<std::endl;

        }
        last_normal = normal;
        normals.push_back(normal);                
        //construct submap correspondence
        if(i > 0)
        {
            correspondences.push_back(SubmapCorrespondence(i-1, i, normals[i-1], normals[i]));
            correspondences.back().preIntegrateInterSubmap();
        }
    }
    
    correspondences.push_back(SubmapCorrespondence(0, submap_count-1, normals[0], normals[submap_count-1] ));
    correspondences.back().preIntegrateInterSubmap();
    
    //use correspondence to optimize the rotation.

    float init_error = normal_error(correspondences, submapPoses);
    optimizeSubmapRobust(correspondences, submapPoses);
    float final_error = normal_error(correspondences, submapPoses);

    std::cout<<"init_error/final_error: "<<init_error<<"/"<<final_error<<std::endl;
    for(int i = 0; i!= submap_count; ++i)
    {
        //save each pointcloud.
        auto &tmp_pcd = *submap_pcds[i];
        Eigen::Matrix3f R = submapPoses[i].rotationMatrix().cast<float>();
        Eigen::Vector3f t = submapPoses[i].translation().cast<float>();
        //std::cout<<std::endl;
        for(int j = 0; j!= tmp_pcd.labels.size(); ++j )
        {
            tmp_pcd.normals[j] =  R * tmp_pcd.normals[j];
            tmp_pcd.vertices[j] = R * tmp_pcd.vertices[j];
        }
        std::string path = "./for_normal_optimization/ply/" + std::to_string(i + start_point)+".ply";
        chisel::SaveSemanticPointCloudPLYASCII(path, submap_pcds[i]);
    }
    return 1;
}