/*
This code is for rebuttal. We read the keyframe and submap information, as well as their correspondences from files. 
Then, we do keyframe-based optimization and submap-based optimization, and compare  their running time and pose accuracy.
*/
#include <string>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <sophus/se3.hpp>
#include <Eigen/Sparse>
#include "./Tools/TickTock.h"
#include <tuple>
typedef std::vector<Eigen::Vector3d , Eigen::aligned_allocator<Eigen::Vector3d> > Point3dList;
typedef Eigen::Vector3d Point3d;
typedef std::vector<Sophus::SE3d , Eigen::aligned_allocator<Eigen::Vector3d> > PoseSE3dList;
typedef Sophus::SE3d PoseSE3d;
tool::Timer timer;

struct SubmapInfo
{
    int submap_id;
    PoseSE3d pose;
};

struct KeyframeInfo
{
    int keyframe_id;
    int submap_id;
    PoseSE3d pose;
    PoseSE3d relative_pose;
};

struct Correspondence
{
    int ref_id;
    int new_id;
    Eigen::Vector3d sum_p_ref;		// sum(p_ref)
    Eigen::Vector3d sum_p_new;		// sum(p_new)
    Eigen::Matrix3d sum_p_ref_new;  // sum(p_ref * p_new^T)
    Eigen::Matrix3d sum_p_ref_ref;  // sum(p_ref * p_ref^T)
    Eigen::Matrix3d sum_p_new_new;  // sum(p_new * p_new^T)
    Eigen::Matrix3d sum_p_new_ref;  // sum(p_new * p_ref^T)   
    double sum_weight;
};


float reprojection_error_3Dto3D(const Correspondence &fC,  const Sophus::SE3d &relative_pose_from_ref_to_new)
{
    Eigen::MatrixXd R_ref = relative_pose_from_ref_to_new.rotationMatrix();
    Eigen::Vector3d t_ref = relative_pose_from_ref_to_new.translation();
    //std::cout<<(relative_pose_from_ref_to_new).log().transpose()<<std::endl;
    // pre-integration method for norm-2 distance
    float total_error = 0; 

    if (fC.sum_weight > 0)
    {
        total_error = fC.sum_p_ref_ref(0,0) + fC.sum_p_ref_ref(1,1) + fC.sum_p_ref_ref(2,2) +
            fC.sum_p_new_new(0,0) + fC.sum_p_new_new(1,1) + fC.sum_p_new_new(2,2) +
            fC.sum_weight * t_ref.transpose() * t_ref
        - 2 * (float)(t_ref.transpose() * fC.sum_p_new) + 2 * (float)(t_ref.transpose() * R_ref * fC.sum_p_ref)
        - 2 * R_ref.cwiseProduct(fC.sum_p_new_ref).sum();

        if(total_error < 0)
        {
        //std::cout << "total error: " << total_error << std::endl;
        // std::cout<<R_ref<<std::endl;
        }
        else
        {
        total_error = sqrt(total_error)  / fC.sum_weight;
        }
    }
    //std::cout << "KeyFrame Index Submap: "<<fC.frame_new.frame_index <<" "<<fC.frame_ref.frame_index <<" "<<total_error<< std::endl;
    return total_error;
}
float reprojection_error_3Dto3D(const Correspondence &fC, std::map<int, KeyframeInfo> &keyframes)
{
    
    return reprojection_error_3Dto3D(fC, keyframes[fC.new_id].pose.inverse() * keyframes[fC.ref_id].pose);
}

float reprojection_error_3Dto3D(std::vector<Correspondence> &fCList, std::map<int, KeyframeInfo> &keyframes)
{
    float average_reprojection_error = 0;
    float count = 0;
    for (int i = 0; i < fCList.size(); i++)
    {
        // auto &fC = fCList[i];
        // std::cout<<fC.ref_id<<" "<<fC.new_id<<std::endl;
        // std::cout<<fC.sum_p_ref.transpose()<<std::endl;
        // std::cout<<fC.sum_p_new.transpose()<<std::endl;
        // std::cout<<fC.sum_p_ref_new<<std::endl;
        // std::cout<<fC.sum_p_ref_ref<<std::endl;
        // std::cout<<fC.sum_p_new_new<<std::endl;
        // std::cout<<fC.sum_p_new_ref<<std::endl;    
        // std::cout<<fC.sum_weight<<std::endl;
        average_reprojection_error += reprojection_error_3Dto3D(fCList[i], keyframes) ;
        //std::cout<<std::endl;
        count += 1;
    }
    return average_reprojection_error / count;
}

float reprojection_error_3Dto3D(const Correspondence &fC, std::vector<SubmapInfo> &keyframes)
{
    return reprojection_error_3Dto3D(fC, keyframes[fC.new_id].pose.inverse() * keyframes[fC.ref_id].pose);
}

float reprojection_error_3Dto3D(std::vector<Correspondence> &fCList, std::vector<SubmapInfo> &keyframes)
{
    float average_reprojection_error = 0;
    float count = 0;
    for (int i = 0; i < fCList.size(); i++)
    {
        average_reprojection_error += reprojection_error_3Dto3D(fCList[i], keyframes) * fCList[i].sum_weight;
        count += fCList[i].sum_weight;
    }
    return average_reprojection_error / count;
}
void addBlockToTriplets(std::vector<Eigen::Triplet<double>> &coeff, Eigen::MatrixXd b,
int start_x,int start_y)
{
    int rows = b.rows();
    int cols = b.cols();
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            coeff.push_back(Eigen::Triplet<double>(start_x+i,start_y+j,b(i,j)));
        }
    }
}

Eigen::Matrix3d skewMatrixProduct(Eigen::Vector3d t1, Eigen::Vector3d t2)
{
    Eigen::Matrix3d M;
    M(0, 0) = -t1(1)*t2(1) - t1(2)*t2(2); M(0, 1) = t1(1)*t2(0); M(0, 2) = t1(2)*t2(0);
    M(1, 0) = t1(0)*t2(1);	 M(1, 1) = -t1(2)*t2(2) - t1(0)*t2(0); M(1, 2) = t1(2)*t2(1);
    M(2, 0) = t1(0)*t2(2);   M(2, 1) = t1(1)*t2(2); M(2, 2) = -t1(1)*t2(1) - t1(0)*t2(0);
    return M;
}

Eigen::Matrix3d getSkewSymmetricMatrix(Eigen::Vector3d t)
{
    Eigen::Matrix3d t_hat;
    t_hat << 0, -t(2), t(1),
        t(2), 0, -t(0),
        -t(1), t(0), 0;
    return t_hat;
}
size_t StdVectorToEigenMatrix(Eigen::Vector3d &mat, const std::vector<double> &buffer, size_t ptr)
{
    int rows = mat.rows();
    int cols = mat.cols();
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            mat(i,j) = buffer[ptr ++ ];
        }
    }
    return ptr;
}
size_t StdVectorToEigenMatrix(Eigen::Matrix3d &mat, const std::vector<double> &buffer, size_t ptr)
{
    int rows = mat.rows();
    int cols = mat.cols();
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            mat(i,j) = buffer[ptr ++ ];
        }
    }
    return ptr;
}
void read_submap_info(const std::string &filename, std::vector<SubmapInfo> &submaps)
{
    std::ifstream ifs(filename, std::ifstream::binary);

    std::vector<double> buffer;
    ifs.seekg (0, ifs.end);
    size_t length = ifs.tellg();
    ifs.seekg (0, ifs.beg);
    buffer.resize(length/sizeof(double));
    ifs.read((char *)&buffer[0],length);    
    
    int submap_num = 0, start = 0;
    submap_num = (int)buffer[0];
    size_t ptr = 1;
    while(start < submap_num)
    {
        SubmapInfo tmp_submap; 
        Eigen::Matrix4d pose_m = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        ifs >> tmp_submap.submap_id;
        tmp_submap.submap_id = (int)buffer[ptr++];
        
        ptr = StdVectorToEigenMatrix(R, buffer, ptr);
        ptr = StdVectorToEigenMatrix(t, buffer, ptr);

        pose_m.block<3, 3>(0, 0) = R;
        pose_m.block<3, 1>(0, 3) = t;
        tmp_submap.pose = PoseSE3d(pose_m);
        submaps.push_back(tmp_submap);
        start += 1;
    }
    ifs.close();
}

void read_keyframe_info(const std::string &filename, std::map<int, KeyframeInfo> &keyframes)
{
    std::ifstream ifs(filename, std::ifstream::binary);
    int keyframe_num = 0, start = 0;

    std::vector<double> buffer;
    ifs.seekg (0, ifs.end);
    size_t length = ifs.tellg();
    ifs.seekg (0, ifs.beg);
    buffer.resize(length/sizeof(double));
    ifs.read((char *)&buffer[0],length);    

    keyframe_num = (int)buffer[0];
    size_t ptr = 1;
    while(start < keyframe_num)
    {
        KeyframeInfo tmp_keyframe;
        Eigen::Matrix4d pose_m = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d R;
        Eigen::Vector3d t;
        tmp_keyframe.keyframe_id = (int) buffer[ptr++];
        tmp_keyframe.submap_id = (int) buffer[ptr++];

        ptr = StdVectorToEigenMatrix(R, buffer, ptr);
        ptr = StdVectorToEigenMatrix(t, buffer, ptr);

        pose_m.block<3, 3>(0, 0) = R;
        pose_m.block<3, 1>(0, 3) = t;
        tmp_keyframe.pose = PoseSE3d(pose_m);
        
        ptr = StdVectorToEigenMatrix(R, buffer, ptr);
        ptr = StdVectorToEigenMatrix(t, buffer, ptr);

        pose_m.block<3, 3>(0, 0) = R;
        pose_m.block<3, 1>(0, 3) = t;
        tmp_keyframe.relative_pose = PoseSE3d(pose_m);

        keyframes[tmp_keyframe.keyframe_id]= tmp_keyframe;
        start += 1;
    }
    ifs.close();
}

void read_correspondence(const std::string &filename, std::vector<Correspondence> &corrs)
{
    std::ifstream ifs(filename);
    int keyframe_corr_num = 0, start = 0;
    std::vector<double> buffer;
    ifs.seekg (0, ifs.end);
    size_t length = ifs.tellg();
    ifs.seekg (0, ifs.beg);
    buffer.resize(length/sizeof(double));
    ifs.read((char *)&buffer[0],length);  

    keyframe_corr_num = (int)buffer[0];
    size_t ptr = 1;

    while(start < keyframe_corr_num)
    {
        Correspondence corr;
        corr.ref_id = (int) buffer[ptr++];
        corr.new_id = (int) buffer[ptr++];

        
        ptr = StdVectorToEigenMatrix(corr.sum_p_ref, buffer, ptr);
        ptr = StdVectorToEigenMatrix(corr.sum_p_new, buffer, ptr);

        ptr = StdVectorToEigenMatrix(corr.sum_p_ref_new, buffer, ptr);
        ptr = StdVectorToEigenMatrix(corr.sum_p_ref_ref, buffer, ptr);
        ptr = StdVectorToEigenMatrix(corr.sum_p_new_new, buffer, ptr);
        ptr = StdVectorToEigenMatrix(corr.sum_p_new_ref, buffer, ptr);

        corr.sum_weight = buffer[ptr++];

        corrs.push_back(corr);
        start += 1;
    }
    ifs.close();
}

void ComputeJacobianInfo(Correspondence &fC, std::map<int, KeyframeInfo> &keyframes,
Eigen::MatrixXd &Pre_JiTr,
Eigen::MatrixXd &Pre_JjTr,
Eigen::MatrixXd &Pre_JiTJi,
Eigen::MatrixXd &Pre_JiTJj,
Eigen::MatrixXd &Pre_JjTJi,
Eigen::MatrixXd &Pre_JjTJj)
{

    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();

    //prepare data
    int refID = fC.ref_id;
    int newID = fC.new_id;
    Eigen::Matrix3d R_ref = keyframes[refID].pose.rotationMatrix();
    Eigen::Vector3d t_ref = keyframes[refID].pose.translation();
    Eigen::Matrix3d R_new = keyframes[newID].pose.rotationMatrix();
    Eigen::Vector3d t_new = keyframes[newID].pose.translation();
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


}


void ComputeJacobianInfo(Correspondence &fC, std::vector<SubmapInfo> &submaps,
Eigen::MatrixXd &Pre_JiTr,
Eigen::MatrixXd &Pre_JjTr,
Eigen::MatrixXd &Pre_JiTJi,
Eigen::MatrixXd &Pre_JiTJj,
Eigen::MatrixXd &Pre_JjTJi,
Eigen::MatrixXd &Pre_JjTJj)
{

    Pre_JiTr.setZero();
    Pre_JjTr.setZero();
    Pre_JiTJi.setZero();
    Pre_JiTJj.setZero();
    Pre_JjTJi.setZero();
    Pre_JjTJj.setZero();

    //prepare data
    int refID = fC.ref_id;
    int newID = fC.new_id;
    Eigen::Matrix3d R_ref = submaps[refID].pose.rotationMatrix();
    Eigen::Vector3d t_ref = submaps[refID].pose.translation();
    Eigen::Matrix3d R_new = submaps[newID].pose.rotationMatrix();
    Eigen::Vector3d t_new = submaps[newID].pose.translation();
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
}

float optimize(std::vector<Correspondence> &keyframe_corrs, std::map<int, KeyframeInfo> &keyframes)
{
    //keyframe based optimization
    std::vector<int> keyframe_candidate_fcorrs;
    std::cout<<"OptimizeKeyFrame FrameCorrespondence: "<<keyframe_corrs.size()<<std::endl;
    if (keyframes.size() <= 3 )
    {
      std::cout << "no need to optimize!" << std::endl;
      return -1;
    }
    std::map<int, int> keyframe_pos;
    std::vector<int> pos_to_keyframe(keyframes.size());
    {
        int i = 0;
        for(auto iter = keyframes.begin(); iter != keyframes.end(); ++iter, ++i)
        {
            keyframe_pos[iter->first] = i;
            pos_to_keyframe[i] = iter->first;
        }
    }



    // if(true)
    // {
    //   double init_error = reprojection_error_3Dto3D(keyframe_corrs, keyframe_candidate_fcorrs);
    //   std::cout << "init error		: " << init_error << std::endl;
    // }
    // will be replaced by conjugate gradient descent.
    int optNum = keyframes.size() - 1;
    Eigen::MatrixXd J, err;
    Eigen::MatrixXd delta(6 * optNum, 1), JTe(6 * optNum, 1);
    Eigen::SparseMatrix<double> JTJ(6 * optNum, 6 * optNum);

    double prev_err = 10000;

    clock_t start, end;

    // the solver is only built at the first iteration
    Eigen::SimplicialLDLT	<Eigen::SparseMatrix<double> > SimplicialLDLTSolver;
    std::vector<Eigen::Triplet<double>> coeff;
    coeff.reserve(6 * 6 * 4 * keyframe_corrs.size());
    Eigen::MatrixXd JiTJi_pre(6, 6), JiTJj_pre(6, 6), JjTJi_pre(6, 6), JjTJj_pre(6, 6), JiTe_pre(6, 1), JjTe_pre(6, 1);

    clock_t start_opt, end_opt;
    double time_opt;
    start_opt = clock();


    //float init_error = reprojection_error_3Dto3D(keyframe_corrs, keyframes);
    //float init_total_error_p = reprojection_error_3Dto3D_plane(keyframe_corrs, keyframe_candidate_fcorrs);

    for (int iter = 0; iter < 3; iter++)
    {
      JTe.setZero();
      JTJ.setZero();
      err.setZero();
      coeff.clear();


      for (int i = 0; i < keyframe_corrs.size(); ++i)
      {
        int ref_id = keyframe_corrs[i].ref_id;
        int new_id = keyframe_corrs[i].new_id;
        
        int frame_ref_pos = keyframe_pos[ref_id];
        int frame_new_pos = keyframe_pos[new_id];
        
        
        if (frame_ref_pos < 0 || frame_new_pos < 0)
        {
            continue;
        }

        ComputeJacobianInfo(keyframe_corrs[i], keyframes,
          JiTe_pre,
          JjTe_pre,
          JiTJi_pre,
          JiTJj_pre,
          JjTJi_pre,
          JjTJj_pre);

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

        
        JTJ.setFromTriplets(coeff.begin(), coeff.end());
        // std::cout<<JTJ<<std::endl<<std::endl;
        
        // std::cout<<JTe<<std::endl<<std::endl;
        //timer.Tick("keyframe based optimization");
        // std::cout<<coeff.size()<<std::endl;
        SimplicialLDLTSolver.compute(JTJ);
        //timer.Tock("keyframe based optimization");
        delta = SimplicialLDLTSolver.solve(JTe);

        // std::cout<<JTe.transpose()<<std::endl;

      for (int i = 1; i < pos_to_keyframe.size(); i++)
      {
          Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (i - 1), 0);
          if(std::isnan(delta_i(0)))
          {
            std::cout << "nan detected in pose update! " << std::endl;
            continue;
          }
          //std::cout<<i<<" "<<delta_i<<std::endl;
            //   std::cout<<std::endl;
          keyframes[pos_to_keyframe[i]].pose = Sophus::SE3d::exp(delta_i).inverse() *
                  keyframes[pos_to_keyframe[i]].pose;

      }

    }

    //float final_error = reprojection_error_3Dto3D(keyframe_corrs, keyframes);

    //std::cout << "init/final error " << init_error << "/" << final_error<<std::endl;


    return 0.1;
}

float optimize(std::vector<Correspondence> &submap_corrs, std::vector<SubmapInfo> &submaps)
{
    //submap based optimization
    std::vector<int> submap_candidate_fcorrs;
    std::cout<<"Optimizesubmap FrameCorrespondence: "<<submap_corrs.size()<<std::endl;
    if (submaps.size() <= 3 )
    {
      std::cout << "no need to optimize!" << std::endl;
      return -1;
    }
    // if(true)
    // {
    //   double init_error = reprojection_error_3Dto3D(submap_corrs, submap_candidate_fcorrs);
    //   std::cout << "init error		: " << init_error << std::endl;
    // }
    // will be replaced by conjugate gradient descent.
    int optNum = submaps.size() - 1;
    Eigen::MatrixXd J, err;
    Eigen::MatrixXd delta(6 * optNum, 1), JTe(6 * optNum, 1);
    Eigen::SparseMatrix<double> JTJ(6 * optNum, 6 * optNum);

    double prev_err = 10000;

    clock_t start, end;

    // the solver is only built at the first iteration
    Eigen::SimplicialLDLT	<Eigen::SparseMatrix<double> > SimplicialLDLTSolver;
    std::vector<Eigen::Triplet<double>> coeff;
    coeff.reserve(6 * 6 * 4 * submap_corrs.size());
    Eigen::MatrixXd JiTJi_pre(6, 6), JiTJj_pre(6, 6), JjTJi_pre(6, 6), JjTJj_pre(6, 6), JiTe_pre(6, 1), JjTe_pre(6, 1);

    clock_t start_opt, end_opt;
    double time_opt;
    start_opt = clock();


    //float init_error_p = reprojection_error_3Dto3D_plane(optimized_fc);
    //float init_total_error_p = reprojection_error_3Dto3D_plane(submap_corrs, submap_candidate_fcorrs);

    for (int iter = 0; iter < 10; iter++)
    {
      JTe.setZero();
      JTJ.setZero();
      err.setZero();
      coeff.clear();


      for (int i = 0; i < submap_corrs.size(); ++i)
      {
        int ref_id = submap_corrs[i].ref_id;
        int new_id = submap_corrs[i].new_id;

        int frame_ref_pos = ref_id;
        int frame_new_pos = new_id;
        if (frame_ref_pos < 0 || frame_new_pos < 0)
        {
            continue;
        }

        ComputeJacobianInfo(submap_corrs[i], submaps,
          JiTe_pre,
          JjTe_pre,
          JiTJi_pre,
          JiTJj_pre,
          JjTJi_pre,
          JjTJj_pre);

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


      JTJ.setFromTriplets(coeff.begin(), coeff.end());
      //timer.Tick("submap based optimization");
      SimplicialLDLTSolver.compute(JTJ);
    //timer.Tock("submap based optimization");
      delta = SimplicialLDLTSolver.solve(JTe);

      for (int i = 1; i < submaps.size(); i++)
      {
          Eigen::VectorXd delta_i = delta.block<6, 1>(6 * (i - 1), 0);
          if(std::isnan(delta_i(0)))
          {
            std::cout << "nan detected in pose update! " << std::endl;
            continue;
          }
          //std::cout<<i<<" "<<delta_i<<std::endl;
          //std::cout<<std::endl;
          submaps[i].pose = Sophus::SE3d::exp(delta_i).inverse() *
                  submaps[i].pose;

      }

    }

    // float final_error = reprojection_error_3Dto3D(optimized_fc);
    // float final_total_error = reprojection_error_3Dto3D(submap_corrs,submap_candidate_fcorrs);

    // //float final_error_p = reprojection_error_3Dto3D_plane(optimized_fc);
    // //float final_total_error_p = reprojection_error_3Dto3D_plane(submap_corrs,submap_candidate_fcorrs);
    // std::cout << "init/final error " << init_error << "/" << final_error
    //      << "       " << init_total_error << "/" << final_total_error << std::endl;

    return 0.1;        
}
int main()
{
    std::vector<SubmapInfo> submaps;
    std::map<int, KeyframeInfo> keyframes;
    std::vector<Correspondence> submap_corrs;
    std::vector<Correspondence> keyframe_corrs;

    read_keyframe_info("./keyframes_info.txt", keyframes);
    read_submap_info("./submaps_info.txt", submaps);
    read_correspondence("./keyframe_corrs.txt", keyframe_corrs);
    read_correspondence("./keyframe_corrs_server.txt", keyframe_corrs);
    read_correspondence("./submap_corrs.txt", submap_corrs);

    std::cout<<"read: "<<std::endl;
    std::cout<<keyframes.size()<<" keyframes"<<std::endl;
    std::cout<<submaps.size()<<" submaps"<<std::endl;
    std::cout<<keyframe_corrs.size()<<" keyframe corrs"<<std::endl;
    std::cout<<submap_corrs.size()<<" submap corrs"<<std::endl;

    float init_error = reprojection_error_3Dto3D(keyframe_corrs, keyframes);
    std::ofstream ofs_before("./trajectory_before.txt");
    for(auto iter = keyframes.begin(); iter != keyframes.end(); ++iter)
    {
        {
        Sophus::SE3d p = iter->second.pose;
        ofs_before<<iter->first<<" "
                <<p.translation()(0)<<" "
                <<p.translation()(1)<<" "
                <<p.translation()(2)<<" "
                <<p.unit_quaternion().coeffs().data()[0]<<" "
                <<p.unit_quaternion().coeffs().data()[1]<<" "
                <<p.unit_quaternion().coeffs().data()[2]<<" "
                <<p.unit_quaternion().coeffs().data()[3]
                <<std::endl;
        }
    } 
    // for(int i = 0; i != keyframe_corrs.size(); ++i)
    // {
    //     auto &fC = keyframe_corrs[i];
    //     std::cout<<fC.ref_id<<" "<<fC.new_id<<std::endl;
    //     std::cout<<fC.sum_p_ref.transpose()<<std::endl;
    //     std::cout<<fC.sum_p_new.transpose()<<std::endl;
    //     std::cout<<fC.sum_p_ref_new<<std::endl;
    //     std::cout<<fC.sum_p_ref_ref<<std::endl;
    //     std::cout<<fC.sum_p_new_new<<std::endl;
    //     std::cout<<fC.sum_p_new_ref<<std::endl;    
    //     std::cout<<std::endl;
    // }
    // float init_error_2 = reprojection_error_3Dto3D(keyframe_corrs, keyframes);
    // float init_error_submap = reprojection_error_3Dto3D(submap_corrs, submaps);
    std::vector<int> pos_to_keyframe;
    for(auto iter = keyframes.begin(); iter != keyframes.end(); ++iter)
    {
        pos_to_keyframe.push_back(iter->first);
    }
    /*
    trash code
    */
    std::map<int, std::tuple<float, float, int, int, float, float>> frame_id_to_statis;


    for(int i = 50; i < pos_to_keyframe.back(); i += 50)
    {
        // std::cout<<i<<": "<<std::endl;

        int max_frame_id = i;
        int max_keyframe_id = 0;
        //std::cout<<max_keyframe_id<<" "<<max_submap_id<<std::endl;
        std::map<int, KeyframeInfo> tmp_keyframes;
        std::vector<SubmapInfo> tmp_submaps;
        std::vector<Correspondence> tmp_submap_corrs;
        std::vector<Correspondence> tmp_keyframe_corrs;
        
        for(auto iter = keyframes.begin(); iter != keyframes.end(); ++iter)
        {

            if(iter->first > max_frame_id)
            break;
            max_keyframe_id = iter->first;
            tmp_keyframes.insert(tmp_keyframes.end(), *iter) ;

        }        
        int max_submap_id = keyframes[max_keyframe_id].submap_id;
        for(auto iter = submaps.begin(); iter != submaps.end(); ++iter)
        {
            if(iter->submap_id > max_submap_id)
            break;
            
            tmp_submaps.push_back(*iter);
        }
        // std::cout<<tmp_submaps.size()<<std::endl;



        for(auto iter = submap_corrs.begin(); iter != submap_corrs.end(); ++iter)
        {
            if(iter->ref_id <= max_submap_id && iter-> new_id <= max_submap_id)
            tmp_submap_corrs.push_back(*iter);
        }
        
        for(auto iter = keyframe_corrs.begin(); iter != keyframe_corrs.end(); ++iter)
        {
            if(iter->ref_id <= max_keyframe_id && iter-> new_id <= max_keyframe_id)
            tmp_keyframe_corrs.push_back(*iter);
        }
        timer.Tick("keyframe based optimization");
        optimize(tmp_keyframe_corrs, tmp_keyframes);
        timer.Tock("keyframe based optimization");


        timer.Tick("submap based optimization");
        optimize(tmp_submap_corrs, tmp_submaps);
        timer.Tock("submap based optimization");

        float running_time_keyframe, running_time_submap;
        int submap_number = tmp_submaps.size(), keyframe_number = tmp_keyframes.size();

        running_time_keyframe = timer.Elapsed("keyframe based optimization");
        running_time_submap = timer.Elapsed("submap based optimization");
        
        float error_k = reprojection_error_3Dto3D(tmp_keyframe_corrs, tmp_keyframes);
        for(auto iter = tmp_keyframes.begin(); iter != tmp_keyframes.end(); ++iter)
        {
            iter->second.pose = tmp_submaps[iter->second.submap_id].pose * iter->second.relative_pose;
        }    
        float error_s = reprojection_error_3Dto3D(tmp_keyframe_corrs, tmp_keyframes);

        frame_id_to_statis[i] = std::make_tuple(running_time_keyframe, running_time_submap, keyframe_number, submap_number, error_k, error_s);

    }

    timer.Tick("keyframe based optimization");
    optimize(keyframe_corrs, keyframes);
    timer.Tock("keyframe based optimization");


    timer.Tick("submap based optimization");
    optimize(submap_corrs, submaps);
    timer.Tock("submap based optimization");


    float final_error_keyframe = reprojection_error_3Dto3D(keyframe_corrs, keyframes);
    std::ofstream traj_keyframe("./trajectory_keyframe.txt");
    std::ofstream traj_submap("./trajectory_submap.txt");
    std::vector<Eigen::Vector3d> positions;
    for(auto iter = keyframes.begin(); iter != keyframes.end(); ++iter)
    {
        positions.push_back(iter->second.pose.translation());
        {
        Sophus::SE3d p = iter->second.pose;
        traj_keyframe<<iter->first<<" "
                <<p.translation()(0)<<" "
                <<p.translation()(1)<<" "
                <<p.translation()(2)<<" "
                <<p.unit_quaternion().coeffs().data()[0]<<" "
                <<p.unit_quaternion().coeffs().data()[1]<<" "
                <<p.unit_quaternion().coeffs().data()[2]<<" "
                <<p.unit_quaternion().coeffs().data()[3]
                <<std::endl;
        }
        iter->second.pose = submaps[iter->second.submap_id].pose * iter->second.relative_pose;
        {
        Sophus::SE3d p = iter->second.pose;
        traj_submap<<iter->first<<" "
                <<p.translation()(0)<<" "
                <<p.translation()(1)<<" "
                <<p.translation()(2)<<" "
                <<p.unit_quaternion().coeffs().data()[0]<<" "
                <<p.unit_quaternion().coeffs().data()[1]<<" "
                <<p.unit_quaternion().coeffs().data()[2]<<" "
                <<p.unit_quaternion().coeffs().data()[3]
                <<std::endl;
        }
    }    
    float distance = 0;
    for(int i = 1; i != positions.size(); ++i)
    {
        distance += (positions[i] - positions[i-1]).norm();
    }
    std::cout<<"distance: "<<distance<<std::endl;
    traj_keyframe.close();
    traj_submap.close();
    float final_error_submap = reprojection_error_3Dto3D(keyframe_corrs, keyframes);
    std::cout<<"init error / final error (keyframe) / final error (submap): "<<init_error<<"/"<<final_error_keyframe<<"/"<<final_error_submap<<std::endl;
    {
        float running_time_keyframe, running_time_submap;
        int submap_number = submaps.size(), keyframe_number = keyframes.size();
        running_time_keyframe = timer.Elapsed("keyframe based optimization");
        running_time_submap = timer.Elapsed("submap based optimization");
        frame_id_to_statis[pos_to_keyframe.back()] = std::make_tuple(running_time_keyframe, running_time_submap, keyframe_number, submap_number, final_error_keyframe, final_error_submap);
    }
    std::ofstream ofs("./optimization_comp.txt");
    for(auto iter = frame_id_to_statis.begin(); iter != frame_id_to_statis.end(); ++ iter)
    {
        float running_time_keyframe, running_time_submap;
        int submap_number, keyframe_number;
        float error_k, error_s;
        std::tie(running_time_keyframe, running_time_submap, keyframe_number, submap_number, error_k, error_s) = iter->second;
        ofs<<iter->first<<" "<<running_time_keyframe<<" "<<running_time_submap<<" "<<keyframe_number<<" "<<submap_number<<" "<<error_k<<" "<<error_s <<std::endl;
    }

    ofs.close();
    return 0;
}