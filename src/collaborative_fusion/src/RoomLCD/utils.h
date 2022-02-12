#ifndef UTILS_H_
#define UTILS_H_

#include <torch/torch.h>
#include <Eigen/Eigen>
#include <vector>
#include <map>
#include <tuple>
#include <iostream>
#include <cstring>
#include <cnpy/cnpy.h>
#include "graphmatching/graphmatching.h"
#include <numeric>      // std::iota
#include <algorithm>    // std::sort, std::stable_sort
#include <opencv2/opencv.hpp>
#include <open_chisel/mesh/PointCloud.h>
#include <open_chisel/ForLabel.h>
#define DEBUG
#include "debugtools.h"
#include "../GCFusion/RoomOptimization.h"
#include "../GCSLAM/GlobalParameter.h"
#define MIN_INSTANCE_SIZE 200
#define NUM_CLASS 20
#define CHAIR 4
#define SOFA 5
#define TABLE 6
#define DESK 12
#define CABINET 2
#define BOOKSHELF 9
// using namespace Eigen;
using Tensors = std::vector<torch::Tensor>;
typedef Eigen::VectorXd VectorHistgram;

#include "TransformationModel.hpp"
#include "GRANSAC.hpp"


Tensors LoadNpz(std::string path);
Eigen::Vector3f cv_point_to_eigen(cv::Point3f point);
/* Graph Generation */
struct ObjGraph {
    ObjGraph(std::vector<float> &vertices,
             std::vector<int> &labels,
             VectorHistgram _histogram, 
             int _scene_id = -1, int _camera_id = -1)
    {
        num_vertices = vertices.size() / 3;
        assert(num_vertices == labels.size());
        vertices_ptr = std::make_shared<std::vector<float>>(
            vertices);
        labels_ptr = std::make_shared<std::vector<int>>(
            labels);
        feature2nd_ptr = nullptr;
        embedding_ptr = nullptr;
        histogram = _histogram;
        scene_id  = _scene_id;
        camera_id = _camera_id;
    }

    ObjGraph() : num_vertices(0) {}

    long get_size() {return num_vertices;}

    size_t set_id(size_t id) {scene_id = id; return id;}
    chisel::PcdPtr visualize_to_ply()
    {
        chisel::PointCloud pcd;
        for(int i = 0; i <vertices_ptr->size();i+=3)
        {
            Eigen::Vector3f point = Eigen::Vector3f((*vertices_ptr)[i], 
                (*vertices_ptr)[i+1], (*vertices_ptr)[i+2]);
            
            pcd.vertices.push_back(point);
            //pcd.colors.push_back();
            Eigen::Vector3f point_tmp;
            for(int x = -4; x <5; ++x)
            {
                point_tmp(0) = point(0) + x*0.005;
                for(int y = -4; y <5; ++y)
                {
                    point_tmp(1) = point(1) + y*0.005;
                    for(int z = -4; z <5; ++z)
                    {
                        point_tmp(2) = point(2) + z*0.005;
                        pcd.vertices.push_back(point_tmp);
                        //pcd.color.push_back()
                    }
                }
            }     
        }

        return std::make_shared<chisel::PointCloud>(pcd);
        
    }
    std::shared_ptr<std::vector<double>> get_feature2nd();

    // N x 3 coords
    std::shared_ptr<std::vector<float>> vertices_ptr;
    std::shared_ptr<std::vector<int>> labels_ptr;
    VectorHistgram histogram;
    std::shared_ptr<std::vector<double>> feature2nd_ptr;
    std::shared_ptr<std::vector<double>> embedding_ptr;
    long num_vertices;
    int scene_id = -1;
    int camera_id = -1;
};




/* Graph database */
struct SceneDatabase {
    SceneDatabase() {

    }
    void insert(ObjGraph G1) { obj_database.push_back(G1); }
    /*Query returns: a list of corresponding vertices*/
    void query(ObjGraph& G1, std::vector<RoomCorrespondence> &outputs);
    // Members
    void query_with_time(ObjGraph& G2, std::vector<RoomCorrespondence> &outputs);
    void query_with_matches(ObjGraph& G2, 
        std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>> &graph_matching, 
        std::vector<std::vector<int>> &matches);
    size_t get_size()
    {
        return obj_database.size();
    }
    void initialize_estimator()
    {
        Estimator.Initialize(MultiViewGeometry::g_para.room_lcd_ransac_threshold, 2000);
    }
    float get_inliers(ObjGraph &G1, ObjGraph &G2, 
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &inliers, bool inverse = false);
    float get_inliers(ObjGraph &G1, ObjGraph &G2, 
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &graph_matching, 
        std::vector<int> &inlier_matches, bool inverse = false);
    std::vector<ObjGraph> obj_database;
    GRANSAC::RANSAC<TransformationModel, MIN_INLIER_SIZE> Estimator;

};

ObjGraph generate_graph_from_pcd(Tensors &inputs);

void dump_graph_to_file(ObjGraph graph, std::string filename);

torch::Tensor match_graphs(ObjGraph& G1, ObjGraph& G2);

void dump_matrix_to_file(torch::Tensor matrix, std::string filename);

// APIs
ObjGraph get_graph(chisel::PointCloud &pcd, int scene_id = -1, int camera_id = -1);


#endif