#include "utils.h"
#include "../Tools/TickTock.h"
// static constexpr double PI = 3.14159265359;

Tensors LoadNpz(std::string path) {
    std::cout << std::string("Loading npz from ") + path << std::endl;

    cnpy::npz_t arr_ = cnpy::npz_load(path);
    cnpy::NpyArray arr_xyz = arr_["xyz"];
    cnpy::NpyArray arr_semantic = arr_["pred_semantic"];
    cnpy::NpyArray arr_instance = arr_["pred_instance"];
    
    // Load xyz
    assert(arr_xyz.word_size == sizeof(float));
    size_t npoints = arr_xyz.shape[0];
    torch::Tensor xyz = torch::empty({(long)npoints ,3}, torch::dtype(torch::kFloat32));
    memcpy( xyz.data_ptr(), (void*) arr_xyz.data<float>(), arr_xyz.num_bytes());

    // Load pred_semantic (kInt64)
    assert(arr_semantic.word_size == sizeof(int64_t));
    assert(npoints == arr_semantic.shape[0]);
    torch::Tensor pred_semantic = torch::empty({(long)npoints}, 
                    torch::dtype(torch::kInt64));
    memcpy( pred_semantic.data_ptr(), 
                    (void*) arr_semantic.data<int64_t>(),
                     arr_semantic.num_bytes());
    pred_semantic = pred_semantic.toType(torch::kInt32);

    // Load pred_instance (kInt32)
    assert(arr_instance.word_size == sizeof(int));
    assert(npoints == arr_instance.shape[0]);
    torch::Tensor pred_instance = torch::empty({(long)npoints}, 
                    torch::dtype(torch::kInt32));
    memcpy(pred_instance.data_ptr(), 
                    (void*) arr_instance.data<int>(),
                     arr_instance.num_bytes());

    return Tensors({xyz, pred_semantic, pred_instance});
}

ObjGraph generate_graph_from_pcd(Tensors &inputs) {
    torch::Tensor xyz = inputs[0]; //float
    torch::Tensor pred_semantic = inputs[1]; // int
    torch::Tensor pred_instance = inputs[2]; // int
    // get each instance and compute the center of it
    size_t max_id = pred_instance.max().item<long>();
    // size_t max_label = pred_semantic.max().item<long>();
    VectorHistgram histogram;
    histogram.setZero(20);
    std::vector<float> vertices;
    std::vector<int> labels;
    DBGVAR(std::cout, max_id);
    assert(max_id > 0);
    for(size_t i = 0; i <= max_id; i++)  {
        auto instance_mask = (pred_instance == (long)i).view({-1,1});
        auto instance_size = instance_mask.sum(0).item<long>();
        if (instance_size < MIN_INSTANCE_SIZE)  {
            continue;
        }
        auto instance_coords = 
                torch::masked_select(xyz, instance_mask.repeat({1,3})).view({-1,3});
        auto instance_labels = 
                torch::masked_select(pred_semantic, instance_mask.view({-1}));
        // Get the object id
        int label = std::get<0>(at::mode(instance_labels)).item<int>();
        /*
        if(label == 0 || label == 1) {
            continue;
        }*/
        if(label != CHAIR &&label != SOFA && label != CABINET 
            && label != TABLE && label != BOOKSHELF && label != DESK)
            continue;
        auto center = at::mean(instance_coords, 0);
        vertices.insert(vertices.end(), 
            {center[0].item<float>(), center[1].item<float>(), center[2].item<float>()});
        labels.push_back(label);
        histogram[label] += 1;
        
        // DBGVAR(std::cout, center);
         DBGVAR(std::cout, label);
        // DBGVAR(std::cout, instance_size);
    }
    DBGVAR(std::cout, vertices.size());
    DBGVAR(std::cout, labels.size());
    DBGVAR(std::cout, histogram);

    return ObjGraph(vertices, labels, histogram);
}

void dump_graph_to_file(ObjGraph graph, std::string filename)  {
    std::cout << "Saving graph..." << std::endl;
    unsigned long N = (unsigned long)graph.get_size();
    DBGVAR(std::cout, N);
    cnpy::npz_save(filename,"vertices",
            graph.vertices_ptr->data(), {N, 3},"w");
    cnpy::npz_save(filename,"labels",
            graph.labels_ptr->data(), {N}, "a");
}

/* graph matching */
std::shared_ptr<std::vector<double>> ObjGraph::get_feature2nd() {
    if (feature2nd_ptr != nullptr)  {
        return feature2nd_ptr;
    }
    else {
        feature2nd_ptr = std::make_shared<std::vector<double>>(num_vertices*num_vertices);
        for(size_t i = 0; i < num_vertices; i++)  {
            for(size_t j = 0; j < num_vertices; j++)  {
                double dist = sqrt(((double)(*vertices_ptr)[i*3]-(*vertices_ptr)[j*3])*
                                   ((double)(*vertices_ptr)[i*3]-(*vertices_ptr)[j*3])+
                                   ((double)(*vertices_ptr)[i*3+1]-(*vertices_ptr)[j*3+1])*
                                   ((double)(*vertices_ptr)[i*3+1]-(*vertices_ptr)[j*3+1])+
                                   ((double)(*vertices_ptr)[i*3+2]-(*vertices_ptr)[j*3+2])*
                                   ((double)(*vertices_ptr)[i*3+2]-(*vertices_ptr)[j*3+2]));
                (*feature2nd_ptr)[i * num_vertices + j] = dist;
                // DBGVAR(std::cout, dist);
            }
        }
        return feature2nd_ptr;
    }
}

torch::Tensor match_graphs(ObjGraph& G1, ObjGraph& G2) {
    long nP1 = G1.get_size();
    long nP2 = G2.get_size();
    assert(nP1*nP2 != 0);
    auto feature1 = G1.get_feature2nd();
    auto feature2 = G2.get_feature2nd();
    auto label1_ptr = G1.labels_ptr;
    auto label2_ptr = G2.labels_ptr;

    std::vector<double> dists2(nP1*nP1*nP2*nP2, 0.0);
    std::vector<double> label_mask(nP1*nP1*nP2*nP2, 0.0);
    std::vector<int> indH2(nP1*nP1*nP2*nP2*2);

    for(size_t k = 0; k < nP1*nP1; k++)  {
        for(size_t l = 0; l < nP2*nP2; l++)  {
            dists2[k*nP2*nP2+l] = std::abs((*feature1)[k] - (*feature2)[l]);
            // DBGVAR(std::cout, dists2[k*nP2*nP2+l]);
            indH2[k*nP2*nP2+l] = k/nP1*nP2 + l/nP2;
            indH2[nP1*nP1*nP2*nP2+ k*nP2*nP2+l] = (k%nP1)*nP2 + (l%nP2);
            if ((*label1_ptr)[k/nP1] == (*label2_ptr)[l/nP2] && 
                (*label1_ptr)[k%nP1] == (*label2_ptr)[l%nP2])
                label_mask[k*nP2*nP2 + l] = 1.0;
        }
    }
    torch::Tensor dists2_tensor = torch::from_blob(dists2.data(), {nP1*nP1*nP2*nP2}, 
                                            torch::dtype(torch::kFloat64));
    torch::Tensor mask_tensor = torch::from_blob(label_mask.data(), {nP1*nP1*nP2*nP2}, 
                                            torch::dtype(torch::kFloat64));
    auto valH2 = at::exp(-dists2_tensor/at::mean(dists2_tensor)) * mask_tensor;
    torch::Tensor Xin = torch::empty({nP1, nP2}, 
                        torch::dtype(torch::kFloat64)).fill_(1.0/(double)nP2);
    torch::Tensor Xout = torch::empty({nP1, nP2}, torch::dtype(torch::kFloat64));

    // due to MATLAB/C++ addressing issue, here nP2 and nP1 should be switched
    double ScoreOut;
    tensorMatching(Xin.data_ptr<double>(), nP2 , nP1, 
                    nullptr, nullptr, 0,
                    indH2.data(), valH2.data_ptr<double>(), nP1*nP1*nP2*nP2,
                    nullptr, nullptr, 0,
                    200, 1, 1,
                    Xout.data_ptr<double>(), &ScoreOut);
    
    DBGVAR(std::cout, ScoreOut);
    
    return Xout;
}

void dump_matrix_to_file(torch::Tensor matrix, std::string filename)  {
    std::cout << "Saving graph..." << std::endl;
    size_t N1 = matrix.size(0);
    size_t N2 = matrix.size(1);
    cnpy::npz_save(filename,"x",
            matrix.data_ptr<double>(), {N1, N2},"w");
}

/* Database query*/

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T> &v)
{
    // initialize original index locations
    std::vector<size_t> idx(v.size());
    std::iota(idx.begin(), idx.end(), 0);

    // sort indexes based on comparing values in v
    // using std::stable_sort instead of std::sort
    // to avoid unnecessary index re-orderings
    // when v contains elements of equal values
    std::stable_sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

    return idx;
}

Eigen::Vector3f cv_point_to_eigen(cv::Point3f point) {
    return Eigen::Vector3f(point.x, point.y, point.z);
}

float SceneDatabase::get_inliers(ObjGraph &G1, ObjGraph &G2, 
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &graph_matching, 
    std::vector<int> &inlier_matches, bool inverse)
{
    auto assign_matrix = match_graphs(G1, G2);
    // DEBUG 
    // dump_matrix_to_file(assign_matrix, 
    //         "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/matrix.npz");
    // dump_graph_to_file(G1, 
    //         "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/graph1.npz");
    // dump_graph_to_file(G2, 
    //         "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/graph2.npz");
    // [12  8  9  5  8 13  7  6 22 18 23 29 19  0 11 21 17 22 31 30 25 29 29 25 15 26 21  5  4 22]
    auto Xmax = assign_matrix.argmax(1);
    // DBGVAR(std::cout, assign_matrix[0]);
    //DBGVAR(std::cout, Xmax);
    //DBGVAR(std::cout, assign_matrix.sizes());
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;

    graph_matching.clear();
    for(size_t i = 0; i < G1.get_size() && i < Xmax.size(0); i++) {
        Eigen::Vector3f p1((*(G1.vertices_ptr))[3*i],
                    (*(G1.vertices_ptr))[3*i+1],
                    (*(G1.vertices_ptr))[3*i+2] );

        long index = Xmax[i].item<long>();
        Eigen::Vector3f p2((*(G2.vertices_ptr))[3*index],
                    (*(G2.vertices_ptr))[3*index+1],
                    (*(G2.vertices_ptr))[3*index+2] );
        
        std::shared_ptr<GRANSAC::AbstractParameter> CandPt;
        
        if(inverse)
        {
            CandPt = std::make_shared<Point3fPair>(p2, p1, i);        
            graph_matching.push_back(std::make_pair(p2, p1));
        }
        else
        {
            CandPt = std::make_shared<Point3fPair>(p1, p2, i);
            graph_matching.push_back(std::make_pair(p1, p2));
        }
        CandPoints.push_back(CandPt);
    }

    cv::Mat aff_est;
// 0/1 value
    //estimateAffine3D(src, dst, aff_est, inliers, threshold);
    //DBGVAR(std::cout, inliers.size());
    inlier_matches.clear();
    if(CandPoints.size() < MIN_INLIER_SIZE)
    {
        
        return 0;
    }
    Estimator.Estimate(CandPoints);
    auto bestInliers = Estimator.GetBestInliers();
    for(int i = 0; i < bestInliers.size();++i)
    {
        auto inlier =  std::dynamic_pointer_cast<Point3fPair>(bestInliers[i]);
        inlier_matches.push_back(inlier->index);
    }
/*
    for(int iter = 5; iter > 0; --iter)
    {
        index = 0;
        for(size_t i = 0; i < inliers.size(); i++) {
            if(inliers[i])
            {
                src[index] = src[i];
                dst[index] = dst[i];
                index++;
            }
        }
        threshold *= 0.8;
        src.resize(index);
        dst.resize(index);
        inliers.clear();
        estimateAffine3D(src, dst, aff_est, inliers, threshold);            
        //DBGVAR(std::cout, inliers.size());
        if(index == last_index) break;
        last_index = index;
    }
*/
    //DBGVAR(std::cout, aff_est);
    DBGVAR(std::cout, float(bestInliers.size()) / G1.get_size());
    return float(bestInliers.size()) / G1.get_size();
}
float SceneDatabase::get_inliers(ObjGraph &G1, ObjGraph &G2, std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &inliers, bool inverse)
{
    auto assign_matrix = match_graphs(G1, G2);
    // DEBUG 
    // dump_matrix_to_file(assign_matrix, 
    //         "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/matrix.npz");
    // dump_graph_to_file(G1, 
    //         "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/graph1.npz");
    // dump_graph_to_file(G2, 
    //         "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/graph2.npz");
    // [12  8  9  5  8 13  7  6 22 18 23 29 19  0 11 21 17 22 31 30 25 29 29 25 15 26 21  5  4 22]
    auto Xmax = assign_matrix.argmax(1);
    // DBGVAR(std::cout, assign_matrix[0]);
    //DBGVAR(std::cout, Xmax);
    //DBGVAR(std::cout, assign_matrix.sizes());
    std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;

    inliers.clear();
    for(size_t i = 0; i < G1.get_size() && i < Xmax.size(0); i++) {
        Eigen::Vector3f p1((*(G1.vertices_ptr))[3*i],
                    (*(G1.vertices_ptr))[3*i+1],
                    (*(G1.vertices_ptr))[3*i+2] );

        long index = Xmax[i].item<long>();
        Eigen::Vector3f p2((*(G2.vertices_ptr))[3*index],
                    (*(G2.vertices_ptr))[3*index+1],
                    (*(G2.vertices_ptr))[3*index+2] );
        
        std::shared_ptr<GRANSAC::AbstractParameter> CandPt;
        if(inverse)
        CandPt = std::make_shared<Point3fPair>(p2, p1);        
        else
        CandPt = std::make_shared<Point3fPair>(p1, p2);
        
        CandPoints.push_back(CandPt);
    }

    cv::Mat aff_est;
// 0/1 value
    //estimateAffine3D(src, dst, aff_est, inliers, threshold);
    //DBGVAR(std::cout, inliers.size());
    if(CandPoints.size() < MIN_INLIER_SIZE)
    {
        inliers.clear();
        return 0;
    }
    Estimator.Estimate(CandPoints);
    auto bestInliers = Estimator.GetBestInliers();
    for(int i = 0; i < bestInliers.size();++i)
    {
        auto inlier =  std::dynamic_pointer_cast<Point3fPair>(bestInliers[i]);
        inliers.push_back(inlier->_3d_pair);
    }
    std::cout<<inliers.size()<<std::endl;
/*
    for(int iter = 5; iter > 0; --iter)
    {
        index = 0;
        for(size_t i = 0; i < inliers.size(); i++) {
            if(inliers[i])
            {
                src[index] = src[i];
                dst[index] = dst[i];
                index++;
            }
        }
        threshold *= 0.8;
        src.resize(index);
        dst.resize(index);
        inliers.clear();
        estimateAffine3D(src, dst, aff_est, inliers, threshold);            
        //DBGVAR(std::cout, inliers.size());
        if(index == last_index) break;
        last_index = index;
    }
*/
    //DBGVAR(std::cout, aff_est);
    DBGVAR(std::cout, float(bestInliers.size()) / G1.get_size());
    return float(bestInliers.size()) / G1.get_size();
}
void SceneDatabase::query(ObjGraph& G2, std::vector<RoomCorrespondence> &outputs) {
    // if empty
    if (obj_database.size() == 0) {
        return ;
    }

    // Step1: histogram comparison and pick N nearest candidates
    // Compare histograms thru exhaustive search right now
    std::vector<double> difference_list(obj_database.size());
    for(size_t i = 0; i < obj_database.size(); i++) {
        ObjGraph& G1 = obj_database[i];
        VectorHistgram diff = G1.histogram - G2.histogram;
        difference_list[i] = diff.squaredNorm();
        //std::cout<<difference_list[i]<<std::endl;
    }

    auto sorted_idx = sort_indexes(difference_list);
    //DBGVAR(std::cout, sorted_idx);

    // Step 2: graph matching
    int n_nearest = MultiViewGeometry::g_para.room_lcd_n_nearest;
    
    outputs.clear();
    
    for (int j = 0; j < n_nearest && j < obj_database.size(); j++) {
        std::cout << "Matching: " << obj_database[sorted_idx[j]].camera_id <<" "<<obj_database[sorted_idx[j]].scene_id << std::endl;
        ObjGraph &G1 = obj_database[sorted_idx[j]];
        std::vector<cv::Point3f> src, dst;
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> inliers;
        float ratio = get_inliers(G1, G2, inliers);
        //information building: 0.3 8
        //i-park: 0.35 6
        if(ratio <= MultiViewGeometry::g_para.room_lcd_min_ratio || 
            inliers.size() < MultiViewGeometry::g_para.room_lcd_min_inlier) 
            ratio = get_inliers(G2, G1, inliers, true);
        if(ratio > MultiViewGeometry::g_para.room_lcd_min_ratio 
        && inliers.size() >= MultiViewGeometry::g_para.room_lcd_min_inlier) {
            RoomCorrespondence output;
            output.source_id = G1.scene_id;
            output.target_id = G2.scene_id;
            if(output.source_id == -1)
            output.source_id = sorted_idx[j];
            output.source_camera_id = G1.camera_id;
            output.target_camera_id = G2.camera_id;
            std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &correspondences 
                = output.instance_center_correspondences;
                correspondences = inliers;
/*
            for(size_t i = 0; i < inliers.size(); i++) {
                if(inliers[i] == 1) {
#if 1
                    correspondences.push_back(
                            std::make_pair( 
                                    cv_point_to_eigen(src[i]),
                                    cv_point_to_eigen(dst[i])
                                    ));
#else
                    correspondences.push_back(
                            std::make_pair( 
                                    cv_point_to_eigen(dst[i]),
                                    cv_point_to_eigen(src[i])

                                    ));
#endif

                }
            }
                //std::cout<<output.transformation<<std::endl;
            DBGVAR(std::cout, correspondences.size());
            DBGVAR(std::cout, ratio);
*/
            //if(correspondences.size() >= 8)
            outputs.push_back(output);
            //return output;
        }
    }

    // Step 3: RANSAC eliminate outliers
  //return nooutput;
}

void SceneDatabase::query_with_time(ObjGraph& G2, std::vector<RoomCorrespondence> &outputs) {
    // if empty
    if (obj_database.size() == 0) {
        return ;
    }

    // Step1: histogram comparison and pick N nearest candidates
    // Compare histograms thru exhaustive search right now
    std::vector<double> difference_list(obj_database.size());
    for(size_t i = 0; i < obj_database.size(); i++) {
        ObjGraph& G1 = obj_database[i];
        VectorHistgram diff = G1.histogram - G2.histogram;
        difference_list[i] = diff.squaredNorm();
        //std::cout<<difference_list[i]<<std::endl;
    }

    auto sorted_idx = sort_indexes(difference_list);
    //DBGVAR(std::cout, sorted_idx);

    // Step 2: graph matching
    int n_nearest = MultiViewGeometry::g_para.room_lcd_n_nearest;
    
    outputs.clear();
    tool::Timer timer;
    timer.Tick("Ransac Geometry Check");
    for (int j = 0; j < n_nearest && j < obj_database.size(); j++) {
        std::cout << "Matching: " << obj_database[sorted_idx[j]].camera_id <<" "<<obj_database[sorted_idx[j]].scene_id << std::endl;
        ObjGraph &G1 = obj_database[sorted_idx[j]];
        std::vector<cv::Point3f> src, dst;
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> inliers;
        float ratio = get_inliers(G1, G2, inliers);
        //information building: 0.3 8
        //i-park: 0.35 6
        if(ratio <= MultiViewGeometry::g_para.room_lcd_min_ratio || 
            inliers.size() < MultiViewGeometry::g_para.room_lcd_min_inlier) 
            ratio = get_inliers(G2, G1, inliers, true);
        if(ratio > MultiViewGeometry::g_para.room_lcd_min_ratio 
        && inliers.size() >= MultiViewGeometry::g_para.room_lcd_min_inlier) {
            RoomCorrespondence output;
            output.source_id = G1.scene_id;
            output.target_id = G2.scene_id;
            if(output.source_id == -1)
            output.source_id = sorted_idx[j];
            output.source_camera_id = G1.camera_id;
            output.target_camera_id = G2.camera_id;
            std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> &correspondences 
                = output.instance_center_correspondences;
                correspondences = inliers;
/*
            for(size_t i = 0; i < inliers.size(); i++) {
                if(inliers[i] == 1) {
#if 1
                    correspondences.push_back(
                            std::make_pair( 
                                    cv_point_to_eigen(src[i]),
                                    cv_point_to_eigen(dst[i])
                                    ));
#else
                    correspondences.push_back(
                            std::make_pair( 
                                    cv_point_to_eigen(dst[i]),
                                    cv_point_to_eigen(src[i])

                                    ));
#endif

                }
            }
                //std::cout<<output.transformation<<std::endl;
            DBGVAR(std::cout, correspondences.size());
            DBGVAR(std::cout, ratio);
*/
            //if(correspondences.size() >= 8)
            outputs.push_back(output);
            //return output;
        }
    }
    timer.Tock("Ransac Geometry Check");
    timer.LogAll();
    // Step 3: RANSAC eliminate outliers
  //return nooutput;
}

void SceneDatabase::query_with_matches(ObjGraph& G2, 
    std::vector<std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>>> &graph_matching, 
    std::vector<std::vector<int>> &inlier_matches) {
    // if empty
    if (obj_database.size() == 0) {
        return ;
    }

    // Step1: histogram comparison and pick N nearest candidates
    // Compare histograms thru exhaustive search right now
    std::vector<double> difference_list(obj_database.size());
    for(size_t i = 0; i < obj_database.size(); i++) {
        ObjGraph& G1 = obj_database[i];
        VectorHistgram diff = G1.histogram - G2.histogram;
        difference_list[i] = diff.squaredNorm();
        //std::cout<<difference_list[i]<<std::endl;
    }

    auto sorted_idx = sort_indexes(difference_list);
    //DBGVAR(std::cout, sorted_idx);

    // Step 2: graph matching
    int n_nearest = MultiViewGeometry::g_para.room_lcd_n_nearest;
    
    graph_matching.clear();
    inlier_matches.clear();
    
    for (int j = 0; j < n_nearest && j < obj_database.size(); j++) {
        std::cout << "Matching: " << obj_database[sorted_idx[j]].camera_id <<" "<<obj_database[sorted_idx[j]].scene_id << std::endl;
        ObjGraph &G1 = obj_database[sorted_idx[j]];
        std::vector<cv::Point3f> src, dst;
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> tmp_graph_matching;
        std::vector<int> tmp_inlier_matches;
        float ratio = get_inliers(G1, G2, tmp_graph_matching, tmp_inlier_matches);
        //information building: 0.3 8
        //i-park: 0.35 6
        if(ratio <= MultiViewGeometry::g_para.room_lcd_min_ratio || 
            tmp_inlier_matches.size() < MultiViewGeometry::g_para.room_lcd_min_inlier) 
            ratio = get_inliers(G2, G1, tmp_graph_matching, tmp_inlier_matches, true);
        if(ratio > MultiViewGeometry::g_para.room_lcd_min_ratio 
        && tmp_inlier_matches.size() >= MultiViewGeometry::g_para.room_lcd_min_inlier) {
            graph_matching.push_back(tmp_graph_matching);
            inlier_matches.push_back(tmp_inlier_matches);
            //return output;
        }
    }

    // Step 3: RANSAC eliminate outliers
  //return nooutput;
}
// Main API
ObjGraph get_graph(chisel::PointCloud &pcd, int scene_id, int camera_id) {
    // Bridge PointCloud and Tensor
    size_t N = pcd.vertices.size();
    torch::Tensor xyz = torch::empty({N ,3}, torch::dtype(torch::kFloat32));
    // torch::Tensor features = torch::empty({N ,3}, torch::dtype(torch::kFloat32));
    torch::Tensor pred_semantic = torch::empty({N}, torch::dtype(torch::kInt32));
    torch::Tensor pred_instance = torch::empty({N}, torch::dtype(torch::kInt32));
    float *coords_ptr = xyz.data<float>();
    int *pred_semantic_ptr = pred_semantic.data<int>();
    int *pred_instance_ptr = pred_instance.data<int>();
    //std::cout<<N<<std::endl;
    int problem_counter = 0;
    for (size_t i = 0; i < N; i++) {
        coords_ptr[i*3] = pcd.vertices[i][0];
        coords_ptr[i*3+1] = pcd.vertices[i][1];
        coords_ptr[i*3+2] = pcd.vertices[i][2];
        pred_semantic_ptr[i] = get_semantic_label(pcd.labels[i]);

        // assert(pcd.labels[i] / 10000 >= 0);

            // pcd.colors[i][0] = 0;
            // pcd.colors[i][1] = 1.0;
            // pcd.colors[i][2] = 0;

        // assert(pcd.labels[i] % 10000 > 1000);
        pred_instance_ptr[i] = get_instance_label(pcd.labels[i]);
    }

    //DBGVAR(std::cout, problem_counter);
    
    // torch::Tensor xyz = inputs[0]; //float
    // torch::Tensor pred_semantic = inputs[1]; // int
    // torch::Tensor pred_instance = inputs[2]; // int
    // get each instance and compute the center of it
    int max_id = pred_instance.max().item<int>();
    size_t class_num = 20;
    // size_t max_label = pred_semantic.max().item<long>();
    VectorHistgram histogram;
    histogram.setZero(class_num);
    std::vector<float> vertices;
    std::vector<int> labels;
    //DBGVAR(std::cout, max_id);
    //std::cout<<max_id<<std::endl;
    assert(max_id > 0);
    for(size_t i = 0; i <= max_id; i++)  {
        auto instance_mask = (pred_instance == (long)i).view({-1,1});
        auto instance_size = instance_mask.sum(0).item<long>();
        if (instance_size < MIN_INSTANCE_SIZE)  {
            continue;
        }
        auto instance_coords = 
                torch::masked_select(xyz, instance_mask.repeat({1,3})).view({-1,3});
        auto instance_labels = 
                torch::masked_select(pred_semantic, instance_mask.view({-1}));
        // Get the object id
        int label = std::get<0>(at::mode(instance_labels)).item<int>();
        if(label == 0 || label == 1) {
            continue;
        }
        auto center = at::mean(instance_coords, 0);
        vertices.insert(vertices.end(), 
            {center[0].item<float>(), center[1].item<float>(), center[2].item<float>()});
        labels.push_back(label);
        histogram[label] += 1;
        // DBGVAR(std::cout, i);
        // DBGVAR(std::cout, center);
        // DBGVAR(std::cout, label);
        // DBGVAR(std::cout, instance_size);
    }
    //DBGVAR(std::cout, vertices.size());
    //DBGVAR(std::cout, labels.size());
    // DBGVAR(std::cout, histogram);

    return ObjGraph(vertices, labels, histogram, scene_id, camera_id);
}