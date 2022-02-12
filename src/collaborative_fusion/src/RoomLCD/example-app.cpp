#include <torch/torch.h>
#include "utils.h"
#include <iostream>
#include <cstring>
#include <cstdlib>
#include <random>

std::vector<std::string> read_lines_into_list(std::string path1)  {
    std::ifstream file (path1);
    std::vector<std::string> output;
    if (!file) {
        std::cout << "unable to open file; "<< path1 << std::endl;
        return output;
    }
    std::string tmp_string;
    while (getline (file, tmp_string))
    {
        output.push_back(tmp_string);
        // DBGVAR(std::cout, tmp_string);
    }
    return output;
}

int _main(int argc, char **argv)
{
    if (argc != 3) {
        std::cerr << "Wrong Arguments." << std::endl;
    }
    // ./loop database_scene_list.txt query_scene_list.txt
    std::string path1 = argv[1];
    std::string path2 = argv[2];

    auto database_list = read_lines_into_list(path1);
    auto query_list = read_lines_into_list(path2);

    SceneDatabase scene_database;

    for(size_t i = 0; i < database_list.size(); i++) {
        Tensors input_tensors = LoadNpz(database_list[i]);
        scene_database.insert(generate_graph_from_pcd(input_tensors));
    }

    // Simple test 1 retrieval
    Tensors query_tensors = LoadNpz(query_list[0]);
    ObjGraph query = generate_graph_from_pcd(query_tensors);
    //auto some_output = scene_database.query(query);


    // Eigen::Vector3f test1;
    // Eigen::VectorXd test2;
    // Eigen::Vector<float,NUM_CLASS,1> test;

/*
    Tensors scene1 = LoadNpz(path1);
    Tensors scene2 = LoadNpz(path2);
    auto graph1 = generate_graph_from_pcd(scene1);
    auto graph2 = generate_graph_from_pcd(scene2);
    auto assign_matrix = match_graphs(graph1, graph2);
    dump_matrix_to_file(assign_matrix, 
            "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/matrix.npz");

    dump_graph_to_file(graph1, 
            "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/graph1.npz");
    dump_graph_to_file(graph2, 
            "/home/zheng/Desktop/LargeScaleFlashFusion/test_results/graph/graph2.npz");
            
    // auto assign_matrix = match_graphs(test_graph, test_graph);
*/
    /*
    int N = 10;
    std::vector<float> random_vert(N*3);
    std::vector<int> random_label(N, 0);

    std::random_device rd;  //Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<> dis(-1.0, 1.0);

    for(int i = 0; i < N*3; i++)    {
        random_vert[i] = dis(gen);
    }

    ObjGraph a(random_vert, random_label);

    DBGVAR(std::cout, random_vert);
    DBGVAR(std::cout, random_label);
    DBGVAR(std::cout, a.get_size());

    auto assign_matrix = match_graphs(a, a);

    std::cout << assign_matrix << std::endl;
    */
    return 0;
}
