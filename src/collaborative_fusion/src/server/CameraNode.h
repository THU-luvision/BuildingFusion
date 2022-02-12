#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

#include "Correspondence.h"
#include "NormalCorrespondence.h"
#include <set>
namespace Server 
{
class CameraNode
{
    public:
    int node_id = 0;
    std::set<int > camera;
    std::vector<Correspondence> correspondences;
    std::vector<RoomCorrespondence> room_correspondences;
    //std::vector<NormalCorrespondence> normal_correspondences;
    CameraNode()=default;
    CameraNode(int camera_id)
    {
        node_id = camera_id;
        camera.insert(camera_id);
    }
    void merge(CameraNode &another_node)
    {
        std::cout<<"start to merge node "<<node_id<<" "<<another_node.node_id<<" !"<<std::endl;
        camera.insert(another_node.camera.begin(), another_node.camera.end());
        for(int i = 0;i!=another_node.correspondences.size();++i)
        correspondences.push_back(another_node.correspondences[i]);

        room_correspondences.insert(room_correspondences.end(), 
            another_node.room_correspondences.begin(), another_node.room_correspondences.end());
        //correspondences.insert(correspondences.end(), another_node.correspondences.begin(), another_node.correspondences.end());
        //normal_correspondences.insert(normal_correspondences.end(),
        //    another_node.normal_correspondences.begin(), another_node.normal_correspondences.end());
        another_node.clear();
        std::cout<<"finish merging node!"<<std::endl;
        last_final_error = 1.0;
    } 
    void clear()
    {
        camera.clear();
        std::vector<Correspondence>().swap(correspondences);
        //correspondences.clear();
        std::vector<RoomCorrespondence>().swap(room_correspondences);
        //std::vector<NormalCorrespondence>().swap(normal_correspondences);
    }

    double last_final_error=1.0;
};
};

#endif