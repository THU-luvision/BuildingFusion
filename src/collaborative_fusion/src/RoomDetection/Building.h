#ifndef BUILDING_H
#define BUILDING_H

#include "Arrangements.h"
#include "Clustering.h"
#include "PatchDetection.h"
#include "DrawImage.h"
#include <algorithm>
class Building
{
    public:
    std::shared_ptr<DCEL> dcel;
    std::vector<LinePatch> lines;
    Eigen::MatrixXf affinity_matrix;
    Eigen::MatrixXf diffusion_matrix;
    Eigen::MatrixXf distance_matrix;
    std::vector<Eigen::VectorXf> embeddings;//embeddings for each face
    std::vector<std::vector<int>> rooms;
    std::vector<int> face_to_room;
    std::set<int> unlabeled_face;
    float t = 40;//diffusion time
    int m = 40;//length of embedding
    void Reset()
    {
        dcel = nullptr;
        lines.clear();
        embeddings.clear();
        rooms.clear();
    }
    void SetEmbeddingDimension(int _d = 0)
    {
        if(_d == 0)
        {
            int f_n = dcel->faces.size();
            m = std::min(f_n, 80);
        }
        else m = _d;
    }
    void ComputeWeightsForEachEdge()
    {
        for(int i = 0; i!=dcel->edges.size(); ++i)
        {
            if(dcel->edges[i].weight != -1 || dcel->edges[i].line_id == -1)
                continue;
            int l_id = dcel->edges[i].line_id;
            LinePatch &line = lines[l_id];
            LineSegment ls = dcel->DCELEdgeSegment(i);
             
            int points_on_line = 0;
            /*
            for(int j = 0; j!= line.items.size(); ++j)
            {
                Eigen::Vector2f &inlier_point = line.items[j];
                Eigen::Vector2f pro_point = 
                    ProjectionPointToLineSegment(ls, inlier_point);
                if(InSegBounding(ls, pro_point ))
                {
                    ++points_on_line;
                }
            }
            dcel->edges[i].weight = (float)points_on_line / ls.Length();
            std::cout<<points_on_line<<" "<<ls.Length()<<std::endl;
            std::cout<<"weights: "<<dcel->edges[i].weight<<std::endl;
            if(dcel->edges[i].weight < 200) dcel->edges[i].weight = 0;
            dcel->edges[dcel->edges[i].twin_eid].weight = dcel->edges[i].weight;
            */

            int main_axis = 0;//0 for x, 1 for y
            if(std::fabs(line.rep[0]) > std::fabs(line.rep[1])) main_axis = 1;
            Eigen::Vector2f min_p = Eigen::Vector2f(1e6, 1e6);
            Eigen::Vector2f max_p = Eigen::Vector2f(-1e6, -1e6);
            for(int j = 0; j!= line.items.size(); ++j)
            {
                Eigen::Vector2f &inlier_point = line.items[j];
                Eigen::Vector2f pro_point = 
                    ProjectionPointToLineSegment(ls, inlier_point);
                
                if(InSegBounding(ls, pro_point ))
                {
                    ++points_on_line;
                    if(pro_point[main_axis] > max_p[main_axis])
                    max_p = pro_point;
                    if(pro_point[main_axis] < min_p[main_axis])
                    min_p = pro_point;
                }
            }
            if(points_on_line < 100) 
            {
                dcel->edges[i].weight = 0;
            }
            else
            {
                //std::cout<<max_p<<" "<<min_p<< std::endl;
                float len_ratio = (max_p - min_p).norm() / ls.Length();
                if(len_ratio > 1) std::cout<<"Something wrong about line segment."<<std::endl;
                if(len_ratio <= 0.4) dcel->edges[i].weight = 0;
                else
                {
                    dcel->edges[i].weight = len_ratio * points_on_line;
                    dcel->edges[dcel->edges[i].twin_eid].weight = dcel->edges[i].weight;
                    std::cout<<len_ratio<<" points on line: "<<points_on_line<<" weight: "<<dcel->edges[i].weight<<std::endl;
                    std::cout<<"Line: "<<line.rep[0]<<" "<<line.rep[1]<<" "<<line.rep[2]<<std::endl;
                    std::cout<<"------------------------------------------------------------------------------"<<std::endl;
                }
            }
        }
        /*
        int start_eid = dcel->faces[5].inc_eid;
        int eid = start_eid;
        std::cout<<"weights: "<<dcel->edges[eid].weight <<" "<<dcel->edges[dcel->edges[eid].twin_eid].left_fid<<std::endl;
        eid = dcel->edges[eid].succ_eid;
        while(eid != start_eid)
        {
            std::cout<<"weights: "<<dcel->edges[eid].weight<<" "<<dcel->edges[dcel->edges[eid].twin_eid].left_fid<<std::endl;   
            eid = dcel->edges[eid].succ_eid;         
        }*/
    }

    void ComputeEmbedding()
    {
        affinity_matrix.resize(dcel->faces.size() + 1, dcel->faces.size()+1);
        affinity_matrix.setZero();
        int infty_id = dcel->faces.size();

        for(int i = 0; i!=dcel->faces.size(); ++i)
        {
            affinity_matrix(i, i) = 1;
            int eid = dcel->faces[i].inc_eid;
            int start_eid = eid;
            int fid = dcel->edges[dcel->edges[eid].twin_eid].left_fid;
            float weight = dcel->edges[eid].weight;
            if(fid != -1)
                affinity_matrix(i,fid) = std::exp(-weight);
            else
            {
                affinity_matrix(i, infty_id) = 1;
                affinity_matrix(infty_id, i) = 1;
            }
            eid = dcel->edges[eid].succ_eid;
            while(eid != start_eid)
            {
                fid = dcel->edges[dcel->edges[eid].twin_eid].left_fid;  
                weight = dcel->edges[eid].weight;
                // double free or corruption! Looking for this bug spend me 1 hour.
                if(fid != -1)
                    affinity_matrix(i, fid) = std::exp(-weight);
                else
                {
                    affinity_matrix(i, infty_id) = 1;
                    affinity_matrix(infty_id, i) = 1;
                }
                eid = dcel->edges[eid].succ_eid;            
            }
        }
        affinity_matrix(infty_id, infty_id) = 1;
        
        Eigen::MatrixXf D;// = Eigen::Matrix<float, dcel->faces.size(), dcel->faces.size()>();
        D.resize(dcel->faces.size()+1, dcel->faces.size()+1);
        D.setZero();
        for(int i = 0; i != dcel->faces.size() + 1 ; ++i)
        {
            D(i,i) = affinity_matrix.block(i,0,1, dcel->faces.size() + 1).sum();
        }
        diffusion_matrix = D.inverse() * affinity_matrix;
        //std::cout<<diffusion_matrix<<std::endl<<std::endl;

        Eigen::EigenSolver<Eigen::MatrixXf> eig(diffusion_matrix);       
        Eigen::MatrixXf eigenvalue = eig.eigenvalues().real().asDiagonal().toDenseMatrix();                
        Eigen::MatrixXf eigenvector = eig.eigenvectors().real();
        //std::cout<<eig.eigenvalues()<<std::endl<<std::endl;
        //std::cout<<eigenvalue<<std::endl<<std::endl;

        //std::cout<< eigenvector * eigenvalue * eigenvector.inverse()<<std::endl;
        
        for(int i = 0; i!=dcel->faces.size() + 1; ++i)
            eigenvalue(i,i) = std::pow(eigenvalue(i,i),t);       
    
        Eigen::MatrixXf help = eigenvalue * eigenvector.transpose();
        for(int i = 0; i!=dcel->faces.size() + 1; ++i)
        {
            auto tmp = help.block(0,i, m, 1);
            embeddings.push_back(tmp);
            //std::cout<<"embedding "<<i<<": "<<embeddings[i].transpose()<<std::endl;
        }

        distance_matrix.resize(dcel->faces.size() + 1, dcel->faces.size() + 1);

        for(int i = 0; i!= dcel->faces.size()+ 1; ++i)
        {
            for(int j = 0; j!= dcel->faces.size() + 1; ++j)
            distance_matrix(i,j) = (embeddings[i] - embeddings[j]).norm();
        }
        //std::cout<<distance_matrix<<std::endl;
        cv::Mat img_matrix = visualization::MatrixImage(distance_matrix);
        //cv::imwrite("./distance_matrix.png",img_matrix);
    }
    void ResetUnlabeledFace()
    {
        unlabeled_face.clear();
        for(int i = 0; i != dcel->faces.size()+1; ++i)
        unlabeled_face.insert(i);
    }

    bool SeparateRoomOnce(int room_id, std::vector<int> &room)
    {
        PointXfList wait_to_clustering;
        std::vector<ClusterDynamic> cluster_result;
        int max_index = -1;
        int infty_id = dcel->faces.size();
        std::vector<int> unlabeled_face_v(unlabeled_face.begin(), unlabeled_face.end());
            
        float max_distance = std::numeric_limits<float>::lowest ();
        for(int j = 0; j != unlabeled_face_v.size(); ++j)
        {
            float distance = distance_matrix(infty_id,unlabeled_face_v[j]);
            if( distance > max_distance)
            {
                max_index = j;
                max_distance = distance; 
            }
            //std::cout<<"distance: "<<distance<<std::endl;
            wait_to_clustering.push_back(embeddings[unlabeled_face_v[j]]);
        }
        std::cout<<"max_distance: "<<max_distance<<std::endl;
        if(max_distance < 0.0001)
        {
            std::cout<<"No need to clustter."<<std::endl;
            return false;
        }
        std::vector<int> initial_index;
        initial_index.push_back(max_index);
        initial_index.push_back(unlabeled_face_v.size()-1);
        std::cout<<"Begin to cluster..."<<std::endl;
        KMedoidsClusteringDynamic(wait_to_clustering, cluster_result, 2, true, initial_index);
        std::cout<<"Finish clustering."<<std::endl;
        
        room.clear();
        std::cout<<"calculating faces of room: "<<std::endl;
        for(int j = 0; j != cluster_result[0].indexs.size(); ++j)
        {
            //std::cout<<cluster_result[0].indexs[j]<<std::endl;
            unlabeled_face.erase(unlabeled_face_v[cluster_result[0].indexs[j]]);
            room.push_back(unlabeled_face_v[cluster_result[0].indexs[j]]);
            face_to_room[unlabeled_face_v[cluster_result[0].indexs[j]]] = room_id;
        }
        std::cout<<"contained face: "<<std::endl;
        for(int i = 0; i != room.size(); ++i)
        {
            std::cout<<" "<<room[i];
        }
        
        return true;
    }
    bool IsInRoom(const Point3f &point, int room_id)
    {
        std::vector<int> &room_to_face = rooms[room_id];
        int face_id = dcel->GetFaceID(point.block<2,1>(0,0));
        if(std::find(room_to_face.begin(), room_to_face.end(), face_id) == room_to_face.end())
        return false;
        
        return true;
    }
    void SeparateRoomUsingCameraCoord(const Point3fList &coords, std::vector<std::vector<int>> & room_to_submap,
    const std::set<int> &unlabeled_submap, std::vector<float> &room_area_s)
    {
        std::map<int, std::vector<int>> face_to_coor;
        face_to_room.resize(dcel->faces.size());
        //coor is the center camera pose of submap
        int current_submap_id = *(unlabeled_submap.rbegin());
        for(int i = 0; i != coords.size()-1; ++i)
        {
            if(unlabeled_submap.find(i) == unlabeled_submap.end() || i >= current_submap_id )
                continue;
            int face_id = dcel->GetFaceID(coords[i].block<2,1>(0,0));
            face_to_coor[face_id].push_back(i);
            std::cout<<i<<" ("<<coords[i](0)<<", "<<coords[i](1)<<")"<<" face id: "<<face_id<<std::endl;
        }
        
        int room_id = 0;
        ResetUnlabeledFace();
        rooms.clear();
        room_to_submap.clear();
        //while(face_to_coor.size())
        if(face_to_coor.size())
        {
            std::cout<<"separating the "<<room_id<<"th room..."<<std::endl;
            rooms.push_back(std::vector<int>());

            bool separate_succeed = SeparateRoomOnce(room_id, rooms[room_id]);
            if(!separate_succeed) 
            {
                rooms.pop_back();
                //break;
            }
            else
            {
                room_to_submap.push_back(std::vector<int>());
                room_area_s.push_back(0);
                //std::cout<<rooms[room_id].size()<<std::endl;
                float room_area = 0;
                for(int i = 0; i != rooms[room_id].size(); ++i)
                {
                    int face_id = rooms[room_id][i];

                    if(face_to_coor.find(face_id) != face_to_coor.end())
                    {
                        room_area += dcel->GetFaceArea(face_id);
                        room_to_submap[room_id].insert(room_to_submap[room_id].end(), 
                        face_to_coor[face_id].begin(), face_to_coor[face_id].end() );
                        face_to_coor.erase(face_id);
                        //std::cout<<i<<std::endl;
                    }
                }
                std::cout<<"ROOM AREA: "<<room_area<<std::endl;
                if(room_area > 200) {
                    room_to_submap.pop_back();
                    room_area_s.pop_back();
                    rooms.pop_back();
                    }
                else
                {
                    std::cout<<"contained submap: ";
                    for(int i = 0; i != room_to_submap[room_id].size(); ++i)
                    std::cout<<" "<<room_to_submap[room_id][i];
                    std::cout<<std::endl;
                    room_area_s[room_id] = room_area;
                    room_id += 1;
                }
            }
        }

        std::cout<<GREEN<<"Finish Room Clustering! "<<room_to_submap.size()<<RESET<<std::endl;
    }
    void SeparateRoom()
    {
        //Use kmedoids to separate room
        //SeparateRoom
        //when there are submaps which are not clustered into a room, we do the separation.
        int room_number = 1;
        ResetUnlabeledFace();
        int infty_id = dcel->faces.size();    
        face_to_room.resize(dcel->faces.size());
    
        for(int i = 0; i != room_number; ++i)
        {
            std::cout<<"separating the "<<i<<"th room..."<<std::endl;
            std::vector<int> unlabeled_face_v(unlabeled_face.begin(), unlabeled_face.end());
            float max_distance = std::numeric_limits<float>::lowest ();
            PointXfList wait_to_clustering;
            std::vector<ClusterDynamic> cluster_result;
            int max_index = -1;
            for(int j = 0; j != unlabeled_face_v.size(); ++j)
            {
                float distance = distance_matrix(infty_id,unlabeled_face_v[j]);
                if( distance > max_distance)
                {
                    max_index = j;
                    max_distance = distance; 
                }
                wait_to_clustering.push_back(embeddings[unlabeled_face_v[j]]);
            }
            std::vector<int> initial_index;
            initial_index.push_back(max_index);
            initial_index.push_back(unlabeled_face_v.size()-1);
            KMedoidsClusteringDynamic(wait_to_clustering, cluster_result, 2, true, initial_index);
            int room_id = rooms.size();
            rooms.push_back(std::vector<int>());
            for(int j = 0; j != cluster_result[0].indexs.size(); ++j)
            {
                //std::cout<<cluster_result[0].indexs[j]<<std::endl;
                unlabeled_face.erase(unlabeled_face_v[cluster_result[0].indexs[j]]);
                rooms.back().push_back(unlabeled_face_v[cluster_result[0].indexs[j]]);
                face_to_room[unlabeled_face_v[cluster_result[0].indexs[j]]] = room_id;
            }
        }
        for(int i = 0; i != rooms.size(); ++i)
        {
            std::cout<<"Room "<<i<<":";
            for(int j = 0; j != rooms[i].size(); ++j)
            std::cout<<" "<<rooms[i][j];
            std::cout<<std::endl;

            for(int j = 0; j != rooms[i].size(); ++j)
            {
                for(int k = 0; k != rooms[i].size(); ++k)
                    std::cout<<distance_matrix(rooms[i][j], rooms[i][k])<<" ";
                    std::cout<<distance_matrix(rooms[i][j], infty_id);
                std::cout<<std::endl;
            }
        }
    }

    cv::Mat RoomImage()
    {
        cv::Mat img (700, 700, CV_8UC3);
        img = cv::Scalar::all(0);

        Point2f max_xy = dcel->right_up - dcel->left_bottom;
        float max_c = std::max(max_xy(0), max_xy(1));
        float scalar = 600 / max_c; 
        for(int i = 0; i != rooms.size(); ++i)
        {
            for(int j = 0; j != rooms[i].size(); ++ j)
            {
                int fid = rooms[i][j];
                std::vector<cv::Point> poly; 
                int start_eid = dcel->faces[fid].inc_eid;
                int start_vid = dcel->edges[start_eid].origin_vid;
                poly.push_back(cv::Point2i((dcel->vertexs[start_vid].coor(0) - dcel->left_bottom(0) ) *scalar, 
                    (dcel->vertexs[start_vid].coor(1)-dcel->left_bottom(1))* scalar));
                start_eid = dcel->edges[start_eid].succ_eid;
                start_vid = dcel->edges[start_eid].origin_vid;
                while(start_eid != dcel->faces[fid].inc_eid)
                {

                    poly.push_back(cv::Point2i((dcel->vertexs[start_vid].coor(0) - dcel->left_bottom(0) ) *scalar, 
                        (dcel->vertexs[start_vid].coor(1)-dcel->left_bottom(1))* scalar));
                    start_eid = dcel->edges[start_eid].succ_eid;
                    start_vid = dcel->edges[start_eid].origin_vid;
                }
                cv::fillConvexPoly(img,&poly[0],poly.size(),visualization::color_tab[i]);                
            }
        }
        return img;
    }
};
#endif