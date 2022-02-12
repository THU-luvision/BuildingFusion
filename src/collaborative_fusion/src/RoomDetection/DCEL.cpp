#include "DCEL.h"
#include "DrawImage.h"

cv::Mat DCEL::Draw(const Point2f &pos, const Point3fList &points)
{
    return visualization::DCELImage(*this, pos, points);
}
float DCEL::GetFaceArea(int fid)
{
    Point2fList poly;
    poly.clear();
    int eid = faces[fid].inc_eid;
    int start_eid = eid;
    poly.push_back(vertexs[edges[eid].origin_vid].coor);
    eid = edges[eid].succ_eid;
    while(eid != start_eid)
    {
        poly.push_back(vertexs[edges[eid].origin_vid].coor);  
        eid = edges[eid].succ_eid;              
    }
    return ComputeAreaConvexPoly(poly);
        
}
int DCEL::GetFaceID(const Point2f &point)
{
    // compute the face id where the point is

    int in_bounding = 0;
    Point2fList poly;
    poly.push_back(left_bottom);
    poly.push_back(right_bottom);
    poly.push_back(right_up);
    poly.push_back(left_up);
    in_bounding = CheckPointInConvexPoly(poly, point);
    if(in_bounding == 0)
    {
        std::cout<<"Not in the bounding box!"<<std::endl;
        return -1;
    }
    for(int i = 0; i != faces.size(); ++ i)
    {
        poly.clear();
        int eid = faces[i].inc_eid;
        int start_eid = eid;
        poly.push_back(vertexs[edges[eid].origin_vid].coor);
        eid = edges[eid].succ_eid;
        while(eid != start_eid)
        {
            poly.push_back(vertexs[edges[eid].origin_vid].coor);  
            eid = edges[eid].succ_eid;              
        }
        
        in_bounding = CheckPointInConvexPoly(poly,point);
        if(in_bounding == 1)
        return i;
        
    }
    return -1;

}
void DCEL::InitialWithBB(const Point2f & _left_bottom, const Point2f &_right_bottom,
const Point2f &_right_up, const Point2f &_left_up)
{
    left_bottom = _left_bottom;
    right_bottom = _right_bottom;
    left_up  = _left_up;
    right_up = _right_up;

    Vertex v0(left_bottom, 0), v1(right_bottom, 1), v2(right_up, 2), v3(left_up, 3);
    
    // add vertex
    vertexs.push_back(v0);
    vertexs.push_back(v1);
    vertexs.push_back(v2);
    vertexs.push_back(v3);
    // add edge
    int eid_1, eid_2, eid_3, eid_4, eid_5, eid_6, eid_7, eid_8;


    
    eid_1 = AddDCELEdge(v0.id, v1.id);
    eid_2 = AddDCELEdge(v1.id, v2.id);
    eid_3 = AddDCELEdge(v2.id, v3.id);
    eid_4 = AddDCELEdge(v3.id, v0.id);
    Face f(0);
    f.inc_eid = eid_1;
    faces.push_back(f);
    edges[eid_1].left_fid = f.id;

    ConnectDCELEdge(eid_1, eid_2);
    ConnectDCELEdge(eid_2, eid_3);
    ConnectDCELEdge(eid_3, eid_4);
    ConnectDCELEdge(eid_4, eid_1);

    eid_5 = AddDCELEdge(v1.id, v0.id);
    eid_6 = AddDCELEdge(v2.id, v1.id);
    eid_7 = AddDCELEdge(v3.id, v2.id);
    eid_8 = AddDCELEdge(v0.id, v3.id);

    MakeTwin(eid_1, eid_5);
    MakeTwin(eid_2, eid_6);
    MakeTwin(eid_3, eid_7);
    MakeTwin(eid_4, eid_8);
    
}
//Only compare x, to find the leftmost edge
bool CompareWithDCELEdgeIndex(const std::pair<Eigen::Vector2f, int> & a, const std::pair<Eigen::Vector2f, int> &b)
{
    return a.first(0) < b.first(0);
}
void DCEL::IncrementLine(const Line &line)
{
    std::set<int> edge_ids;
    for(int i = 0; i!=edges.size(); ++i)
    {
        if(edge_ids.find(edges[i].twin_eid) == edge_ids.end())
            edge_ids.insert(edges[i].id);
    }
    std::vector<std::pair<Eigen::Vector2f, int>> inter_points;
    for(auto iter = edge_ids.begin(); iter != edge_ids.end(); ++iter)
    {
        LineSegment line_seg = DCELEdgeSegment(*iter);
        if(IsIntersecting(line, line_seg))
        {
            Point2f point = LineSegIntersect(line, line_seg);
            auto item = std::make_pair(point,*iter);
            inter_points.push_back(item);
        }
    }
    //from left to right
    std::sort(inter_points.begin(), inter_points.end(), CompareWithDCELEdgeIndex);
    int new_eid0;
    int old_eid1;
    int vid1;
    int vid2;
    //std::cout<<"Add "<<inter_points.size()<<" vertexs."<<std::endl;
    for(int i = 0; i != inter_points.size(); ++i)
    {
        //Check if the point is on the edge
        int on_edge = false;
        if(Distance(inter_points[i].first, vertexs[edges[inter_points[i].second].des_vid].coor) < EPS)
        {
            vid2 = edges[inter_points[i].second].des_vid;
            on_edge = 1;
        }
        else if(Distance(inter_points[i].first, vertexs[edges[inter_points[i].second].origin_vid].coor) < EPS)
        {
            vid2 = edges[inter_points[i].second].origin_vid;
            on_edge = 2;
        }
        else
        {
            Vertex v(inter_points[i].first);
            v.id = vertexs.size();
            vertexs.push_back(v);
            vid2 = v.id;
        }

        if(i == 0)
        {
            old_eid1 = inter_points[i].second;
            if(edges[old_eid1].left_fid == -1)
                old_eid1 = edges[old_eid1].twin_eid;
            if(on_edge == 0)
            {
                new_eid0 = AddDCELEdge(vid2, edges[old_eid1].des_vid);
                int new_eid = AddDCELEdge(vid2, edges[old_eid1].origin_vid);
                edges[new_eid0].line_id = edges[old_eid1].line_id;
                edges[new_eid].line_id = edges[old_eid1].line_id;
                MakeTwin(new_eid0, edges[old_eid1].twin_eid);
                MakeTwin(old_eid1, new_eid);
                edges[old_eid1].des_vid = vid2;
                edges[edges[new_eid0].twin_eid].des_vid = vid2;
            }
            else
            {
                new_eid0 = old_eid1;
            }
            vid1 = vid2;
            //std::cout<<old_eid1<<" " <<edges[old_eid1].left_fid<<std::endl; 
            continue;
        }

        Vertex &v1 = vertexs[vid1];
        Vertex &v2 = vertexs[vid2];

        int face_id = edges[old_eid1].left_fid;
        int old_eid2 = inter_points[i].second;


        if(edges[old_eid1].left_fid != edges[old_eid2].left_fid)
        {
            old_eid2 = edges[old_eid2].twin_eid;
        }

        int des2 = edges[old_eid2].des_vid;
        int origin2 = edges[old_eid2].origin_vid;
        int new_eid1, new_eid2, new_eid3, new_eid4;
        if(on_edge == 0)
        {
            new_eid1 = AddDCELEdge(v2.id, des2);
            new_eid2 = AddDCELEdge(v2.id, origin2);
            edges[new_eid1].line_id = edges[old_eid2].line_id;
            edges[new_eid2].line_id = edges[old_eid2].line_id;
        }
        else 
        {
            new_eid1 = old_eid2;
            new_eid2 = edges[old_eid2].twin_eid;
        }
        new_eid3 = AddDCELEdge(v2.id, v1.id);
        new_eid4 = AddDCELEdge(v1.id, v2.id);
        edges[new_eid3].line_id = lines_number;
        edges[new_eid4].line_id = lines_number;
        MakeTwin(new_eid1, edges[old_eid2].twin_eid);  
        MakeTwin(old_eid2, new_eid2);
        MakeTwin(new_eid3, new_eid4);

        if(on_edge == 0)
        {
            edges[old_eid2].des_vid = v2.id; 
            edges[edges[new_eid1].twin_eid].des_vid = v2.id;
        }


        DCELEdge & old_edge1 = edges[old_eid1];
        DCELEdge & old_edge2 = edges[old_eid2];
        


        ConnectDCELEdge(new_eid0, old_edge1.succ_eid);
        ConnectDCELEdge(new_eid1, old_edge2.succ_eid);

        ConnectDCELEdge(old_eid2, new_eid3);
        ConnectDCELEdge(new_eid3, new_eid0);

        ConnectDCELEdge(old_eid1, new_eid4);
        ConnectDCELEdge(new_eid4, new_eid1);

        faces[face_id].inc_eid = old_edge1.id;
        int new_face_id = faces.size();
        Face new_face(new_face_id);

        new_face.inc_eid = old_edge2.id;
        faces.push_back(new_face);
        
        old_edge1.left_fid = face_id;
        old_edge2.left_fid = new_face_id;

        int tmp_eid = old_edge1.succ_eid;
        while(tmp_eid != old_edge1.id)
        {
            edges[tmp_eid].left_fid = face_id;
            tmp_eid = edges[tmp_eid].succ_eid;
        }
        tmp_eid = old_edge2.succ_eid;
        while(tmp_eid != old_edge2.id)
        {
            edges[tmp_eid].left_fid = new_face_id;
            tmp_eid = edges[tmp_eid].succ_eid;
        }
        
        old_eid1 = edges[new_eid1].twin_eid;
        //std::cout<<old_eid1<<" " <<edges[old_eid1].left_fid<<std::endl; 
        new_eid0 = new_eid2;
        vid1 = vid2;
    }
    lines_number += 1;
}
