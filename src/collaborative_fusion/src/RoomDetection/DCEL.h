#ifndef DCEL_H
#define DCEL_H

#include "../Geometry/Geometry.h"

struct Vertex;
struct DCELEdge;
struct Face;
//LineSegment
struct Vertex
{
    int id;//index
    Point2f coor;//coordinate of v
    int inc_eid = -1;//Reference to the first outgoing incident half-edge
    Vertex() = default;
    Vertex(const Point2f &c, int _id = 0)
    {
        coor = c;
        id = _id;
    }
};

struct DCELEdge//Half DCELEdge
{
    int id;//index
    int twin_eid = -1;// Another twin 
    int left_fid = -1;//reference to left incident face;
    int origin_vid = -1;// origin vertex
    int des_vid = -1;//destination vertex
    int pred_eid = -1;// previous half edge
    int succ_eid = -1;// next half edge
    int line_id = -1;//$(line_id) line create this edge
    float weight = -1;
    DCELEdge() = default;
    DCELEdge(int _id)
    {
        id = _id;
    }
};

struct Face
{
    int id;//index
    int inc_eid = -1;//Reference to the first incident half-edge
    Face()=default;
    Face(int _id)
    {
        id = _id;
    }
};


class DCEL
{
    public:
    //Initial With Bounding Box
    void InitialWithBB(const Point2f & _left_bottom, const Point2f & _right_bottom,
        const Point2f &_right_up, const Point2f &_left_up);
    int AddDCELEdge(int vid1, int vid2)
    {
        if(vid1 == vid2)
        return -1;
        DCELEdge e1;
        e1.id = edges.size();

        e1.origin_vid = vid1;
        e1.des_vid = vid2;

        if(vertexs[vid1].inc_eid == -1)
            vertexs[vid1].inc_eid = e1.id;
        edges.push_back(e1);
        return e1.id;
    }
    void MakeTwin(int eid1, int eid2)
    {
        if(eid1 == eid2) return;
        edges[eid1].twin_eid = eid2;
        edges[eid2].twin_eid = eid1;
    }

    void ConnectDCELEdge(int eid1, int eid2)
    {
        if(eid1 == eid2) return;
        edges[eid1].succ_eid = eid2;

        edges[eid2].pred_eid = eid1;

        edges[eid2].left_fid = edges[eid1].left_fid;
    }
    cv::Mat Draw(const Point2f &pos= Point2f(), const Point3fList &points = Point3fList());
    int GetFaceID(const Point2f &point);    
    LineSegment DCELEdgeSegment(int eid)
    {
        return LineSegment(vertexs[edges[eid].origin_vid].coor, vertexs[edges[eid].des_vid].coor);
    }
    void IncrementLine(const Line &line);

    void Reset()
    {
        vertexs.clear();
        edges.clear();
        faces.clear();
        lines_number = 0;
        left_up = Point2f();
        left_bottom = Point2f();
        right_up = Point2f();
        right_bottom = Point2f();
    }
    float GetFaceArea(int fid);
    std::vector<Vertex> vertexs;
    std::vector<DCELEdge> edges;
    std::vector<Face> faces;
    Eigen::Vector2f left_up, left_bottom, right_up, right_bottom;
    int lines_number = 0;

};
#endif