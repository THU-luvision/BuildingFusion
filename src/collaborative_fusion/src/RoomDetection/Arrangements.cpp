#include "Arrangements.h"
void ComputeIntersect(const std::vector<Line> &lines, 
    Point2fList &intersect_points)
{
    intersect_points.clear();
    for(int i = 0; i!= lines.size(); ++i)
    {
        for(int j = i+1; j < lines.size(); ++j)
        {
            if(std::fabs(lines[i].n.transpose() * lines[j].n ) < 0.9)
            {
            Eigen::Vector2f point = LineIntersect(lines[i], lines[j]);
            //std::cout<<point(0) <<" "<<point(1)<<std::endl;
            intersect_points.push_back(point);
            }
        }
    }
}
std::shared_ptr<DCEL> CreateBoundingBoxDcel(const Point2fList &points)
{
    DCEL dcel;
    float max_x = std::numeric_limits<float>::lowest (), 
        min_x = std::numeric_limits<float>::max (),
        max_y = std::numeric_limits<float>::lowest (),
        min_y = std::numeric_limits<float>::max ();
    for(int i = 0; i!=points.size(); ++i)
    {
        if(points[i](0) > max_x) max_x = points[i](0);
        if(points[i](0) < min_x) min_x = points[i](0);
        if(points[i](1) > max_y) max_y = points[i](1);
        if(points[i](1) < min_y) min_y = points[i](1);
    }
    min_x -= 0.1;
    min_y -= 0.1;
    max_x += 0.1;
    max_y += 0.1;
    Point2f left_bottom(min_x,min_y);
    Point2f left_up(min_x,max_y);
    Point2f right_bottom(max_x,min_y);
    Point2f right_up(max_x,max_y);
    dcel.InitialWithBB(left_bottom, right_bottom, right_up, left_up);
#if DEBUG_MODE
    std::cout<<BLUE<<"[CreateBoundingBoxDcel]::[DEBUG]::Corner: \n"<<left_bottom <<"\n\n"<< right_bottom <<
        "\n\n"<< right_up<<"\n\n"<<left_up<<RESET<<std::endl;
#endif
    return std::make_shared<DCEL>(dcel);
}