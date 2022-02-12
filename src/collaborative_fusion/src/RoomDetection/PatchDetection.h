#ifndef PATCH_DETECTION_H
#define PATCH_DETECTION_H
#include "../Geometry/Geometry.h"

template <int T > class Patch
{

    public:
    Patch()=default;
    Patch(const Eigen::Matrix<float,T,1> & c)
    {
        //representation
        rep = c;
    }
    Eigen::Matrix<float,T,1> rep;
    std::vector<Eigen::Matrix<float,T - 1,1>, Eigen::aligned_allocator<Eigen::Matrix<float,T - 1,1>> > items;
    std::vector<int > indexs;
};  

typedef Patch<3> LinePatch;
typedef Patch<4> PlanePatch;


int ChooseSeed(const std::set<int> &un_visited, const std::vector<double> &residuals);
void LineDetection(const Point2fList &points, 
    std::vector<LinePatch> &results);
void PlaneDetection(const Point3fList &points, 
    std::vector<PlanePatch> &Patches);
void PlaneDetection(const Point3fList &points, Point3fList & normals,
    std::vector<double> &residuals, std::vector<PlanePatch> &Patches);
bool IsInlier(const Eigen::Vector3f &point, const Eigen::Vector3f &normals, 
    const Eigen::Vector4f &plane, float radius);
bool IsInlier(const Eigen::Vector3f &point, const Eigen::Vector4f &tangnet, 
    const Eigen::Vector4f &plane, float radius);
bool IsInlier(const Eigen::Vector2f &point, const Eigen::Vector3f &tangnet, 
    const Eigen::Vector3f &plane, float radius);
#endif