#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>

#include "../Tools/ConsoleColor.h"
#define DEBUG_MODE 1
#define EPS 1e-6
namespace Eigen {

/// Extending Eigen namespace by adding frequently used matrix type
typedef Eigen::Matrix<float, 6, 6> Matrix6f;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
//typedef Eigen::Matrix<float, 2, 1> Vector2f;
}
//namespace geometry { 
    typedef Eigen::Matrix4f TransformationMatrix;
    typedef Eigen::Vector3f Point3f;
    typedef Eigen::Vector2f Point2f;
    typedef Eigen::Matrix<int, 2, 1> Point2i;
    typedef std::pair<Point3f, Point3f> PointCorrespondence;
    typedef std::vector<PointCorrespondence> PointCorrespondenceSet;
    typedef std::vector<cv::KeyPoint> KeyPointSet;
    typedef std::vector<cv::DMatch> DMatchSet;
    typedef cv::Mat Descriptor;
    typedef std::vector<std::pair<Point2i, Point2i>> PixelCorrespondenceSet;
    typedef std::vector<std::vector<Point3f>> ImageXYZ;
    typedef std::vector<Point2f, Eigen::aligned_allocator<Point2f> > Point2fList;
    typedef std::vector<Point3f, Eigen::aligned_allocator<Point3f> > Point3fList;
    typedef std::vector<Eigen::VectorXf, Eigen::aligned_allocator<Eigen::VectorXf> > PointXfList;
    typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > Point3dList;
    typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > Mat4dList;
    typedef std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > Mat4fList;
    typedef Eigen::Vector3d Point3d;
    typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > Point3iList;
    //lie group and lie algebra

    typedef Eigen::Matrix<float, 6, 1> se3f;
    typedef Eigen::Matrix<float, 3, 1> so3f;
    typedef Eigen::Vector4f Plane;
    typedef Eigen::Vector3f Line3f;
    typedef Eigen::Matrix4f SE3f;
    typedef Eigen::Matrix3f SO3f;
    typedef Eigen::Vector3i CubeID;
    Eigen::Matrix4f TransformVector6fToMatrix4f(const Eigen::Vector6f &input);
    Eigen::Vector6f TransformMatrix4fToVector6f(const Eigen::Matrix4f &input);
    void TransformPoints(const Eigen::Matrix4f &T, std::vector<Point3f> &points);
    void TransformNormals(const Eigen::Matrix4f &T, std::vector<Point3f> &normals);
    struct CubeHasher
    {
            // Three large primes are used for spatial hashing.
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;
            static constexpr size_t p3 = 83492791;

            std::size_t operator()(const CubeID& key) const
            {
                return ( key(0) * p1 ^ key(1) * p2 ^ key(2) * p3);
            }
    };

    Plane GetPlane(const Point3f &p1, const Point3f &p2, const Point3f &p3);
    std::tuple<Eigen::Vector3f, float ,float> FitPlane(const Point3fList & _points);
    std::tuple<Eigen::Vector2f, float ,float> FitLine(const Point2fList & _points);
    
    struct LineSegment;
    struct Line;
    TransformationMatrix ICPTransformation(const  PointCorrespondenceSet & correspondence_set);
    float Cross3(const Point2f &a, const Point2f &b, const Point2f &c);
    bool InSegBounding(const LineSegment &l, const Point2f &p);
    bool InSegBoundingX(const LineSegment &l, const Point2f &p);
    bool InSegBoundingY(const LineSegment &l, const Point2f &p);
    bool IsIntersecting(const LineSegment &l1, const LineSegment &l2);
    bool IsIntersecting(const Line & l1, const LineSegment &l2);
    Line LineFromSeg(const LineSegment &s);
    Point2f LineIntersect(const Line &a, const Line &b);
    Point2f SegIntersect(const LineSegment &s1, const LineSegment &s2);
    Point2f LineSegIntersect(const Line &a, const LineSegment & b);
    float Distance(const Point2f &a, const Point2f &b);
    Point2f ProjectionPointToLine(const Line &a, const Point2f &p);
    Point2f ProjectionPointToLineSegment(const LineSegment &a, const Point2f &p);
    int CheckPointInConvexPoly(const Point2fList &points, const Point2f &p);
    int CheckPointToLine(const Line &line, const Point2f &point);
    int CheckPointToLine(const Point2f &a, const Point2f &b, const Point2f &z );
    int CheckPointInTriangle(const Point2f & a, const Point2f &b, const Point2f & c, const Point2f &p);
    bool CompareTwoLine(const Line3f &a, const Line3f &b);
    bool CompareTwoPlane(const Plane &a, const Plane &b);
    float ComputeAreaTriangle(Point2f a, Point2f b, Point2f c);
    float ComputeAreaConvexPoly(const Point2fList &points);
    struct LineSegment
    {
        Point2f p0, p1;
        LineSegment(){}
        LineSegment(Point2f _p0, Point2f _p1) : p0(_p0), p1(_p1) {}
        float Length()
        {
            return Distance(p0, p1);
        }
    };

    struct Line
    {
        Eigen::Vector2f n;
        float d;
        Line() = default;
        Line(float a, float b, float c)
        {
            n(0) = a;
            n(1) = b;
            d = c;
        }
        Line(const Eigen::Vector2f &_n, float _d)
        {
            n = _n;
            d = _d;
        }
        Line(const Eigen::Vector3f &l)
        {
            n = l.head<2>();
            d = l(2);
        }
    };
    
//};
#endif