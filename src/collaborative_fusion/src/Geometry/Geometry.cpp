#include "Geometry.h"


    Eigen::Matrix4f TransformVector6fToMatrix4f(const Eigen::Vector6f &input) {
        Eigen::Matrix4f output;
        output.setIdentity();
        output.block<3, 3>(0, 0) =
                (Eigen::AngleAxisf(input(2), Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(input(1), Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(input(0), Eigen::Vector3f::UnitX()))
                        .matrix();
        output.block<3, 1>(0, 3) = input.block<3, 1>(3, 0);
        return output;
    }

    Eigen::Vector6f TransformMatrix4fToVector6f(const Eigen::Matrix4f &input) {
        Eigen::Vector6f output;
        Eigen::Matrix3f R = input.block<3, 3>(0, 0);
        float sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
        if (!(sy < 1e-6)) {
            output(0) = atan2(R(2, 1), R(2, 2));
            output(1) = atan2(-R(2, 0), sy);
            output(2) = atan2(R(1, 0), R(0, 0));
        } else {
            output(0) = atan2(-R(1, 2), R(1, 1));
            output(1) = atan2(-R(2, 0), sy);
            output(2) = 0;
        }
        output.block<3, 1>(3, 0) = input.block<3, 1>(0, 3);
        return output;
    }
    void TransformPoints(const Eigen::Matrix4f &T, std::vector<Point3f> &points)
    {
        for (auto& point : points) 
        {
            Eigen::Vector4f new_point =
                T *Eigen::Vector4f(point(0), point(1), point(2), 1.0);
            point = new_point.head<3>() / new_point(3);
        }
    }
    void TransformNormals(const Eigen::Matrix4f &T, std::vector<Point3f> &normals)
    {
        for (auto& normal : normals) 
        {
            Eigen::Vector4f new_normal =
                T *Eigen::Vector4f(normal(0), normal(1), normal(2), 0.0);
            normal = new_normal.head<3>() / new_normal(3);
        }
    }
    Plane GetPlane(const Point3f &p1, const Point3f &p2, const Point3f &p3)
    {
        Point3f normal = (p2 - p1).cross(p3 - p1);
        normal.normalize();
        float d = -p1.dot(normal);
        return Plane(normal(0), normal(1), normal(2), d);
    }
    bool CompareTwoLine(const Line3f &a, const Line3f &b)
    {
        float production = a.block<2,1>(0,0).transpose() * b.block<2,1>(0,0);
        if(std::fabs(production) < 0.75)
        return false;
/*
        float bias = a(2);
        if(production < 0) bias = -bias;
        if(std::fabs(bias - b(2)) > 0.15)
        return false;
*/

        float xa = -a(2) / a(0);
        float xb = -b(2) / b(0);

        float ya = -a(2) / a(1);
        float yb = -b(2) / b(1);
        float edge1 = std::fabs(xa - xb), edge2 = std::fabs(ya - yb);
        float dist =  edge1 * edge2 / std::sqrt(edge1*edge1 + edge2*edge2);
        if(dist > 0.3) return false;
        return true; 
    }
    TransformationMatrix ICPTransformation(const  PointCorrespondenceSet & correspondence_set)
    {
        //Eigen::MatrixXf origin_points, target_points;
        TransformationMatrix T;
        //AT=B
        //ICP algorithm, Compute R, then compute t.
        Eigen::Matrix3f W;
        Eigen::Matrix3f R;
        Eigen::Vector3f t;
        Eigen::Vector3f mean_source, mean_target;
        mean_source.setZero();
        mean_target.setZero();
        W.setZero();
        T.setZero();
        
        for(int i = 0;i!=correspondence_set.size();++i)
        {

            mean_source += correspondence_set[i].first;
            mean_target += correspondence_set[i].second;
        }            
        mean_source /= correspondence_set.size();
        mean_target /= correspondence_set.size();
        for(int i = 0;i!=correspondence_set.size();++i)
        {
            W+=  (correspondence_set[i].first - mean_source) * (correspondence_set[i].second - mean_target).transpose();
        }
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto UT = svd.matrixU().transpose();
        auto V = svd.matrixV();
        R = V*UT;
        if (R.determinant() < 0)
        {
          V(0, 2) = -V(0, 2);
          V(1, 2) = -V(1, 2);
          V(2, 2) = -V(2, 2);
          R = V * UT;
        }
        t = mean_target - R * mean_source;
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        T(3,3) = 1;
        return T;
    }
    bool CompareTwoPlane(const Plane &a, const Plane &b)
    {
        /*
        float production = a.block<3, 1>(0,0).transpose() * b.block<3,1>(0,0);
        if(std::fabs(production) < 0.65)
        return false;
        float bias = a(3);
        if(production < 0) bias = -bias;
        if(std::fabs(bias - b(3)) > 0.15)
        return false;
        return true; 
        */
       Line3f la = Line3f(a(0), a(1), a(3));
       Line3f lb = Line3f(b(0), b(1), b(3));
       return CompareTwoLine(la, lb);

    }
    std::tuple<Eigen::Vector3f, float , float> FitPlane(const Point3fList & _points)
    {
        if(_points.size() < 3)
        {
            std::cout<<YELLOW<< "[FitPlane]::[WARNING]::The Number of points is less than 3."<<RESET << std::endl;
            return std::make_tuple(Eigen::Vector3f(0,0,0),0,0);
        }
        Eigen::Vector3f mean_point;
        Eigen::Vector3f sum_point;
        sum_point.setZero();
        for(int i = 0;i < _points.size(); ++i)
        {
            sum_point+=_points[i];
        }
        mean_point = sum_point / _points.size();
        Eigen::Matrix3f W, W_tmp;
        W.setZero();
        for(int i = 0; i!= _points.size(); ++i)
        {
            W += (_points[i] - mean_point)* (_points[i] - mean_point).transpose();
        }
        W = W / _points.size();
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto singular_values = svd.singularValues();
        Eigen::Vector3f normal = U.block<3,1>(0,2); 
        //std::cout<<U<<std::endl;   
        normal.normalize();    

/*
        Eigen::Vector3f temp = (_points[0] - mean_point);
        temp.normalize();
        if(temp.dot(normal) >= 0 )
        normal = -normal;
*/
        float d = - mean_point.transpose() * normal;
        
        /*
        float residual = 0;
        
        for(int i = 0; i!= _points.size(); ++i)
        {
            residual = residual +  std::fabs((_points[i].transpose() * normal)(0) + d );
        }*/
        float indicator = singular_values(2) / singular_values(1);
        return std::make_tuple(normal,d, indicator);
    }

    std::tuple<Eigen::Vector2f, float, float > FitLine(const Point2fList & _points)
    {
        if(_points.size() < 2)
        {
            std::cout<<YELLOW<< "[FitLine]::[WARNING]::The Number of points is less than 2."<<RESET << std::endl;
            return std::make_tuple(Eigen::Vector2f(0,0),0,0);
        }
        Eigen::Vector2f mean_point;
        Eigen::Vector2f sum_point;
        sum_point.setZero();
        for(int i = 0;i < _points.size(); ++i)
        {
            sum_point+=_points[i];
        }
        mean_point = sum_point / _points.size();
        Eigen::Matrix2f W, W_tmp;
        W.setZero();
        for(int i = 0; i!= _points.size(); ++i)
        {
            W += (_points[i] - mean_point)* (_points[i] - mean_point).transpose();
        }
        W = W / _points.size();
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto singular_values = svd.singularValues();
        Eigen::Vector2f normal = U.block<2,1>(0,1); 
        //std::cout<<singular_values(0) << " "<<singular_values(1)<<std::endl;   
        normal.normalize();    
        float d = - mean_point.transpose() * normal;
        /*
        float residual = 0;
        for(int i = 0; i!= _points.size(); ++i)
        {
            
            residual = residual +  std::fabs((_points[i].transpose() * normal)(0) + d );
        }
        residual /= _points.size();
        */
        //indicator of the curve, noise, and residual.
        double indicator = singular_values(1)/singular_values(0);
        //std::cout<<residual<<" "<<indicator<<std::endl;
        return std::make_tuple(normal,d, indicator);
    }
    float Cross3(const Point2f &a, const Point2f &b, const Point2f &c)
    { 
        return (c(0) - b(0)) * (a(1) - b(1)) - (c(1) - b(1)) * (a(0) - b(0));
    }
    bool InSegBoundingX(const LineSegment &l, const Point2f &p)
    {
        if (std::min(l.p0(0), l.p1(0)) <= p(0) && p(0) <= std::max(l.p0(0), l.p1(0)))
            return 1;
        return 0;        
    }
    bool InSegBoundingY(const LineSegment &l, const Point2f &p)
    {
        if (std::min(l.p0(1), l.p1(1)) <= p(1) && p(1) <= std::max(l.p0(1), l.p1(1)))
            return 1;
        return 0;        
    }
    bool InSegBounding(const LineSegment &l, const Point2f &p)
    {
        if (std::min(l.p0(0), l.p1(0)) <= p(0) && p(0) <= std::max(l.p0(0), l.p1(0)) 
            && std::min(l.p0(1), l.p1(1)) <= p(1) && p(1) <= std::max(l.p0(1), l.p1(1)))
            return 1;
        return 0;
    }
    bool IsIntersecting(const Line & l1, const LineSegment &l2)
    {
        float min_x = std::min(l2.p0(0), l2.p1(0));
        float max_x = std::max(l2.p0(0), l2.p1(0));

        float y1 = (-l1.n(0) * min_x - l1.d) / l1.n(1);
        float y2 = (-l1.n(0) * max_x - l1.d) / l1.n(1);
        
        LineSegment l(Point2f(min_x,y1), Point2f(max_x, y2));

        return IsIntersecting(l,l2);
    }
    bool IsIntersecting(const LineSegment &l1, const LineSegment &l2)
    {
        float d1 = Cross3(l1.p0, l1.p1, l2.p0);
        float d2 = Cross3(l1.p0, l1.p1, l2.p1);
        float d3 = Cross3(l2.p0, l2.p1, l1.p0);
        float d4 = Cross3(l2.p0, l2.p1, l1.p1);
        
        if (((d1 < -EPS && d2 > EPS) || (d1 > EPS && d2 < -EPS)) && ((d3 < -EPS && d4 > EPS) || (d3 > EPS && d4 < -EPS)))
            return 1;
        if (fabs(d1) <= EPS && InSegBounding(l1, l2.p0) || (fabs(d2) <= EPS && InSegBounding(l1, l2.p1)) 
            || (fabs(d3) <= EPS && InSegBounding(l2, l1.p0)) || (fabs(d4) <= EPS && InSegBounding(l2, l1.p1)))
            return 1;
        return 0;
    }

    Line LineFromSeg(const LineSegment &s)
    {
        Line line;
        line.n = Eigen::Vector2f(s.p0(1) - s.p1(1), s.p1(0) - s.p0(0));
        line.n.normalize();
        line.d = - line.n.transpose() * s.p0;
        return line;
    }
    Point2f LineIntersect(const Line &a, const Line &b)
    {
        float x = a.n(1) * b.d - b.n(1) * a.d;
        float y = b.n(0) * a.d - a.n(0) * b.d;
        return Point2f(x,y) / (a.n(0) * b.n(1) - b.n(0) * a.n(1));
    }
    Point2f SegIntersect(const LineSegment &s1, const LineSegment &s2)
    {
        Line l1 = LineFromSeg(s1);
        Line l2 = LineFromSeg(s2);
        return LineIntersect(l1, l2);
    }
    Point2f LineSegIntersect(const Line &a, const LineSegment & b)
    {
        Line l = LineFromSeg(b);
        return LineIntersect(a,l);
    }
    float Distance(const Point2f &a, const Point2f &b)
    {
        return (a - b).norm();
    }
    Point2f ProjectionPointToLine(const Line &a, const Point2f &p)
    {
        float tmp = a.n.transpose() * a.n;
        return  Point2f( (a.n(1) * a.n(1) * p(0) - a.n(0) * a.n(1) * p(1) - a.n(0)* a.d)/ tmp,
         (a.n(0) * a.n(0) * p(1) -a.n(0) * a.n(1) * p(0) - a.n(1) * a.d) / tmp ) ;
    }
    Point2f ProjectionPointToLineSegment(const LineSegment &a, const Point2f &p)
    {
        return ProjectionPointToLine(LineFromSeg(a), p);
    }
    int CheckPointToLine(const Line &line, const Point2f &point)
    {
        float distance = line.n.transpose() * point + line.d;
        if(std::fabs(distance) < EPS) return 0;
        if(distance > 0) return 1;
        if(distance < 0) return -1;
        //if(distance == 0) return 0;
        //return distance; 
    }
    int CheckPointToLine(const Point2f &a, const Point2f &b, const Point2f &z )
    {
        // equal to CheckPointToLine(LineFromSeg(LineSegment(a,b)), z);
        float d =  a(0)* b(1) + a(1) * z(0) + b(0) * z(1) - z(0)* b(1) - a(1)*b(0) - a(0) * z(1);
        //std::cout<<d<<std::endl;
        if(std::fabs(d) < EPS) return 0;
        if(d > 0) return 1;
        if(d < 0) return -1;
    }
    int CheckPointInTriangle(const Point2f & a, const Point2f &b, const Point2f & c, const Point2f &p)
    {
        // Check if a point is in the triangle. 
        // And we consider "the point is on the triangle" as "the point is in the triangle"
        int a_r = CheckPointToLine(a, b, p);
        int b_r = CheckPointToLine(b, c, p);
        int c_r = CheckPointToLine(c, a, p);
        //std::cout<<a_r<<" "<<b_r<<" "<<c_r<<std::endl;
        if(a_r == b_r && b_r == c_r)
        return 1;
        if(a_r == b_r &&  c_r == 0 || a_r == c_r && b_r == 0 || c_r == b_r && a_r == 0)
        return 1;
        if(a_r == b_r && a_r == 0 || a_r == c_r && a_r == 0 || c_r == b_r && c_r == 0)
        return 1;

        return 0;
    }
    float ComputeAreaTriangle(Point2f a, Point2f b, Point2f c)
    {
        return std::fabs(0.5*(a(0) * b(1) + b(0) * c(1) + c(0) * a(1) - 
            a(0) * c(1) - b(0) * a(1) - c(0) * b(1))) ;
    }
    float ComputeAreaConvexPoly(const Point2fList &points)
    {
        int seed = 0; 
        int start = 1;
        int end = points.size();
        float area = 0;
        if(end < 3) 
        {
            std::cout<<YELLOW<<"[ComputeTriangleArea]::[Warning]::Not a triangle."<<RESET<<std::endl;
            return 0;
        }

        for(int i = start; i < end-1; ++i)
        {
            area += ComputeAreaTriangle(points[seed], points[i], points[i + 1]);
        }
        return area;
    }
    int CheckPointInConvexPoly(const Point2fList &points, const Point2f &p )
    {
        // Same as the in-triangle-test
        int seed = 0;
        int start = 1;
        int end = points.size(); 
        int check = (start + end )/2;        
        if(end < 3) 
        {
            std::cout<<YELLOW<<"[CheckPointInConvexPoly]::[Warning]::Not a convex polygon."<<RESET<<std::endl;
            return -1;
        }

        while(true)
        {
            if(end - start == 2)
            {
                return CheckPointInTriangle(points[seed], points[start], points[end-1], p);
            }
            if(end <= start)
            {
                std::cout<<RED<<"[CheckPointInConvexPoly]::[ERROR]::Something wrong."<<RESET<<std::endl;
                //std::cout<<end<<" "<<start<<std::endl;
                return 0;                
            }

            int d = CheckPointToLine(points[seed], points[check], p);
            //CheckPointToLine(points[check],points[seed],p);
            if(d == 1)
            {
                start = check;
            }
            else
            {
                end = check + 1;
            }
            check = (start + end )/2;
        }
        return 0;
    }
