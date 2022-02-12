#include "PatchDetection.h"
#include <algorithm>

bool CompareResidual(const std::pair<int, double> &a, const std::pair<int, double> &b)
{
    return a.second < b.second;
}
bool IsInlier(const Eigen::Vector3f &point, const Eigen::Vector4f &tangnet, 
    const Eigen::Vector4f &plane, float radius)
{
    float dist = std::fabs( point.transpose() * plane.head<3>() + plane(3));
    float normal_prod = std::fabs(tangnet.head<3>().transpose() * plane.head<3>());
    //A another judgement: is visible.
    return dist <= radius/2 && normal_prod >= 0.7; 
}
bool IsInlier(const Eigen::Vector3f &point, const Eigen::Vector3f &normal, 
    const Eigen::Vector4f &plane, float radius)
{
    float dist = std::fabs( point.transpose() * plane.head<3>() + plane(3));
    float normal_prod = normal.transpose() * plane.head<3>();
    //A another judgement: is visible.
    return dist <= radius/1.8 && std::fabs(normal_prod) >= 0.7 ;//&& normal_prod > 0.0; 
}
bool IsInlier(const Eigen::Vector2f &point, const Eigen::Vector3f &tangnet, 
    const Eigen::Vector3f &plane, float radius)
{
    float dist = std::fabs( point.transpose() * plane.head<2>() + plane(2));
    float normal_prod = std::fabs(tangnet.head<2>().transpose() * plane.head<2>());
    //A another judgement: is visible.
    return dist <= radius/2 && normal_prod >= 0.8; 
}
int ChooseSeed(const std::set<int> &un_visited, const std::vector<double> &residuals)
{
    std::vector<std::pair<int, double>> seed_r; 
    for(auto iter = un_visited.begin(); iter != un_visited.end(); ++iter)
    {
        seed_r.push_back(std::make_pair(*iter, residuals[*iter]));
    }
    std::sort(seed_r.begin(), seed_r.end(), CompareResidual);
    return seed_r[0].first;
}
void LineDetection(const Point2fList &points, 
    std::vector<LinePatch> &results)
{
    //ComputeNormalResidual
    //Choose Seed
    //Use Seed itelatively grow patch
    results.clear();
    Point3fList k_lines(points.size());
    std::vector<double> residuals(points.size());      
    std::set<int> un_visited;

    std::vector<cv::Point2f> cv_pcd;

    for(int i = 0; i < points.size(); ++i)
    {
        cv_pcd.push_back(cv::Point2f(points[i](0), points[i](1)));

        un_visited.insert(i);
    } 
    cv::flann::KDTreeIndexParams indexParams; 
    cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);


#if 0 
    int k = 100;
    std::cout<<BLUE<<"[EstimateNormals]::[INFO]::Search "<<k<<" nearest points."<<RESET<<std::endl;
    for(int i = 0; i < cv_pcd.size(); ++i)
    {
        std::vector<float> query={points[i](0), points[i](1)};
        std::vector<int> indices(k); 
        std::vector<float> dists(k); 
        kdtree.knnSearch(query, indices, dists,k,cv::flann::SearchParams(64));

        Point2fList nearest_points;
        for(int j = 0; j!= indices.size();++j)
        {
            nearest_points.push_back(points[indices[j]]);
            //std::cout <<indices[j]<<" "<<std::endl;
        }
        //std::cout <<std::endl;
        auto result = FitLine(nearest_points);
        k_lines[i].block<2,1>(0,0) = std::get<0>(result);
        k_lines[i](2) = std::get<1>(result);
        
        residuals[i] = std::get<2>(result);
        //std::cout<<residuals[i] <<" "<<std::get<2>(result)<<std::endl;
    }
#else 
    float radius = 0.1;
    float squared_radius = radius * radius;
    std::cout<<BLUE<<"[LineDetection]::[INFO]::radius: "<<radius<<RESET<<std::endl;
    for(int i = 0; i < cv_pcd.size(); ++i)
    {
        std::vector<float> query={points[i](0), points[i](1)};
        std::vector<int> indices; 
        std::vector<float> dists; 
        int points_num = kdtree.radiusSearch(query, indices, dists,squared_radius ,1024 ,cv::flann::SearchParams(1024));

        Point2fList radius_points;
        for(int j = 0; j!= points_num;++j)
        {
            radius_points.push_back(points[indices[j]]);
            //std::cout <<indices[j]<<" "<<std::endl;
        }
        //adj_points[i] = indices;
        //std::cout <<std::endl;
        auto result = FitLine(radius_points);

        
        residuals[i] = std::get<2>(result);
        int remain_iter_times = 2;
        while(remain_iter_times > 0)
        {
            Point2fList inliers;
            Eigen::Vector2f n = std::get<0>(result);
            float d  =std::get<1>(result);
            
            for(int i = 0; i!= radius_points.size(); ++i)
            {
                if(std::fabs(n.transpose() * radius_points[i] + d ) <= radius / 2)
                {
                    inliers.push_back(radius_points[i]);
                }
            }
            result = FitLine(inliers);
            radius_points = inliers;
            remain_iter_times -- ;
        }
        k_lines[i].block<2,1>(0,0) = std::get<0>(result);
        k_lines[i](2) = std::get<1>(result);
        //std::cout<<residuals[i] <<" "<<std::get<2>(result)<<std::endl;
    }
#endif
    int iter_th = 0;
    while(un_visited.size() > 0)
    {
        int seed_id = ChooseSeed(un_visited, residuals);
        Eigen::Vector3f line = k_lines[seed_id];
        Point2fList tmp_points;
        std::vector<int > tmp_indexs;
        int last_number = 0;
        int add_number = 1000;
        double threshold = 0.1;
        float indicator;
#if DEBUG_MODE
        std::cout<<BLUE<<"[LineDetection]::[Debug]::"<<iter_th<<"th iteration."<<RESET<<std::endl;
#endif
        iter_th += 1;
        while(add_number >= 100)
        {
        
            std::vector<int> deleted_id;
            
            for(auto iter = un_visited.begin(); iter != un_visited.end(); ++iter)
            {
                int i = *iter;
                //std::cout<<residual<<std::endl;
                if(IsInlier(points[i],k_lines[i], line, radius))
                {
                    tmp_points.push_back(points[i]);
                    tmp_indexs.push_back(i);
                    deleted_id.push_back(i);
                }
            }
            for(int i = 0; i != deleted_id.size(); ++i)
            {
                un_visited.erase(deleted_id[i]);
            }
            un_visited.erase(seed_id);
            add_number = deleted_id.size();
            std::cout<<add_number <<" points are deleted from un_visited_set."<<std::endl;
            // update the line
            if(tmp_points.size() >= 2)
            {
                auto result = FitLine(tmp_points);
                line.block<2,1>(0,0) = std::get<0>(result);
                line(2) = std::get<1>(result);
                indicator = std::get<2>(result);

            }
        }
        std::cout<<BLUE<<"[Debug]::[LineDetection]::The indicator of noise of this line: "
            <<indicator<<RESET<<std::endl;
#if 1
        if(tmp_points.size() > 500 && indicator < 0.1)
        {
            LinePatch result(line);
            result.indexs = tmp_indexs;
            result.items = tmp_points;
            results.push_back(result);
        } 
#else
        LinePatch result(line);
        result.indexs = tmp_indexs;
        result.items = tmp_points;
        results.push_back(result);
#endif
    }

}


void PlaneDetection(const Point3fList &points, 
    std::vector<PlanePatch> &results)
{
    //ComputeNormalResidual
    //Choose Seed
    //Use Seed itelatively grow patch
    results.clear();
    std::vector<Eigen::Vector4f> each_plane(points.size());
    std::vector<double> residuals(points.size());      
    std::vector<std::vector<int>> adj_points;
    std::set<int> un_visited;
    std::vector<cv::Point3f> cv_pcd;
    for(int i = 0; i < points.size(); ++i)
    {
        cv_pcd.push_back(cv::Point3f(points[i](0), points[i](1), points[i](2)));
        un_visited.insert(i);
    } 
    cv::flann::KDTreeIndexParams indexParams; 
    cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);
    //int k = 100;
    float radius = 0.1;
    float squared_radius = radius * radius;
    std::cout<<BLUE<<"[PlaneDetection]::[INFO]::Search "<<radius<<RESET<<std::endl;
    
    for(int i = 0; i < cv_pcd.size(); ++i)
    {
        std::vector<float> query={points[i](0), points[i](1), points[i](2)};
        std::vector<int> indices; 
        std::vector<float> dists; 
        int points_num = kdtree.radiusSearch(query, indices, dists,squared_radius ,1024 ,cv::flann::SearchParams(1024));

        Point3fList radius_points;
        for(int j = 0; j!= points_num;++j)
        {
            radius_points.push_back(points[indices[j]]);
            //std::cout <<indices[j]<<" "<<std::endl;
        }
        //adj_points[i] = indices;
        //std::cout <<std::endl;
        //std::cout<<points_num<<std::endl;
        auto result = FitPlane(radius_points);
        //std::cout<<points_num<<std::endl;
        
        residuals[i] = std::get<2>(result);
        int remain_iter_times = 2;
        while(remain_iter_times > 0)
        {
            Point3fList inliers;
            Eigen::Vector3f n = std::get<0>(result);
            float d  =std::get<1>(result);
            
            for(int i = 0; i!= radius_points.size(); ++i)
            {
                if(std::fabs(n.transpose() * radius_points[i] + d ) <= radius / 2)
                {
                    inliers.push_back(radius_points[i]);
                }
            }
            result = FitPlane(inliers);
            radius_points = inliers;
            remain_iter_times -- ;
        }
        //std::cout<<<<std::endl;
        each_plane[i].block<3,1>(0,0) = std::get<0>(result);
        each_plane[i](3) = std::get<1>(result);
        //std::cout<<residuals[i] <<" "<<std::get<2>(result)<<std::endl;
    }

    int iter_th = 0;
    while(un_visited.size() > 0)
    {
        int seed_id = ChooseSeed(un_visited, residuals);
        Eigen::Vector4f plane = each_plane[seed_id];
        Point3fList tmp_points;
        std::vector<int > tmp_indexs;
        int last_number = 0;
        int add_number = 1000;
        double threshold = 0.1;
        float indicator;
#if DEBUG_MODE
        std::cout<<BLUE<<"[LineDetection]::[Debug]::"<<iter_th<<"th iteration."<<RESET<<std::endl;
#endif
        iter_th += 1;
        /*
        std::queue<int> wait_to_search;
        wait_to_search.push(seed_id);
        un_visited.erase(seed_id);
        while(wait_to_search.size())
        {
            int search_id = wait_to_search.front();
            wait_to_search.pop();
            int out_lier = 0;
            std::set<int> inlier_seed;
            for(int i = 0; i!= adj_points[search_id]; ++i)
            {
                int index = adj_points[search_id][i];
                if(un_visited.find(index) == un_visited.end())
                continue;
                un_visited.erase(index);
                
            }
        }*/
        while(add_number >= 100)
        {
        
            std::vector<int> deleted_id;
            
            for(auto iter = un_visited.begin(); iter != un_visited.end(); ++iter)
            {
                int i = *iter;
                //std::cout<<residual<<std::endl;
                if(IsInlier(points[i], each_plane[i], plane, radius))
                {
                    tmp_points.push_back(points[i]);
                    tmp_indexs.push_back(i);
                    deleted_id.push_back(i);
                }
            }
            for(int i = 0; i != deleted_id.size(); ++i)
            {
                un_visited.erase(deleted_id[i]);
            }
            un_visited.erase(seed_id);
            add_number = deleted_id.size();
            std::cout<<add_number <<" points are deleted from un_visited_set."<<std::endl;
            // update the line
            if(tmp_points.size() >= 3)
            {
                auto result = FitPlane(tmp_points);
                plane.block<3,1>(0,0) = std::get<0>(result);
                plane(3) = std::get<1>(result);
                indicator = std::get<2>(result);
            }
        }
#if 1
        if(tmp_indexs.size() > 500 && indicator < radius)
        {
            PlanePatch result(plane);
            result.indexs = tmp_indexs;
            result.items = tmp_points;
            results.push_back(result);
        } 
#else
        PlanePatch result(plane);
        result.indexs = tmp_indexs;
        result.items = tmp_points;
        results.push_back(result);
#endif
    }
}
void PlaneDetection(const Point3fList &points,  Point3fList & normals,
         std::vector<double> &residuals, std::vector<PlanePatch> &results)
{
    //ComputeNormalResidual
    //Choose Seed
    //Use Seed itelatively grow patch
    results.clear();      
    std::set<int> un_visited;
    std::vector<std::vector<int>> adj_points(points.size());
    std::vector<cv::Point3f> cv_pcd;
    for(int i = 0; i < points.size(); ++i)
    {
        cv_pcd.push_back(cv::Point3f(points[i](0), points[i](1), points[i](2)));
        un_visited.insert(i);
    } 
    cv::flann::KDTreeIndexParams indexParams; 
    cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);
    //int k = 100;
    float radius = 0.2;
    float squared_radius = radius * radius;
    std::cout<<BLUE<<"[PlaneDetection]::[INFO]::Search "<<radius<<RESET<<std::endl;
    
    for(int i = 0; i < cv_pcd.size(); ++i)
    {
        std::vector<float> query={points[i](0), points[i](1), points[i](2)};
        std::vector<int> indices; 
        std::vector<float> dists; 
        int points_num = kdtree.radiusSearch(query, indices, dists,squared_radius ,1024 ,cv::flann::SearchParams(1024));
        indices.resize(points_num);


        adj_points[i] = indices;
        //std::cout <<std::endl;
        //std::cout<<points_num<<std::endl;
        /*
        Point3fList radius_points;
        for(int j = 0; j!= points_num;++j)
        {
            radius_points.push_back(points[indices[j]]);
            //std::cout <<indices[j]<<" "<<std::endl;
        }
        auto result = FitPlane(radius_points);
        //std::cout<<points_num<<std::endl;
        
        residuals[i] = std::get<2>(result);
        int remain_iter_times = 2;
        while(remain_iter_times > 0)
        {
            Point3fList inliers;
            Eigen::Vector3f n = std::get<0>(result);
            float d  =std::get<1>(result);
            
            for(int i = 0; i!= radius_points.size(); ++i)
            {
                if(std::fabs(n.transpose() * radius_points[i] + d ) <= radius / 2)
                {
                    inliers.push_back(radius_points[i]);
                }
            }
            result = FitPlane(inliers);
            radius_points = inliers;
            remain_iter_times -- ;
        }
        //std::cout<<<<std::endl;
        if(normals[i].transpose() * std::get<0>(result) < 0)
        normals[i] = - std::get<0>(result);
        else normals[i] = std::get<0>(result);
        /*
        each_plane[i].block<3,1>(0,0) = std::get<0>(result);
        each_plane[i](3) = std::get<1>(result);
        */
        //std::cout<<residuals[i] <<" "<<std::get<2>(result)<<std::endl;
        
    }

    int iter_th = 0;
    while(un_visited.size() > 0)
    {
        int seed_id = ChooseSeed(un_visited, residuals);
        std::cout<<"Seed residual: "<<residuals[seed_id]<<std::endl;
        std::vector<int> &indices = adj_points[seed_id]; 
        std::vector<float> dists; 
        int points_num = indices.size();
        Point3fList radius_points;
        for(int j = 0; j!= points_num;++j)
        {
            radius_points.push_back(points[indices[j]]);
            //std::cout <<indices[j]<<" "<<std::endl;
        }
        auto result = FitPlane(radius_points);
        Eigen::Vector4f plane;
        plane.block<3,1>(0,0) = std::get<0>(result);
        plane(3) = std::get<1>(result);
        if( plane.block<3,1>(0,0).transpose() * normals[seed_id] < 0 )
            plane = -plane;
        Point3fList tmp_points;
        std::vector<int > tmp_indexs;
        int last_number = 0;
        int add_number = 1000;
        double threshold = 0.1;
        float indicator;
#if DEBUG_MODE
        std::cout<<BLUE<<"[LineDetection]::[Debug]::"<<iter_th<<"th iteration."<<RESET<<std::endl;
#endif
        iter_th += 1;
#if 1
        //using region growing...       
        std::queue<int> wait_to_search;
        wait_to_search.push(seed_id);
        un_visited.erase(seed_id);
        tmp_points.push_back(points[seed_id]);
        tmp_indexs.push_back(seed_id);  
        while(wait_to_search.size())
        {
            int search_id = wait_to_search.front();
            wait_to_search.pop();
            //int out_lier = 0;
            //std::set<int> inlier_seed;
            for(int i = 0; i!= adj_points[search_id].size(); ++i)
            {
                int index = adj_points[search_id][i];
                if(un_visited.find(index) == un_visited.end())
                continue;
                un_visited.erase(index);
                if(IsInlier(points[index], normals[index], plane, radius))
                {
                    wait_to_search.push(index);
                    tmp_points.push_back(points[index]);
                    tmp_indexs.push_back(index);                    
                }
            }
            if(tmp_points.size() >= 3)
            {
                auto result = FitPlane(tmp_points);
                plane.block<3,1>(0,0) = std::get<0>(result);
                plane(3) = std::get<1>(result);
                indicator = std::get<2>(result);
            }
        }
#else
        while(add_number >= 100)
        {
        
            std::vector<int> deleted_id;
            
            for(auto iter = un_visited.begin(); iter != un_visited.end(); ++iter)
            {
                int i = *iter;
                //std::cout<<residual<<std::endl;
                if(IsInlier(points[i], normals[i], plane, radius))
                {
                    tmp_points.push_back(points[i]);
                    tmp_indexs.push_back(i);
                    deleted_id.push_back(i);
                }
            }
            for(int i = 0; i != deleted_id.size(); ++i)
            {
                un_visited.erase(deleted_id[i]);
            }
            un_visited.erase(seed_id);
            add_number = deleted_id.size();
            std::cout<<add_number <<" points are deleted from un_visited_set."<<std::endl;
            // update the line
            if(tmp_points.size() >= 3)
            {
                auto result = FitPlane(tmp_points);
                plane.block<3,1>(0,0) = std::get<0>(result);
                plane(3) = std::get<1>(result);
                indicator = std::get<2>(result);
            }
        }
#endif
#if 1
        std::cout<<tmp_indexs.size()<<std::endl;
        //merge the same plane
        bool merged = false;
        /*
        for(int i = 0; i != results.size(); ++i)
        {
            if(CompareTwoPlane(results[i].rep, plane))
            {
                results[i].indexs.insert(results[i].indexs.end(),tmp_indexs.begin(), tmp_indexs.end());
                results[i].items.insert(results[i].items.end(), tmp_points.begin(), tmp_points.end());
                
                auto result =  FitPlane(results[i].items);
                results[i].rep.block<3,1>(0,0) = std::get<0>(result);
                results[i].rep(3) = std::get<1>(result);
                merged = true;
            }
        }*/
        if(merged == false && tmp_indexs.size() > 400&& indicator < radius)
        {
            PlanePatch result(plane);
            result.indexs = tmp_indexs;
            result.items = tmp_points;
            results.push_back(result);
        } 
#else
        PlanePatch result(plane);
        result.indexs = tmp_indexs;
        result.items = tmp_points;
        results.push_back(result);
#endif
    }
}
