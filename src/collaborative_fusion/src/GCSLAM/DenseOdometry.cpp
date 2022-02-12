#include "DenseOdometry.h"
#include <Eigen/Cholesky>

    bool DenseEstimation(const cv::Mat &source_color, const cv::Mat &target_color, 
    const cv::Mat &source_depth, const cv::Mat &target_depth, 
    TransformationMatrix &transformation, PointCorrespondenceSet &correspondence_set,bool use_hybrid)
    {
        //Real-time visual odometry from dense RGB-D images.
        cv::Mat source_gray;
        cv::Mat source_refined_depth;
        cv::Mat target_gray;
        cv::Mat target_refined_depth;
        //std::cout<<"Initialize Dense Tracking.."<<std::endl;
        int valid_count = InitializeRGBDDenseTracking(source_color,source_depth,target_color,target_depth,
        source_gray,source_refined_depth, target_gray,target_refined_depth);
#if DEBUG_MODE
        if(use_hybrid)
        std::cout<<BLUE<<"[DEBUG]::Using hybrid term"<<RESET<<std::endl;
        else
        std::cout<<BLUE<<"[DEBUG]::Using photo term"<<RESET<<std::endl;
#endif
        //std::cout<<"Transformation: "<<transformation<<std::endl;
       return MultiScaleComputing(source_gray,target_gray, source_refined_depth, 
        target_refined_depth, transformation,correspondence_set,use_hybrid) / (valid_count+0.0) >= MIN_INLIER_RATIO_DENSE;

    }
    int InitializeRGBDDenseTracking(const cv::Mat &source_color, const cv::Mat &source_depth, 
        const cv::Mat &target_color, const cv::Mat &target_depth,
        cv::Mat &source_gray, cv::Mat &source_refined_depth, cv::Mat &target_gray, cv::Mat &target_refined_depth)
    {
    //frame precessing, including transfer to intensity, and filtering
        cv::Mat source_refined_depth_tmp , target_refined_depth_tmp, source_intensity_tmp, target_intensity_tmp;
        ConvertColorToIntensity32F(source_color, source_intensity_tmp,255.0);
        ConvertColorToIntensity32F(target_color, target_intensity_tmp,255.0);
        int valid_count_ref = ConvertDepthTo32FNaN(source_depth, source_refined_depth_tmp,fc_camera.GetDepthScale());
        int valid_count_new = ConvertDepthTo32FNaN(target_depth, target_refined_depth_tmp,fc_camera.GetDepthScale());
        
        GaussianFiltering(source_intensity_tmp,source_gray);
        GaussianFiltering(target_intensity_tmp,target_gray);
        //source_refined_depth = source_refined_depth_tmp;
        //target_refined_depth = target_refined_depth_tmp;
        GaussianFiltering(source_refined_depth_tmp,source_refined_depth);
        GaussianFiltering(target_refined_depth_tmp,target_refined_depth);
        PixelCorrespondenceSet correspondences;
        
        ComputeCorrespondencePixelWise(source_refined_depth,target_refined_depth,fc_camera,TransformationMatrix::Identity(),correspondences);
        NormalizeIntensity(source_gray,target_gray,correspondences);
        return std::min(valid_count_ref, valid_count_new);
    }
    void AddElementToCorrespondenceMap(cv::Mat &wraping_map,cv::Mat &wraping_depth,
        int u_s, int v_s, int u_t, int v_t, float transformed_d_s)
    {
        float existing_depth = wraping_depth.at<float>(v_t,u_t);
        //std::cout<<existing_depth<<std::endl;
        if(existing_depth == -1)
        {

            wraping_map.at<cv::Vec2i>(v_s,u_s) = cv::Vec2i(v_t,u_t);
            wraping_depth.at<float>(v_s,u_s) = transformed_d_s;
        }
        else
        {

            if(existing_depth > transformed_d_s)
            {
            wraping_map.at<cv::Vec2i>(v_s,u_s) = cv::Vec2i(v_t,u_t);
            wraping_depth.at<float>(v_s,u_s) = transformed_d_s;
            }
        }
    }
    int ConvertDepthTo32FNaN(const cv::Mat &depth, cv::Mat &refined_depth, float depth_scale)
    {
    refined_depth.create(depth.rows, depth.cols, CV_32FC1);
    //std::cout <<depth.depth()<<std::endl;
    int valid_count = 0;
    if(depth.depth() == CV_32FC1)
    {
        for(int i = 0; i < depth.rows * depth.cols; ++i)
        {
            if(depth.at<float>(i) > MIN_DEPTH  && depth.at<float>(i) < MAX_DEPTH )
            {
                refined_depth.at<float>(i) = depth.at<float>(i);
                valid_count ++;
            }
            else 
            refined_depth.at<float>(i) = std::numeric_limits<float>::quiet_NaN();
        }
        //PrintMat(refined_depth);
    }
    else if(depth.depth() == CV_16UC1)
    {
        for(int i = 0; i < depth.rows * depth.cols; ++i)
        {
            if(depth.at<unsigned short>(i) > MIN_DEPTH * depth_scale && depth.at<unsigned short>(i) < MAX_DEPTH * depth_scale)
            {
                refined_depth.at<float>(i) = depth.at<unsigned short>(i) /depth_scale;
                valid_count ++;
            }
            else 
            refined_depth.at<float>(i) = std::numeric_limits<float>::quiet_NaN();
        }
    }
    else
    std::cout <<RED<< "[DenseOdometry]::[ERROR]::Unknown depth image type."<<RESET<<std::endl;
    //PrintMat(refined_depth);
    return valid_count;
    }

    void ConvertColorToIntensity32F(const cv::Mat &color, cv::Mat &intensity, float scale)
    {
        cv::Mat gray;
        Convert2Gray(color,gray);
        intensity.create(gray.rows, gray.cols, CV_32FC1);
        for(int i = 0; i < gray.rows; ++i)
        {
            for(int j = 0; j < gray.cols;++j)
                intensity.at<float>(i,j) = gray.at<unsigned char>(i,j)/scale;
        }
        //PrintMat(intensity);
        
    }
    void ComputeCorrespondencePixelWise(const cv::Mat &source, const cv::Mat &target, const PinholeCamera &fc_camera,
        const TransformationMatrix & relative_pose, PixelCorrespondenceSet &correspondences)
    {


        int width = fc_camera.GetWidth();
        int height = fc_camera.GetHeight();
        float fx = fc_camera.GetFx(), fy = fc_camera.GetFy(), cx = fc_camera.GetCx(), cy = fc_camera.GetCy();
        //std::cout << width << " " << height << std::endl;
        //std::cout<<fx<<fy<<cx<<cy<<std::endl;
        cv::Mat wraping_depth(height,width,CV_32FC1,cv::Scalar(-1)); 
        cv::Mat wraping_map(height,width,CV_32SC2,cv::Scalar(-1));       
        Eigen::Matrix3f K = fc_camera.ToCameraMatrix();
        Eigen::Matrix3f R = relative_pose.block<3,3>(0,0);
        Eigen::Vector3f t = relative_pose.block<3,1>(0,3);
        Eigen::Vector3f Kt = K * t;
        Eigen::Matrix3f K_inv = K.inverse();
        Eigen::Matrix3f KRK_inv = K * R * K_inv;
        //std::cout<<"K: "<<K<<std::endl;

        //std::cout<<"relative_pose: "<<relative_pose<<std::endl;
        for(int i = 0; i!=height; ++i)
        {
            for(int j = 0;j!=width; ++j)
            {
                float d_s = source.at<float>(i, j) ;
                //std::cout<<d_s<<std::endl;
                if(!std::isnan(d_s))
                {
                    Eigen::Vector3f uv_in_s =
                            d_s * KRK_inv * Eigen::Vector3f(j, i, 1.0) + Kt;
                    float transformed_d_s = uv_in_s(2);

                    //round up
                    int u_t = (int)(uv_in_s(0) / transformed_d_s + 0.5);
                    int v_t = (int)(uv_in_s(1) / transformed_d_s + 0.5);

                    if (u_t >= 0 && u_t < width && v_t >= 0 &&
                        v_t < height)
                        {
                            float d_t = target.at<float>(v_t, u_t);
                            //std::cout<<d_t<<" "<<transformed_d_s<<std::endl;
                            if(!std::isnan(d_t) && std::abs(d_t - transformed_d_s)< MAX_DIFF_DEPTH)
                            {
                                AddElementToCorrespondenceMap(wraping_map,wraping_depth,j,i,u_t,v_t,transformed_d_s);
                            }
                        }
                }
            }
        }
        for(int i = 0;i!=height;++i)//v_s
        {
            cv::Vec2i vu_target;
            for(int j = 0;j!=width;++j)
            {
            //std::cout<<i<<" "<<j<<" "<<wraping_depth.at<float>(i,j)<<std::endl;  
                if(wraping_depth.at<float>(i,j) != -1)//u_s
                {

                    vu_target = wraping_map.at<cv::Vec2i>(i,j);
                    correspondences.push_back(std::make_pair(Point2i(i,j), Point2i(vu_target(0), vu_target(1))));
                }
            }
        }
    
    }
    void NormalizeIntensity(cv::Mat &source, cv::Mat & target, const PixelCorrespondenceSet &correspondence)
    {
        float mean_s = 0.0, mean_t = 0.0;
        for (size_t row = 0; row < correspondence.size(); row++) {
            int v_s = correspondence[row].first(0);
            int u_s = correspondence[row].first(1);
            int v_t = correspondence[row].second(0);
            int u_t = correspondence[row].second(1);
            mean_s += source.at<float>(v_s, u_s);
            mean_t += target.at<float>(v_t, u_t);
        }
        mean_s /= (float)correspondence.size();
        mean_t /= (float)correspondence.size();
        LinearTransform(source, 0.5 / mean_s, 0.0);
        LinearTransform(target, 0.5 / mean_t, 0.0);
    }
    void TransformToMatXYZ(const cv::Mat &image, const PinholeCamera &fc_camera, ImageXYZ &imageXYZ)
    {

        int width = image.cols;
        int height = image.rows;
        imageXYZ.resize(height);
        float fx = fc_camera.GetFx(), fy = fc_camera.GetFy(), cx = fc_camera.GetCx(), cy = fc_camera.GetCy();
        for(int i = 0;i!= height; ++i)
        {
            imageXYZ[i].resize(width);
            for(int j = 0; j!=width; ++j)
            {
                float  z;
                z = image.at<float>(i, j) ;
                //if(std::isnan(z) || z < 0 || z > far)
                if (z > 0) 
                {
                    float x = (j - cx) * z / fx;
                    float y =
                            (i - cy) * z / fy;

                    imageXYZ[i][j] = Point3f(x,y,z);
                }
                else imageXYZ[i][j] =Point3f(-1,-1,-1);
            }
        }
        //PointCloud pcd;
        //pcd.LoadFromXYZ(imageXYZ);
    }
    void ComputeJacobianPhotoTerm(
        int row,std::vector<Eigen::Vector6f> &J, std::vector<float> &residual, 
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy, 
        const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, 
        const TransformationMatrix & relative_pose, 
        const PixelCorrespondenceSet &correspondences)
    {
        int width = fc_camera.GetWidth();
        int height = fc_camera.GetHeight();
        float fx = fc_camera.GetFx(), fy = fc_camera.GetFy(), cx = fc_camera.GetCx(), cy = fc_camera.GetCy();
        Eigen::Matrix3f R = relative_pose.block<3,3>(0,0);
        Eigen::Vector3f t = relative_pose.block<3,1>(0,3);

        int v_s = correspondences[row].first(0);
        int u_s = correspondences[row].first(1);
        int v_t = correspondences[row].second(0);
        int u_t = correspondences[row].second(1);
        float diff_photo = target_color.at<float>(v_t, u_t) -
                            source_color.at<float>(v_s, u_s);
        float dIdx = SOBEL_SCALE * (target_color_dx.at<float>(v_t, u_t));
        float dIdy = SOBEL_SCALE * (target_color_dy.at<float>(v_t, u_t));

        Eigen::Vector3f p3d_mat(source_XYZ[v_s][u_s]);
        Eigen::Vector3f p3d_trans = R * p3d_mat + t;

        float invz = 1. / p3d_trans(2);
        float c0 = dIdx * fx * invz;
        float c1 = dIdy * fy * invz;
        float c2 = -(c0 * p3d_trans(0) + c1 * p3d_trans(1)) * invz;

        J.resize(1);
        residual.resize(1);
        J[0](0) =  (-p3d_trans(2) * c1 + p3d_trans(1) * c2);
        J[0](1) =  (p3d_trans(2) * c0 - p3d_trans(0) * c2);
        J[0](2) =  (-p3d_trans(1) * c0 + p3d_trans(0) * c1);
        J[0](3) =  (c0);
        J[0](4) =  (c1);
        J[0](5) =  (c2);
        float r_photo =  diff_photo;
        residual[0] = r_photo;
        //std::cout <<J[0]<<" \n"<< std::endl;
    }
    std::vector<PinholeCamera > CreatePyramidCameras()
    {
        std::vector<PinholeCamera > result;
        for(int i=0;i!=multi_scale_level;++i)
        {
            if(i == 0) result.push_back(fc_camera);
            else 
            result.push_back(result[i-1].GenerateNextPyramid());
        }
        return result;
    }
    size_t MultiScaleComputing(const cv::Mat& source_color, const cv::Mat& target_color, 
    const cv::Mat& source_depth, const cv::Mat& target_depth, TransformationMatrix &T, 
    PointCorrespondenceSet &correspondence_set, bool use_hybrid)
    {

        //compute pyramid
        
        float fx = fc_camera.GetFx(), fy = fc_camera.GetFy(), cx = fc_camera.GetCx(), cy = fc_camera.GetCy();   
        int width = fc_camera.GetWidth(), height = fc_camera.GetHeight();
        std::vector<cv::Mat> source_color_pyramid ,target_color_pyramid, source_depth_pyramid, target_depth_pyramid;
        CreatePyramid(source_color, source_color_pyramid,multi_scale_level);
        CreatePyramid(target_color, target_color_pyramid,multi_scale_level);
        CreatePyramid(source_depth, source_depth_pyramid,multi_scale_level);
        CreatePyramid(target_depth, target_depth_pyramid,multi_scale_level);
        PixelCorrespondenceSet correspondences;
        std::vector<cv::Mat> target_depth_dx_pyramid, target_depth_dy_pyramid,target_color_dx_pyramid, target_color_dy_pyramid;
        if(use_hybrid)
        {
            SobelFiltering(target_depth_pyramid, target_depth_dx_pyramid,'x');
            SobelFiltering(target_depth_pyramid, target_depth_dy_pyramid,'y');
        }
        SobelFiltering(target_color_pyramid, target_color_dx_pyramid,'x');
        SobelFiltering(target_color_pyramid, target_color_dy_pyramid,'y');
        //std::cout <<"T: "<<T<<std::endl;
        std::vector<PinholeCamera > PyramidCamera =  CreatePyramidCameras();
        for(int i = multi_scale_level-1;i>=0;--i)
        {
            //std::cout<<"Scale "<<i<<"..."<<std::endl;
            ImageXYZ source_XYZ;
            //std::cout<<"Getting XYZ:"<<std::endl;
            TransformToMatXYZ(source_depth_pyramid[i], PyramidCamera[i], source_XYZ);
            //std::cout<<"Finish Getting XYZ."<<std::endl;
            //PrintMat(source_depth_pyramid[i]);
            for(int j=0;j!= iter_count_per_level[i]; ++j)
            {
                correspondences.clear();
                if(use_hybrid)
                {
                    //std::cout<<"Start the DosingIteration"<<std::endl;
                    DoSingleIteration(source_color_pyramid[i], source_depth_pyramid[i], target_color_pyramid[i], target_depth_pyramid[i],
                        target_color_dx_pyramid[i], target_depth_dx_pyramid[i],
                        target_color_dy_pyramid[i], target_depth_dy_pyramid[i], source_XYZ, PyramidCamera[i], T, correspondences);
                    //std::cout<<"Finish the DosingIteration"<<std::endl;
                }
                else
                {
                    DoSingleIteration(source_color_pyramid[i], source_depth_pyramid[i], target_color_pyramid[i], target_depth_pyramid[i],
                        target_color_dx_pyramid[i], target_color_dy_pyramid[i], 
                        source_XYZ, PyramidCamera[i], T, correspondences);
                }
                if((float)correspondences.size()/(height * width) > MAX_INLIER_RATIO_DENSE)
                break;
            }
        }
        //std::cout<<"Finish computing..."<<std::endl;
        int step = correspondences.size()/feature_number;
        if(step <=0) step = 1; 
        for(int i = 0;i<correspondences.size(); i+=step)
        {
            int v_s = correspondences[i].first(0);
            int u_s = correspondences[i].first(1);
            int v_t = correspondences[i].second(0);
            int u_t = correspondences[i].second(1);

            float z_s = source_depth.at<float>(v_s,u_s);
            float z_t = target_depth.at<float>(v_t,u_t);
            float x_s = (u_s - cx) *z_s / fx;
            float y_s = (v_s - cy) *z_s / fy;
            float x_t = (u_t - cx) *z_t / fx;
            float y_t = (v_t - cy) *z_t / fy;

            correspondence_set.push_back(std::make_pair(Point3f(x_s,y_s,z_s), Point3f(x_t,y_t,z_t)));
        }
        return correspondences.size();
    }
    void ComputeJacobianHybridTerm(
        int row,std::vector<Eigen::Vector6f> &J, std::vector<float> &residual, 
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, const TransformationMatrix & relative_pose, 
        const PixelCorrespondenceSet &correspondences)
    {
        float sqrt_lamba_dep, sqrt_lambda_img;
        sqrt_lamba_dep = sqrt(LAMBDA_HYBRID_DEPTH);
        sqrt_lambda_img = sqrt(1.0 - LAMBDA_HYBRID_DEPTH);

        int width = fc_camera.GetWidth();
        int height = fc_camera.GetHeight();
        float fx = fc_camera.GetFx(), fy = fc_camera.GetFy(), cx = fc_camera.GetCx(), cy = fc_camera.GetCy();
        Eigen::Matrix3f R = relative_pose.block<3,3>(0,0);
        Eigen::Vector3f t = relative_pose.block<3,1>(0,3);

        int v_s = correspondences[row].first(0);
        int u_s = correspondences[row].first(1);
        int v_t = correspondences[row].second(0);
        int u_t = correspondences[row].second(1);
        float diff_photo = target_color.at<float>(v_t, u_t) -
                            source_color.at<float>(v_s, u_s);
        float dIdx = SOBEL_SCALE * (target_color_dx.at<float>(v_t, u_t));
        float dIdy = SOBEL_SCALE * (target_color_dy.at<float>(v_t, u_t));
        float dDdx = SOBEL_SCALE * (target_depth_dx.at<float>(v_t, u_t));
        float dDdy = SOBEL_SCALE * (target_depth_dy.at<float>(v_t, u_t));
        if (std::isnan(dDdx)) dDdx = 0;
        if (std::isnan(dDdy)) dDdy = 0;
        Eigen::Vector3f p3d_mat(source_XYZ[v_s][u_s]);
        Eigen::Vector3f p3d_trans = R * p3d_mat + t;

        float diff_geo = target_depth.at<float>(v_t, u_t) - p3d_trans(2);
        float invz = 1. / p3d_trans(2);
        float c0 = dIdx * fx * invz;
        float c1 = dIdy * fy * invz;
        float c2 = -(c0 * p3d_trans(0) + c1 * p3d_trans(1)) * invz;
        float d0 = dDdx * fx * invz;
        float d1 = dDdy * fy * invz;
        float d2 = -(d0 * p3d_trans(0) + d1 * p3d_trans(1)) * invz;

        J.resize(2);
        residual.resize(2);
        J[0](0) = sqrt_lambda_img * (-p3d_trans(2) * c1 + p3d_trans(1) * c2);
        J[0](1) = sqrt_lambda_img * (p3d_trans(2) * c0 - p3d_trans(0) * c2);
        J[0](2) = sqrt_lambda_img * (-p3d_trans(1) * c0 + p3d_trans(0) * c1);
        J[0](3) = sqrt_lambda_img * (c0);
        J[0](4) = sqrt_lambda_img * (c1);
        J[0](5) = sqrt_lambda_img * (c2);
        float r_photo = sqrt_lambda_img * diff_photo;
        residual[0] = r_photo;

        J[1](0) = sqrt_lamba_dep *
                    ((-p3d_trans(2) * d1 + p3d_trans(1) * d2) - p3d_trans(1));
        J[1](1) = sqrt_lamba_dep *
                    ((p3d_trans(2) * d0 - p3d_trans(0) * d2) + p3d_trans(0));
        J[1](2) = sqrt_lamba_dep * ((-p3d_trans(1) * d0 + p3d_trans(0) * d1));
        J[1](3) = sqrt_lamba_dep * (d0);
        J[1](4) = sqrt_lamba_dep * (d1);
        J[1](5) = sqrt_lamba_dep * (d2 - 1.0f);
        float r_geo = sqrt_lamba_dep * diff_geo;
        residual[1] = r_geo;
        //std::cout <<J[0]<<" \n"<< std::endl;
    }
    std::tuple<Eigen::Matrix6f,Eigen::Vector6f,float> ComputeJTJandJTrHybridTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, const TransformationMatrix & relative_pose, 
        const PixelCorrespondenceSet &correspondences)
    {
        Eigen::Matrix6f JTJ_private;
        Eigen::Vector6f JTr_private;
        float r2_sum_private = 0.0;
        JTJ_private.setZero();
        JTr_private.setZero();
        for(int i = 0;i!=correspondences.size();++i)
        {
            std::vector<se3f> J;
            std::vector<float > r;
            ComputeJacobianHybridTerm(i,J,r,source_color,source_depth,target_color,target_depth,target_color_dx ,
                target_depth_dx, target_color_dy, target_depth_dy, source_XYZ, fc_camera, relative_pose,correspondences);            
            for(int j = 0; j!=J.size();++j)
            {
                JTJ_private.noalias() += J[j] * J[j].transpose();
                JTr_private.noalias() += J[j] * r[j];
                r2_sum_private += r[j] * r[j];
            }
        }
        //std::cout <<"JTJ: \n"<<JTJ_private<<std::endl;
        return std::make_tuple(std::move(JTJ_private), std::move(JTr_private), r2_sum_private);
    }
    std::tuple<Eigen::Matrix6f,Eigen::Vector6f,float> ComputeJTJandJTrPhotoTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy, 
        const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, const TransformationMatrix & relative_pose, 
        const PixelCorrespondenceSet &correspondences)
    {
        Eigen::Matrix6f JTJ_private;
        Eigen::Vector6f JTr_private;
        float r2_sum_private = 0.0;
        JTJ_private.setZero();
        JTr_private.setZero();
        for(int i = 0;i!=correspondences.size();++i)
        {
            std::vector<se3f> J;
            std::vector<float > r;
            ComputeJacobianPhotoTerm(i,J,r,source_color,source_depth,target_color,target_depth,target_color_dx ,
                 target_color_dy, source_XYZ, fc_camera, relative_pose,correspondences);            
            for(int j = 0; j!=J.size();++j)
            {
                JTJ_private.noalias() += J[j] * J[j].transpose();
                JTr_private.noalias() += J[j] * r[j];
                r2_sum_private += r[j] * r[j];
            }
        }
        //std::cout <<"JTJ: \n"<<JTJ_private<<std::endl;
        return std::make_tuple(std::move(JTJ_private), std::move(JTr_private), r2_sum_private);
    }
    void DoSingleIteration(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, TransformationMatrix & relative_pose, 
        PixelCorrespondenceSet &correspondences)
    {
        //find correspondences based on the current pose
        //std::cout<<"find correspondences[pixel]..."<<std::endl;
        ComputeCorrespondencePixelWise(source_depth,target_depth,fc_camera,relative_pose,correspondences);
        //std::cout<<"correspondences: "<<correspondences.size()<<std::endl;
        int valid_count = correspondences.size();
        Eigen::Matrix6f JTJ;
        se3f JTr;
        float r;
        //std::cout<<"computing Jacobian..."<<std::endl;
        
        std::tie(JTJ,JTr,r) = ComputeJTJandJTrHybridTerm(source_color,source_depth,target_color,target_depth,target_color_dx ,
                target_depth_dx, target_color_dy, target_depth_dy, source_XYZ, fc_camera, relative_pose,correspondences );
        //std::cout<<"JTJ: "<<JTJ<<std::endl;
        se3f delta = JTJ.ldlt().solve(-JTr);
        
        TransformationMatrix delta_matrix = TransformVector6fToMatrix4f(delta);
#if DEBUG_MODE        
        std::cout<<BLUE <<"[DEBUG]::residual: "<<r/valid_count<<" element_num: "<<valid_count <<RESET<<std::endl;
        //std::cout<<"delta: "<<delta<<std::endl;
#endif
        relative_pose = delta_matrix *  relative_pose;

    }
    void DoSingleIteration(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy,
        const ImageXYZ & source_XYZ, const PinholeCamera &fc_camera, TransformationMatrix & relative_pose, 
        PixelCorrespondenceSet &correspondences)
    {
        //find correspondences based on the current pose
        //std::cout<<"find correspondences[pixel]..."<<std::endl;
        ComputeCorrespondencePixelWise(source_depth,target_depth,fc_camera,relative_pose,correspondences);
        //std::cout<<"correspondences: "<<correspondences.size()<<std::endl;
        int valid_count = correspondences.size();
        Eigen::Matrix6f JTJ;
        se3f JTr;
        float r;
        //std::cout<<"computing Jacobian..."<<std::endl;
        
        std::tie(JTJ,JTr,r) = ComputeJTJandJTrPhotoTerm(source_color,source_depth,target_color,target_depth,target_color_dx ,
                 target_color_dy,  source_XYZ, fc_camera, relative_pose,correspondences );
        se3f delta = JTJ.ldlt().solve(-JTr);
        
        TransformationMatrix delta_matrix = TransformVector6fToMatrix4f(delta);
#if DEBUG_MODE        
        std::cout<<BLUE <<"[DEBUG]::residual: "<<r/valid_count<<" element_num: "<<valid_count <<" all:"<<fc_camera.GetHeight() * fc_camera.GetWidth() <<RESET<<std::endl;
        //std::cout<<"delta: "<<delta<<std::endl;
#endif
        relative_pose = delta_matrix *  relative_pose;

    }