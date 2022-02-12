#ifndef TRANSFORMATION_MODEL
#define TRANSFORMATION_MODEL
#include "AbstractModel.hpp"
#include <Eigen/Core>
#define MIN_INLIER_SIZE 5
typedef std::array<GRANSAC::VPFloat, 2> Vector2VP;

class Point3fPair:
	public GRANSAC::AbstractParameter
{
public:
    Point3fPair(const Eigen::Vector3f &src, const Eigen::Vector3f &dst, int _index = 0)
    {
        _3d_pair.first = src;
        _3d_pair.second = dst;
		index = _index;
    }

    std::pair<Eigen::Vector3f, Eigen::Vector3f> _3d_pair;
	int index;
};
//rigid
class TransformationModel
	: public GRANSAC::AbstractModel<MIN_INLIER_SIZE>
{
protected:
	// Parametric form
    Eigen::Matrix3f R;
    Eigen::Vector3f t;
	GRANSAC::VPFloat m_DistDenominator; // = sqrt(a^2 + b^2). Stored for efficiency reasons
	Eigen::Vector3f gravity;
	virtual GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter> Param) override
	{
        auto pair_3d = std::dynamic_pointer_cast<Point3fPair>(Param);
		if (pair_3d == nullptr)
			std::cout<<"ERROR::3D RANSAC."<<std::endl;

		// Return distance between passed "point" and this line
		// http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
		auto dist = R * pair_3d->_3d_pair.first + t - pair_3d->_3d_pair.second;
		
		float error = dist.norm();
		if( (R * gravity).transpose() * gravity < 0.2) error = 9999; 
		//// Debug
		//std::cout << "Point: " << ExtPoint2D->m_Point2D[0] << ", " << ExtPoint2D->m_Point2D[1] << std::endl;
		//std::cout << "Line: " << m_a << " x + " << m_b << " y + "  << m_c << std::endl;
		//std::cout << "Distance: " << Dist << std::endl << std::endl;

		return error;
	};

public:
	TransformationModel(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams)
	{
		Initialize(InputParams);
	};

	virtual void Initialize(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams) override
	{
		if (InputParams.size() != MIN_INLIER_SIZE)
			throw std::runtime_error("TransformationModel - Number of input parameters does not match minimum number required for this model.");

		// Check for AbstractParamter types
		std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());   
        std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f>> correspondences;
        for(int  i = 0 ; i != MIN_INLIER_SIZE; ++i)
        {
            auto point3 = std::dynamic_pointer_cast<Point3fPair>(InputParams[i]);
            correspondences.push_back(point3->_3d_pair);
        }  
        auto transformation = ICPTransformation(correspondences);
		R = transformation.block<3,3>(0,0);
		t = transformation.block<3,1>(0,3);
		gravity << 0,0,1.0;
	};

	virtual std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>> Evaluate(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& EvaluateParams, GRANSAC::VPFloat Threshold)
	{
		std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
		int nTotalParams = EvaluateParams.size();
		int nInliers = 0;

		for (auto& Param : EvaluateParams)
		{
			if (ComputeDistanceMeasure(Param) < Threshold)
			{
				Inliers.push_back(Param);
				nInliers++;
			}
		}

		GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

		return std::make_pair(InlierFraction, Inliers);
	};
};

#endif