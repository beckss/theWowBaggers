/*
 * CollisionChecker.cpp
 *
 *  Created on: Oct 22, 2013
 *      Author: lestefan
 */

#include <tedusar/CollisionChecker.hpp>
#include <sm/assert_macros.hpp>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

namespace Eigen{
// stolen from Eigen2 sources
template<typename VectorType, typename HyperplaneType>
void fitHyperplane(int numPoints,
                   VectorType **points,
                   HyperplaneType *result,
                   typename NumTraits<typename VectorType::Scalar>::Real* soundness = 0)
{
  typedef typename VectorType::Scalar Scalar;
  typedef Matrix<Scalar,VectorType::SizeAtCompileTime,VectorType::SizeAtCompileTime> CovMatrixType;
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(VectorType)
  int size = points[0]->size();

  // compute the mean of the data
  VectorType mean = VectorType::Zero(size);
  for(int i = 0; i < numPoints; ++i)
    mean += *(points[i]);
  mean /= numPoints;

  // compute the covariance matrix
  CovMatrixType covMat = CovMatrixType::Zero(size, size);
  VectorType remean = VectorType::Zero(size);
  for(int i = 0; i < numPoints; ++i)
  {
    VectorType diff = (*(points[i]) - mean).conjugate();
    covMat += diff * diff.adjoint();
  }

  // now we just have to pick the eigen vector with smallest eigen value
  SelfAdjointEigenSolver<CovMatrixType> eig(covMat);
  result->normal() = eig.eigenvectors().col(0);
  if (soundness)
    *soundness = eig.eigenvalues().coeff(0)/eig.eigenvalues().coeff(1);

  // let's compute the constant coefficient such that the
  // plane pass trough the mean point:
  result->offset() = - (result->normal().array()* mean.array()).sum();
}
template<typename VectorType>
void linearRegression(int numPoints,
                      VectorType **points,
                      VectorType *result,
                      int funcOfOthers )
{
	typedef typename VectorType::Scalar Scalar;
	typedef Hyperplane<Scalar, VectorType::SizeAtCompileTime> HyperplaneType;
	const int size = points[0]->size();
	result->resize(size);
	HyperplaneType h(size);
	fitHyperplane(numPoints, points, &h);
	for(int i = 0; i < funcOfOthers; i++)
	result->coeffRef(i) = - h.coeffs()[i] / h.coeffs()[funcOfOthers];
	for(int i = funcOfOthers; i < size; i++)
	result->coeffRef(i) = - h.coeffs()[i+1] / h.coeffs()[funcOfOthers];
}
}

namespace tedusar{
void CollisionChecker::setParameters(Parameters parameters){
	_parameters=parameters;
	// points involved in polygons assessed
	_a0_B=sm::kinematics::HomogeneousPoint (Eigen::Vector4d(
			parameters.l/2.0+parameters.x_off,
			parameters.w/2.0+parameters.y_off,
			parameters.z_off,1.0));
	_b0_B=sm::kinematics::HomogeneousPoint (Eigen::Vector4d(
			parameters.l/2.0+parameters.x_off,
			parameters.w/2.0+parameters.y_off-parameters.t,
			parameters.z_off,1.0));
	_c0_B=sm::kinematics::HomogeneousPoint (Eigen::Vector4d(
			parameters.l/2.0+parameters.x_off,
			-parameters.w/2.0+parameters.y_off+parameters.t,
			parameters.z_off,1.0));
	_d0_B=sm::kinematics::HomogeneousPoint (Eigen::Vector4d(
			parameters.l/2.0+parameters.x_off,
			-parameters.w/2.0+parameters.y_off,
			parameters.z_off,1.0));
	_a1_B=sm::kinematics::HomogeneousPoint (Eigen::Vector4d(
			-parameters.l/2.0+parameters.x_off,
			parameters.w/2.0+parameters.y_off,
			parameters.z_off,1.0));
	_b1_B=sm::kinematics::HomogeneousPoint (Eigen::Vector4d(
			-parameters.l/2.0+parameters.x_off,
			parameters.w/2.0+parameters.y_off-parameters.t,
			parameters.z_off,1.0));
	_c1_B=sm::kinematics::HomogeneousPoint (Eigen::Vector4d(
			-parameters.l/2.0+parameters.x_off,
			-parameters.w/2.0+parameters.y_off+parameters.t,
			parameters.z_off,1.0));
	_d1_B=sm::kinematics::HomogeneousPoint (Eigen::Vector4d(
			-parameters.l/2.0+parameters.x_off,
			-parameters.w/2.0+parameters.y_off,
			parameters.z_off,1.0));

	/*std::cout<<_a0_B.toEuclidean().transpose()<<std::endl;
	std::cout<<_b0_B.toEuclidean().transpose()<<std::endl;
	std::cout<<_c0_B.toEuclidean().transpose()<<std::endl;
	std::cout<<_d0_B.toEuclidean().transpose()<<std::endl;
	std::cout<<_a1_B.toEuclidean().transpose()<<std::endl;
	std::cout<<_b1_B.toEuclidean().transpose()<<std::endl;
	std::cout<<_c1_B.toEuclidean().transpose()<<std::endl;
	std::cout<<_d1_B.toEuclidean().transpose()<<std::endl;*/

	// now fill the polygons
	_L.push_back(cv::Point2f(_a0_B.toEuclidean()[0],_a0_B.toEuclidean()[1]));
	_L.push_back(cv::Point2f(_b0_B.toEuclidean()[0],_b0_B.toEuclidean()[1]));
	_L.push_back(cv::Point2f(_b1_B.toEuclidean()[0],_b1_B.toEuclidean()[1]));
	_L.push_back(cv::Point2f(_a1_B.toEuclidean()[0],_a1_B.toEuclidean()[1]));

	_C.push_back(cv::Point2f(_b0_B.toEuclidean()[0],_b0_B.toEuclidean()[1]));
	_C.push_back(cv::Point2f(_c0_B.toEuclidean()[0],_c0_B.toEuclidean()[1]));
	_C.push_back(cv::Point2f(_c1_B.toEuclidean()[0],_c1_B.toEuclidean()[1]));
	_C.push_back(cv::Point2f(_b1_B.toEuclidean()[0],_b1_B.toEuclidean()[1]));

	_R.push_back(cv::Point2f(_c0_B.toEuclidean()[0],_c0_B.toEuclidean()[1]));
	_R.push_back(cv::Point2f(_d0_B.toEuclidean()[0],_d0_B.toEuclidean()[1]));
	_R.push_back(cv::Point2f(_d1_B.toEuclidean()[0],_d1_B.toEuclidean()[1]));
	_R.push_back(cv::Point2f(_c1_B.toEuclidean()[0],_c1_B.toEuclidean()[1]));
}

bool CollisionChecker::calculateSupport(double x, double y, double theta){
	SM_ASSERT_TRUE_DBG(Exception,_dem.cols>0&&_dem.rows>0,"DEM not set");

	_collision=false;

	// bring the transformation to initial pose
	_T_WB.setIdentity();
	Eigen::Matrix<double,6,1> deltaT;
	deltaT.setZero();
	deltaT.head<3>() = Eigen::Vector3d(x,y,0);
	deltaT[5]=theta;
	_T_WB.oplus(deltaT);

	// fit a hyperplane
	int min = _a0_B.toEuclidean().norm()/_resolution+1;
	int center_x = _T_WB.t()[0]/_resolution;
	int center_y = _T_WB.t()[1]/_resolution;
	std::vector<Eigen::Vector3d> points;
	size_t i=0;
	for(double xi=center_x-min; xi<center_x+min; ++xi){
		for(double yi=center_y-min; yi<center_y+min; ++yi){
			double z=_dem.at<float>(_dem.rows-yi-1,xi);
			double x=(0.5+double(xi))*_resolution;
			double y=(0.5+double(yi))*_resolution;

			if(z==std::numeric_limits<float>::min())
				continue;

			// assemble homogeneous point and transform
			sm::kinematics::HomogeneousPoint p_W(Eigen::Vector3d(x,y,z));
			sm::kinematics::HomogeneousPoint p_B=_T_WB.inverse()*p_W;

			// check if in polygon
			cv::Point2f pt(p_B.toEuclidean()[0],p_B.toEuclidean()[1]);
			bool l=(cv::pointPolygonTest(_L,pt,false)>0.0);
			//bool c=(cv::pointPolygonTest(_C,pt,false)>0.0);
			bool r=(cv::pointPolygonTest(_R,pt,false)>0.0);

			if(l||r){
				points.push_back(p_W.toEuclidean());
				++i;

				if(_dem.at<float>(_dem.rows-yi-1,xi+1)==std::numeric_limits<float>::min()) continue;
				if(_dem.at<float>(_dem.rows-yi-1,xi-1)==std::numeric_limits<float>::min()) continue;
				if(_dem.at<float>(_dem.rows-(yi+1)-1,xi)==std::numeric_limits<float>::min()) continue;
				if(_dem.at<float>(_dem.rows-(yi-1)-1,xi)==std::numeric_limits<float>::min()) continue;

				// check step
				double step_x=_dem.at<float>(_dem.rows-yi-1,xi+1)-_dem.at<float>(_dem.rows-yi-1,xi-1);
				double step_y=_dem.at<float>(_dem.rows-(yi+1)-1,xi)-_dem.at<float>(_dem.rows-(yi-1)-1,xi);
				//double step_xy=_dem.at<float>(_dem.rows-(yi+1)-1,xi+1)-_dem.at<float>(_dem.rows-(yi-1)-1,xi-1);
				//double step_yx=_dem.at<float>(_dem.rows-(yi+1)-1,xi-1)-_dem.at<float>(_dem.rows-(yi-1)-1,xi+1);

				double step=_T_WB.C().col(0).transpose()*Eigen::Vector3d(step_x,step_y,0.0);
				if(step>0.3||step<-0.4)
					_collision=true;
			}
		}
	}
	Eigen::Vector3d** points_ptrs=new Eigen::Vector3d*[points.size()];
	for(size_t i=0; i<points.size(); ++i) points_ptrs[i]=&points[i];
	Eigen::Vector3d coeffs; // will store the coefficients a, b, c
	Eigen::linearRegression(int(i),
			points_ptrs,
			&coeffs,2 // the coord to express as a function of
			// the other ones. 0 means x, 1 means y, 2 means z.
	);
	delete[] points_ptrs;
	Eigen::Vector3d normal=Eigen::Vector3d(coeffs[0],coeffs[1],1).normalized();
	Eigen::Vector3d e_z_B = _T_WB.C().col(2);

	if((normal.transpose()*Eigen::Vector3d(0.0,0.0,1.0))<cos(40.0/180*M_PI))
		_collision=true; // hardcoded hack max 40° inclination

	/*/ place it z-wise such that it gets one support point: get the maximum z
	sm::kinematics::HomogeneousPoint p_B_max;
	double z_max=std::numeric_limits<float>::min();
	for(double xi=center_x-min; xi<center_x+min; ++xi){
		for(double yi=center_y-min; yi<center_y+min; ++yi){
			double z=_dem.at<float>(_dem.rows-yi-1,xi);
			double x=(0.5+double(xi))*_resolution;
			double y=(0.5+double(yi))*_resolution;

			// assemble homogeneous point and transform
			sm::kinematics::HomogeneousPoint p_W(Eigen::Vector3d(x,y,z));
			sm::kinematics::HomogeneousPoint p_B=_T_WB.inverse()*p_W;

			// check if in polygon
			cv::Point2f pt(p_B.toEuclidean()[0],p_B.toEuclidean()[1]);
			bool l=(cv::pointPolygonTest(_L,pt,false)>0.0);
			bool c=(cv::pointPolygonTest(_C,pt,false)>0.0);
			bool r=(cv::pointPolygonTest(_R,pt,false)>0.0);

			if(p_W.toEuclidean()[2]>z_max&&(l||c||r)){
				z_max=p_W.toEuclidean()[2];
				p_B_max=p_B;
			}
		}
	}
	_T_WB.tptr()[2]=z_max;

	if(z_max==std::numeric_limits<float>::min()){
		//no support found - leave the robot at z minus infinity
		return false;
	}

	//std::cout<<_T_WB.T()<<std::endl;*/

	// add the contact point

	// now let the robot "fall"

	return true;
}

}
