/*
 * CollisionChecker.hpp
 *
 *  Created on: Oct 21, 2013
 *      Author: lestefan
 */

#ifndef COLLISIONCHECKER_HPP_
#define COLLISIONCHECKER_HPP_

#include <sm/kinematics/Transformation.hpp>
#include <opencv2/opencv.hpp>

namespace tedusar{

class CollisionChecker{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	SM_DEFINE_EXCEPTION(Exception,std::runtime_error);

	struct Parameters{
		Parameters():
			t(0.1),l(0.8),w(0.5), c(0.08),
			x_off(0.0),y_off(0.0),z_off(0.0) {}
		double t; // track width
		double l; // length
		double w; // width
		double c; // ground clearance
		double x_off; // CoG offset x default 0
		double y_off; // CoG offset y default 0
		double z_off; // CoG offset z default 0
	};

	void setDem(const cv::Mat& dem, double resolution){
		_dem=dem;
		_resolution=resolution;
	}

	// recalculates the robot points and sets parameters
	void setParameters(Parameters parameters);
	// calculates the support - call this first
	bool calculateSupport(double x, double y, double theta);
	// now the analysis is available TODO: implement these
	bool bottomContact() const ;
	bool flipsOver() const ;
	bool willCrashIntoStructure() const ;
	bool collision() const {return _collision;}

	// some accessors for convenience
	const sm::kinematics::Transformation& Pose() const {
		return _T_WB;
	}
	const std::vector<cv::Point2f>& supportPolygon() const{
		return _supportPolygon;
	}

protected:
	Parameters _parameters; // store the robot parameters
	// points of polygons (tracks and bottom)
	sm::kinematics::HomogeneousPoint _a0_B;
	sm::kinematics::HomogeneousPoint _b0_B;
	sm::kinematics::HomogeneousPoint _c0_B;
	sm::kinematics::HomogeneousPoint _d0_B;
	sm::kinematics::HomogeneousPoint _a1_B;
	sm::kinematics::HomogeneousPoint _b1_B;
	sm::kinematics::HomogeneousPoint _c1_B;
	sm::kinematics::HomogeneousPoint _d1_B;

	std::vector<cv::Point2f> _L; // x-y coordinates polygon left track
	std::vector<cv::Point2f> _C; // x-y coordinates polygon center track
	std::vector<cv::Point2f> _R; // x-y coordinates polygon right track
	cv::Mat _dem; // the DEM: 32S depth as [mm]
	double _resolution; // DEM resolution [m]
	std::vector<cv::Point2f> _supportPolygon;
	sm::kinematics::Transformation _T_WB; // World->Robot pose

	// hack:
	bool _collision;
};

} // namespace tedusar


#endif /* COLLISIONCHECKER_HPP_ */
