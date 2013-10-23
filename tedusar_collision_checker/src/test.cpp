/*
 * test.cpp
 *
 *  Created on: Oct 22, 2013
 *      Author: lestefan
 */


#include <tedusar/check_collision.hpp>
#include <tedusar/CollisionChecker.hpp>

int main(){
	tedusar::CollisionChecker collisionChecker;
	collisionChecker.setParameters(tedusar::CollisionChecker::Parameters());
	cv::Mat dem;
	const int M=1000;
	const int N=1000;
	dem.create(N,M,CV_32FC1);
	double h=0.0;
	for(int y=0; y<N; y++){
		for(int x=0; x<M; x++){
			dem.at<float>(dem.rows-y-1,x)=h;
			h+=0.01;
		}
		h=y*0.01;
	}
	collisionChecker.setDem(dem,0.05);
	/*for(int y=0; y<N; y++){
		for(int x=0; x<M; x++){
			std::cout<<dem.at<float>(y,x)<<" ";
		}
		std::cout<<std::endl;
	}*/
	//cv::imshow("DEM",dem*(1.0/h)*std::numeric_limits<float>::max());
	//cv::waitKey();

	collisionChecker.calculateSupport(1.0,1.0,0.0);
	std::cout<<"collision: "<<int(collisionChecker.collision())<<std::endl;

	return 0;
}

