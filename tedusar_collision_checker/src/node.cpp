/*
 * node.cpp
 *
 *  Created on: Oct 23, 2013
 *      Author: lestefan
 */

#include "ros/ros.h"
#include <cstdlib>
#include <elevation_mapping/GetElevationMap.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tedusar/CollisionChecker.hpp>

/// Global Variables
const int theta_slider_max = 360;
int theta_slider;
double theta;
cv::Mat image;
std::vector<cv::Mat> collision_maps;
const int THETA_DISCRETIZATION = 5.0; // [deg]

/**
 * @function on_trackbar
 * @brief Callback for trackbar
 */
void on_trackbar( int, void* )
{
 theta = 360*(double(theta_slider)/double(theta_slider_max)) ;

 // select collision map and display
 cv::Mat outimage;
 int theta_idx=int(theta/double(THETA_DISCRETIZATION));
 theta_idx=theta_idx%(collision_maps.size());
 //std::cout<<"theta_idx"<<theta_idx<<std::endl;
 std::cout.flush();
 outimage=collision_maps.at(theta_idx);

 cv::imshow( "Collision Map", outimage );
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "collision_map_demo");
  /*if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y"); // TODO
    return 1;
  }*/

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<elevation_mapping::GetElevationMap>("get_elevation_map");
  elevation_mapping::GetElevationMap srv;

  if (!client.call(srv))
  {
	ROS_ERROR("Failed to call service add_two_ints");
	return 1;
  }

  const cv::Mat raw(srv.response.height_map.height, srv.response.height_map.width, CV_32FC1,
  	                    reinterpret_cast<float*>(&srv.response.height_map.data[0]), srv.response.height_map.step);

  std::cout<<"map size: "<<raw.cols*srv.response.resolution<<"m x "<<raw.rows*srv.response.resolution<<"m"<<std::endl;

  // create the collision maps
  std::cout<<"creating collision maps "<<std::flush;

  // geometry
  tedusar::CollisionChecker collisionChecker;
  tedusar::CollisionChecker::Parameters parameters;
  parameters.l=0.625;
  parameters.w=0.530;
  parameters.t=0.090;
  collisionChecker.setParameters(parameters);
  collisionChecker.setDem(raw,srv.response.resolution);

  // fill maps
  int margin = 0.5*sqrt(parameters.l*parameters.l+parameters.w*parameters.w)/srv.response.resolution+1;
  int start_x=8.0/srv.response.resolution;
  int start_y=3.0/srv.response.resolution;
  int end_x=17.0/srv.response.resolution;
  int end_y=10.0/srv.response.resolution;
  for(int theta_deg = 0; theta_deg<theta_slider_max; theta_deg+=int(THETA_DISCRETIZATION)){
	  collision_maps.push_back(cv::Mat::zeros(raw.rows,raw.cols,CV_8UC1));
	  for(int x=std::max(margin,start_x); x<std::min(raw.cols-margin,end_x); ++x){
		  for(int y=std::max(margin,start_y); y<std::min(raw.rows-margin,end_y); ++y){

			  collisionChecker.calculateSupport((x+0.5)*srv.response.resolution,(y+0.5)*srv.response.resolution,theta_deg/180.0*M_PI);
			  if(collisionChecker.collision())
				  collision_maps.back().at<uchar>(raw.rows-y-1,x) = 0;
			  else
				  collision_maps.back().at<uchar>(raw.rows-y-1,x) = 255;
		  }
	  }
	  std::cout<<"."<<std::flush;
  }

  std::cout<<" ok."<<std::endl;


  // the angle theta:
  /// Create Windows
  image.create(raw.rows,raw.cols,CV_32FC3);
  image=raw.clone();
  cv::namedWindow("DEM", 1);
  cv::createTrackbar( "theta", "DEM", &theta_slider, theta_slider_max, on_trackbar );

  cv::imshow("DEM",raw);
  cv::waitKey();

  return 0;
}


