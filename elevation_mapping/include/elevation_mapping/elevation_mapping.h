#ifndef travers_analysis_h___
#define travers_analysis_h___

#include <ros/ros.h>
#include <octomap_msgs/GetOctomap.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <limits>
#include <octomap_msgs/conversions.h>
#include <elevation_mapping/GetElevationMap.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace elevation_mapping
{

class ElevationMapping
{
protected:
    ros::NodeHandle nh_;

    ros::ServiceClient get_octomap_client_;

    ros::ServiceServer get_elevation_map_client_;

    double max_z_;

    int frame_count_;
    std::string frame_id_;

public:
    ElevationMapping(ros::NodeHandle nh);

    virtual ~ElevationMapping();

    void init();

    bool getHeightfieldMap(elevation_mapping::GetElevationMap::Request &request, elevation_mapping::GetElevationMap::Response &response);

protected:

    void worldCoordToCellCoord(double x, double y, double map_origin_x, double map_origin_y, double map_res, int &raw, int &rol);

};

} // end namespace elevation_mapping

#endif // travers_analysis_h___
