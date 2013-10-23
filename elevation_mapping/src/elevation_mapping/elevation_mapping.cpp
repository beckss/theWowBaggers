#include <elevation_mapping/elevation_mapping.h>


namespace elevation_mapping
{

ElevationMapping::ElevationMapping(ros::NodeHandle nh) :
    nh_(nh),
    frame_count_(0)
{

}

ElevationMapping::~ElevationMapping()
{

}

void
ElevationMapping::init()
{
    ros::NodeHandle private_nh("~");

    get_octomap_client_ = nh_.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");

    get_elevation_map_client_ = nh_.advertiseService("get_elevation_map", &ElevationMapping::getHeightfieldMap, this);

    private_nh.param<double>("max_z", max_z_, 1.5);
    private_nh.param<std::string>("frame_id", frame_id_, std::string("/world"));
}

bool ElevationMapping::getHeightfieldMap(elevation_mapping::GetElevationMap::Request &request, elevation_mapping::GetElevationMap::Response &response)
{   
    octomap_msgs::GetOctomap srv;

    if (!get_octomap_client_.call(srv))
    {
        ROS_ERROR("Failed to call service octomap_binary");
        return false;
    }

    boost::shared_ptr<octomap::OcTree> oc_tree(octomap_msgs::binaryMsgToMap(srv.response.map));

    double max_x, max_y, max_z;
    oc_tree->getMetricMax(max_x, max_y, max_z);

    double min_x, min_y, min_z;
    oc_tree->getMetricMin(min_x, min_y, min_z);

    double map_origin_x = min_x;
    double map_origin_y = min_y;

    double map_res = oc_tree->getNodeSize(oc_tree->getTreeDepth());

    double delta_x = max_x - min_x;
    double delta_y = max_y - min_y;

    int rows = static_cast<int>((delta_x) / map_res);
    int cols = static_cast<int>((delta_y) / map_res);

    cv_bridge::CvImagePtr elevation_image_ptr = cv_bridge::CvImagePtr(new cv_bridge::CvImage);
    elevation_image_ptr->encoding = std::string("mono16");
    elevation_image_ptr->header.frame_id = frame_id_;
    elevation_image_ptr->header.stamp = ros::Time::now();
    elevation_image_ptr->header.seq = frame_count_++;

    cv::scaleAdd(cv::Mat::ones(rows, cols, CV_32FC1), std::numeric_limits<float>::min(),
                 cv::Mat::zeros(rows, cols, CV_32FC1), elevation_image_ptr->image);


    for (octomap::OcTree::iterator it = oc_tree->begin(), end = oc_tree->end(); it != end; ++it)
    {
        if (oc_tree->isNodeOccupied(*it))
        {
            octomap::point3d cell_coord = it.getCoordinate();

            if(cell_coord.z() > max_z_)
                continue;

            int row, col;
            worldCoordToCellCoord(cell_coord.x(), cell_coord.y(), map_origin_x, map_origin_y, map_res, row, col);

            if(elevation_image_ptr->image.at<float>(rows - row - 1, col) < cell_coord.z())
            {
                elevation_image_ptr->image.at<float>(rows - row - 1, col) = cell_coord.z();
            }

        }
    }

    response.resolution = map_res;
    response.origin_x = map_origin_x;
    response.origin_y = map_origin_y;

    elevation_image_ptr->toImageMsg(response.height_map);

    return true;
}

void ElevationMapping::worldCoordToCellCoord(double x, double y, double map_origin_x, double map_origin_y, double map_res, int &row, int &col)
{
    row = (x - map_origin_x) / map_res;
    col = (y - map_origin_y) / map_res;
}

} // end namespace elevation_mapping
