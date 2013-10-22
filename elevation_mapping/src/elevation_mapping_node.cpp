#include <elevation_mapping/elevation_mapping.h>

int main(int argc, char** argv)
{

    try
    {
        ros::init(argc, argv, "elevation_mapping_node");

        ros::NodeHandle nh;

        elevation_mapping::ElevationMapping elevation_mapping(nh);

        elevation_mapping.init();

        ros::spin();
    }
    catch (...)
    {
        ROS_ERROR("Unhandled exception!");
        return -1;
    }

    return 0;
}
