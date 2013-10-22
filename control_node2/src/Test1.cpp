/*
rosmsg show nav_msgs/Odometry 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/PoseWithCovariance pose
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  geometry_msgs/Twist twist
    geometry_msgs/Vector3 linear
      float64 x
      float64 y
      float64 z
    geometry_msgs/Vector3 angular
      float64 x
      float64 y
      float64 z
  float64[36] covariance
*/
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
void callback(const nav_msgs::Odometry::ConstPtr& str)
{
	printf("pose.pose/n");
	printf(" position/n");
	printf("  x,y,z %f,%f\n",str->pose.pose.position.x, str->pose.pose.position.y, str->pose.pose.position.z);
	printf(" orientation/n");
	printf("  x,y,z,w %f,%f\n",str->pose.pose.orientation.x, str->pose.pose.orientation.y, str->pose.pose.orientation.z, str->pose.pose.orientation.w);
	printf("twist.twist/n");
	printf(" linear/n");
	printf("  x,y,z %f,%f\n",str->twist.twist.linear.x, str->twist.twist.linear.y, str->twist.twist.linear.z);
	printf(" angular/n");
	printf("  x,y,z %f,%f\n",str->twist.twist.angular.x, str->twist.twist.angular.y, str->twist.twist.angular.z);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Test1");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/nav_msgs/Odometry/odom", 1000, callback);
	ros::spin();
return 0;
}
