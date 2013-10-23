#ifndef THEROWBAGGERS_PATH_CONTROLLER_H_
#define THEROWBAGGERS_PATH_CONTROLLER_H_
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <nav_msgs/Path.h>

namespace control_node {
  class PathController
  {
    public:
        PathController();
        double ControlAngle(geometry_msgs::PoseWithCovariance& robotPose)
