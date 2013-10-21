/*********************************************************************
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Johannes Maurer,
 *                      Institute for Software Technology,
 *                      Graz University of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * All advertising materials mentioningfeatures or use of this
 *     software must display the following acknowledgement: “This product
 *     includes software developed by Graz University of Technology and
 *     its contributors.”
 *   * Neither the name of Graz University of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <tedusar_mesa_element/mesa_element.h>

#include <boost/regex.hpp>

using namespace std;

namespace MesaElement
{

/****************************************************************
 *
 */
MesaElement::MesaElement(ros::NodeHandle nh) :
    nh_(nh),
    io_service_(),
    serial_port_(io_service_)
{
    robot_state_.x = 0.0;
    robot_state_.y = 0.0;
    robot_state_.theta = 0.0;

}

/****************************************************************
 *
 */
MesaElement::~MesaElement()
{
    serial_port_.close();
    io_service_.stop();
    read_thread_.join();
}

/****************************************************************
 *
 */
void MesaElement::init(const MesaElementConfig &robot_config)
{
    robot_config_ = robot_config;

    try
    {
        odometry_msg_.header.frame_id = robot_config_.frame_id;
        odometry_msg_.child_frame_id = robot_config_.child_frame_id;

        odometry_tf_.header.frame_id = robot_config_.frame_id;
        odometry_tf_.child_frame_id = robot_config_.child_frame_id;

        serial_port_.open(robot_config_.port);

        serial_port_.set_option(boost::asio::serial_port::baud_rate(robot_config_.baud_rate));
        serial_port_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_port_.set_option(boost::asio::serial_port::character_size(boost::asio::serial_port::character_size(8)));
        serial_port_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_port_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));

        ROS_INFO_STREAM("MesaElement::init" << "Sucessfull connected to robot (" << robot_config_.port << ";" << robot_config_.baud_rate <<").");
        ROS_DEBUG_STREAM("MesaElement::init: Robot parameters:" << std::endl <<
                         "publish_tf:" << robot_config_.publish_tf << std::endl <<
                         "max_trans_velocity:" << robot_config_.max_trans_velocity << std::endl <<
                         "max_rot_velocity:" << robot_config_.max_rot_velocity << std::endl <<
                         "frame_id:" << robot_config_.frame_id << std::endl <<
                         "child_frame_id:" << robot_config_.child_frame_id << std::endl <<
                         "wheel_base:" << robot_config_.wheel_base << std::endl <<
                         "velocity_raw_factor:" << robot_config_.velocity_raw_factor << std::endl <<
                         "raw_odometry_factor:" << robot_config_.raw_odometry_factor);

        boost::asio::async_read_until(serial_port_, buffer_, MESSAGE_DELIMITER, boost::bind(&MesaElement::readCallback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        read_thread_ = boost::thread(boost::bind(&MesaElement::run, this));

        odometry_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 10);
    }
    catch (boost::system::system_error& error)
    {
        ROS_ERROR_STREAM("MesaElement::init" << error.what());
    }
}

/****************************************************************
 *
 */
void MesaElement::run()
{
    io_service_.run();
}

/****************************************************************
 *
 */
void  MesaElement::setVelocity(const double &trans, const double &rot)
{
    try
    {
        if((trans <= robot_config_.max_trans_velocity) && (rot <= robot_config_.max_rot_velocity))
        {
            double vel_left = trans - 0.5 * rot * robot_config_.wheel_base;
            double vel_right = trans + 0.5 * rot * robot_config_.wheel_base;

            int32_t left =  (int32_t) (vel_left * robot_config_.velocity_raw_factor);
            int32_t right =  (int32_t) (vel_right * robot_config_.velocity_raw_factor);

            stringstream msg_stream;
            msg_stream << RIGHT_MOTOR_ADDR << "p=" << left << MESSAGE_DELIMITER << RIGHT_MOTOR_ADDR << "s=" << right << MESSAGE_DELIMITER << RIGHT_MOTOR_ADDR << "m=4" << MESSAGE_DELIMITER;

            sendMessage(msg_stream.str());
        }
        else
        {
            if(trans > robot_config_.max_trans_velocity)
            {
                ROS_WARN_STREAM("MesaElement::setVelocity: Trans Velocity to high: " << trans << ">" << robot_config_.max_trans_velocity);
            }
            else
            {
                ROS_WARN_STREAM("MesaElement::setVelocity: Rot Velocity to high: " << rot << ">" << robot_config_.max_rot_velocity);
            }
        }
    }
    catch (boost::system::system_error& error)
    {
        ROS_ERROR_STREAM("MesaElement::setVelocity: " << error.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("MesaElement::setVelocity: " << "Unhandled exception!");
    }
}


/****************************************************************
 *
 */
MesaElementConfig MesaElement::getRobotConfig()
{
    return robot_config_;
}

/****************************************************************
 *
 */
MesaElementState MesaElement::getRobotState()
{
    return robot_state_;
}

/****************************************************************
 *
 */
void MesaElement::emergencyStop()
{
    try
    {
        stringstream msg_stream;
        msg_stream << ALL_MOTOR_ADDR << "S" << MESSAGE_DELIMITER;

        sendMessage(msg_stream.str());
    }
    catch (boost::system::system_error& error)
    {
        ROS_ERROR_STREAM("MesaElement::emergencyStop: " << error.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("MesaElement::emergencyStop: " << "Unhandled exception!");
    }
}

/****************************************************************
 *
 */
void  MesaElement::readCallback(const boost::system::error_code &error_code, std::size_t bytes_transferred)
{
    ros::Time now = ros::Time::now();
    if(error_code)
    {
        ROS_ERROR_STREAM("MesaElement::readCallback: " << boost::system::system_error(error_code).what());
    }
    else
    {
        istream is(&buffer_);
        string new_msg;
        getline(is, new_msg);

        ROS_DEBUG_STREAM("MesaElement::readCallback: " << new_msg);

        try
        {
            boost::regex reg_ex("ll=(\\-?\\d+)\\$rr=(\\-?\\d+)");
            boost::cmatch matches;

            if(boost::regex_search(new_msg.c_str(), matches, reg_ex))
            {
                int32_t left_pose =  boost::lexical_cast<int32_t>(matches[1].str());
                int32_t right_pose = boost::lexical_cast<int32_t>(matches[2].str());

                static bool first_call = true;

                if (first_call)
                {
                    robot_state_.last_msg_time = ros::Time::now();
                    robot_state_.last_left_position = left_pose;
                    robot_state_.last_right_position = right_pose;
                    first_call = false;
                }
                else
                {
                    int32_t delta_left_pose;
                    if(robot_state_.last_left_position > OVERFLOW_INDOCATOR && left_pose < -OVERFLOW_INDOCATOR) // positiv overflow
                    {
                        delta_left_pose = (left_pose - INT32_MIN) + (INT32_MAX - robot_state_.last_left_position);
                    }
                    else if(robot_state_.last_left_position < -OVERFLOW_INDOCATOR && left_pose > OVERFLOW_INDOCATOR) // negativ overflow
                    {
                        delta_left_pose = (left_pose - INT32_MAX) + (INT32_MIN - robot_state_.last_left_position);
                    }
                    else
                    {
                        delta_left_pose = left_pose - robot_state_.last_left_position;
                    }

                    int32_t delta_right_pose;
                    if(robot_state_.last_right_position > OVERFLOW_INDOCATOR && right_pose < -OVERFLOW_INDOCATOR) // positiv overflow
                    {
                        delta_right_pose = (right_pose - LONG_MIN) + (LONG_MAX - robot_state_.last_right_position);
                    }
                    else if(robot_state_.last_right_position < -OVERFLOW_INDOCATOR && right_pose > OVERFLOW_INDOCATOR) // negativ overflow
                    {
                        delta_right_pose = (right_pose - LONG_MAX) + (LONG_MIN - robot_state_.last_right_position);
                    }
                    else
                    {
                        delta_right_pose = right_pose - robot_state_.last_right_position;
                    }

                    robot_state_.last_left_position = left_pose;
                    robot_state_.last_right_position = right_pose;

                    double delta_left = (double) delta_left_pose * robot_config_.raw_odometry_factor;
                    double delta_right = (double) delta_right_pose * robot_config_.raw_odometry_factor;

                    ROS_DEBUG_STREAM("MesaElement::readCallback: delta_left= " << delta_left << " delta_right= " << delta_right);

                    double delta_trans = (delta_left + delta_right) / 2;
                    double delta_rot = (delta_right - delta_left) / robot_config_.wheel_base;
                    ros::Duration delta_t = now - robot_state_.last_msg_time;

                    ROS_DEBUG_STREAM("MesaElement::readCallback: delta_trans= " << delta_trans << " delta_rot= " << delta_rot);

                    double delta_x, delta_y;
                    if(delta_rot >= 0.0005)
                    {
                        delta_x = -delta_trans/delta_rot * sin(robot_state_.theta) + delta_trans/delta_rot * sin(robot_state_.theta + delta_rot);
                        delta_y =  delta_trans/delta_rot * cos(robot_state_.theta) - delta_trans/delta_rot * cos(robot_state_.theta + delta_rot);
                    }
                    else
                    {
                        delta_x = delta_trans * cos(robot_state_.theta);
                        delta_y = delta_trans * sin(robot_state_.theta);
                    }

                    robot_state_.last_msg_time = now;
                    robot_state_.x += delta_x;
                    robot_state_.y += delta_y;
                    robot_state_.theta += delta_rot;
                    robot_state_.trans = delta_trans / delta_t.toSec();
                    robot_state_.rot = delta_rot / delta_t.toSec();

                    odometry_msg_.header.seq++;
                    odometry_msg_.header.stamp = now;
                    odometry_msg_.pose.pose.position.x = robot_state_.x;
                    odometry_msg_.pose.pose.position.y = robot_state_.y;
                    odometry_msg_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(robot_state_.theta);
                    odometry_msg_.twist.twist.linear.x = robot_state_.trans;
                    odometry_msg_.twist.twist.angular.z = robot_state_.rot;

                    odometry_pub_.publish(odometry_msg_);

                    if(robot_config_.publish_tf)
                    {
                        odometry_tf_.header = odometry_msg_.header;
                        odometry_tf_.transform.translation.x = robot_state_.x;
                        odometry_tf_.transform.translation.y = robot_state_.y;
                        odometry_tf_.transform.rotation = tf::createQuaternionMsgFromYaw(robot_state_.theta);

                        odometry_tf_broadcaster_.sendTransform(odometry_tf_);
                    }
                }

            }
            else
            {
                ROS_WARN_STREAM(new_msg << " do not match " << reg_ex);
            }
        }
        catch (boost::regex_error& ex)
        {
            ROS_FATAL_STREAM("MesaElement::readCallback: " << "This should not happen! RegEx is not valid");
        }
    }

    boost::asio::async_read_until(serial_port_, buffer_, MESSAGE_DELIMITER, boost::bind(&MesaElement::readCallback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

/****************************************************************
 *
 */
void  MesaElement::sendMessage(const std::string &msg)
{
    boost::asio::write(serial_port_, boost::asio::buffer(msg), boost::asio::transfer_all());
}

/****************************************************************
 *
 */
void  MesaElement::resetErrors()
{
    try
    {
        stringstream msg_stream;
        msg_stream << ALL_MOTOR_ADDR << "ZS" << MESSAGE_DELIMITER;

        sendMessage(msg_stream.str());
    }
    catch (boost::system::system_error& error)
    {
        ROS_ERROR_STREAM("MesaElement::resetErrors: " << error.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("MesaElement::resetErrors: " << "Unhandled exception!");
    }
}

/****************************************************************
 *
 */
void  MesaElement::restartMotors()
{
    try
    {
        stringstream msg_stream;
        msg_stream << ALL_MOTOR_ADDR << "END" << MESSAGE_DELIMITER << ALL_MOTOR_ADDR << "ZS" << MESSAGE_DELIMITER << ALL_MOTOR_ADDR << "RUN" << MESSAGE_DELIMITER;

        sendMessage(msg_stream.str());
    }
    catch (boost::system::system_error& error)
    {
        ROS_ERROR_STREAM("MesaElement::restartMotors: " << error.what());
    }
    catch(...)
    {
        ROS_ERROR_STREAM("MesaElement::restartMotors: " << "Unhandled exception!");
    }
}

} // end namespace MesaElement
