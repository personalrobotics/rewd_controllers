/**
 * @file RosMsgConverter.h
 * @brief Provides helper functions for converting to and from ros messages.
 * @author Ross Taylor
 * @date Aug 31, 2012
 */

#ifndef ROS_MSG_CONVERTER_H
#define ROS_MSG_CONVERTER_H

#include <sensor_msgs/JointState.h>

#include <kdl/frameacc.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayacc.hpp>
#include <kdl/jntarrayvel.hpp>

namespace rewd_controllers {

    struct JointVel
    {
        double q;
        double qdot;
    };

    struct JointAcc
    {
        double q;
        double qdot;
        double qdotdot;
    };

    struct JointCommand
    {
        double desiredPosition;
        double desiredPositionVelocityLimit;
        double feedForwardTorque;
        double porportionalGain;
        double derivativeGain;
        double integralGain;
        double positionLoopTorqueLimit;
        double positionLoopWindupLimit;
        double torqueLoopVelocityLimit;
    };

    void JointStateToJntArray(const sensor_msgs::JointState& jointState, const std::vector<std::string> jointNames, KDL::JntArray& jointArray);

    void JointStateToJntArrayVel(const sensor_msgs::JointState& jointState, const std::vector<std::string> jointNames, KDL::JntArrayVel& jointArray);

    /// Acc term contains effort ///
    void JointStateToJntArrayAcc(const sensor_msgs::JointState& jointState, const std::vector<std::string> jointNames, KDL::JntArrayAcc& jointArray);
}

#endif
