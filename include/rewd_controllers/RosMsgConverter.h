/**
 * @file RosMsgConverter.h
 * @brief Provides helper functions for converting to and from ros messages.
 * @author Ross Taylor
 * @date Aug 31, 2012
 */

#ifndef ROS_MSG_CONVERTER_H
#define ROS_MSG_CONVERTER_H

// #include <trajectory_msgs/JointTrajectory.h>
// #include "nasa_r2_common_msgs/PoseTrajectory.h"
#include <sensor_msgs/JointState.h>
// #include "nasa_r2_common_msgs/PoseState.h"
// #include "nasa_r2_common_msgs/JointControlDataArray.h"
// #include "nasa_r2_common_msgs/WrenchState.h"
// #include "nasa_r2_common_msgs/JointCommand.h"

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

    // void JointStateToJntArrayVel(const sensor_msgs::JointState& jointState, const std::vector<std::string> jointNames, KDL::JntArrayVel& jointArray);

    // /// Acc term contains effort ///
    // void JointStateToJntArrayAcc(const sensor_msgs::JointState& jointState, const std::vector<std::string> jointNames, KDL::JntArrayAcc& jointArray);

//     void JointStateToJntMap(const sensor_msgs::JointState& jointState, std::map<std::string, double> &jointMap);

//     void JointStateToJointVelMap(const sensor_msgs::JointState& jointState, std::map<std::string, JointVel > &jointMap);

//     void JointStateToJointAccMap(const sensor_msgs::JointState& jointState, std::map<std::string, JointAcc> &jointMap);

//     void PoseStateToFrame(const nasa_r2_common_msgs::PoseState& poseState, const std::string& baseName,
//                                 const std::string& tipName, KDL::Frame& frame);

//     void PoseStateToFrameVel(const nasa_r2_common_msgs::PoseState& poseState, const std::string& baseName,
//                                 const std::string& tipName, KDL::FrameVel& frame);

//     void PoseStateToFrameAcc(const nasa_r2_common_msgs::PoseState& poseState, const std::string& baseName,
//                                 const std::string& tipName, KDL::FrameAcc& frame);

//     void PoseStateToFrameAccMap(const nasa_r2_common_msgs::PoseState& poseState, const std::string &base,
//                                 std::map<std::string, KDL::FrameAcc> &frameaccs);

//     void PoseStateToFrameVelMap(const nasa_r2_common_msgs::PoseState& poseState, const std::string &base,
//                                 std::map<std::string, KDL::FrameVel> &framevels);

//     void PoseStateToFrameMap(const nasa_r2_common_msgs::PoseState& poseState, const std::string &base,
//                                 std::map<std::string, KDL::Frame> &frames);

//     void JointControlDataToCoeffStateMap(const nasa_r2_common_msgs::JointControlDataArray& jointData,
//                                           std::map<std::string, nasa_r2_common_msgs::JointControlCoeffState>& jointMap);
//     KDL::Wrench wrenchMsgToWrench(const geometry_msgs::Wrench& wrenchMsg);

//     geometry_msgs::Wrench wrenchToWrenchMsg(const KDL::Wrench &wrench);

//     void JointCommandToJointCommandMap(const nasa_r2_common_msgs::JointCommand& jointCommand, std::map<std::string, JointCommand> &jointCommandMap);

//     void WrenchStateToWrenchMap(const nasa_r2_common_msgs::WrenchState& wrenchState, std::map<std::string, KDL::Wrench> &wrenchMap);
}

#endif
