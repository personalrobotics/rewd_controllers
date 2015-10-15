#include <rewd_controllers/RosMsgConverter.h>

// #include "nasa_common_utilities/Logger.h"
#include <tf_conversions/tf_kdl.h>
// #include <boost/tuple/tuple.hpp>

using namespace rewd_controllers;
/**
 * @brief JointStateToJntArray
 * @param jointState jointState of joints
 * @param jointNames names of joints to extract
 * @param jointArray array of joint position values extracted from jointState
 * @return void
 * @exception logic_error jointState name length doesn't match positions length
 * @exception logic_error jointState name length less than trajectory name length
 * @exception logic_error jointState doesn't contain all joints in jointNames
 * @details extracts joints from jointState based on jointNames. If jointNames and jointArray
 * are the same size, resizing jointArray is avoided
 */
void JointStateToJntArray(const sensor_msgs::JointState& jointState,
                          const std::vector<std::string> jointNames,
                          KDL::JntArray& jointArray)
{
    if (jointArray.rows() != jointNames.size())
    {
        jointArray.resize(jointNames.size());
    }

    if (jointState.name.size() != jointState.position.size())
    {
        // length mismatch
        std::stringstream err;
        err << "JointStateToJntArray() - jointState name length doesn't match positions length: " << jointState.name.size() << " vs " << jointState.position.size();
        // RCS::Logger::log("gov.nasa.controllers.JointStateToJntArray", log4cpp::Priority::ERROR, err.str());
        throw std::logic_error(err.str());
        return;
    }

    // if M = jointState.name.size() & N = jointNames.size(),
    // a simple repetitive search would be worst case O(M*N);
    // let's sort instead to get worst case O((M+N)*log(M))
    std::map<std::string, double> jsMap;
    for (unsigned int i = 0; i < jointState.name.size(); ++i)
    {
        jsMap[jointState.name[i]] = jointState.position[i];
    }

    std::string jointName;
    for (unsigned int nameIndex = 0; nameIndex < jointNames.size(); ++nameIndex)
    {
        jointName = jointNames[nameIndex];

        std::map<std::string, double>::iterator jsMapIt = jsMap.find(jointName);
        if (jsMapIt == jsMap.end())
        {
            // not found
            std::stringstream err;
            err << "JointStateToJntArray() - jointState doesn't contain joint: " << jointName;
            // RCS::Logger::log("gov.nasa.controllers.JointStateToJntArray", log4cpp::Priority::ERROR, err.str());
            throw std::logic_error(err.str());
            return;
        }
        else
        {
            jointArray(nameIndex) = jsMapIt->second;
        }
    }
}

/**
 * @brief JointStateToJntArrayVel
 * @param jointState jointState of joints
 * @param jointNames names of joints to extract
 * @param jointArray array of joint values extracted from jointState
 * @return void
 * @exception logic_error jointState name length doesn't match positions length
 * @exception logic_error jointState name length less than trajectory name length
 * @exception logic_error jointState doesn't contain all joints in jointNames
 * @details extracts joints from jointState based on jointNames. If jointNames and jointArray
 * are the same size, resizing jointArray is avoided
 */
// void JointStateToJntArrayVel(const sensor_msgs::JointState& jointState,
//                              const std::vector<std::string> jointNames,
//                              KDL::JntArrayVel& jointArray)
// {
//     if (jointArray.q.rows() != jointNames.size())
//     {
//         jointArray.resize(jointNames.size());
//     }

//     if (jointState.name.size() != jointState.position.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "JointStateToJntArray() - jointState name length doesn't match positions length";
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJntArray", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     bool noVels = false;

//     if (jointState.name.size() != jointState.velocity.size())
//     {
//         if (jointState.velocity.size() == 0)
//         {
//             noVels = true;
//         }
//         else
//         {
//             // length mismatch
//             std::stringstream err;
//             err << "JointStateToJntArray() - jointState name length doesn't match positions length";
//             RCS::Logger::log("gov.nasa.controllers.JointStateToJntArray", log4cpp::Priority::ERROR, err.str());
//             throw std::logic_error(err.str());
//             return;
//         }
//     }

//     // if M = jointState.name.size() & N = jointNames.size(),
//     // a simple repetitive search would be worst case O(M*N);
//     // let's sort instead to get worst case O((M+N)*log(M))
//     std::map<std::string, std::pair<double, double> > jsMap;
//     for (unsigned int i = 0; i < jointState.name.size(); ++i)
//     {
//         if (noVels)
//         {
//             jsMap[jointState.name[i]] = std::make_pair(jointState.position[i], 0.);
//         }
//         else
//         {
//             jsMap[jointState.name[i]] = std::make_pair(jointState.position[i], jointState.velocity[i]);
//         }
//     }

//     std::string jointName;
//     for (unsigned int nameIndex = 0; nameIndex < jointNames.size(); ++nameIndex)
//     {
//         jointName = jointNames[nameIndex];

//         std::map<std::string, std::pair<double, double> >::iterator jsMapIt = jsMap.find(jointName);
//         if (jsMapIt == jsMap.end())
//         {
//             // not found
//             std::stringstream err;
//             err << "JointStateToJntArrayVel() - jointState doesn't contain joint: " << jointName;
//             RCS::Logger::log("gov.nasa.controllers.JointStateToJntArrayVel", log4cpp::Priority::ERROR, err.str());
//             throw std::logic_error(err.str());
//             return;
//         }
//         else
//         {
//             jointArray.q(nameIndex)    = jsMapIt->second.first;
//             jointArray.qdot(nameIndex) = jsMapIt->second.second;
//         }
//     }
// }

// void JointStateToJntArrayAcc(const sensor_msgs::JointState& jointState,
//                              const std::vector<std::string> jointNames,
//                              KDL::JntArrayAcc &jointArray)
// {
//     if (jointArray.q.rows() != jointNames.size())
//     {
//         jointArray.resize(jointNames.size());
//     }

//     if (jointState.name.size() != jointState.position.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "JointStateToJntArray() - jointState name length doesn't match positions length";
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJntArray", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointState.name.size() != jointState.velocity.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "JointStateToJntArray() - jointState name length doesn't match positions length";
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJntArray", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointState.name.size() != jointState.effort.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "JointStateToJntArray() - jointState name length doesn't match positions length";
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJntArray", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     // if M = jointState.name.size() & N = jointNames.size(),
//     // a simple repetitive search would be worst case O(M*N);
//     // let's sort instead to get worst case O((M+N)*log(M))
//     std::map<std::string, boost::tuples::tuple<double, double, double> > jsMap;
//     for (unsigned int i = 0; i < jointState.name.size(); ++i)
//     {
//         jsMap[jointState.name[i]] = boost::tuples::make_tuple(jointState.position[i], jointState.velocity[i], jointState.effort[i]);
//     }

//     for (unsigned int nameIndex = 0; nameIndex < jointNames.size(); ++nameIndex)
//     {
//         std::map<std::string, boost::tuples::tuple<double, double, double> >::iterator jsMapIt = jsMap.find(jointNames[nameIndex]);
//         if (jsMapIt == jsMap.end())
//         {
//             // not found
//             std::stringstream err;
//             err << "JointStateToJntArrayVel() - jointState doesn't contain joint: " << jointNames[nameIndex];
//             RCS::Logger::log("gov.nasa.controllers.JointStateToJntArrayVel", log4cpp::Priority::ERROR, err.str());
//             throw std::logic_error(err.str());
//             return;
//         }
//         else
//         {
//             jointArray.q(nameIndex)       = jsMapIt->second.get<0>();
//             jointArray.qdot(nameIndex)    = jsMapIt->second.get<1>();
//             jointArray.qdotdot(nameIndex) = jsMapIt->second.get<2>();
//         }
//     }
// }

/**
 * @brief PoseStateToFrame
 * @param poseState poseState of robot
 * @param baseFrame name of the reference frame for returned pose
 * @param tipFrame name of the tip frame to return
 * @param frame returned pose
 * @return void
 * @exception logic_error poseState name length doesn't match pose length
 * @exception logic_error poseState doesn't contain the necessary transform information
 * @details extracts poses from poseState and performs necessary transformations
 */
// void PoseStateToFrame(const nasa_r2_common_msgs::PoseState& poseState, const std::string& baseName,
//                       const std::string& tipName, KDL::Frame& frame)
// {
//     if (poseState.name.size() != poseState.positions.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "PoseStateToFrame() - poseState name length doesn't match pose length";
//         RCS::Logger::log("gov.nasa.controllers.PoseStateToFrame", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     // find the needed poses
//     bool       foundBase = false;
//     bool       foundTip  = false;
//     KDL::Frame baseFrame;
//     KDL::Frame tipFrame;

//     // check poseState frame
//     if (poseState.header.frame_id == baseName)
//     {
//         foundBase = true;
//     }
//     if (poseState.header.frame_id == tipName)
//     {
//         foundTip = true;
//     }

//     // check poses
//     for (unsigned int i = 0; i < poseState.name.size(); ++i)
//     {
//         if (!foundBase && poseState.name[i] == baseName)
//         {
//             foundBase = true;
//             tf::PoseMsgToKDL(poseState.positions.at(i), baseFrame);
//             if (foundTip)
//             {
//                 break;
//             }
//         }

//         if (!foundTip && poseState.name[i] == tipName)
//         {
//             foundTip = true;
//             tf::PoseMsgToKDL(poseState.positions.at(i), tipFrame);
//             if (foundBase)
//             {
//                 break;
//             }
//         }
//     }

//     if (!foundBase || !foundTip)
//     {
//         // not found
//         std::stringstream err;
//         err << "PoseStateToFrame() - poseState doesn't contain the necessary transform information for " << baseName << " to " << tipName;
//         RCS::Logger::log("gov.nasa.controllers.PoseStateToFrame", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     // transform
//     frame = baseFrame.Inverse() * tipFrame;
// }

// /**
//  * @brief PoseStateToFrameVel
//  * @param poseState poseState of robot
//  * @param baseFrame name of the reference frame for returned pose
//  * @param tipFrame name of the tip frame to return
//  * @param frame returned pose
//  * @return void
//  * @exception logic_error poseState name length doesn't match pose length
//  * @exception logic_error poseState doesn't contain the necessary transform information
//  * @details extracts poses from poseState and performs necessary transformations
//  */
// void PoseStateToFrameVel(const nasa_r2_common_msgs::PoseState& poseState, const std::string& baseName,
//                          const std::string& tipName, KDL::FrameVel& frame)
// {
//     if (poseState.name.size() != poseState.positions.size() || poseState.name.size() != poseState.velocities.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "PoseStateToFrameVel() - poseState name and value lengths don't match";
//         RCS::Logger::log("gov.nasa.controllers.PoseStateToFrameVel", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     // find the needed poses
//     bool          foundBase = false;
//     bool          foundTip  = false;
//     KDL::FrameVel baseFrame;
//     KDL::FrameVel tipFrame;

//     // check poseState frame
//     if (poseState.header.frame_id == baseName)
//     {
//         foundBase = true;
//     }
//     if (poseState.header.frame_id == tipName)
//     {
//         foundTip = true;
//     }

//     // check poses
//     for (unsigned int i = 0; i < poseState.name.size(); ++i)
//     {
//         if (!foundBase && poseState.name[i] == baseName)
//         {
//             foundBase = true;

//             // position
//             baseFrame.p.p.x(poseState.positions[i].position.x);
//             baseFrame.p.p.y(poseState.positions[i].position.y);
//             baseFrame.p.p.z(poseState.positions[i].position.z);
//             baseFrame.M.R = KDL::Rotation::Quaternion(poseState.positions[i].orientation.x, poseState.positions[i].orientation.y,
//                             poseState.positions[i].orientation.z, poseState.positions[i].orientation.w);

//             //velocity
//             baseFrame.p.v.x(poseState.velocities[i].linear.x);
//             baseFrame.p.v.y(poseState.velocities[i].linear.y);
//             baseFrame.p.v.z(poseState.velocities[i].linear.z);
//             baseFrame.M.w.x(poseState.velocities[i].angular.x);
//             baseFrame.M.w.y(poseState.velocities[i].angular.y);
//             baseFrame.M.w.z(poseState.velocities[i].angular.z);

//             if (foundTip)
//             {
//                 break;
//             }
//         }

//         if (!foundTip && poseState.name[i] == tipName)
//         {
//             foundTip = true;

//             // position
//             tipFrame.p.p.x(poseState.positions[i].position.x);
//             tipFrame.p.p.y(poseState.positions[i].position.y);
//             tipFrame.p.p.z(poseState.positions[i].position.z);
//             tipFrame.M.R = KDL::Rotation::Quaternion(poseState.positions[i].orientation.x, poseState.positions[i].orientation.y,
//                            poseState.positions[i].orientation.z, poseState.positions[i].orientation.w);

//             //velocity
//             tipFrame.p.v.x(poseState.velocities[i].linear.x);
//             tipFrame.p.v.y(poseState.velocities[i].linear.y);
//             tipFrame.p.v.z(poseState.velocities[i].linear.z);
//             tipFrame.M.w.x(poseState.velocities[i].angular.x);
//             tipFrame.M.w.y(poseState.velocities[i].angular.y);
//             tipFrame.M.w.z(poseState.velocities[i].angular.z);

//             if (foundBase)
//             {
//                 break;
//             }
//         }
//     }

//     if (!foundBase || !foundTip)
//     {
//         // not found
//         std::stringstream err;
//         err << "PoseStateToFrame() - poseState doesn't contain the necessary transform information for " << baseName << " to " << tipName;
//         RCS::Logger::log("gov.nasa.controllers.PoseStateToFrame", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     // transform
//     frame = baseFrame.Inverse() * tipFrame;
// }

// /**
//  * @brief PoseStateToFrameAcc
//  * @param poseState poseState of robot
//  * @param baseFrame name of the reference frame for returned pose
//  * @param tipFrame name of the tip frame to return
//  * @param frame returned pose
//  * @return void
//  * @exception logic_error poseState name length doesn't match pose length
//  * @exception logic_error poseState doesn't contain the necessary transform information
//  * @details extracts poses from poseState and performs necessary transformations
//  */
// void PoseStateToFrameAcc(const nasa_r2_common_msgs::PoseState& poseState, const std::string& baseName,
//                          const std::string& tipName, KDL::FrameAcc& frame)
// {
//     if (poseState.name.size() != poseState.positions.size() || poseState.name.size() != poseState.velocities.size()
//             || poseState.name.size() != poseState.accelerations.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "PoseStateToFrameAcc() - poseState name and value lengths don't match";
//         RCS::Logger::log("gov.nasa.controllers.PoseStateToFrameAcc", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     // find the needed poses.at(i)
//     bool          foundBase = false;
//     bool          foundTip  = false;
//     KDL::FrameAcc baseFrame;
//     KDL::FrameAcc tipFrame;

//     // check poseState frame
//     if (poseState.header.frame_id == baseName)
//     {
//         foundBase = true;
//     }
//     if (poseState.header.frame_id == tipName)
//     {
//         foundTip = true;
//     }

//     // check poses
//     for (unsigned int i = 0; i < poseState.name.size(); ++i)
//     {
//         if (!foundBase && poseState.name[i] == baseName)
//         {
//             foundBase = true;

//             // position
//             baseFrame.p.p.x(poseState.positions[i].position.x);
//             baseFrame.p.p.y(poseState.positions[i].position.y);
//             baseFrame.p.p.z(poseState.positions[i].position.z);
//             baseFrame.M.R = KDL::Rotation::Quaternion(poseState.positions[i].orientation.x, poseState.positions[i].orientation.y,
//                             poseState.positions[i].orientation.z, poseState.positions[i].orientation.w);

//             //velocity
//             baseFrame.p.v.x(poseState.velocities[i].linear.x);
//             baseFrame.p.v.y(poseState.velocities[i].linear.y);
//             baseFrame.p.v.z(poseState.velocities[i].linear.z);
//             baseFrame.M.w.x(poseState.velocities[i].angular.x);
//             baseFrame.M.w.y(poseState.velocities[i].angular.y);
//             baseFrame.M.w.z(poseState.velocities[i].angular.z);

//             //acceleration
//             baseFrame.p.dv.x(poseState.accelerations[i].linear.x);
//             baseFrame.p.dv.y(poseState.accelerations[i].linear.y);
//             baseFrame.p.dv.z(poseState.accelerations[i].linear.z);
//             baseFrame.M.dw.x(poseState.accelerations[i].angular.x);
//             baseFrame.M.dw.y(poseState.accelerations[i].angular.y);
//             baseFrame.M.dw.z(poseState.accelerations[i].angular.z);

//             if (foundTip)
//             {
//                 break;
//             }
//         }

//         if (!foundTip && poseState.name[i] == tipName)
//         {
//             foundTip = true;

//             // position
//             tipFrame.p.p.x(poseState.positions[i].position.x);
//             tipFrame.p.p.y(poseState.positions[i].position.y);
//             tipFrame.p.p.z(poseState.positions[i].position.z);
//             tipFrame.M.R = KDL::Rotation::Quaternion(poseState.positions[i].orientation.x, poseState.positions[i].orientation.y,
//                            poseState.positions[i].orientation.z, poseState.positions[i].orientation.w);

//             //velocity
//             tipFrame.p.v.x(poseState.velocities[i].linear.x);
//             tipFrame.p.v.y(poseState.velocities[i].linear.y);
//             tipFrame.p.v.z(poseState.velocities[i].linear.z);
//             tipFrame.M.w.x(poseState.velocities[i].angular.x);
//             tipFrame.M.w.y(poseState.velocities[i].angular.y);
//             tipFrame.M.w.z(poseState.velocities[i].angular.z);

//             //acceleration
//             tipFrame.p.dv.x(poseState.accelerations[i].linear.x);
//             tipFrame.p.dv.y(poseState.accelerations[i].linear.y);
//             tipFrame.p.dv.z(poseState.accelerations[i].linear.z);
//             tipFrame.M.dw.x(poseState.accelerations[i].angular.x);
//             tipFrame.M.dw.y(poseState.accelerations[i].angular.y);
//             tipFrame.M.dw.z(poseState.accelerations[i].angular.z);

//             if (foundBase)
//             {
//                 break;
//             }
//         }
//     }

//     if (!foundBase || !foundTip)
//     {
//         // not found
//         std::stringstream err;
//         err << "PoseStateToFrame() - poseState doesn't contain the necessary transform information for " << baseName << " to " << tipName;
//         RCS::Logger::log("gov.nasa.controllers.PoseStateToFrame", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     // transform
//     frame = baseFrame.Inverse() * tipFrame;
// }

// /** @brief updates a map of all the frames in poseState with their new location and orientations
//   * @param poseState        the poseState calculated from forward kinematics
//   * @param base             the base frame from which to calculate all the poses
//   * @param frames           the output frame map
//   */
// void PoseStateToFrameMap(const nasa_r2_common_msgs::PoseState& poseState, const std::string &base, std::map<std::string, KDL::Frame> &frames)
// {
//     KDL::Frame  frame;
//     std::string tipName;
//     if (frames.size() != poseState.name.size())
//     {
//         RCS::Logger::getCategory("gov.nasa.controllers.PoseStateToFrameMap")<<log4cpp::Priority::DEBUG<<"resizing frames";
//         frames.clear();
//         for (unsigned int i = 0; i < poseState.name.size(); ++i)
//         {
//             frames.insert(std::make_pair(poseState.name[i], KDL::Frame()));
//         }
//     }

//     for (unsigned int i = 0; i < poseState.name.size(); ++i)
//     {
//         tipName = poseState.name[i];
//         try
//         {
//             PoseStateToFrame(poseState, base, tipName, frame );
//             frames[tipName] = frame;
//         }
//         catch (std::exception &e)
//         {
//             RCS::Logger::getCategory("gov.nasa.controllers.PoseStateToFrameMap")<<log4cpp::Priority::WARN<<"Squelched error "<<e.what()<<".. continuing..";
//         }
//     }
// }

// /** @brief updates a map of all the frames in poseState with their new location and orientations
//   * @param poseState        the poseState calculated from forward kinematics
//   * @param base             the base frame from which to calculate all the poses
//   * @param framevels        the output frame map
//   */
// void PoseStateToFrameVelMap(const nasa_r2_common_msgs::PoseState& poseState, const std::string &base, std::map<std::string, KDL::FrameVel> &framevels)
// {
//     KDL::FrameVel frameVel;
//     std::string   tipName;
//     if(framevels.size() != poseState.name.size())
//     {
//         RCS::Logger::getCategory("gov.nasa.controllers.PoseStateToFrameVelMap")<<log4cpp::Priority::DEBUG<<"resizing framevels";
//         framevels.clear();
//         for (unsigned int i = 0; i < poseState.name.size(); ++i)
//         {
//             framevels.insert(std::make_pair(poseState.name[i], KDL::FrameVel()));
//         }
//     }
//     for (unsigned int i = 0; i < poseState.name.size(); ++i)
//     {
//         tipName = poseState.name[i];
//         try
//         {
//             PoseStateToFrameVel(poseState, base, tipName, frameVel );
//             framevels[tipName] = frameVel;
//         }
//         catch (std::exception &e)
//         {
//             RCS::Logger::getCategory("gov.nasa.controllers.PoseStateToFrameVelMap")<<log4cpp::Priority::WARN<<"Squelched error "<<e.what()<<".. continuing..";
//         }
//     }
// }

// /** @brief updates a map of all the frames in poseState with their new location and orientations
//   * @param poseState        the poseState calculated from forward kinematics
//   * @param base             the base frame from which to calculate all the poses
//   * @param frameaccs        the output frame map
//   */
// void PoseStateToFrameAccMap(const nasa_r2_common_msgs::PoseState& poseState, const std::string &base, std::map<std::string, KDL::FrameAcc> &frameaccs)
// {
//     KDL::FrameAcc frameAcc;
//     std::string   tipName;
//     if (frameaccs.size() != poseState.name.size())
//     {
//         RCS::Logger::getCategory("gov.nasa.controllers.PoseStateToFrameAccMap")<<log4cpp::Priority::DEBUG<<"resizing frameaccs";
//         frameaccs.clear();
//         for (unsigned int i = 0; i < poseState.name.size(); ++i)
//         {
//             frameaccs.insert(std::make_pair(poseState.name[i], KDL::FrameAcc()));
//         }
//     }
//     for(unsigned int i = 0; i<poseState.name.size(); ++i)
//     {
//         tipName = poseState.name[i];
//         try
//         {
//             RosMsgConverter::PoseStateToFrameAcc(poseState, base, tipName, frameAcc );
//             frameaccs[tipName] = frameAcc;
//         }
//         catch (std::exception &e)
//         {
//             RCS::Logger::getCategory("gov.nasa.controllers.PoseStateToFrameAccMap")<<log4cpp::Priority::WARN<<"Squelched error "<<e.what()<<".. continuing..";
//         }
//     }
// }

// /** @brief JointStateToJntMap creates a map of jointName and position
//   * @param jointState       the jointState message
//   * @param jointMap         the output map of jointName and position
//   */
// void JointStateToJntMap(const sensor_msgs::JointState& jointState, std::map<std::string, double> &jointMap)
// {
//     if (jointState.name.size() != jointState.position.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointState name length doesn't match positions length: " << jointState.name.size() << " vs " << jointState.position.size();
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJntMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointMap.empty())
//     {
//         //! if jointmap is empty, add all elements
//         for (unsigned int i = 0; i < jointState.name.size(); ++i)
//         {
//             jointMap[jointState.name[i]] = jointState.position[i];
//         }
//     }
//     else
//     {
//         //! otherwise, locate those elements in jointMap and only add them
//         std::map<std::string, double>::iterator jmIt;
//         for (unsigned int i = 0; i < jointState.name.size(); ++i)
//         {
//             jmIt = jointMap.find(jointState.name[i]);
//             if (jmIt != jointMap.end())
//             {
//                 jmIt->second = jointState.position[i];
//             }
//         }
//     }
// }

// /** @brief JointStateVel ToJntMap creates a map of jointName and velocity
//   * @param jointState       the jointState message
//   * @param jointMap         the output map of jointName and velocity
//   */
// void JointStateToJointVelMap(const sensor_msgs::JointState& jointState, std::map<std::string, JointVel> &jointMap)
// {
//     if (jointState.name.size() != jointState.velocity.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointState name length doesn't match velocity length: " << jointState.name.size() << " vs " << jointState.velocity.size();
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJointVelMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointState.name.size() != jointState.position.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointState name length doesn't match positions length: " << jointState.name.size() << " vs " << jointState.position.size();
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJointVelMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointMap.empty())
//     {
//         //! if jointmap is empty, add all elements
//         for (unsigned int i = 0; i < jointState.name.size(); ++i)
//         {
//             jointMap[jointState.name[i]] = (JointVel) {
//                 jointState.position[i], jointState.velocity[i]
//             };
//         }
//     }
//     else
//     {
//         //! otherwise, locate those elements in jointMap and only addd them
//         std::map<std::string, JointVel>::iterator jmIt;
//         for (unsigned int i = 0; i < jointState.name.size(); ++i)
//         {
//             jmIt = jointMap.find(jointState.name[i]);
//             if (jmIt != jointMap.end())
//             {
//                 jmIt->second = (JointVel) {
//                     jointState.position[i], jointState.velocity[i]
//                 };
//             }
//         }
//     }
// }

// /** @brief JointStateEffortToJntMap ToJntMap creates a map of jointName and effort
//   * @param jointState       the jointState message
//   * @param jointMap         the output map of jointName and effort
//   */
// void JointStateToJointAccMap(const sensor_msgs::JointState& jointState, std::map<std::string, JointAcc> &jointMap)
// {
//     if (jointState.name.size() != jointState.effort.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointState name length doesn't match effort length: " << jointState.name.size() << " vs " << jointState.effort.size();
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJointAccMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointState.name.size() != jointState.velocity.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointState name length doesn't match velocity length: " << jointState.name.size() << " vs " << jointState.velocity.size();
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJointAccMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointState.name.size() != jointState.position.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointState name length doesn't match positions length: " << jointState.name.size() << " vs " << jointState.position.size();
//         RCS::Logger::log("gov.nasa.controllers.JointStateToJointAccMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if(jointMap.empty())
//     {
//         //! if jointmap is empty, add all elements
//         for (unsigned int i = 0; i < jointState.name.size(); ++i)
//         {
//             jointMap[jointState.name[i]] = (JointAcc) {
//                 jointState.position[i], jointState.velocity[i], jointState.effort[i]
//             };
//         }
//     }
//     else
//     {
//         //! otherwise, locate those elements in jointMap and only addd them
//         std::map<std::string, JointAcc>::iterator jmIt;
//         for (unsigned int i = 0; i < jointState.name.size(); ++i)
//         {
//             jmIt = jointMap.find(jointState.name[i]);
//             if (jmIt != jointMap.end())
//             {
//                 jmIt->second = (JointAcc) {
//                     jointState.position[i], jointState.velocity[i], jointState.effort[i]
//                 };
//             }
//         }
//     }
// }

// void JointControlDataToCoeffStateMap(const nasa_r2_common_msgs::JointControlDataArray &jointData,
//                                      std::map<std::string, nasa_r2_common_msgs::JointControlCoeffState> &jointMap)
// {
//     if (jointData.joint.size() != jointData.data.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointData name length doesn't match data length: " << jointData.joint.size() << " vs " << jointData.data.size();
//         RCS::Logger::log("gov.nasa.controllers.JointControlDataToCoeffLoadedMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointMap.empty())
//     {
//         //! if jointmap is empty, add all elements
//         for (unsigned int i = 0; i < jointData.joint.size(); ++i)
//         {
//             jointMap[jointData.joint[i]] = jointData.data[i].coeffState;
//         }
//     }
//     else
//     {
//         //! otherwise, locate those elements in jointMap and only addd them
//         std::map<std::string, nasa_r2_common_msgs::JointControlCoeffState>::iterator jmIt;
//         for (unsigned int i = 0; i < jointData.joint.size(); ++i)
//         {
//             jmIt = jointMap.find(jointData.joint[i]);
//             if (jmIt != jointMap.end())
//             {
//                 jmIt->second = jointData.data[i].coeffState;
//             }
//         }
//     }
// }

// KDL::Wrench wrenchMsgToWrench(const geometry_msgs::Wrench& wrenchMsg)
// {
//     return (KDL::Wrench(KDL::Vector(wrenchMsg.force.x, wrenchMsg.force.y, wrenchMsg.force.z),
//                         KDL::Vector(wrenchMsg.torque.x, wrenchMsg.torque.y, wrenchMsg.torque.z)));
// }

// geometry_msgs::Wrench wrenchToWrenchMsg(const KDL::Wrench &wrench)
// {
//     geometry_msgs::Wrench wrenchOut;
//     wrenchOut.force.x  = wrench.force.x();
//     wrenchOut.force.y  = wrench.force.y();
//     wrenchOut.force.z  = wrench.force.z();
//     wrenchOut.torque.x = wrench.torque.x();
//     wrenchOut.torque.y = wrench.torque.y();
//     wrenchOut.torque.z = wrench.torque.z();
//     return wrenchOut;
// }

// void JointCommandToJointCommandMap(const nasa_r2_common_msgs::JointCommand &jointCommand, std::map<std::string, JointCommand> &jointCommandMap)
// {
//     if (jointCommand.name.size() != jointCommand.desiredPosition.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match desiredPosition length: " << jointCommand.name.size() << " vs " << jointCommand.desiredPosition.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointCommand.name.size() != jointCommand.desiredPositionVelocityLimit.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match desiredPositionVelocityLimit length: " << jointCommand.name.size() << " vs " << jointCommand.desiredPositionVelocityLimit.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointCommand.name.size() != jointCommand.feedForwardTorque.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match feedForwardTorque length: " << jointCommand.name.size() << " vs " << jointCommand.feedForwardTorque.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointCommand.name.size() != jointCommand.proportionalGain.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match proportionalGain length: " << jointCommand.name.size() << " vs " << jointCommand.proportionalGain.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }
//     if (jointCommand.name.size() != jointCommand.derivativeGain.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match derivativeGain length: " << jointCommand.name.size() << " vs " << jointCommand.derivativeGain.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }
//     if (jointCommand.name.size() != jointCommand.integralGain.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match integralGain length: " << jointCommand.name.size() << " vs " << jointCommand.integralGain.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }
//     if (jointCommand.name.size() != jointCommand.positionLoopTorqueLimit.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match positionLoopTorqueLimit length: " << jointCommand.name.size() << " vs " << jointCommand.positionLoopTorqueLimit.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }
//     if (jointCommand.name.size() != jointCommand.positionLoopWindupLimit.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match positionLoopWindupLimit length: " << jointCommand.name.size() << " vs " << jointCommand.positionLoopWindupLimit.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }
//     if (jointCommand.name.size() != jointCommand.torqueLoopVelocityLimit.size())
//     {
//         // length mismatch
//         std::stringstream err;
//         err << "jointCommand name length doesn't match torqueLoopVelocityLimit length: " << jointCommand.name.size() << " vs " << jointCommand.torqueLoopVelocityLimit.size();
//         RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     if (jointCommandMap.empty())
//     {
//         //! if jointmap is empty, add all elements
//         for (unsigned int i = 0; i < jointCommand.name.size(); ++i)
//         {
//             jointCommandMap[jointCommand.name[i]] = (JointCommand) {
//                 jointCommand.desiredPosition[i],
//                                              jointCommand.desiredPositionVelocityLimit[i],
//                                              jointCommand.feedForwardTorque[i],
//                                              jointCommand.proportionalGain[i],
//                                              jointCommand.derivativeGain[i],
//                                              jointCommand.integralGain[i],
//                                              jointCommand.positionLoopTorqueLimit[i],
//                                              jointCommand.positionLoopWindupLimit[i],
//                                              jointCommand.torqueLoopVelocityLimit[i]
//             };
//         }
//     }
//     else
//     {
//         //! otherwise, locate those elements in jointMap and only add them
//         for (unsigned int i = 0; i < jointCommand.name.size(); ++i)
//         {
//             try
//             {
//                 jointCommandMap.at(jointCommand.name[i]) = (JointCommand) {
//                     jointCommand.desiredPosition[i],
//                                                  jointCommand.desiredPositionVelocityLimit[i],
//                                                  jointCommand.feedForwardTorque[i],
//                                                  jointCommand.proportionalGain[i],
//                                                  jointCommand.derivativeGain[i],
//                                                  jointCommand.integralGain[i],
//                                                  jointCommand.positionLoopTorqueLimit[i],
//                                                  jointCommand.positionLoopWindupLimit[i],
//                                                  jointCommand.torqueLoopVelocityLimit[i]
//                 };
//             }
//             catch (std::exception &e)
//             {
//                 // not found
//                 std::stringstream err;
//                 err << "jointCommandMap doesn't contain joint: " << jointCommand.name[i] << ".. skipping..";
//                 RCS::Logger::log("gov.nasa.controllers.JointCommandToJointCommandMap", log4cpp::Priority::DEBUG, err.str());
//                 err.str("");
//             }
//         }
//     }
// }

// void WrenchStateToWrenchMap(const nasa_r2_common_msgs::WrenchState &wrenchState, std::map<std::string, KDL::Wrench> &wrenchMap)
// {
//     if (wrenchState.name.size() != wrenchState.wrench.size())
//     {
//         std::stringstream err;
//         err << "wrenchstate name length doesn't match wrench length: " << wrenchState.name.size() << " vs " << wrenchState.wrench.size();
//         RCS::Logger::log("gov.nasa.controllers.WrenchStateToWrenchMap", log4cpp::Priority::ERROR, err.str());
//         throw std::logic_error(err.str());
//         return;
//     }

//     for (unsigned int i = 0; i < wrenchState.name.size(); ++i)
//     {
//         wrenchMap[wrenchState.name.at(i)] = wrenchMsgToWrench(wrenchState.wrench.at(i));
//     }

// }
// }

