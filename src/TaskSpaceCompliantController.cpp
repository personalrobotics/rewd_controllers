#include <rewd_controllers/TaskSpaceCompliantController.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>

#include <aikido/common/Spline.hpp>
#include <aikido/control/ros/Conversions.hpp>
#include <aikido/statespace/Rn.hpp>
#include <aikido/statespace/SO2.hpp>
#include <aikido/statespace/dart/MetaSkeletonStateSpace.hpp>
#include <aikido/statespace/dart/RnJoint.hpp>
#include <aikido/trajectory/Spline.hpp>
#include <dart/dynamics/dynamics.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <pluginlib/class_list_macros.h>

using aikido::statespace::dart::MetaSkeletonStateSpace;
using aikido::statespace::dart::R1Joint;


namespace rewd_controllers {

namespace internal
{

inline std::string getLeafNamespace(const ros::NodeHandle& nh)
{
	const std::string complete_ns = nh.getNamespace();
	std::size_t id   = complete_ns.find_last_of("/");
	return complete_ns.substr(id + 1);
}

} // namespace

TaskSpaceCompliantController::TaskSpaceCompliantController() 
		: MultiInterfaceController(true) // allow_optional_interfaces
		, urdf_model(luca_dynamics::create_model_from_urdf("/home/rkjenamani/controller_ws/src/compliant-controller/models/kinova.urdf"))
		, dyn(new luca_dynamics::luca_dynamics(urdf_model))
{}

//=============================================================================
TaskSpaceCompliantController::~TaskSpaceCompliantController() 
{} 

bool TaskSpaceCompliantController::init(hardware_interface::RobotHW *robot, ros::NodeHandle &n) {

	using hardware_interface::EffortJointInterface;
	using hardware_interface::JointStateInterface;

	mNodeHandle.reset(new ros::NodeHandle{n});

	// Build up the list of controlled DOFs.
	const auto jointParameters = loadJointsFromParameter(n, "joints", "effort");
	if (jointParameters.empty())
		return false;

	ROS_INFO_STREAM("Controlling " << jointParameters.size() << " joints:");
	for (const auto &param : jointParameters) {
		ROS_INFO_STREAM("- " << param.mName << " (type: " << param.mType << ")");

		if (param.mType != "effort") {
			ROS_ERROR_STREAM("Joint '"
											 << param.mName
											 << "' is not effort-controlled and cannot be "
													"used in a gravity compensation controller");
			return false;
		}
	}

	// Load the URDF as a Skeleton.
	mSkeleton = loadRobotFromParameter(n, "robot_description_parameter");
	if (!mSkeleton)
		return false;

	// // Check for zero-mass bodies that will be used incorrectly in calculations
	// bool hasZeroMassBody = false;
	// for (auto body : mSkeleton->getBodyNodes()) {
	//   if (body->getMass() <= 0.0) {
	//     ROS_ERROR_STREAM("Robot link '" << body->getName()
	//                                     << "' has mass = " << body->getMass());
	//     hasZeroMassBody = true;
	//   }
	// }
	// if (hasZeroMassBody)
	//   return false; // TODO is this actually a problem?

	// Extract the subset of the Skeleton that is being controlled.
	mControlledSkeleton =
			getControlledMetaSkeleton(mSkeleton, jointParameters, "Controlled");
	if (!mControlledSkeleton)
		return false;

	// the full skeleton.
	const auto jointStateInterface = robot->get<JointStateInterface>();
	if (!jointStateInterface) {
		ROS_ERROR("Unable to get JointStateInterface from RobotHW instance.");
		return false;
	}

	mSkeletonUpdater.reset(
			new SkeletonJointStateUpdater{mSkeleton, jointStateInterface});

	const auto numControlledDofs = mControlledSkeleton->getNumDofs();

	ROS_INFO_STREAM("numControlledDofs: " << (int)numControlledDofs);

	mControlledJointHandles.resize(numControlledDofs);
	mDofs.resize(numControlledDofs);

	const auto effortJointInterface = robot->get<EffortJointInterface>();
	if (!effortJointInterface) {
		ROS_ERROR("Unable to get EffortJointInterface from RobotHW instance.");
		return false;
	}

	for (size_t idof = 0; idof < numControlledDofs; ++idof) {
		const auto dofName = mControlledSkeleton->getDof(idof)->getName();
		mDofs[idof] = mControlledSkeleton->getDof(idof);
		try {
			auto handle = effortJointInterface->getHandle(dofName);
			mControlledJointHandles[idof] = handle;
		} catch (const hardware_interface::HardwareInterfaceException &e) {
			ROS_ERROR_STREAM(
					"Unable to get interface of type 'EffortJointInterface' for joint '"
					<< dofName << "'.");
			return false;
		}
	}

	mEENode = mSkeleton->getBodyNode(std::string("inline_forque_end_effector"));

	mExtendedJoints = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);
	mExtendedJointsGravity = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);

	mCount = 0;  

	mJointStiffnessMatrix.resize(numControlledDofs, numControlledDofs);
	mJointStiffnessMatrix.setZero();
	mJointStiffnessMatrix.diagonal() << 8000,8000,8000,7000,7000,7000;

	mRotorInertiaMatrix.resize(numControlledDofs, numControlledDofs);
	mRotorInertiaMatrix.setZero();
	mRotorInertiaMatrix.diagonal() << 0.4, 0.4, 0.4, 0.2, 0.2, 0.2;

	mFrictionL.resize(numControlledDofs, numControlledDofs);
	mFrictionL.setZero();
	mFrictionL.diagonal() << 160, 160, 160, 100, 100, 100;

	mFrictionLp.resize(numControlledDofs, numControlledDofs);
	mFrictionLp.setZero();
	mFrictionLp.diagonal() << 10, 10, 10, 7.5, 7.5, 7.5;

	mJointKMatrix.resize(numControlledDofs, numControlledDofs);
	mJointKMatrix.setZero();
	mJointKMatrix.diagonal() << 80,80,80,60,60,60;

	mJointDMatrix.resize(numControlledDofs, numControlledDofs);
	mJointDMatrix.setZero();
	mJointDMatrix.diagonal() << 8,8,8,6,6,6;

	mTaskKMatrix.resize(6, 6);
	mTaskKMatrix.setZero();
	mTaskKMatrix.diagonal() << 200,200,200,150,150,150;
	// mTaskKMatrix.diagonal() << 50,50,50,35,35,35;

	mTaskDMatrix.resize(6, 6);
	mTaskDMatrix.setZero();
	// mTaskDMatrix.diagonal() << 40,40,40,30,30,30;
	// mTaskDMatrix.diagonal() << 20,20,20,15,15,15;
	mTaskDMatrix.diagonal() << 60,60,60,40,40,40;

	mTaskIMatrix.resize(6, 6);
	mTaskIMatrix.setZero();
	mTaskIMatrix.diagonal() << 0.3, 0.3, 0.3, 0.3, 0.3, 0.3;

	mContactKMatrix.resize(6, 6);
	mContactKMatrix.setZero();
	mContactKMatrix.diagonal() << 7.0, 7.0, 7.0, 7.0, 7.0, 7.0;

	mContactIMatrix.resize(6, 6);
	mContactIMatrix.setZero();
	mContactIMatrix.diagonal() << 15, 15, 15, 15, 15, 15;

	mUseIntegralTermMaxThreshold.resize(6);
	mUseIntegralTermMaxThreshold << 0.015, 0.015, 0.015, M_PI/30, M_PI/30, M_PI/30;

	mUseIntegralTermMinThreshold.resize(6);
	mUseIntegralTermMinThreshold << 0.003, 0.003, 0.003, M_PI/180, M_PI/180, M_PI/180;

	mUseIntegralTermForqueFrameMaxThreshold = 0.015;
	mUseIntegralTermForqueFrameMinThreshold = 0.002;

	// Initialize buffers to avoid dynamic memory allocation at runtime.
	mDesiredPosition.resize(numControlledDofs);
	mDesiredVelocity.resize(numControlledDofs);
	mZeros.resize(numControlledDofs);
	mZeros.setZero();

	mContactIntegral.resize(numControlledDofs);
	mContactIntegral.setZero();

	mTaskPoseIntegral.resize(numControlledDofs);
	mTaskPoseIntegral.setZero();

	mName = internal::getLeafNamespace(n);

	// Initialize controlled joints
	std::string param_name = "joints";
	if(!n.getParam(param_name, mJointNames))
	{
		ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << n.getNamespace() << ").");
		return false;
	}

	// Action status checking update rate
	double action_monitor_rate = 20.0;
	n.getParam("action_monitor_rate", action_monitor_rate);
	mActionMonitorPeriod = ros::Duration(1.0 / action_monitor_rate);
	ROS_DEBUG_STREAM_NAMED(mName, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

	// Rajat ToDo: Add initial state of controller here

	// ROS API: Subscribed topics
	mSubCommand = n.subscribe<moveit_msgs::CartesianTrajectoryPoint>("command", 1, &TaskSpaceCompliantController::commandCallback, this);

	mSubFTSensor = n.subscribe("/forque/forqueSensor", 1, &TaskSpaceCompliantController::forceTorqueDataCallback, this);

	// Start the action server. This must be last.
	// using std::placeholders::_1; // Rajat check: is this required?

	// ROS API: Action interface
	mActionServer.reset(new ActionServer(n, "joint_group_command",
																			boost::bind(&TaskSpaceCompliantController::goalCallback, this, _1),
																			boost::bind(&TaskSpaceCompliantController::cancelCallback, this, _1),
																			false));
	mActionServer->start();

	ROS_INFO("TaskSpaceCompliantController initialized successfully");
	return true;
}

//=============================================================================
void TaskSpaceCompliantController::forceTorqueDataCallback(
    const geometry_msgs::WrenchStamped &msg) {
  std::lock_guard<std::mutex> lock(mForceTorqueDataMutex);
  mForce.x() = msg.wrench.force.x;
  mForce.y() = msg.wrench.force.y;
  mForce.z() = msg.wrench.force.z;
  mTorque.x() = msg.wrench.torque.x;
  mTorque.y() = msg.wrench.torque.y;
  mTorque.z() = msg.wrench.torque.z;

  // if (mForce.norm() < 0.005)
  // 	mZeroCount = 0;
  // else
  // 	mZeroCount++;

  // if(mZeroCount < 100)
  // 	mForce << 0.0, 0.0, 0.0;
}

//=============================================================================
void TaskSpaceCompliantController::starting(const ros::Time &time) {

	mExecuteDefaultCommand = true;

	mLastTimePoint = std::chrono::high_resolution_clock::now();

	ROS_DEBUG_STREAM(
			"Initialized desired position: " << mDesiredPosition.transpose());
	ROS_DEBUG_STREAM(
			"Initialized desired velocity: " << mDesiredVelocity.transpose());

	ROS_DEBUG("Reset PID.");

}

//=============================================================================
void TaskSpaceCompliantController::stopping(const ros::Time &time) {
	preemptActiveGoal(); // Rajat check: Is this required?
}

//=============================================================================
void TaskSpaceCompliantController::update(const ros::Time &time, const ros::Duration &period) {

	
	auto current_time = std::chrono::high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(current_time - mLastTimePoint);

	// std::cout<<"Controller Frequency: "<<1000000.0/duration.count()<<std::endl;
	mLastTimePoint = std::chrono::high_resolution_clock::now();

	// Update the state of the Skeleton.
	mSkeletonUpdater->update();
	mActualPosition = mControlledSkeleton->getPositions();
	for (size_t idof = 0; idof < mControlledJointHandles.size(); ++idof) 
	{
		if (mActualPosition[idof] < 0)
			mActualPosition[idof] += 2*M_PI;
	}
	mActualVelocity = mControlledSkeleton->getVelocities();
	mActualEffort = mControlledSkeleton->getForces();
	mActualEETransform =  mEENode->getWorldTransform();

	// if(true)
	//  Gravity compensation for switching controllers
	if(mCount < 2000)
	{
		mSkeleton->computeInverseDynamics();
		mGravity = mControlledSkeleton->getGravityForces();
		
		if(mCount%200 == 0)
			std::cout<<"Initializing controller while in gravity compensation: "<<mCount<<std::endl; 
		mCount++;	

		for (size_t idof = 0; idof < mControlledJointHandles.size(); ++idof) 
		{
			auto jointHandle = mControlledJointHandles[idof];
			jointHandle.setCommand(mGravity[idof]);
		}

		setActionFeedback(time);  // Rajat check: Is this required?
		return;
	}

	Eigen::MatrixXd dart_actual_jacobian(6, 6); // change to numControlledDofs
	{
		mControlledSkeleton->setAccelerations(mZeros); 
		mSkeleton->computeInverseDynamics();
		Eigen::MatrixXd dart_actual_jacobian_flipped(6, 6);
		Eigen::MatrixXd fullJ = mEENode->getWorldJacobian();
		// std::cout<<"\n"<<"Full Jacobian: "<<fullJ<<"\n\n";
		auto fullDofs = mEENode->getDependentDofs();
		// std::cout<<"fullDofs.size(): "<<fullDofs.size()<<std::endl;
		for (size_t fullIndex = 0; fullIndex < fullDofs.size(); fullIndex++)
		{
			auto it = std::find(mDofs.begin(), mDofs.end(), fullDofs[fullIndex]);
			if (it != mDofs.end())
			{
				int index = it - mDofs.begin();
				dart_actual_jacobian_flipped.col(index) = fullJ.col(fullIndex);
			}
		}

		for(int i=0; i<3; i++)
		{
			dart_actual_jacobian.row(i) = dart_actual_jacobian_flipped.row(i+3);
			dart_actual_jacobian.row(i+3) = dart_actual_jacobian_flipped.row(i);
		}
	}

	std::string stopReason;
	bool shouldStopExec = shouldStopExecution(stopReason);

	if(shouldStopExec)
	{
		std::cout<<"Controller halted due to: "<<stopReason<<std::endl;
		preemptActiveGoal();
		mExecuteDefaultCommand = true;
	}
	
	if(shouldStopExec || mExecuteDefaultCommand.load())
	{
		// std::cout<<"1. Updating desired position ..."<<std::endl;
		mDesiredPosition = mActualPosition;
		mDesiredVelocity.setZero();

		mControlledSkeleton->setPositions(mDesiredPosition);
		mControlledSkeleton->setVelocities(mZeros);
		mControlledSkeleton->setAccelerations(mZeros); 
		mDesiredEETransform = mEENode->getWorldTransform();

	}
	else
	{
		std::cout<<"Reading from RT Buffer... "<<std::endl;
		moveit_msgs::CartesianTrajectoryPoint command = *mCommandsBuffer.readFromRT(); // Rajat check: should this be by reference?
		std::cout<<"... Read. :| "<<std::endl;
	
		// std::cout<<"Efforts Size: "<<command.effort.size()<<std::endl;
		// std::cout<<"Positions Size: "<<command.positions.size()<<std::endl;
		// std::cout<<"Velocities Size: "<<command.velocities.size()<<std::endl;
		// std::cout<<"Accelerations Size: "<<command.accelerations.size()<<std::endl;
		geometry_msgs::Pose command_pose = command.point.pose;
		// std::cout<<"Received Command: "<<command_pose.position.x<<" "<<command_pose.position.y<<" "<<command_pose.position.z<<" "<<command_pose.orientation.x<<" "<<
			// command_pose.orientation.y<<" "<<command_pose.orientation.z<<" "<<command_pose.orientation.w<<std::endl;
		tf::poseMsgToEigen(command_pose, mDesiredEETransform);

		// for (const auto &dof : mControlledSkeleton->getDofs()) 
		// {
		// 	// Rajat ToDo: Find better method
		// 	std::size_t index = mControlledSkeleton->getIndexOf(dof);
		// 	mDesiredPosition[index] = (command.positions.size() == 0) ? 0.0 : command.positions[index];
		// 	mDesiredVelocity[index] = (command.velocities.size() == 0) ? 0.0 : command.velocities[index];
		// }
		// std::cout<<"Out. :) "<<std::endl;
		// mTaskPoseIntegral.setZero();
	}

	if (!mExtendedJoints->is_initialized){
		std::cout<<"2. Updating desired position ..."<<std::endl;
		mLastDesiredPosition = mDesiredPosition;
		mLastDesiredEETransform = mDesiredEETransform;
		mExtendedJoints->initializeExtendedJointPosition(mDesiredPosition);
		mExtendedJoints->estimateExtendedJoint(mDesiredPosition);
		mNominalThetaPrev = mExtendedJoints->getExtendedJoint();
		mNominalThetaDotPrev = mActualVelocity;
		mTrueDesiredPosition = mExtendedJoints->getExtendedJoint();
		mTrueDesiredVelocity = mDesiredVelocity;

		mTrueDesiredEETransform = mDesiredEETransform;
	}

	if ((mDesiredPosition != mLastDesiredPosition || !mDesiredEETransform.isApprox(mLastDesiredEETransform,0.0001) ) && mActualPosition != mDesiredPosition){
		std::cout<<"3. Updating desired position ..."<<std::endl;
		std::cout<<"mDesiredEETransform: "<<mDesiredEETransform.translation().transpose()<<std::endl;
		std::cout<<"mLastDesiredEETransform: "<<mLastDesiredEETransform.translation().transpose()<<std::endl;
		std::cout<<"(mDesiredPosition != mLastDesiredPosition): "<<(mDesiredPosition != mLastDesiredPosition)<<std::endl;
		std::cout<<"!mDesiredEETransform.isApprox(mLastDesiredEETransform,0.0001): "<<(!mDesiredEETransform.isApprox(mLastDesiredEETransform,0.0001))<<std::endl;
		mLastDesiredPosition = mDesiredPosition;
		mLastDesiredEETransform = mDesiredEETransform;
		mExtendedJoints->estimateExtendedJoint(mDesiredPosition);
		mTrueDesiredPosition = mExtendedJoints->getExtendedJoint();
		mTrueDesiredVelocity = mDesiredVelocity;
		mTrueDesiredEETransform = mDesiredEETransform;
	}
	// else
	// {
	// 	std::cout<<"Goal Matches.... "<<std::endl;
	// 	std::cout<<"mDesiredEETransform: "<<mDesiredEETransform.translation().transpose()<<std::endl;
	// 	std::cout<<"mLastDesiredEETransform: "<<mLastDesiredEETransform.translation().transpose()<<std::endl;
	// }

	Eigen::Quaterniond qd(mTrueDesiredEETransform.linear());
	// std::cout<<"Desired Pose: "<<mTrueDesiredEETransform.translation().transpose()<<" "<<qd.x()<<" "<<qd.y()<<" "<<qd.z()<<" "<<qd.w()<<std::endl;

	Eigen::Quaterniond q(mActualEETransform.linear());
	// std::cout<<"Current Pose: "<<mActualEETransform.translation().transpose()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;

	{

		mExtendedJointsGravity->is_initialized = false;
		mExtendedJointsGravity->initializeExtendedJointPosition(mDesiredPosition);
		mExtendedJointsGravity->estimateExtendedJoint(mExtendedJointsGravity->mLastDesiredPosition);
		mActualTheta = mExtendedJointsGravity->getExtendedJoint();

		mControlledSkeleton->setPositions(mActualTheta);
		mControlledSkeleton->setVelocities(mZeros);
		mControlledSkeleton->setAccelerations(mZeros); 
		mSkeleton->computeInverseDynamics();
		mGravity = mControlledSkeleton->getGravityForces();

		//compute quasi-static estimate of the link side position
	    //input value is motor side angle theta not link side angle(q);
	    //Number of iteration can be modified by edit int i ( recommend is 1 or 2 for real time computing)

	    int iteration = 2; // number of iteration
	    Eigen::VectorXd qs_estimate_link_pos(6);
	    qs_estimate_link_pos = mNominalThetaPrev;

	    for (int i=0; i<iteration; i++)
	    {
	    	mControlledSkeleton->setPositions(qs_estimate_link_pos);
			mControlledSkeleton->setVelocities(mZeros);
			mSkeleton->computeInverseDynamics();
	        qs_estimate_link_pos = mNominalThetaPrev - mJointStiffnessMatrix.inverse()*(mControlledSkeleton->getGravityForces());
	    }
	    mControlledSkeleton->setPositions(qs_estimate_link_pos);
		mControlledSkeleton->setVelocities(mZeros);
		mSkeleton->computeInverseDynamics();
        mQuasiGravity = mControlledSkeleton->getGravityForces();

		// Restore the state of the Skeleton from JointState interfaces. These values
		// may be used by the adapters below.
		mControlledSkeleton->setPositions(mActualPosition);
		mControlledSkeleton->setVelocities(mActualVelocity);

	}
	
	mExtendedJoints->estimateExtendedJoint(mActualPosition);
	mActualTheta = mExtendedJoints->getExtendedJoint();

	mDesiredTheta = mTrueDesiredPosition + mJointStiffnessMatrix.inverse()*mGravity;
	mDesiredThetaDot = mTrueDesiredVelocity;
	
	// mTaskEffort = -mJointKMatrix*(mNominalThetaPrev-mDesiredTheta) - mJointDMatrix*(mNominalThetaDotPrev - mDesiredThetaDot) + mGravity;

	//Compute error
	Eigen::VectorXd dart_error(6);   
	Eigen::MatrixXd dart_nominal_jacobian(6, 6); // change to numControlledDofs
	{
		mControlledSkeleton->setPositions(mNominalThetaPrev);
		mControlledSkeleton->setVelocities(mNominalThetaDotPrev);
		mControlledSkeleton->setAccelerations(mZeros); 
		mSkeleton->computeInverseDynamics(); // Do we need this to update jacobian?
		mNominalEETransform = mEENode->getWorldTransform();

		Eigen::Quaterniond nominal_ee_quat(mNominalEETransform.linear());
		
		// Get Jacobian relative only to controlled joints -- Rajat ToDo: Check with Ethan
		Eigen::MatrixXd dart_nominal_jacobian_flipped(6, 6);
		Eigen::MatrixXd fullJ = mEENode->getWorldJacobian();
		// std::cout<<"\n"<<"Full Jacobian: "<<fullJ<<"\n\n";
		auto fullDofs = mEENode->getDependentDofs();
		// std::cout<<"fullDofs.size(): "<<fullDofs.size()<<std::endl;
		for (size_t fullIndex = 0; fullIndex < fullDofs.size(); fullIndex++)
		{
			auto it = std::find(mDofs.begin(), mDofs.end(), fullDofs[fullIndex]);
			if (it != mDofs.end())
			{
				int index = it - mDofs.begin();
				dart_nominal_jacobian_flipped.col(index) = fullJ.col(fullIndex);
			}
		}

		for(int i=0; i<3; i++)
		{
			dart_nominal_jacobian.row(i) = dart_nominal_jacobian_flipped.row(i+3);
			dart_nominal_jacobian.row(i+3) = dart_nominal_jacobian_flipped.row(i);
		}

		dart_error.head(3) << mNominalEETransform.translation() - mTrueDesiredEETransform.translation(); // positional error
		
		Eigen::Quaterniond ee_quat_d(mTrueDesiredEETransform.linear());

		if (ee_quat_d.coeffs().dot(nominal_ee_quat.coeffs()) < 0.0) 
		{
				nominal_ee_quat.coeffs() << -nominal_ee_quat.coeffs();
		}
		Eigen::Quaterniond error_qtn(nominal_ee_quat.inverse() * ee_quat_d);
		dart_error.tail(3) << error_qtn.x(), error_qtn.y(), error_qtn.z();
		dart_error.tail(3) << -mNominalEETransform.linear() * dart_error.tail(3);


		// Restore the state of the Skeleton from JointState interfaces. These values
		// may be used by the adapters below.
		mControlledSkeleton->setPositions(mActualPosition);
		mControlledSkeleton->setVelocities(mActualVelocity);
	}

	double step_time;
	step_time = 0.001;

	// std::cout<<"\n"<<"Nominal Jacobian: "<<dart_nominal_jacobian<<"\n\n";

	mTaskEffort = dart_nominal_jacobian.transpose() * (-mTaskKMatrix * dart_error - mTaskDMatrix * (dart_nominal_jacobian * mNominalThetaDotPrev)) + mQuasiGravity;

	std::cout<<"\n dart_error: "<<dart_error.transpose()<<std::endl;

	if(mUseIntegralTerm)
	{
		bool useITerm = true;
		for(int i=0; i<6; i++)
		{
			if(std::abs(dart_error(i)) > mUseIntegralTermMaxThreshold(i))
			{
				useITerm = false;
				break;
			}
		}
		bool allLess = true;
		for(int i=0; i<6; i++)
			if(std::abs(dart_error(i)) > mUseIntegralTermMinThreshold(i))
			{
				allLess = false;
				break;
			}

		if(useITerm && !allLess)
		{
			mTaskPoseIntegral += dart_nominal_jacobian.transpose() * (-mTaskIMatrix * dart_error);

			// cap the i term
			for(int i=0; i<6; i++)
			{
				if(mTaskPoseIntegral(i) > 1)
					mTaskPoseIntegral(i) = 1;
				if(mTaskPoseIntegral(i) < -1)
					mTaskPoseIntegral(i) = -1;
			}

			// std::cout<<"mUseIntegralTermMaxThreshold: "<<mUseIntegralTermMaxThreshold.transpose()<<std::endl;

			mTaskEffort += mTaskPoseIntegral;
		}
		// else
		// {
		// 	mTaskPoseIntegral.setZero();
		// }
		// std::cout<<"mTaskPoseIntegral: "<<mTaskPoseIntegral.transpose()<<std::endl;
	}
	else if(mUseIntegralTermForqueFrame)
	{
		Eigen::Vector3d position_error(dart_error(0), dart_error(1), dart_error(2));
		Eigen::MatrixXd mActualEERotation = mActualEETransform.linear();
		position_error = mActualEERotation.inverse() * position_error; 
		std::cout<<"BF Position error in FT frame: "<<position_error.transpose()<<std::endl;
		position_error(2) = 0; // zero out the z term in FT sensor frame of reference

		// if(std::abs(position_error(0)) > mUseIntegralTermForqueFrameMaxThreshold || std::abs(position_error(1)) > mUseIntegralTermForqueFrameMaxThreshold)
		// {
		// 	position_error(0) = 0;
		// 	position_error(1) = 0;
		// }
		// if(std::abs(position_error(0)) < mUseIntegralTermForqueFrameMinThreshold)
		// 	position_error(0) = 0;
		// if(std::abs(position_error(1)) < mUseIntegralTermForqueFrameMinThreshold)
		// 	position_error(1) = 0;

		if(position_error.norm() > mUseIntegralTermForqueFrameMaxThreshold)
		{
			position_error(0) = 0;
			position_error(1) = 0;
			position_error(2) = 0;
		}

		if(std::abs(position_error(0)) < mUseIntegralTermForqueFrameMinThreshold)
			position_error(0) = 0;
		if(std::abs(position_error(1)) < mUseIntegralTermForqueFrameMinThreshold)
			position_error(1) = 0;

		std::cout<<"AF Position error in FT frame: "<<position_error.transpose()<<std::endl;

		position_error = mActualEERotation * position_error;

		Eigen::VectorXd error_wrench(6);	
		error_wrench << position_error(0), position_error(1), position_error(2), 0.0, 0.0, 0.0;

		if(error_wrench.norm() > 0.00000001)
		{

			mTaskPoseIntegral += dart_nominal_jacobian.transpose() * (-mTaskIMatrix * error_wrench);

			// cap the i term
			for(int i=0; i<6; i++)
			{
				if(mTaskPoseIntegral(i) > 1.0)
					mTaskPoseIntegral(i) = 1.0;
				if(mTaskPoseIntegral(i) < -1.0)
					mTaskPoseIntegral(i) = -1.0;
			}

			mTaskEffort += mTaskPoseIntegral;
		}
		else
		{
			mTaskPoseIntegral.setZero();
		}

		std::cout<<"mTaskPoseIntegral: "<<mTaskPoseIntegral.transpose()<<std::endl;
	}

	if(mUseContactData)
	{
		Eigen::Vector3d force;
		{
			std::lock_guard<std::mutex> lock(mForceTorqueDataMutex);
			force << mForce.x(), mForce.y(), mForce.z();
			// force << mForce.x(), mForce.y(), 0.0;
		}

		Eigen::Vector3d filter(0.1, 0.1, 0.1);

		for(int i=0; i<3; i++)
		{
			// if (force(i) > 4 || force(i) < -4)
				// force(i) = 4;
			// else 
			if (force(i) >= 0)
				force(i) = std::max(0.0,force(i)-filter(i));
			else
				force(i) = std::min(0.0,force(i)+filter(i));
		}

		// force(1) += 2; //maintain 2N force in y-axis
		// force(0) = 0;
		// force(2) = 0;

		force = mActualEETransform.linear() * force; // force in world frame

		std::cout<<"FT Sensor Forces: "<<force.transpose()<<std::endl;

		// if (force.norm() > 0.0)
		{
			Eigen::VectorXd ft_wrench(6);	
			ft_wrench << force(0), force(1), force(2), 0.0, 0.0, 0.0;

			mContactIntegral += step_time*(ft_wrench);

			// // Cap I term
			// for(int i=0; i<3; i++)
			// {
			// 	if(mContactIntegral(i) > 2)
			// 		mContactIntegral(i) = 2;
			// 	if(mContactIntegral(i) < -2)
			// 		mContactIntegral(i) = -2;
			// }

			// mContactEffort = dart_actual_jacobian.transpose() * (mContactKMatrix * ft_wrench);
			// mContactEffort = dart_actual_jacobian.transpose() * (mContactKMatrix * ft_wrench + mContactIMatrix * mContactIntegral);
			mContactEffort = dart_actual_jacobian.transpose() * (mContactIMatrix * mContactIntegral);
		}
		// else
		// {
		// 	mContactEffort = mZeros;
		// 	mContactIntegral.setZero();
		// }

		// SAFETY
		for(int i=0; i<6; i++)
		{
			if(mContactEffort(i) > 20)
				mContactEffort(i) = 20;
			if(mContactEffort(i) < -20)
				mContactEffort(i) = -20;
		}

		// std::cout<<"Contact Effort: "<<mContactEffort.transpose()<<std::endl;
		// std::cout<<"Contact Integral Term: "<< (dart_actual_jacobian.transpose()*mContactIMatrix * mContactIntegral).transpose()<<std::endl;

		if(mCount>5000)
		{
			mTaskEffort = mTaskEffort + mContactEffort;
			// std::cout<<"Adding to mTaskEffort!"<<std::endl;
		}
		else if(mCount >= 3000 && mCount<=5000)
		{
			if(mCount%200 == 0)
				std::cout<<"Initializing controller: "<<mCount<<std::endl; 
			mCount++;		
		}
	}

	mNominalThetaDDot = mRotorInertiaMatrix.inverse()*(mTaskEffort+mActualEffort); // mActualEffort is negative of what is required here
	mNominalThetaDot = mNominalThetaDotPrev + mNominalThetaDDot*step_time;
	mNominalTheta = mNominalThetaPrev + mNominalThetaDot*step_time;

	mNominalThetaPrev = mNominalTheta;
	mNominalThetaDotPrev = mNominalThetaDot;

	mNominalFriction = mRotorInertiaMatrix*mFrictionL*((mNominalThetaDotPrev - mActualVelocity) + mFrictionLp*(mNominalThetaPrev - mActualTheta));

	mDesiredEffort = mTaskEffort + mNominalFriction;


	// Gravity Compensation for initializing nominal values
	if(mCount < 3000)
	{
		mDesiredEffort = mGravity;
		if(mCount%200 == 0)
			std::cout<<"Initializing controller while in gravity compensation: "<<mCount<<std::endl; 
		mCount++;
	}

	// std::cout<<"mTaskEffort: "<<mTaskEffort.transpose()<<std::endl;
	// std::cout<<"mNominalFriction: "<<mNominalFriction.transpose()<<std::endl;
	// std::cout<<"mActualTheta: "<<mActualTheta.transpose()<<std::endl;
	// std::cout<<"mActualEffort: "<<mActualEffort.transpose()<<std::endl;

	for (size_t idof = 0; idof < mControlledJointHandles.size(); ++idof) 
	{
		auto jointHandle = mControlledJointHandles[idof];
		jointHandle.setCommand(mDesiredEffort[idof]);
	}

	setActionFeedback(time);  // Rajat check: Is this required?
}

void TaskSpaceCompliantController::preemptActiveGoal()
{
		// RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

		// // Cancel any goal timeout
		// mGoalDurationTimer.stop();

		// // Cancels the currently active goal
		// if (current_active_goal)
		// {
		// 	// Marks the current goal as canceled
		// 	mRTActiveGoal.reset();
		// 	current_active_goal->gh_.setCanceled();
		// }
}

void TaskSpaceCompliantController::commandCallback(const moveit_msgs::CartesianTrajectoryPointConstPtr& msg)
{
	// Preconditions
	if (!shouldAcceptRequests())
	{
		ROS_ERROR_STREAM_NAMED(mName, "Can't accept new commands. Controller is not running.");
		return;
	}

	if (!msg)
	{
		ROS_WARN_STREAM_NAMED(mName, "Received null-pointer message, skipping.");
		return;
	}

	mCommandsBuffer.writeFromNonRT(*msg);
	preemptActiveGoal();
	mExecuteDefaultCommand = false;
}

void TaskSpaceCompliantController::goalCallback(GoalHandle gh)
{
	// std::cout<<"Joint group command controller: Recieved new goal!"<<std::endl;
	// ROS_DEBUG_STREAM_NAMED(mName,"Received new action goal");
	// pr_control_msgs::TaskCommandResult result;

	// // Preconditions
	// if (!shouldAcceptRequests())
	// {
	// 	result.error_string = "Can't accept new action goals. Controller is not running.";
	// 	ROS_ERROR_STREAM_NAMED(mName, result.error_string);
	// 	result.error_code = pr_control_msgs::TaskCommandResult::INVALID_GOAL;
	// 	gh.setRejected(result);
	// 	return;
	// }

	// // if (gh.getGoal()->joint_names.size() != gh.getGoal()->command.positions.size()) {
	// //   result.error_string = "Size of command must match size of joint_names.";
	// //   ROS_ERROR_STREAM_NAMED(mName, result.error_string);
	// //   result.error_code = pr_control_msgs::TaskCommandResult::INVALID_GOAL;
	// //   gh.setRejected(result);
	// //   return;
	// // }

	// // Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case

	// // update new command
	// RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
	// moveit_msgs::CartesianTrajectoryPoint new_command = gh.getGoal()->command;
	// rt_goal->preallocated_feedback_->joint_names = mJointNames;
	// mCommandsBuffer.writeFromNonRT(new_command);

	// 	// Accept new goal
	// preemptActiveGoal();
	// gh.setAccepted();
	// mRTActiveGoal = rt_goal;
	// mExecuteDefaultCommand = false;

	// // Setup goal status checking timer
	// mGoalHandleTimer = mNodeHandle->createTimer(mActionMonitorPeriod,
	// 																									&RealtimeGoalHandle::runNonRealtime,
	// 																									rt_goal);
	// mGoalHandleTimer.start();

	// // Setup goal timeout
	// if (gh.getGoal()->command.time_from_start > ros::Duration()) {
	// 	mGoalDurationTimer = mNodeHandle->createTimer(gh.getGoal()->command.time_from_start,
	// 																									&TaskSpaceCompliantController::timeoutCallback,
	// 																									this,
	// 																									true);
	// 	mGoalDurationTimer.start();
	// }
}

void TaskSpaceCompliantController::timeoutCallback(const ros::TimerEvent& event)
{
	// RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

	// // Check that timeout refers to currently active goal (if any)
	// if (current_active_goal) {
	// 	ROS_DEBUG_NAMED(mName, "Active action goal reached requested timeout.");

	// 	// Give sub-classes option to update mDefaultCommand
	// 	mExecuteDefaultCommand = true;

	// 	// Marks the current goal as succeeded
	// 	mRTActiveGoal.reset();
	// 	current_active_goal->gh_.setSucceeded();
	// }
}

void TaskSpaceCompliantController::cancelCallback(GoalHandle gh)
{
	// RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

	// // Check that cancel request refers to currently active goal
	// if (current_active_goal && current_active_goal->gh_ == gh)
	// {
	// 	ROS_DEBUG_NAMED(mName, "Canceling active action goal because cancel callback recieved from actionlib.");

	// 	// Give sub-classes option to update mDefaultCommand
	// 	mExecuteDefaultCommand = true;

	// 	preemptActiveGoal();
	// }
}

void TaskSpaceCompliantController::setActionFeedback(const ros::Time& time)
{
	// RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);
	// if (!current_active_goal)
	// {
	// 	return;
	// }

	// current_active_goal->preallocated_feedback_->header.stamp = time;
	// current_active_goal->preallocated_feedback_->desired = current_active_goal->gh_.getGoal()->command;

	// moveit_msgs::CartesianTrajectoryPoint point;
	// tf::poseEigenToMsg(mActualEETransform, point.point.pose);
	// current_active_goal->preallocated_feedback_->actual.point.pose = point.point.pose;

	// current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );
}

//=============================================================================
bool TaskSpaceCompliantController::shouldAcceptRequests() { 
	return isRunning(); 
}

//=============================================================================
// Default for virtual function is do nothing. DO NOT EDIT
bool TaskSpaceCompliantController::shouldStopExecution(std::string &message) {
	return false;
}

} // namespace


//=============================================================================
PLUGINLIB_EXPORT_CLASS(rewd_controllers::TaskSpaceCompliantController,
											 controller_interface::ControllerBase)