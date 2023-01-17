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

	std::string mEENodeName;
	n.getParam("/adaConf/end_effector", mEENodeName);
	mEENode = mSkeleton->getBodyNode(mEENodeName);

	mExtendedJoints = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);
	mExtendedJointsGravity = new ExtendedJointPosition(numControlledDofs, 3 * M_PI / 2);

	mCount = 0;  

	mJointStiffnessMatrix.resize(numControlledDofs, numControlledDofs);
	mJointStiffnessMatrix.setZero();
	mJointStiffnessMatrix.diagonal() << 6000,6000,6000,6000,5000,5000,5000;

	mRotorInertiaMatrix.resize(numControlledDofs, numControlledDofs);
	mRotorInertiaMatrix.setZero();
	mRotorInertiaMatrix.diagonal() << 0.3, 0.3, 0.3, 0.3, 0.18, 0.18, 0.18;

	mFrictionL.resize(numControlledDofs, numControlledDofs);
	mFrictionL.setZero();
	mFrictionL.diagonal() << 160, 160, 160, 160, 100, 100, 100;

	mFrictionLp.resize(numControlledDofs, numControlledDofs);
	mFrictionLp.setZero();
	mFrictionLp.diagonal() << 10, 10, 10, 10, 7.5, 7.5, 7.5;

	mJointKMatrix.resize(numControlledDofs, numControlledDofs);
	mJointKMatrix.setZero();
	mJointKMatrix.diagonal() << 80,80,80,80,60,60,60;

	mJointDMatrix.resize(numControlledDofs, numControlledDofs);
	mJointDMatrix.setZero();
	mJointDMatrix.diagonal() << 8,8,8,8,6,6,6;

	mTaskKMatrix.resize(6, 6);
	mTaskKMatrix.setZero();
	mTaskKMatrix.diagonal() << 200,200,200,100,100,100;

	mTaskDMatrix.resize(6, 6);
	mTaskDMatrix.setZero();
	mTaskDMatrix.diagonal() << 20,20,20,10,10,10;

	// Initialize buffers to avoid dynamic memory allocation at runtime.
	mDesiredPosition.resize(numControlledDofs);
	mDesiredVelocity.resize(numControlledDofs);
	mZeros.resize(numControlledDofs);
	mZeros.setZero();

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
	mSubCommand = n.subscribe<trajectory_msgs::JointTrajectoryPoint>("command", 1, &TaskSpaceCompliantController::commandCallback, this);

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
void TaskSpaceCompliantController::starting(const ros::Time &time) {

	mExecuteDefaultCommand = true;

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
void TaskSpaceCompliantController::update(const ros::Time &time,
																							 const ros::Duration &period) {

	// Update the state of the Skeleton.
	mSkeletonUpdater->update();
	mActualPosition = mControlledSkeleton->getPositions();
	mActualVelocity = mControlledSkeleton->getVelocities();
	mActualEffort = mControlledSkeleton->getForces();
	mActualEETransform =  mEENode->getWorldTransform();
	const auto numControlledDofs = mControlledSkeleton->getNumDofs();

	Eigen::MatrixXd Jq(6, numControlledDofs);
	Eigen::MatrixXd Jq_flipped(6, numControlledDofs);
	Eigen::MatrixXd fullJq = mEENode->getWorldJacobian();
	// std::cout<<"\n"<<"Full Jacobian: "<<fullJ<<"\n\n";
	auto fullDofs = mEENode->getDependentDofs();
	// std::cout<<"fullDofs.size(): "<<fullDofs.size()<<std::endl;
	for (size_t fullIndex = 0; fullIndex < fullDofs.size(); fullIndex++)
	{
		auto it = std::find(mDofs.begin(), mDofs.end(), fullDofs[fullIndex]);
		if (it != mDofs.end())
		{
			int index = it - mDofs.begin();
			Jq_flipped.col(index) = fullJq.col(fullIndex);
		}
	}
	for(int i=0; i<3; i++)
	{
		Jq.row(i) = Jq_flipped.row(i+3);
		Jq.row(i+3) = Jq_flipped.row(i);
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
		std::cout<<"1. Updating desired position ..."<<std::endl;
		mDesiredPosition = mActualPosition;
		mDesiredVelocity.setZero();
	}
	else
	{
		// std::cout<<"Reading from RT Buffer... "<<std::endl;
		trajectory_msgs::JointTrajectoryPoint command = *mCommandsBuffer.readFromRT(); // Rajat check: should this be by reference?
		// std::cout<<"... Read. :| "<<std::endl;
	
		// std::cout<<"Efforts Size: "<<command.effort.size()<<std::endl;
		// std::cout<<"Positions Size: "<<command.positions.size()<<std::endl;
		// std::cout<<"Velocities Size: "<<command.velocities.size()<<std::endl;
		// std::cout<<"Accelerations Size: "<<command.accelerations.size()<<std::endl;

		for (const auto &dof : mControlledSkeleton->getDofs()) 
		{
			// Rajat ToDo: Find better method
			std::size_t index = mControlledSkeleton->getIndexOf(dof);
			mDesiredPosition[index] = (command.positions.size() == 0) ? 0.0 : command.positions[index];
			mDesiredVelocity[index] = (command.velocities.size() == 0) ? 0.0 : command.velocities[index];
		}
		// std::cout<<"Out. :) "<<std::endl;
	}

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
	    Eigen::VectorXd qs_estimate_link_pos(numControlledDofs);
	    qs_estimate_link_pos = mActualTheta;

	    for (int i=0; i<iteration; i++)
	    {
	    	mControlledSkeleton->setPositions(qs_estimate_link_pos);
			mControlledSkeleton->setVelocities(mZeros);
			mSkeleton->computeInverseDynamics();
	        qs_estimate_link_pos = mActualTheta - mJointStiffnessMatrix.inverse()*(mControlledSkeleton->getGravityForces());
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

	if (!mExtendedJoints->is_initialized){
		std::cout<<"2. Updating desired position ..."<<std::endl;
		mLastDesiredPosition = mDesiredPosition;
		mExtendedJoints->initializeExtendedJointPosition(mDesiredPosition);
		mExtendedJoints->estimateExtendedJoint(mDesiredPosition);
		mNominalThetaPrev = mExtendedJoints->getExtendedJoint();
		mNominalThetaDotPrev = mActualVelocity;
		mTrueDesiredPosition = mExtendedJoints->getExtendedJoint();
		mTrueDesiredVelocity = mDesiredVelocity;
	}

	if (mDesiredPosition != mLastDesiredPosition && mActualPosition != mDesiredPosition){
		std::cout<<"3. Updating desired position ..."<<std::endl;
		mLastDesiredPosition = mDesiredPosition;
		mExtendedJoints->estimateExtendedJoint(mDesiredPosition);
		mTrueDesiredPosition = mExtendedJoints->getExtendedJoint();
		mTrueDesiredVelocity = mDesiredVelocity;
	}
	
	mExtendedJoints->estimateExtendedJoint(mActualPosition);
	mActualTheta = mExtendedJoints->getExtendedJoint();

	mDesiredTheta = mTrueDesiredPosition + mJointStiffnessMatrix.inverse()*mGravity;
	mDesiredThetaDot = mTrueDesiredVelocity;
	
	mTaskEffort = -mJointKMatrix*(mNominalThetaPrev-mDesiredTheta) - mJointDMatrix*(mNominalThetaDotPrev - mDesiredThetaDot) + mGravity;

	//Compute error
	Eigen::VectorXd dart_error(6);   
	Eigen::MatrixXd dart_nominal_jacobian(6, numControlledDofs); // change to numControlledDofs
	{
		mControlledSkeleton->setPositions(mTrueDesiredPosition);
		mControlledSkeleton->setVelocities(mZeros);
		mControlledSkeleton->setAccelerations(mZeros); 
		mDesiredEETransform = mEENode->getWorldTransform();

		mControlledSkeleton->setPositions(mNominalThetaPrev);
		mControlledSkeleton->setVelocities(mNominalThetaDotPrev);
		mControlledSkeleton->setAccelerations(mZeros); 
		mSkeleton->computeInverseDynamics(); // Do we need this to update jacobian?
		mNominalEETransform = mEENode->getWorldTransform();

		Eigen::Quaterniond nominal_ee_quat(mNominalEETransform.linear());
		
		// Get Jacobian relative only to controlled joints -- Rajat ToDo: Check with Ethan
		Eigen::MatrixXd dart_nominal_jacobian_flipped(6, numControlledDofs);
		Eigen::MatrixXd fullJ = mEENode->getWorldJacobian();
		// std::cout<<"\n"<<"Full Jacobian: "<<fullJ<<"\n\n";
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

		dart_error.head(3) << mNominalEETransform.translation() - mDesiredEETransform.translation(); // positional error
		
		Eigen::Quaterniond ee_quat_d(mDesiredEETransform.linear());

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

	// std::cout<<"\n"<<"Nominal Jacobian: "<<dart_nominal_jacobian<<"\n\n";

	mTaskEffort = dart_nominal_jacobian.transpose() * (-mTaskKMatrix * dart_error - mTaskDMatrix * (dart_nominal_jacobian * mNominalThetaDotPrev)) + mQuasiGravity;

	double step_time;
	step_time = 0.001;

	mNominalThetaDDot = mRotorInertiaMatrix.inverse()*(mTaskEffort+mActualEffort); // mActualEffort is negative of what is required here
	mNominalThetaDot = mNominalThetaDotPrev + mNominalThetaDDot*step_time;
	mNominalTheta = mNominalThetaPrev + mNominalThetaDot*step_time;

	mNominalThetaPrev = mNominalTheta;
	mNominalThetaDotPrev = mNominalThetaDot;

	mNominalFriction = mRotorInertiaMatrix*mFrictionL*((mNominalThetaDotPrev - mActualVelocity) + mFrictionLp*(mNominalThetaPrev - mActualTheta));

	mDesiredEffort = mTaskEffort + mNominalFriction;

	bool mNullSpace = true;
	if (mNullSpace)
	{
		Eigen::MatrixXd Kp, Kv;
		Kp.resize(numControlledDofs, numControlledDofs);
		Kv.resize(numControlledDofs, numControlledDofs);
		// Kp.diagonal() << 8., 8., 8., 8., 8., 8., 8.;
		Kp.diagonal() << 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2;
		Kv << 2 * Kp.array().sqrt();

		// https://github.com/studywolf/control/blob/master/studywolf_control/controllers/osc.py
		Eigen::VectorXd q_err = mExtendedJoints->normalizeJointPosition(mLastDesiredPosition) - mExtendedJoints->normalizeJointPosition(mActualPosition);
		Eigen::VectorXd qd_err = mTrueDesiredVelocity - mActualVelocity;
		Eigen::VectorXd q_des = Kp * q_err + Kv * qd_err;

		// // Get mass matrix in joint space Mq
		// Eigen::MatrixXd mq = mSkeleton->getMassMatrix();
		// Eigen::MatrixXd Mq = Eigen::Map<Eigen::MatrixXd>(mq.data(), numControlledDofs, numControlledDofs);
		// Eigen::MatrixXd q = mExtendedJoints->normalizeJointPosition(mActualPosition);
		// Eigen::MatrixXd Mx = mExtendedJoints->computeEEMassMatrix(Mq, q, Jq);
		// Eigen::VectorXd tau_null = Mq * q_des;
		// std::cout << "Tau null: " << tau_null.transpose() << std::endl;
		// Eigen::MatrixXd Jdyn_inv = Mx * (Jq * mExtendedJoints->pseudoinverse(Mq, 1e-6));
		// Eigen::MatrixXd null_filter = Eigen::MatrixXd::Identity(numControlledDofs, numControlledDofs) - Jq.transpose() * Jdyn_inv;
		// Eigen::VectorXd tau_signal = null_filter * tau_null;
		// std::cout << "Tau signal: " << tau_signal.transpose() << std::endl;
		// mDesiredEffort = mDesiredEffort + tau_signal;

		// tau_nullspace
		// https://github.com/nbfigueroa/franka_interactive_controllers/blob/main/src/franka_cartesian_controllers/cartesian_pose_impedance_controller.cpp
		// tau_nullspace = I - J.T * (J.T)^-1 * (K * (q_d - q) - 2 * sqrt(K) * dq);
		Eigen::VectorXd tau_nullspace;
		tau_nullspace = (Eigen::MatrixXd::Identity(numControlledDofs, numControlledDofs) - Jq.transpose() * mExtendedJoints->pseudoinverse(Jq.transpose(), 1e-6)) * q_des;
		std::cout << "Tau null space: " << tau_nullspace.transpose() << std::endl;
		mDesiredEffort = mDesiredEffort + tau_nullspace;
	}

	if(mCount < 2000)
	{
		mDesiredEffort = mGravity;
		mCount++;
		if(mCount%200 == 0)
			std::cout<<"Initializing controller: "<<mCount<<std::endl; 
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
		RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

		// Cancel any goal timeout
		mGoalDurationTimer.stop();

		// Cancels the currently active goal
		if (current_active_goal)
		{
			// Marks the current goal as canceled
			mRTActiveGoal.reset();
			current_active_goal->gh_.setCanceled();
		}
}

void TaskSpaceCompliantController::commandCallback(const trajectory_msgs::JointTrajectoryPointConstPtr& msg)
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
	std::cout<<"Joint group command controller: Recieved new goal!"<<std::endl;
	ROS_DEBUG_STREAM_NAMED(mName,"Received new action goal");
	pr_control_msgs::JointGroupCommandResult result;

	// Preconditions
	if (!shouldAcceptRequests())
	{
		result.error_string = "Can't accept new action goals. Controller is not running.";
		ROS_ERROR_STREAM_NAMED(mName, result.error_string);
		result.error_code = pr_control_msgs::JointGroupCommandResult::INVALID_GOAL;
		gh.setRejected(result);
		return;
	}

	// if (gh.getGoal()->joint_names.size() != gh.getGoal()->command.positions.size()) {
	//   result.error_string = "Size of command must match size of joint_names.";
	//   ROS_ERROR_STREAM_NAMED(mName, result.error_string);
	//   result.error_code = pr_control_msgs::JointGroupCommandResult::INVALID_GOAL;
	//   gh.setRejected(result);
	//   return;
	// }

	// Goal should specify valid controller joints (they can be ordered differently). Reject if this is not the case

	// update new command
	RealtimeGoalHandlePtr rt_goal(new RealtimeGoalHandle(gh));
	trajectory_msgs::JointTrajectoryPoint new_command = gh.getGoal()->command;
	rt_goal->preallocated_feedback_->joint_names = mJointNames;
	mCommandsBuffer.writeFromNonRT(new_command);

		// Accept new goal
	preemptActiveGoal();
	gh.setAccepted();
	mRTActiveGoal = rt_goal;
	mExecuteDefaultCommand = false;

	// Setup goal status checking timer
	mGoalHandleTimer = mNodeHandle->createTimer(mActionMonitorPeriod,
																										&RealtimeGoalHandle::runNonRealtime,
																										rt_goal);
	mGoalHandleTimer.start();

	// Setup goal timeout
	if (gh.getGoal()->command.time_from_start > ros::Duration()) {
		mGoalDurationTimer = mNodeHandle->createTimer(gh.getGoal()->command.time_from_start,
																										&TaskSpaceCompliantController::timeoutCallback,
																										this,
																										true);
		mGoalDurationTimer.start();
	}
}

void TaskSpaceCompliantController::timeoutCallback(const ros::TimerEvent& event)
{
	RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

	// Check that timeout refers to currently active goal (if any)
	if (current_active_goal) {
		ROS_DEBUG_NAMED(mName, "Active action goal reached requested timeout.");

		// Give sub-classes option to update mDefaultCommand
		mExecuteDefaultCommand = true;

		// Marks the current goal as succeeded
		mRTActiveGoal.reset();
		current_active_goal->gh_.setSucceeded();
	}
}

void TaskSpaceCompliantController::cancelCallback(GoalHandle gh)
{
	RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);

	// Check that cancel request refers to currently active goal
	if (current_active_goal && current_active_goal->gh_ == gh)
	{
		ROS_DEBUG_NAMED(mName, "Canceling active action goal because cancel callback recieved from actionlib.");

		// Give sub-classes option to update mDefaultCommand
		mExecuteDefaultCommand = true;

		preemptActiveGoal();
	}
}

void TaskSpaceCompliantController::setActionFeedback(const ros::Time& time)
{
	RealtimeGoalHandlePtr current_active_goal(mRTActiveGoal);
	if (!current_active_goal)
	{
		return;
	}

	current_active_goal->preallocated_feedback_->header.stamp = time;
	current_active_goal->preallocated_feedback_->desired = current_active_goal->gh_.getGoal()->command;
	current_active_goal->preallocated_feedback_->actual.positions.clear();
	current_active_goal->preallocated_feedback_->actual.velocities.clear();
	current_active_goal->preallocated_feedback_->actual.effort.clear();
	for (const auto &dof : mControlledSkeleton->getDofs()) 
	{
			std::size_t index = mControlledSkeleton->getIndexOf(dof);
			current_active_goal->preallocated_feedback_->actual.positions.push_back(mActualPosition[index]);
			current_active_goal->preallocated_feedback_->actual.velocities.push_back(mActualVelocity[index]);
			current_active_goal->preallocated_feedback_->actual.effort.push_back(mActualEffort[index]);
	}

	current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );
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