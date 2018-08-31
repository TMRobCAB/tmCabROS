/*

 * Controller.cpp
 *
 *  Created on: Feb 21, 2016
 *      Author: yamil
 */

#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

//#include <kdl/chain.hpp>
//
//#include <kdl/chainfksolver.hpp>
//
//#include <kdl/chainfksolverpos_recursive.hpp>
//
//#include <kdl/frames_io.hpp>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include "msg_shared.hpp"

using namespace boost::interprocess;

#include <moveit/exceptions/exceptions.h>

#include <stdio.h>

#include <iostream>

#include <stdint.h>

#include "stdint.h"

#include <sstream>

#include "ros/ros.h"

#include "DriverFsm/incfiles.h"

#include "DriverFsm/SysFsm.h"

#include "DriverFsm/ComCmds.h"

#include "DriverFsm/SysClk.h"

#include "tm_cab/POS_CART.h"

#include "tm_cab/POS_REF.h"

#include "tm_cab/PTP_TM_CTL.h"

#include "tm_cab/START_TM_CTL.h"

#include "tm_cab/STOP_TM_CTL.h"

#include "tm_cab/set_par.h"

#include "tm_cab/TOPIC_STATUS.h"

#include "tm_cab/TOPIC_VARIABLES.h"

#include <tm_cab/SYS_STATUS.h>

#include <tm_cab/CTL_MODE.h>

#include <tm_cab/SELECT_CTL.h>

#include "geometry_msgs/WrenchStamped.h"

#include "geometry_msgs/Pose.h"

#include <sensor_msgs/JointState.h>

#include <moveit_msgs/DisplayRobotState.h>

#include <eigen_conversions/eigen_msg.h>

#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/robot_model/robot_model.h>

#include <moveit/robot_state/robot_state.h>

#include <moveit/robot_state/conversions.h>

#include <moveit/dynamics_solver/dynamics_solver.h>

#include <math.h>

#include <vector>

#include <string>

#define TIMEOUT_PAR_SET 1.0f

jnt_ref_shared * _jnt1RefMsg;
jnt_ref_shared * _jnt2RefMsg;
jnt_ref_shared * _jnt3RefMsg;

jnt_state_shared * _jnt1StateMsg;
jnt_state_shared * _jnt2StateMsg;
jnt_state_shared * _jnt3StateMsg;

force_shared * _forceMsg;

enum CtlMode {
	PTP, FORCE_POS
} _mode;

ros::Publisher _refQ1P, _refQ2P, _refQ3P;

Eigen::Affine3d _posDes;

geometry_msgs::WrenchStamped _forceCart;

Eigen::Vector3d _qMeas, _qDes, _dqMeas, _dqDes, _qIdxM, _qIdxS;

bool _fPTPCart = false, _fQ1m = false, _fQ2m = false, _fQ3m = false, _fRunning =
false, _fAdvPTP = false, _fShDwnPTP = false, _fSubForce = false, _fUnSubForce =
false, _fForcem = false, _fPTPJnt = false, _compG = false, _idxPose = false;

float _forceK = 0.5, _scaleK = 1;

void selJntCtlOpMode(
		tm_cab::CTL_MODE::Request::_mode_type ctlMode) {

	ros::NodeHandle n;
	ros::ServiceClient client;

	std::stringstream srvName;

	tm_cab::SELECT_CTL srv;

	switch (ctlMode) {
	case tm_cab::CTL_MODE::Request::PTP:
	case tm_cab::CTL_MODE::Request::FORCE_POS:

		srv.request.id = tm_cab::SELECT_CTL::Request::POS_CTL;

		break;
	default:

		srv.request.id = tm_cab::SELECT_CTL::Request::POS_CTL;

		break;
	}

	for (int i = 1; i < 4; ++i) {

		srvName.str("");

		srvName << "/joint_" << i << "_select_Ctl";

		client = n.serviceClient<tm_cab::SELECT_CTL>(srvName.str());

		if (client.call(srv)) {

			if (!srv.response.ack) {
				ROS_ERROR("Joint Ctl not Set");

			}
		} else {

			ROS_ERROR("Failed to call %s", srvName.str().data());
		}
	}
}

bool setJointRef(const int jointNum, const float jointValue) {

	tm_cab::POS_REF posMsg;

	posMsg.q_D = jointValue;

	switch (jointNum) {
	case 1:

		_jnt1RefMsg->mutex.lock();

		_jnt1RefMsg->q = jointValue;

		_jnt1RefMsg->mutex.unlock();

		_refQ1P.publish(posMsg);

		break;
	case 2:

		_jnt2RefMsg->mutex.lock();

		_jnt2RefMsg->q = jointValue;

		_jnt2RefMsg->mutex.unlock();

		_refQ2P.publish(posMsg);
		break;
	case 3:

		_jnt3RefMsg->mutex.lock();

		_jnt3RefMsg->q = jointValue;

		_jnt3RefMsg->mutex.unlock();

		_refQ3P.publish(posMsg);
		break;

	default:

		ROS_DEBUG("Joint number not found");

		return false;

		break;
	}
	return true;
}

bool setJointRefs(const std::vector<double> jointValues) {

	bool ret = true;

	for (int i = 0; i < jointValues.size(); ++i) {
		if (!setJointRef(i + 1, (float) jointValues[i]))
			ret = false;
	}

	return ret;
}

bool setJointRefs(const Eigen::VectorXd jointValues) {

	bool ret = true;

	for (int i = 0; i < jointValues.size(); ++i) {
		if (!setJointRef(i + 1, (float) jointValues[i]))
			ret = false;
	}

	return ret;
}

void ForceCallback(const geometry_msgs::WrenchStamped & msg) {

	//_forceCart = geometry_msgs::WrenchStamped(msg);

	//_fForcem = true;

}

void Joint1VarsCallback(const tm_cab::TOPIC_VARIABLES & msg) {

	//_qMeas(0) = msg.q;

	//_fQ1m = true;

}

void Joint2VarsCallback(const tm_cab::TOPIC_VARIABLES& msg) {

	//_qMeas(1) = msg.q;
	//_fQ2m = true;

}

void Joint3VarsCallback(const tm_cab::TOPIC_VARIABLES& msg) {

	//_qMeas(2) = msg.q;
	//_fQ3m = true;

}

bool TM_SetMode(tm_cab::CTL_MODE::Request &req,
		tm_cab::CTL_MODE::Response &res) {

	if (req.mode == tm_cab::CTL_MODE::Request::PTP) {

		_mode = PTP;
	}
	if (req.mode == tm_cab::CTL_MODE::Request::FORCE_POS) {

		_mode = FORCE_POS;

		_forceK = req.force_k;

		_fSubForce = true;

		_fShDwnPTP = true;
	}

	res.ack = true;

	return true;
}

bool TM_SetFPosKP(tm_cab::set_par::Request &req,
		tm_cab::set_par::Response &res) {

	_forceK = req.parVal;

	res.ack = true;

	return true;
}

bool TM_SetScalingK(tm_cab::set_par::Request &req,
		tm_cab::set_par::Response &res) {

	_scaleK = req.parVal;

	res.ack = true;

	return true;
}

bool TM_Ptp(tm_cab::PTP_TM_CTL::Request &req,
		tm_cab::PTP_TM_CTL::Response &res) {

	if (req.mode == tm_cab::PTP_TM_CTL::Request::PTP_CART) {

		_posDes.translation() = Eigen::Vector3d(req.Xd, req.Yd, req.Zd);

		_fPTPCart = true;

		_fPTPJnt = false;

	} else if (req.mode == tm_cab::PTP_TM_CTL::Request::PTP_JNT) {

		_qDes = Eigen::Vector3d(req.Xd, req.Yd, req.Zd);

		_fPTPJnt = true;

		_fPTPCart = false;
	}

	res.ack = true;

	return true;
}
void SetGComp(const std_msgs::Bool & msg) {

	if (msg.data) {

		_compG = true;

		ROS_INFO("Gravity Compensation Enabled");
	} else {

		_compG = false;

		ROS_INFO("Gravity Compensation Disabled");
	}
}

void IdxPose(const std_msgs::Bool & msg) {

	if (msg.data) {

		_idxPose = true;

		_qIdxM = _qMeas;

		ROS_INFO("Slave mapped");
	} else {

		_idxPose = false;

		_qIdxS = _qIdxS + (_qMeas - _qIdxM) * _scaleK;

		ROS_INFO("Slave Unmapped");
	}
}

void RUNCallback(const std_msgs::Bool & msg) {

	if (msg.data) {

		switch (_mode) {
		case PTP:

			_fAdvPTP = true;
			_fUnSubForce = true;
			break;
		case FORCE_POS:

			_fSubForce = true;

			_fShDwnPTP = true;

			_forceCart.wrench.force.x = 0;
			_forceCart.wrench.force.y = 0;
			_forceCart.wrench.force.z = 0;

			_forceCart.wrench.torque.x = 0;
			_forceCart.wrench.torque.x = 0;
			_forceCart.wrench.torque.x = 0;
			break;
		default:
			break;
		}

	} else {

		_fShDwnPTP = true;
		_fUnSubForce = true;
	}

}

int main(int argc, char **argv) {

	bool fMemMapped;

	////////////////////////////////////////////////////////////////////

	shared_memory_object jnt1StateShm;

	fMemMapped = false;

	while (fMemMapped == false) {

		fMemMapped = true;

		try {

			jnt1StateShm = shared_memory_object(open_only,
					"/joint_1_state_shared", read_write);

		} catch (interprocess_exception &ex) {

			fMemMapped = false;
		}

		sleep(1);
	}

	// Shared ref objects j1
	////////////////////////////////////////////////////////////////////

	//Map the whole shared memory in this process
	mapped_region regionState1(jnt1StateShm, read_write);

	//Get the address of the mapped region
	void * jnt1StateAddr = regionState1.get_address();

	//Construct the shared structure in memory
	_jnt1StateMsg = static_cast<jnt_state_shared*>(jnt1StateAddr);

	////////////////////////////////////////////////////////////////////

	shared_memory_object jnt2StateShm;

	fMemMapped = false;

	while (fMemMapped == false) {

		fMemMapped = true;

		try {

			jnt2StateShm = shared_memory_object(open_only,
					"/joint_2_state_shared", read_write);

		} catch (interprocess_exception &ex) {

			fMemMapped = false;
		}

		sleep(1);
	}

	// Shared ref objects j1
	////////////////////////////////////////////////////////////////////

	//Map the whole shared memory in this process
	mapped_region regionState2(jnt2StateShm, read_write);

	//Get the address of the mapped region
	void * jnt2StateAddr = regionState2.get_address();

	//Construct the shared structure in memory
	_jnt2StateMsg = static_cast<jnt_state_shared*>(jnt2StateAddr);

	////////////////////////////////////////////////////////////////////

	shared_memory_object jnt3StateShm;

	fMemMapped = false;

	while (fMemMapped == false) {

		fMemMapped = true;

		try {

			jnt3StateShm = shared_memory_object(open_only,
					"/joint_3_state_shared", read_write);

		} catch (interprocess_exception &ex) {

			fMemMapped = false;
		}

		sleep(1);
	}

	// Shared ref objects j1
	////////////////////////////////////////////////////////////////////

	//Map the whole shared memory in this process
	mapped_region regionState3(jnt3StateShm, read_write);

	//Get the address of the mapped region
	void * jnt3StateAddr = regionState3.get_address();

	//Construct the shared structure in memory
	_jnt3StateMsg = static_cast<jnt_state_shared*>(jnt3StateAddr);

	////////////////////////////////////////////////////////////////////

	shared_memory_object jnt1RefShm;

	fMemMapped = false;

	while (fMemMapped == false) {

		fMemMapped = true;

		try {

			jnt1RefShm = shared_memory_object(open_only, "/joint_1_ref_shared",
					read_write);

		} catch (interprocess_exception &ex) {

			fMemMapped = false;
		}

		sleep(1);
	}

	// Shared ref objects j1
	////////////////////////////////////////////////////////////////////

	//Map the whole shared memory in this process
	mapped_region regionRef1(jnt1RefShm, read_write);

	//Get the address of the mapped region
	void * jnt1Refaddr = regionRef1.get_address();

	//Construct the shared structure in memory
	_jnt1RefMsg = static_cast<jnt_ref_shared*>(jnt1Refaddr);

	////////////////////////////////////////////////////////////////////

	// Shared ref objects j2
	////////////////////////////////////////////////////////////////////

	shared_memory_object jnt2RefShm;

	fMemMapped = false;

	while (fMemMapped == false) {

		fMemMapped = true;

		try {

			jnt2RefShm = shared_memory_object(open_only, "/joint_2_ref_shared",
					read_write);

		} catch (interprocess_exception &ex) {

			fMemMapped = false;
		}

		sleep(1);
	}

	//Map the whole shared memory in this process
	mapped_region regionRef2(jnt2RefShm, read_write);

	//Get the address of the mapped region
	void * jnt2Refaddr = regionRef2.get_address();

	//Construct the shared structure in memory
	_jnt2RefMsg = static_cast<jnt_ref_shared*>(jnt2Refaddr);

	////////////////////////////////////////////////////////////////////

	// Shared ref objects j3
	////////////////////////////////////////////////////////////////////

	shared_memory_object jnt3RefShm;

	fMemMapped = false;

	while (fMemMapped == false) {

		fMemMapped = true;

		try {

			jnt3RefShm = shared_memory_object(open_only, "/joint_3_ref_shared",
					read_write);

		} catch (interprocess_exception &ex) {

			fMemMapped = false;
		}

		sleep(1);
	}

	//Map the whole shared memory in this process
	mapped_region regionRef3(jnt3RefShm, read_write);

	//Get the address of the mapped region
	void * jnt3Refaddr = regionRef3.get_address();

	//Construct the shared structure in memory
	_jnt3RefMsg = static_cast<jnt_ref_shared*>(jnt3Refaddr);

	////////////////////////////////////////////////////////////////////

	shared_memory_object forceShm;

	fMemMapped = false;

	while (fMemMapped == false) {

		fMemMapped = true;

		try {

			forceShm = shared_memory_object(open_only, "/force_cart_shared",
					read_write);

		} catch (interprocess_exception &ex) {

			fMemMapped = false;
		}

		sleep(1);
	}

	//Map the whole shared memory in this process
	mapped_region regionForce(forceShm, read_write);

	//Get the address of the mapped region
	void * forceAddr = regionForce.get_address();

	//Construct the shared structure in memory
	_forceMsg = static_cast<force_shared*>(forceAddr);

	////////////////////////////////////////////////////////////////////

	_mode = PTP;

	_forceK = 0.1;

//Definition of a kinematic chain & add segments to the chain

	Eigen::Affine3d forcePosDes;

	geometry_msgs::PoseStamped posDesMsg;

	std::vector<double> qDes(3);

	std::vector<double> jntTorques(3);

	_qMeas = Eigen::Vector3d(0, 0, 0);

	_qIdxM = Eigen::Vector3d(0, 0, 0);

	_qIdxS = Eigen::Vector3d(0, 0, 0);

	ros::init(argc, argv, "Controller");

	ros::NodeHandle m;

	std::string ctlName = ros::this_node::getName();

	ctlName = ctlName.substr(1, ctlName.size() - 1);

	Eigen::Matrix3d fMass;
	Eigen::Vector3d fDC;

	std::vector<double> v;

	m.getParam(ctlName + "/fMass", v);

	for (int i = 0; i < 3; ++i) {
		for (int j = 0; j < 3; ++j) {

			fMass(i, j) = v[i * 3 + j];
		}
	}

	m.getParam(ctlName + "/fDC", v);

	for (int i = 0; i < 3; ++i) {
		fDC[i] = v[i];
	}

	geometry_msgs::WrenchStamped forceCartW;

	Eigen::Vector3d forceTip, qFake;

	qFake = Eigen::Vector3d(0, 0, 0);

	geometry_msgs::PoseStamped meas_pose_Msg;
	tm_cab::POS_REF refMsg;
	sensor_msgs::JointState fakeJntMsg;

	moveit_msgs::DisplayRobotState rStateMsg;

	ros::Subscriber jnt1Svar = m.subscribe("/joint_1_state_vars", 100,
			Joint1VarsCallback);
	ros::Subscriber jnt2Svar = m.subscribe("/joint_2_state_vars", 100,
			Joint2VarsCallback);
	ros::Subscriber jnt3Svar = m.subscribe("/joint_3_state_vars", 100,
			Joint3VarsCallback);

	ros::Subscriber setGCmpS = m.subscribe("/set_g_comp", 100, SetGComp);

	ros::Subscriber setIdxPose = m.subscribe("/index_pose", 100, IdxPose);

	ros::Subscriber sysRunS = m.subscribe("/sys_run", 100, RUNCallback);

	ros::Publisher cartPosP = m.advertise<geometry_msgs::PoseStamped>(
			"sys_meas_pose", 100);

	ros::Publisher desPosP = m.advertise<geometry_msgs::PoseStamped>(
			"sys_des_pose", 100);

	ros::Publisher gCompQ2P = m.advertise<tm_cab::POS_REF>(
			"/joint_2_g_comp_effort", 100);
	ros::Publisher gCompQ3P = m.advertise<tm_cab::POS_REF>(
			"/joint_3_g_comp_effort", 100);

	ros::Publisher forceWP = m.advertise<geometry_msgs::WrenchStamped>(
			"force_cart_w", 100);

	ros::Publisher ctlFakeP = m.advertise<sensor_msgs::JointState>(
			"/move_group/fake_controller_joint_states", 100);

	ros::Publisher measRobotStateP =
			m.advertise<moveit_msgs::DisplayRobotState>("/meas_robot_state",
					100);

	ros::Publisher gTipP = m.advertise<geometry_msgs::Vector3>("/g_tip", 100);

	_refQ1P = m.advertise<tm_cab::POS_REF>("/joint_1_pos_ref", 100);
	_refQ2P = m.advertise<tm_cab::POS_REF>("/joint_2_pos_ref", 100);
	_refQ3P = m.advertise<tm_cab::POS_REF>("/joint_3_pos_ref", 100);

	ros::ServiceServer srvSetCtlMode = m.advertiseService("tm_ctl_set_mode",
			TM_SetMode);

	ros::ServiceServer srvSetSetFPosKP = m.advertiseService("tm_set_f_pos_kp",
			TM_SetFPosKP);

	ros::ServiceServer srvSetSetScaleK = m.advertiseService("tm_set_scaling_k",
			TM_SetScalingK);

	ros::Subscriber forceSub;

	ros::ServiceServer srvPtpCTL;

	float ctlFs = 100;

	ros::Rate loopRate(ctlFs);

	int fkStatus = 0;

	robot_model_loader::RobotModelLoader robot_model_loader(
			"robot_description");

//------Create Robot Desired Model----
//------------------------------------

	robot_model::RobotModelPtr des_kin_model = robot_model_loader.getModel();

	robot_state::RobotStatePtr des_kin_state(
			new robot_state::RobotState(des_kin_model));

	const robot_state::JointModelGroup* des_arm_group =
			des_kin_model->getJointModelGroup("arm");

	const std::vector<std::string> &joint_names =
			des_arm_group->getJointModelNames();

	const std::vector<std::string> linkNames(
			des_kin_model->getLinkModelNames());

	des_kin_state->setToDefaultValues();
	const Eigen::Affine3d &des_sensor_tip_state =
			des_kin_state->getGlobalLinkTransform(linkNames.back().c_str());

//------Create Robot Measured Model----
//-------------------------------------

	robot_model::RobotModelPtr meas_kin_model = robot_model_loader.getModel();

	robot_state::RobotStatePtr meas_kin_state(
			new robot_state::RobotState(meas_kin_model));

	const robot_state::JointModelGroup* meas_arm_group =
			meas_kin_model->getJointModelGroup("arm");

	meas_kin_state->setToDefaultValues();
	const Eigen::Affine3d &meas_sensor_tip_state =
			meas_kin_state->getGlobalLinkTransform(linkNames.back().c_str());

	geometry_msgs::Vector3 gVecMsg, gVecTipMsg;
	gVecMsg.x = 0;
	gVecMsg.y = 0;
	gVecMsg.z = -9.806;

	Eigen::Vector3d gVector, gVectorTip;

	tf::vectorMsgToEigen(gVecMsg, gVector);

	dynamics_solver::DynamicsSolver meas_dyn_solver(meas_kin_model,
			meas_arm_group->getName(), gVecMsg);

	meas_pose_Msg.header.frame_id = linkNames.front().c_str();

	fakeJntMsg.name = std::vector<std::string>(joint_names.begin() + 1,
			joint_names.end());

	forceCartW.header.frame_id = linkNames.front().c_str();

	Eigen::MatrixXd jacobian;

	while (ros::ok()) {

		/////////////////////////////////////////////////////

		_jnt1StateMsg->mutex.lock();

		if (_jnt1StateMsg->newState) {

			_jnt1StateMsg->newState = false;
			_fQ1m = true;

			_qMeas(0) = _jnt1StateMsg->q;
			_dqMeas(0) = _jnt1StateMsg->dq;
		}

		_jnt1StateMsg->mutex.unlock();

		/////////////////////////////////////////////////////

		_jnt2StateMsg->mutex.lock();

		if (_jnt2StateMsg->newState) {

			_jnt2StateMsg->newState = false;
			_fQ2m = true;

			_qMeas(1) = _jnt2StateMsg->q;
			_dqMeas(1) = _jnt2StateMsg->dq;
		}

		_jnt2StateMsg->mutex.unlock();

		/////////////////////////////////////////////////////

		_jnt3StateMsg->mutex.lock();

		if (_jnt3StateMsg->newState) {

			_jnt3StateMsg->newState = false;
			_fQ3m = true;

			_qMeas(2) = _jnt3StateMsg->q;
			_dqMeas(2) = _jnt3StateMsg->dq;
		}

		_jnt3StateMsg->mutex.unlock();

		/////////////////////////////////////////////////////

		if (_fQ1m && _fQ2m && _fQ3m) {

			_fQ1m = false;
			_fQ2m = false;
			_fQ3m = false;

			meas_kin_state->setJointGroupPositions(meas_arm_group, _qMeas);

			meas_kin_state->update(true);

			if (_compG) {

				meas_dyn_solver.getPayloadTorques(
						std::vector<double>(_qMeas.data(),
								_qMeas.data() + _qMeas.size()), 0, jntTorques);

				//////////////////////////////////////////////

				_jnt2RefMsg->mutex.lock();

				_jnt2RefMsg->pwmComp = (float) jntTorques[1];

				_jnt2RefMsg->mutex.unlock();

				_jnt3RefMsg->mutex.lock();

				_jnt3RefMsg->pwmComp = (float) jntTorques[2];

				_jnt3RefMsg->mutex.unlock();

				//////////////////////////////////////////////

				refMsg.q_D = jntTorques[1];
				gCompQ2P.publish(refMsg);

				refMsg.q_D = jntTorques[2];
				gCompQ3P.publish(refMsg);
			}

			// TODO check bounds
			//des_kin_state->satisfiesBounds();

			tf::poseEigenToMsg(meas_sensor_tip_state, meas_pose_Msg.pose);

			cartPosP.publish(meas_pose_Msg);

			robotStateToRobotStateMsg(*meas_kin_state, rStateMsg.state,false);

			measRobotStateP.publish(rStateMsg);

			gVectorTip = meas_sensor_tip_state.rotation().inverse() * gVector;

			tf::vectorEigenToMsg(gVectorTip, gVecTipMsg);

			gTipP.publish(gVecTipMsg);
		}

		_forceMsg->mutex.lock();

		if (_forceMsg->newSample) {

			_forceMsg->newSample = false;
			_fForcem = true;

			_forceCart.wrench.force.x = _forceMsg->fX;
			_forceCart.wrench.force.y = _forceMsg->fY;
			_forceCart.wrench.force.z = _forceMsg->fZ;

		}

		_forceMsg->mutex.unlock();

		tf::vectorMsgToEigen(_forceCart.wrench.force, forceTip);

		forceTip -= fMass * gVectorTip + fDC;

		tf::vectorEigenToMsg(meas_sensor_tip_state.rotation() * forceTip,
				forceCartW.wrench.force);

		forceCartW.wrench.torque.x = 0;
		forceCartW.wrench.torque.y = 0;
		forceCartW.wrench.torque.z = 0;

		forceWP.publish(forceCartW);

		switch (_mode) {
		case PTP:

			fakeJntMsg.position = std::vector<double>(_qMeas.data(),
					_qMeas.data() + _qMeas.size());

			fakeJntMsg.velocity = std::vector<double>(_dqMeas.data(),
					_dqMeas.data() + _dqMeas.size());

			ctlFakeP.publish(fakeJntMsg);

			if (_fAdvPTP) {

				_fAdvPTP = false;

				srvPtpCTL = m.advertiseService("tm_ptp", TM_Ptp);

				setJointRefs(Eigen::Vector3d(0, M_PI / 2, 0));
			}

			if (_fShDwnPTP) {

				_fShDwnPTP = false;

				srvPtpCTL.shutdown();
			}

			if (_fPTPCart) {

				_fPTPCart = false;

				tf::poseEigenToMsg(_posDes, posDesMsg.pose);

				if (des_kin_state->setFromIK(des_arm_group, posDesMsg.pose, 10,
						0.001)) {

					des_kin_state->update(true);

					des_kin_state->copyJointGroupPositions(des_arm_group, qDes);

					setJointRefs(qDes);

					tf::poseEigenToMsg(des_sensor_tip_state, posDesMsg.pose);

					ROS_INFO("PTP: Des Pos: (%f,%f,%f) Des q: (%f,%f,%f)",
							des_sensor_tip_state.translation()(0),
							des_sensor_tip_state.translation()(1),
							des_sensor_tip_state.translation()(2), qDes[0],
							qDes[1], qDes[2]);

					desPosP.publish(posDesMsg);

				} else {

					ROS_ERROR("Error Solving IKP");
				}
			}

			if (_fPTPJnt) {

				_fPTPJnt = false;

				setJointRefs(_qDes);

				des_kin_state->setJointGroupPositions(des_arm_group, _qDes);
				des_kin_state->update(true);

				tf::poseEigenToMsg(des_sensor_tip_state, posDesMsg.pose);

				ROS_INFO("PTP: Des Pos: (%f,%f,%f) Des q: (%f,%f,%f)",
						des_sensor_tip_state.translation()(0),
						des_sensor_tip_state.translation()(1),
						des_sensor_tip_state.translation()(2), _qDes[0],
						_qDes[1], _qDes[2]);

				desPosP.publish(posDesMsg);
			}

			break;

		case FORCE_POS:

			if (_fSubForce) {

				_fSubForce = false;

				des_kin_state->setJointGroupPositions(des_arm_group,
						meas_kin_state->getVariablePositions());

				des_kin_state->update(true);

				des_kin_state->copyJointGroupPositions(meas_arm_group, qDes);

				setJointRefs(qDes);

				tf::poseEigenToMsg(des_sensor_tip_state, posDesMsg.pose);

				desPosP.publish(posDesMsg);

				forceSub = m.subscribe("force_cart", 100, ForceCallback);

				fakeJntMsg.position = std::vector<double>(_qMeas.data(),
						_qMeas.data() + _qMeas.size());

				fakeJntMsg.velocity = std::vector<double>(_dqMeas.data(),
						_dqMeas.data() + _dqMeas.size());

				_qIdxS = _qMeas;

				ctlFakeP.publish(fakeJntMsg);

				_scaleK = 1;
			}
			if (_fUnSubForce) {

				_fUnSubForce = false;

				forceSub.shutdown();
			}

			if (_fForcem) {

				_fForcem = false;

//				forcePosDes.translation() = meas_sensor_tip_state.translation()
//						+ _forceK
//								* Eigen::Vector3d(forceCartW.wrench.force.x,
//										forceCartW.wrench.force.y,
//										forceCartW.wrench.force.z);

				//jacobian = meas_kin_state->getJacobian(meas_arm_group);

				try {

					meas_kin_state->getJacobian(meas_arm_group,
							meas_kin_state->getLinkModel(
									meas_arm_group->getLinkModelNames().back()),
							Eigen::Vector3d(0.0, 0.0, 0.0), jacobian);

					//jacobian = jacobian.block<3, 3>(0, 0);

					//jacobian = jacobian.inverse();

					//ROS_INFO_STREAM("Jacobian: \n" << jacobian);

					//_dqDes = jacobian.transpose().block<3, 3>(0, 0);
					//_dqDes = jacobian.inverse();
					_dqDes = jacobian.transpose().block<3, 3>(0, 0)
							* (_forceK
									* Eigen::Vector3d(forceCartW.wrench.force.x,
											forceCartW.wrench.force.y,
											forceCartW.wrench.force.z));

					if (des_kin_state->integrateVariableVelocity(des_arm_group,
							_dqDes, 1 / ctlFs)) {

						des_kin_state->enforceBounds(des_arm_group);

						des_kin_state->update(true);

						des_kin_state->copyJointGroupPositions(des_arm_group,
								qDes);

						setJointRefs(qDes);

						tf::poseEigenToMsg(des_sensor_tip_state,
								posDesMsg.pose);

						desPosP.publish(posDesMsg);

					} else {

						ROS_ERROR("Error Solving IKP");
					}

				} catch (moveit::Exception::exception & e) {

					ROS_ERROR(" Unable to calculate Jacobian");
				}

				if (_idxPose) {

					qFake = _qIdxS + (_qMeas - _qIdxM) * _scaleK;

					fakeJntMsg.position = std::vector<double>(qFake.data(),
							qFake.data() + qFake.size());

					qFake = (_qMeas - _qIdxM) * _scaleK * ctlFs;

					fakeJntMsg.velocity = std::vector<double>(qFake.data(),
							qFake.data() + qFake.size());

					ctlFakeP.publish(fakeJntMsg);

				}

//				tf::poseEigenToMsg(forcePosDes, posDesMsg.pose);
//
//				desPosP.publish(posDesMsg);

//				if (des_kin_state->setFromIK(des_arm_group, posDesMsg.pose, 10,
//						0.001)) {

			}

			break;
		}

		ros::spinOnce();

		loopRate.sleep();
	}
}

