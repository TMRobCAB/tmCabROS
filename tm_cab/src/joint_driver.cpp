#include "stdint.h"

extern "C" {
#include "sflash/sflash.h"
}

#include <sstream>

#include "ros/ros.h"

#include <ros/package.h>

#include <std_msgs/Bool.h>

#include <std_srvs/Trigger.h>

#include "tm_cab/AddTwoInts.h"

#include "tm_cab/SAFE_CURR.h"

#include "tm_cab/SAFE_FORCE.h"

#include "tm_cab/SAFE_POS.h"

#include "tm_cab/SAFE_POS_E.h"

#include "tm_cab/SAFE_PWM.h"

#include "tm_cab/SAFE_SPEED.h"

#include "tm_cab/SELECT_CTL.h"

#include "tm_cab/SET_INTER.h"

#include "tm_cab/SAFE_STATE.h"

#include "tm_cab/SET_SYS_FS.h"

#include "tm_cab/SET_PAR_PID.h"

#include "tm_cab/START.h"

#include "tm_cab/STOP.h"

#include "tm_cab/POS_REF.h"

#include "tm_cab/Num.h"

#include "tm_cab/set_pwm_pars.h"

#include "tm_cab/TOPIC_STATUS.h"

#include "tm_cab/TOPIC_VARIABLES.h"

#include "DriverFsm/incfiles.h"

#include "DriverFsm/SysFsm.h"

#include "DriverFsm/ComCmds.h"

#include "DriverFsm/SysClk.h"

#include "DriverFsm/CommDefines.h"

#include <ros/callback_queue.h>

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include "msg_shared.hpp"

using namespace boost::interprocess;

#define QEI_MAX_CNT_DEFAULT 10000

#define TIMEOUT_PAR_SET 5.0f

jnt_ref_shared * _jntRefMsg;
jnt_state_shared * _jntStateMsg;

SysClk g_SysClk = SysClk();

ComPort _tivaPort;

ComCmds _tivaCmds(&_tivaPort);

SysFSM _tivaFSM(&_tivaCmds, 1);

std::string _port_name;

tm_cab::TOPIC_STATUS _statusMsg;

ros::Publisher _statusPublisher;
ros::Publisher _varPublisher;

std::string _jointName;

float _qOffset, _qeiMaxCnt, _qKVW, _qKVT, _dbPos, _dbNeg, _slopePos, _slopeNeg;
int _qSign;

bool _fSendRefs = false;

struct PIDPars {
	float kp;
	float ki;
	float kd;
	float sat;
	float isat;
} _posPID, _speedPID;

enum SchemeVector {
	POS, SPEED, FORCE, IDX, BREAK
} _jntScheme;

StateVector _oldState;

tm_cab::TOPIC_STATUS::_status_type _oldStatus;

void UpdatePubState(void) {

	switch (_tivaFSM.getState()) {
	case OUT_OF_SYNCH:

		_statusMsg.status = tm_cab::TOPIC_STATUS::S_OUT_OF_SYNCH;

		ROS_INFO("%s status: Out of synch", _jointName.data());

//		_tivaPort.~ComPort();
//
//		ros::shutdown();

		break;
	case IDLE:

		_statusMsg.status = tm_cab::TOPIC_STATUS::S_IDLE;

		ROS_INFO("%s status: Idle", _jointName.data());

		break;
	case UNINDEXED:

		_statusMsg.status = tm_cab::TOPIC_STATUS::S_UNINDEXED;

		ROS_INFO("%s status: Unindexed", _jointName.data());

		break;
	case RUNNING:

		_statusMsg.status = tm_cab::TOPIC_STATUS::S_RUNNING;

		ROS_INFO("%s status: Running", _jointName.data());

		break;
	case MONITOR_BREAK:

		_statusMsg.status = tm_cab::TOPIC_STATUS::S_MONITOR_BREAK;

		ROS_INFO("%s status: Monitor break", _jointName.data());
		break;
	case UNSAFE:

		_statusMsg.status = tm_cab::TOPIC_STATUS::S_UNSAFE;

		switch (_tivaFSM.getSafeErrorId()) {

		case CMDS_ERR_SAFE_POS:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_POS;

			ROS_ERROR("%s : Position safe bounds breached", _jointName.data());

			break;
		case CMDS_ERR_SAFE_SPEED:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_SPEED;

			ROS_ERROR("%s : Speed safe bounds breached", _jointName.data());

			break;
		case CMDS_ERR_SAFE_PWM:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_PWM;

			ROS_ERROR("%s : PWM safe bounds breached", _jointName.data());

			break;
		case CMDS_ERR_SAFE_CURR:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_CURR;

			ROS_ERROR("%s : Current safe bounds breached", _jointName.data());

			break;
		case CMDS_ERR_SAFE_CURR_E:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_CURR_E;

			ROS_ERROR("%s : Current error safe bounds breached",
					_jointName.data());

			break;
		case CMDS_ERR_SAFE_SPEED_E:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_SPEED_E;

			ROS_ERROR("%s : Speed error safe bounds breached",
					_jointName.data());

			break;
		case CMDS_ERR_SAFE_POS_E:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_POS_E;

			ROS_ERROR("%s : Position error safe bounds breached",
					_jointName.data());

			break;
		case CMDS_ERR_SAFE_FORCE_E:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_FORCE_E;

			ROS_ERROR("%s : Force error safe bounds breached",
					_jointName.data());

			break;
		case CMDS_ERR_SAFE_FORCE:

			_statusMsg.safeErrId =
					tm_cab::TOPIC_STATUS::ERR_SAFE_FORCE;

			ROS_ERROR("%s : Force safe bounds breached", _jointName.data());

			break;
		default:
			break;
		}

		break;
	default:
		break;
	}
}

bool TivaNode_AssertSet(unsigned char cmdId);

int32_t JointDriver_RunLoop(void) {

	int32_t ret;

	tm_cab::TOPIC_VARIABLES varMsg;

	ret = _tivaFSM.RunLoop();

	if (_oldState != _tivaFSM.getState() || _oldStatus != _statusMsg.status) {

		_statusMsg.index = _tivaFSM.isIndex();
		_statusMsg.safe = _tivaFSM.isSafe();

		UpdatePubState();

		_oldState = _tivaFSM.getState();
		_oldStatus = _statusMsg.status;
	}

	_statusPublisher.publish(_statusMsg);

	if (_tivaFSM.isVarsAvail()) {

		//ROS_INFO("new Vars");

		_tivaFSM.GetVar(SIG_ID_POS, &varMsg.q);

		varMsg.q = (float) (varMsg.q * _qSign - _qOffset);

		_tivaFSM.GetVar(SIG_ID_SPEED, &varMsg.dq);

		varMsg.dq *= _qSign;

		_tivaFSM.GetVar(SIG_ID_CURR, &varMsg.Ia);

		varMsg.Ia *= _qSign;

		_tivaFSM.GetVar(SIG_ID_PWM, &varMsg.Ua);

		varMsg.Ua *= _qSign;


		_jntStateMsg->mutex.lock();

		_jntStateMsg->q = varMsg.q;

		_jntStateMsg->dq = varMsg.dq;

		_jntStateMsg->ua = varMsg.Ua;

		_jntStateMsg->newState = true;

		_jntStateMsg->mutex.unlock();


		_varPublisher.publish(varMsg);

		_tivaFSM.VarsPublished();
	}

	return ret;

}

bool TivaNode_AssertSet(unsigned char cmdId) {

	int32_t ret;

	ros::Rate loop_rate(100);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(TIMEOUT_PAR_SET); // Timeout of 2 seconds

	while (ret != cmdId) {

		ret = JointDriver_RunLoop();

		if (ret == CMDS_ERR_SAFE_NOT_FOUND) {

			_statusMsg.err = true;

			_statusMsg.errId = tm_cab::TOPIC_STATUS::ERR_SET_PAR;

			_statusPublisher.publish(_statusMsg);

			ROS_INFO("safe par not found");

			return false;
		}

		if (ret == CMDS_ERR_SEL_SCH) {

			ROS_INFO("Error Selecting Scheme");

			return false;
		}

		if (ret == CMDS_SYS_RUNNING) {

			ROS_INFO("Sys running");

			return false;
		}

		if (ret == CMDS_ERR_BAD_CMD) {

			_statusMsg.err = true;

			_statusMsg.errId = tm_cab::TOPIC_STATUS::ERR_BAD_CMD;

			_statusPublisher.publish(_statusMsg);

			ROS_INFO("Error Bad CMD");

			return false;
		}

		if (ret == CMDS_ERR_CTL_MOD_NOT_FOUND) {

			_statusMsg.err = true;

			_statusMsg.errId = tm_cab::TOPIC_STATUS::ERR_SET_PAR;

			_statusPublisher.publish(_statusMsg);

			ROS_INFO("Error control mode not found ");

			return false;

		}

		if (ros::Time::now() - start_time > timeout) {

			ROS_INFO("Return time out");

			return false;
		}

		loop_rate.sleep();
	}

	return true;
}
bool StartCtl(void) {

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_START_SCHEME;

	_statusPublisher.publish(_statusMsg);

	_tivaCmds.AddCmd(CMDM_START_CTL, 0);

	if (TivaNode_AssertSet(CMDS_CTL_STARTED)) {
		_fSendRefs = true;
		return true;
	}

	return false;

}

bool StopCtl(void) {

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_STOP_SCHEME;

	_statusPublisher.publish(_statusMsg);

	_tivaCmds.AddCmd(CMDM_STOP_CTL, 0);

	if (TivaNode_AssertSet(CMDS_CTL_STOPPED)) {
		_fSendRefs = false;
		return true;
	}

	return false;

}

bool TivaNode_UpdateSys(std_srvs::TriggerRequest &req,
		std_srvs::TriggerResponse &res) {
	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.success = false;

		return false;
	}

	_tivaCmds.AddCmd(CMDM_UPDATE, 0);

	if (TivaNode_AssertSet(CMDS_UPDATING)) {

		std::string updateAppPath = ros::package::getPath("tm_cab")
				+ "/tiva_app/" + _jointName.data() + ".bin";

		std::string updateCmd = "./sflash " + updateAppPath + " -p 0x2800 -c "
				+ _port_name + " -b 115200 -s 20 -d";

		std::vector<char *> args;
		std::istringstream iss(updateCmd);

		std::string token;
		while (iss >> token) {
			char *arg = new char[token.size() + 1];
			copy(token.begin(), token.end(), arg);
			arg[token.size()] = '\0';
			args.push_back(arg);
		}
		args.push_back(0);

		_tivaPort.~ComPort();

		ROS_INFO("Tiva Updating");
		ROS_INFO("%s", updateCmd.data());

		UpdateJointFlash((int32_t) args.size(), &args[0]);

		_tivaPort.Open(_port_name.data());

		for (size_t i = 0; i < args.size(); i++)
			delete[] args[i];

		res.success = true;

		return true;
	}

	res.success = false;

	return false;

}

bool TivaNode_StartCtl(tm_cab::START::Request &req,
		tm_cab::START::Response &res) {
	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	if (StartCtl()) {

		ROS_INFO("Tiva Controller Started");

		//_tivaFSM.setStart(true);

		res.ack = true;

		return true;
	}

	res.ack = false;

	return false;

}

bool TivaNode_StopCtl(tm_cab::STOP::Request &req,
		tm_cab::STOP::Response &res) {
	if (_tivaFSM.getState() != RUNNING) {

		res.ack = false;

		return false;
	}

	if (StopCtl()) {

		ROS_INFO("Tiva Controller Stopped");

		//_tivaFSM.setStop(true);

		res.ack = true;

		return true;
	}

	res.ack = false;

	return false;
}

bool TivaNode_Safe_State(tm_cab::SAFE_STATE::Request &req,
		tm_cab::SAFE_STATE::Response &res) {
	if (_tivaFSM.getState() != UNSAFE) {

		res.ack = false;

		return false;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_ACK, 0);

	if (TivaNode_AssertSet(CMDS_SAFE_ACK)) {

		ROS_INFO("Tiva safety acknowledge");

		_tivaFSM.setSafe(true);

		res.ack = true;

		return true;
	}

	res.ack = false;

	return false;

}

bool TivaNode_SelectCtl(tm_cab::SELECT_CTL::Request &req,
		tm_cab::SELECT_CTL::Response &res) {
	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SELECT_SCHEME;

	_statusPublisher.publish(_statusMsg);

	float ctlID;

	switch (req.id) {
	case tm_cab::SELECT_CTL::Request::BREAK_CTL:

		_jntScheme = BREAK;

		ctlID = CTL_ID_BREAK;
		break;

	case tm_cab::SELECT_CTL::Request::POS_CTL:

		_jntScheme = POS;

		ctlID = CTL_ID_POS;
		break;

	case tm_cab::SELECT_CTL::Request::SPEED_CTL:

		_jntScheme = SPEED;

		ctlID = CTL_ID_SPEED;
		break;

	case tm_cab::SELECT_CTL::Request::FORCE_CTL:

		_jntScheme = FORCE;

		ctlID = CTL_ID_FORCE;
		break;
	case tm_cab::SELECT_CTL::Request::IDX_CTL:

		_jntScheme = IDX;

		ctlID = CTL_ID_INDEX;
		break;

	default:
		res.ack = false;
		return false;
		break;
	}

	_tivaCmds.AddCmd(CMDM_SEL_SCHEME, ctlID);
	if (TivaNode_AssertSet(CMDS_SCHEME_SELECTED)) {

		res.ack = true;
		return true;
	}
	res.ack = false;
	return false;
}

bool TivaNode_SetFS(tm_cab::SET_SYS_FS::Request &req,
		tm_cab::SET_SYS_FS::Response &res) {
	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	_tivaCmds.AddCmd(CMDM_SET_SYS_FS, req.sysFS);

	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		res.ack = false;
		return false;
	}

	res.ack = true;
	return true;
}

bool TivaNode_SetInterp(tm_cab::SET_INTER::Request &req,
		tm_cab::SET_INTER::Response &res) {
	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	_tivaCmds.AddCmd(CMDM_SET_BREAK_T_CHANGE, req.time_change);

	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		res.ack = false;

		ROS_INFO("Error setting interpolator's parameters");

		return false;
	}

	switch (req.id) {
	case tm_cab::SET_INTER::Request::FLIN:

		_tivaCmds.AddCmd(CMDM_SET_BREAK_INT_FUN, INT_FUN_LINEAR);

		if (TivaNode_AssertSet(CMDS_PAR_SET)) {

			res.ack = true;
			return true;
		}
		res.ack = false;
		return false;

		break;

	case tm_cab::SET_INTER::Request::FARC:

		_tivaCmds.AddCmd(CMDM_SET_BREAK_INT_FUN, INT_FUN_ARCTG);

		if (TivaNode_AssertSet(CMDS_PAR_SET)) {

			res.ack = true;
			return true;
		}

		res.ack = false;
		return false;

		break;

	default:
		res.ack = false;
		return false;
		break;
	}
}

bool TivaNode_SetSafeSpeed(tm_cab::SAFE_SPEED::Request &req,
		tm_cab::SAFE_SPEED::Response &res) {

	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	int ret = 0;

	float maxP, minP;

	if (_qSign == 1) {
		maxP = req.maxS;
		minP = req.minS;
	} else {
		minP = _qSign * req.maxS;
		maxP = _qSign * req.minS;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_SPEED_MAX, maxP);

	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_SPEED_MIN, minP);

	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}
	res.ack = true;
	return true;

}

bool TivaNode_SetSafeSpeedE(tm_cab::SAFE_SPEED::Request &req,
		tm_cab::SAFE_SPEED::Response &res) {

	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	int ret = 0;

	float maxP, minP;

	if (_qSign == 1) {
		maxP = req.maxS;
		minP = req.minS;
	} else {
		minP = _qSign * req.maxS;
		maxP = _qSign * req.minS;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_SPEED_E_MAX, maxP);

	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_SPEED_E_MIN, minP);

	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}
	res.ack = true;
	return true;

}

bool TivaNode_SetParPwm(tm_cab::set_pwm_pars::Request &req,
		tm_cab::set_pwm_pars::Response &res) {
	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	float dbPos, dbNeg, slopePos, slopeNeg;

	if (_qSign == 1) {
		dbPos = req.deadBandPos;
		dbNeg = req.deadBandNeg;
		slopePos = req.slopePos;
		slopeNeg = req.slopeNeg;
	} else {
		dbPos = req.deadBandNeg;
		dbNeg = req.deadBandPos;
		slopePos = req.slopeNeg;
		slopeNeg = req.slopePos;
	}

	_tivaCmds.AddCmd(CMDM_SET_DB_POS_PWM, dbPos);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_dbPos = req.deadBandPos;

	_tivaCmds.AddCmd(CMDM_SET_DB_NEG_PWM, dbNeg);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_dbNeg = req.deadBandNeg;

	_tivaCmds.AddCmd(CMDM_SET_K_POS_PWM, slopePos);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_slopePos = req.slopePos;

	_tivaCmds.AddCmd(CMDM_SET_K_NEG_PWM, slopeNeg);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_slopeNeg = req.slopeNeg;

	res.ack = true;
	return true;
}

bool TivaNode_SetParPID(tm_cab::SET_PAR_PID::Request &req,
		tm_cab::SET_PAR_PID::Response &res) {
	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	unsigned char cmd;

	struct PIDPars * selPID = NULL;
	float * selPar = NULL;

	switch (req.pid_id) {
	case tm_cab::SET_PAR_PID::Request::POS_PID:

		selPID = &_posPID;

		cmd = CMDM_POS_PARS_START;
		break;
	case tm_cab::SET_PAR_PID::Request::SPEED_PID:

		selPID = &_speedPID;

		cmd = CMDM_SPEED_PARS_START;
		break;
	case tm_cab::SET_PAR_PID::Request::BREAK_PID:

		cmd = CMDM_BREAK_PARS_START;
		break;
	case tm_cab::SET_PAR_PID::Request::FORCE_PID:

		cmd = CMDM_FORCE_PARS_START;
		break;
	case tm_cab::SET_PAR_PID::Request::IDX_PID:

		cmd = CMDM_IDX_PARS_START;
		break;
	default:
		res.ack = false;
		return false;
		break;
	}

	switch (req.par_id) {
	case tm_cab::SET_PAR_PID::Request::KP:

		selPar = &selPID->kp;

		cmd += KP_OFFSET;
		break;
	case tm_cab::SET_PAR_PID::Request::KI:

		selPar = &selPID->ki;

		cmd += KI_OFFSET;
		break;
	case tm_cab::SET_PAR_PID::Request::KD:

		selPar = &selPID->kd;

		cmd += KD_OFFSET;
		break;
	case tm_cab::SET_PAR_PID::Request::ISAT:

		selPar = &selPID->sat;

		cmd += ISAT_OFFSET;
		break;
	case tm_cab::SET_PAR_PID::Request::SAT:

		selPar = &selPID->isat;

		cmd += SAT_OFFSET;
		break;
	default:
		res.ack = false;
		return false;
		break;
	}

	_tivaCmds.AddCmd(cmd, req.value);

	if (TivaNode_AssertSet(CMDS_PAR_SET)) {

		if (selPID != NULL)
			*selPar = req.value;

		res.ack = true;
		return true;
	}
	res.ack = false;
	return false;
}

bool setJointParameters(void) {

	ros::NodeHandle nh;

	tm_cab::SELECT_CTL selSrv;

	tm_cab::SET_PAR_PID pidSrv;

	tm_cab::set_pwm_pars pwmSrv;

	_tivaCmds.AddCmd(CMDM_SET_QEI_MAX_CNT, _qeiMaxCnt);

	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		ROS_DEBUG("%s QEI max count not set", _jointName.data());

		return false;
	}

	ROS_DEBUG("%s QEI max count set to %f", _jointName.data(), _qeiMaxCnt);

	selSrv.request.id = tm_cab::SELECT_CTL::Request::SPEED_CTL;

	TivaNode_SelectCtl(selSrv.request, selSrv.response);
	if (!selSrv.response.ack) {
//	_tivaCmds.AddCmd(CMDM_SEL_SCHEME, CTL_ID_SPEED);
//	if (!TivaNode_AssertSet(CMDS_SCHEME_SELECTED)) {
		ROS_ERROR("ERROR selecting speed ctl");
		return false;
	}

//kp
	_tivaCmds.AddCmd(CMDM_SET_SPEED_KP, _speedPID.kp);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set speed kp");
		return false;
	}

//ki
	_tivaCmds.AddCmd(CMDM_SET_SPEED_KI, _speedPID.ki);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set speed ki");
		return false;
	}

//kd
	_tivaCmds.AddCmd(CMDM_SET_SPEED_KD, _speedPID.kd);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set speed kd");
		return false;
	};

//sat
	_tivaCmds.AddCmd(CMDM_SET_SPEED_SAT, _speedPID.sat);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set speed sat");
		return false;
	}

//isat
	_tivaCmds.AddCmd(CMDM_SET_SPEED_ISAT, _speedPID.isat);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set speed isat");
		return false;
	}

	JointDriver_RunLoop();

	selSrv.request.id = tm_cab::SELECT_CTL::Request::POS_CTL;

	TivaNode_SelectCtl(selSrv.request, selSrv.response);
	if (!selSrv.response.ack) {

		// Set pos pid pars
//	_tivaCmds.AddCmd(CMDM_SEL_SCHEME, CTL_ID_POS);
//	if (!TivaNode_AssertSet(CMDS_SCHEME_SELECTED)) {
		ROS_ERROR("Failed to set pos ctl");
		return false;
	}

	//kp
	_tivaCmds.AddCmd(CMDM_SET_POS_KP, _posPID.kp);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set pos kp");
		return false;
	}

	//ki
	_tivaCmds.AddCmd(CMDM_SET_POS_KI, _posPID.ki);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set pos ki");
		return false;
	}

	//kd
	_tivaCmds.AddCmd(CMDM_SET_POS_KD, _posPID.kd);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set pos kd");
		return false;
	}

	//sat
	_tivaCmds.AddCmd(CMDM_SET_POS_SAT, _posPID.sat);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set pos sat");
		return false;
	}

	//isat
	//sat
	_tivaCmds.AddCmd(CMDM_SET_POS_ISAT, _posPID.isat);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {
		ROS_ERROR("Failed to set pos sat");
		return false;
	}

	pwmSrv.request.deadBandPos = _dbPos;
	pwmSrv.request.deadBandNeg = _dbNeg;
	pwmSrv.request.slopePos = _slopePos;
	pwmSrv.request.slopeNeg = _slopeNeg;

	if (!TivaNode_SetParPwm(pwmSrv.request, pwmSrv.response)) {
		if (!pwmSrv.response.ack) {

			ROS_ERROR("Failed to set pwm Pars");
			return false;
		}
	}

	ROS_INFO("%s parametes set", _jointName.data());

	return true;
}

bool TivaNode_SetSafePWM(tm_cab::SAFE_PWM::Request &req,
		tm_cab::SAFE_PWM::Response &res) {

	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	int ret = 0;

	float maxP, minP;

	if (_qSign == 1) {
		maxP = req.maxP;
		minP = req.minP;
	} else {
		minP = _qSign * req.maxP;
		maxP = _qSign * req.minP;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_PWM_MAX, maxP);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_PWM_MIN, minP);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}
	res.ack = true;
	return true;
}

bool TivaNode_SetSafePos(tm_cab::SAFE_POS::Request &req,
		tm_cab::SAFE_POS::Response &res) {

	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	int ret = 0;

	float maxP, minP;

	if (_qSign == 1) {
		maxP = req.maxPos + _qOffset;
		minP = req.minPos + _qOffset;
	} else {
		minP = _qSign * (req.maxPos + _qOffset);
		maxP = _qSign * (req.minPos + _qOffset);
	}

	_tivaCmds.AddCmd(CMDM_SAFE_POS_MAX, maxP);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_POS_MIN, minP);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}
	res.ack = true;
	return true;

}

bool TivaNode_SetSafePosE(tm_cab::SAFE_POS_E::Request &req,
		tm_cab::SAFE_POS_E::Response &res) {

	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	int ret = 0;

	float maxP, minP;

	if (_qSign == 1) {
		maxP = req.SPEM;
		minP = req.SPEm;
	} else {
		minP = _qSign * req.SPEM;
		maxP = _qSign * req.SPEm;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_POS_E_MAX, maxP);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_POS_E_MIN, minP);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}
	res.ack = true;
	return true;

//	ROS_INFO("Tiva Controller Started");
//return true;

}

bool TivaNode_SetSafeCurr(tm_cab::SAFE_CURR::Request &req,
		tm_cab::SAFE_CURR::Response &res) {

	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	int ret = 0;

	float maxP, minP;

	if (_qSign == 1) {
		maxP = req.maxC;
		minP = req.minC;
	} else {
		minP = _qSign * req.maxC;
		maxP = _qSign * req.minC;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_CURR_MAX, maxP);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_CURR_MIN, minP);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}
	res.ack = true;
	return true;

}

bool TivaNode_SetSafeForce(tm_cab::SAFE_FORCE::Request &req,
		tm_cab::SAFE_FORCE::Response &res) {

	if (_tivaFSM.getState() != IDLE && _tivaFSM.getState() != UNINDEXED) {

		res.ack = false;

		return false;
	}

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_SET_PAR;

	_statusPublisher.publish(_statusMsg);

	_tivaCmds.AddCmd(CMDM_SAFE_FORCE_MAX, req.maxF);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}

	_tivaCmds.AddCmd(CMDM_SAFE_FORCE_MIN, req.minF);
	if (!TivaNode_AssertSet(CMDS_PAR_SET)) {

		res.ack = false;
		return false;
	}
	res.ack = true;
	return true;

}

void TivaNode_JointSetPosRef(const tm_cab::POS_REF & msg) {
//ROS_INFO("PosSet");

	if (!_tivaFSM.isIndex())
		_tivaCmds.AddCmd(REF_ID_POS_D, msg.q_D);
}

void TivaNode_JointSetGCompEffort(const tm_cab::POS_REF & msg) {

//_tivaCmds.AddCmd(REF_ID_PWM_CMP, (float) (msg.q_D / _qKVT));
}

void TivaNode_JointSetSpeedRef(const tm_cab::POS_REF & msg) {

//_tivaCmds.AddCmd(REF_ID_SPEED_D, (float) _qSign * msg.q_D);
}

void TivaNode_Run(const std_msgs::Bool & msg) {

//TODO CHECK!!!!!
	if (msg.data) {

		if (_tivaFSM.getState() == IDLE) {

			_jntRefMsg->mutex.lock();

			_jntRefMsg->q = 0;
			_jntRefMsg->dq = 0;
			_jntRefMsg->pwmComp = 0;
			_jntRefMsg->newRef = false;

			_jntRefMsg->mutex.unlock();

			StartCtl();
			//UpdatePubState();
		}

	} else {

		if (_tivaFSM.getState() == RUNNING) {
			StopCtl();

			//UpdatePubState();
		}
	}
}

bool TivaNode_LoadParas(void) {

	ros::NodeHandle nh;

	if (!nh.getParam(_jointName + "/port_name", _port_name)) {

		ROS_ERROR("Parameter /port_name not found");

		return false;
	}

	if (!nh.getParam(_jointName + "/q_offset", _qOffset)) {

		ROS_ERROR("Parameter /q_offset not found");

		return false;
	}

	if (!nh.getParam(_jointName + "/q_sign", _qSign)) {

		ROS_ERROR("Parameter /q_sign not found");

		return false;
	}

	if (!nh.getParam(_jointName + "/qei_max_cnt", _qeiMaxCnt)) {

		ROS_INFO(
				"Parameter /qei_max_cnt not found, setting QEI max count to default");

		_qeiMaxCnt = QEI_MAX_CNT_DEFAULT;
	}

// Load Joint's Motor Pars
///////////////////////////////

	if (!nh.getParam(_jointName + "/kVW", _qKVW)) {

		ROS_ERROR("Parameter /kVW not found");

		return false;
	}

	if (!nh.getParam(_jointName + "/kVT", _qKVT)) {

		ROS_ERROR("Parameter /kVT not found");

		return false;
	}

// Load Position PID's Pars
///////////////////////////////

	if (!nh.getParam(_jointName + "/kp_pos", _posPID.kp)) {

		ROS_ERROR("Parameter /kp_pos not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/ki_pos", _posPID.ki)) {

		ROS_ERROR("Parameter /ki_pos not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/kd_pos", _posPID.kd)) {

		ROS_ERROR("Parameter /kd_pos not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/sat_pos", _posPID.sat)) {

		ROS_ERROR("Parameter /sat_pos not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/isat_pos", _posPID.isat)) {

		ROS_ERROR("Parameter /isat_pos not found");

		return false;
	}

// Load Speed PID's Pars
///////////////////////////////

	if (!nh.getParam(_jointName + "/kp_speed", _speedPID.kp)) {

		ROS_ERROR("Parameter /kp_speed not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/ki_speed", _speedPID.ki)) {

		ROS_ERROR("Parameter /ki_speed not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/kd_speed", _speedPID.kd)) {

		ROS_ERROR("Parameter /kd_speed not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/sat_speed", _speedPID.sat)) {

		ROS_ERROR("Parameter /sat_speed not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/isat_speed", _speedPID.isat)) {

		ROS_ERROR("Parameter /isat_speed not found");

		return false;
	}

	// Load PWM pars
	if (!nh.getParam(_jointName + "/dbPos", _dbPos)) {

		ROS_ERROR("Parameter /dbPos not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/dbNeg", _dbNeg)) {

		ROS_ERROR("Parameter /dbNeg not found");

		return false;
	}
	if (!nh.getParam(_jointName + "/slopePos", _slopePos)) {

		ROS_ERROR("Parameter /slopePos not found");

		return false;
	}

	if (!nh.getParam(_jointName + "/slopeNeg", _slopeNeg)) {

		ROS_ERROR("Parameter /slopeNeg not found");

		return false;
	}

	return true;
}

int main(int argc, char **argv) {

	_jntScheme = IDX;

	_tivaFSM.AddVar(SIG_ID_POS);
	_tivaFSM.AddVar(SIG_ID_SPEED);
	_tivaFSM.AddVar(SIG_ID_CURR);
	_tivaFSM.AddVar(SIG_ID_PWM);

	ros::init(argc, argv, "joint_driver");
	ros::NodeHandle n;

	_jointName = ros::this_node::getName();

	_jointName = _jointName.substr(1, _jointName.size() - 1);

/////////////////////////////////////////////////////////////////////////////////

	std::stringstream shmName;

	shmName.str("");
	shmName << _jointName << "_ref_shared";

	shared_memory_object::remove(shmName.str().data());

//Create a shared memory object.
	shared_memory_object jntRefShm(create_only, shmName.str().data(),
			read_write);

//Set size
	jntRefShm.truncate(sizeof(jnt_ref_shared));

//Map the whole shared memory in this process
	mapped_region regionRef(jntRefShm, read_write);

//Get the address of the mapped region
	void * jntRefaddr = regionRef.get_address();

//Construct the shared structure in memory
	_jntRefMsg = new (jntRefaddr) jnt_ref_shared;

	/////////////////////////////////////////////////////////////////////////////////

	shmName.str("");
	shmName << _jointName << "_state_shared";

	shared_memory_object::remove(shmName.str().data());

	//Create a shared memory object.
	shared_memory_object jntStateShm(create_only, shmName.str().data(),
			read_write);

	//Set size
	jntStateShm.truncate(sizeof(jnt_state_shared));

	//Map the whole shared memory in this process
	mapped_region regionState(jntStateShm, read_write);

	//Get the address of the mapped region
	void * jntStateaddr = regionState.get_address();

	//Construct the shared structure in memory
	_jntStateMsg = new (jntStateaddr) jnt_state_shared;

/////////////////////////////////////////////////////////////////////////////////

//ROS_INFO("%s", jointName.data());

	if (!TivaNode_LoadParas())
		return -1;

	ROS_INFO("opening port %s", _port_name.data());

	_tivaPort.Open(_port_name.data());

	ros::ServiceServer srvSafeCurr = n.advertiseService(
			_jointName + "_set_safe_curr", TivaNode_SetSafeCurr);
	ros::ServiceServer srvSafeForce = n.advertiseService(
			_jointName + "_set_safe_force", TivaNode_SetSafeForce);
	ros::ServiceServer srvSafePos_E = n.advertiseService(
			_jointName + "_set_safe_Pos_E", TivaNode_SetSafePosE);
	ros::ServiceServer srvSafePos = n.advertiseService(
			_jointName + "_set_safe_Pos", TivaNode_SetSafePos);
	ros::ServiceServer srvSafePWM = n.advertiseService(
			_jointName + "_set_safe_PWM", TivaNode_SetSafePWM);
	ros::ServiceServer srvSafeState = n.advertiseService(
			_jointName + "_set_safe", TivaNode_Safe_State);
	ros::ServiceServer srvSafeSpeed = n.advertiseService(
			_jointName + "_set_safe_Speed", TivaNode_SetSafeSpeed);
	ros::ServiceServer srvSafeSpeed_E = n.advertiseService(
			_jointName + "_set_safe_Speed_E", TivaNode_SetSafeSpeedE);
	ros::ServiceServer srvSelect_CTL = n.advertiseService(
			_jointName + "_select_Ctl", TivaNode_SelectCtl);
	ros::ServiceServer srvSetInter = n.advertiseService(
			_jointName + "_set_inter", TivaNode_SetInterp);
	ros::ServiceServer srvSetPID_pos = n.advertiseService(
			_jointName + "_set_par_pid", TivaNode_SetParPID);
	ros::ServiceServer srvSetPwmpars = n.advertiseService(
			_jointName + "_set_PWM_pars", TivaNode_SetParPwm);

	ros::ServiceServer srvSetFs = n.advertiseService(_jointName + "_set_sys_FS",
			TivaNode_SetFS);
	ros::ServiceServer srvStart = n.advertiseService(_jointName + "_start",
			TivaNode_StartCtl);
	ros::ServiceServer srvStop = n.advertiseService(_jointName + "_stop",
			TivaNode_StopCtl);
	ros::ServiceServer srvUpdate = n.advertiseService(_jointName + "_update",
			TivaNode_UpdateSys);

	ROS_INFO("TivaDriver's services running!");

	ros::Subscriber refPosSub = n.subscribe(_jointName + "_pos_ref", 1000,
			TivaNode_JointSetPosRef);

	ros::Subscriber refSpeedSub = n.subscribe(_jointName + "_speed_ref", 1000,
			TivaNode_JointSetSpeedRef);

	ros::Subscriber gCompRefSub = n.subscribe(_jointName + "_g_comp_effort",
			1000, TivaNode_JointSetGCompEffort);

	ros::Subscriber runSub = n.subscribe("sys_run", 1000, TivaNode_Run);

	_varPublisher = n.advertise<tm_cab::TOPIC_VARIABLES>(
			_jointName + "_state_vars", 1000);

	_statusPublisher = n.advertise<tm_cab::TOPIC_STATUS>(
			_jointName + "_status", 1000);

	ros::Rate loop_rate(100);

	tm_cab::TOPIC_VARIABLES varMsg;

	_statusMsg.status = tm_cab::TOPIC_STATUS::S_OUT_OF_SYNCH;
	_statusMsg.safe = true;
	_statusMsg.safeErrId = 0;
	_statusMsg.err = false;
	_statusMsg.errId = 0;
	_statusMsg.index = false;

	_tivaFSM.Rst();

	_oldState = _tivaFSM.getState();
	_oldStatus = _statusMsg.status;

	float qBuff, dqBuff, pwmCompBuff;
	bool newBuf;
	timespec subTime;

	while (ros::ok()) {

		if (_tivaFSM.getState() == RUNNING && _tivaFSM.isIndex()) {

			_jntRefMsg->mutex.lock();

			qBuff = _jntRefMsg->q;
			dqBuff = _jntRefMsg->dq;
			pwmCompBuff = _jntRefMsg->pwmComp;

			_jntRefMsg->mutex.unlock();

			_tivaCmds.AddCmd(REF_ID_PWM_CMP,
					(float) (_qSign * pwmCompBuff / _qKVT));

			switch (_jntScheme) {
			case POS:

				_tivaCmds.AddCmd(REF_ID_POS_D, (float) (_qSign * (qBuff + _qOffset)));

				break;
			case SPEED:

				_tivaCmds.AddCmd(REF_ID_SPEED_D, (float) _qSign * dqBuff);

				break;
			}

		}

		JointDriver_RunLoop();

		if (_tivaFSM.isJustSynched() && _tivaFSM.getState() == IDLE) {
			if (!setJointParameters()) {

				ROS_ERROR("Unable to set %s parameters, Shuting down node",
						_jointName.data());

				n.shutdown();
			}

			_tivaFSM.setJustSynched(false);
		}

		ros::spinOnce();

		loop_rate.sleep();
	}

	shared_memory_object::remove(shmName.str().data());
}
