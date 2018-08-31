#include "ros/ros.h"

#include <std_msgs/Bool.h>

#include <sstream>

#include <tm_cab/SYS_STATUS.h>

#include <tm_cab/START_TM_CTL.h>

#include <tm_cab/STOP_TM_CTL.h>

#include <tm_cab/TOPIC_STATUS.h>

#include <tm_cab/SAFE_STATE.h>

#define TIMEOUT_START_RUN	0.5f

using namespace tm_cab;

TOPIC_STATUS _jntStatus[3];

ros::ServiceClient _safeClient;

ros::ServiceServer _startSrv, _stopSrv;

ros::Publisher _runPub, _tmStatusPub;

ros::Subscriber _jnt1Sub, _jnt2Sub, _jnt3Sub;

enum TMStateVector {
	JNTS_OUT_OF_SYNCH,
	UNINDEXED,
	SYS_READY,
	RUNNING,
	MONITOR_BREAK,
	STOP_JOINTS,
	RESTORE_SAFE
} _tmState;

ros::Duration _tOutStartRun(TIMEOUT_START_RUN);
ros::Time _tStartRun;

bool _fStart = false, _fStop = false;

bool StartTmCtl(START_TM_CTL::Request &req, START_TM_CTL::Response &res) {

	if (_jntStatus[0].status != TOPIC_STATUS::S_IDLE
			|| _jntStatus[1].status != TOPIC_STATUS::S_IDLE
			|| _jntStatus[2].status != TOPIC_STATUS::S_IDLE) {

		res.ack = false;

		return false;
	}

	_fStart = true;

	res.ack = true;

	std_msgs::Bool runMsg;
	runMsg.data = true;
	_runPub.publish(runMsg);

	return true;
}

bool StopTmCtl(STOP_TM_CTL::Request &req, STOP_TM_CTL::Response &res) {

	_fStop = true;

	res.ack = true;

	std_msgs::Bool runMsg;
	runMsg.data = false;
	_runPub.publish(runMsg);

	return true;
}

void JntStatInit(void) {
	for (int i = 0; 0 < 3; ++i) {

		_jntStatus[i].status = TOPIC_STATUS::S_OUT_OF_SYNCH;

		_jntStatus[i].err = false;
		_jntStatus[i].errId = 0;

		_jntStatus[i].safe = true;
		_jntStatus[i].safeErrId = 0;

	}
}

void RstFSM(void) {

	_tmState = JNTS_OUT_OF_SYNCH;

	_fStart = false;
	_fStop = false;

	_startSrv.shutdown();
	_stopSrv.shutdown();

	std_msgs::Bool runMsg;
	runMsg.data = false;
	_runPub.publish(runMsg);
}

void Joint1StatusCallback(const tm_cab::TOPIC_STATUS & msg) {
	_jntStatus[0] = msg;
}

void Joint2StatusCallback(const tm_cab::TOPIC_STATUS & msg) {
	_jntStatus[1] = msg;
}

void Joint3StatusCallback(const tm_cab::TOPIC_STATUS & msg) {
	_jntStatus[2] = msg;
}

void TM_Init(ros::NodeHandle nodeH) {

	//ROS_INFO("mng_alive");

	//JntStatInit();

	_tmStatusPub = nodeH.advertise<SYS_STATUS>("sys_status", 1000);

	_runPub = nodeH.advertise<std_msgs::Bool>("sys_run", 1000);

	_jnt1Sub = nodeH.subscribe("/joint_1_status", 1000, Joint1StatusCallback);
	_jnt2Sub = nodeH.subscribe("/joint_2_status", 1000, Joint2StatusCallback);
	_jnt3Sub = nodeH.subscribe("/joint_3_status", 1000, Joint3StatusCallback);

	RstFSM();
}

void RunLoopTM(ros::NodeHandle nodeH) {

	//int32_t fsmRet = 0;

	bool jointsStopped;
	SAFE_STATE srv;

	if (_tmState != JNTS_OUT_OF_SYNCH &&
			(_jntStatus[0].status == TOPIC_STATUS::S_OUT_OF_SYNCH
			|| _jntStatus[1].status == TOPIC_STATUS::S_OUT_OF_SYNCH
			|| _jntStatus[2].status == TOPIC_STATUS::S_OUT_OF_SYNCH))
		RstFSM();

	switch (_tmState) {

	case JNTS_OUT_OF_SYNCH:

		// If this slave got synched: notify the master
		if (_jntStatus[0].status != TOPIC_STATUS::S_OUT_OF_SYNCH
				&& _jntStatus[1].status != TOPIC_STATUS::S_OUT_OF_SYNCH
				&& _jntStatus[2].status != TOPIC_STATUS::S_OUT_OF_SYNCH) {

			if (!_jntStatus[0].index || !_jntStatus[1].index
					|| !_jntStatus[2].index) {

				//Change active state
				_tmState = UNINDEXED;
			} else {

				_startSrv = nodeH.advertiseService("start_ctl", StartTmCtl);

				//Change active state
				_tmState = SYS_READY;
			}
		}
		break;

	case UNINDEXED:

		if (_jntStatus[0].index && _jntStatus[1].index && _jntStatus[2].index) {

			_startSrv = nodeH.advertiseService("start_ctl", StartTmCtl);

			//Change active state
			_tmState = SYS_READY;
		}

		break;

	case SYS_READY:

		if (!_jntStatus[0].safe || !_jntStatus[1].safe || !_jntStatus[2].safe) {

			_startSrv.shutdown();

			_tmState = RESTORE_SAFE;
		} else if (_fStart) {

			//Clear flag
			_fStart = false;

			_startSrv.shutdown();

			_stopSrv = nodeH.advertiseService("stop_ctl", StopTmCtl);

			_tStartRun = ros::Time::now();

			//Change active state
			_tmState = RUNNING;
		}

		break;

	case RUNNING:

		if (!_jntStatus[0].safe || !_jntStatus[1].safe || !_jntStatus[2].safe) {

			_stopSrv.shutdown();

			std_msgs::Bool runMsg;
			runMsg.data = false;
			_runPub.publish(runMsg);

			//Change active state
			_tmState = RESTORE_SAFE;

		} else if ((_jntStatus[0].status
						!= tm_cab::TOPIC_STATUS::S_RUNNING
				|| _jntStatus[1].status
						!= tm_cab::TOPIC_STATUS::S_RUNNING
				|| _jntStatus[2].status
						!= tm_cab::TOPIC_STATUS::S_RUNNING)
						&& ros::Time::now() - _tStartRun > _tOutStartRun) {

			_stopSrv.shutdown();

			std_msgs::Bool runMsg;
			runMsg.data = false;
			_runPub.publish(runMsg);

			//Change active state
			_tmState = STOP_JOINTS;

		} else if (_fStop) {

			//Clear flag
			_fStop = false;

			_stopSrv.shutdown();

			//Change active state
			_tmState = STOP_JOINTS;

			break;
		}

		break;

	case STOP_JOINTS:

		jointsStopped = true;

		for (int i = 0; i < 3; ++i) {

			if (_jntStatus[i].status == TOPIC_STATUS::S_IDLE)
				continue;

			jointsStopped = false;
		}

		// Stop scheme if requested of limits are exceeded
		if (jointsStopped) {
			//Change active state
			if (!_jntStatus[0].index || !_jntStatus[1].index
					|| !_jntStatus[2].index) {

				//Change active state
				_tmState = UNINDEXED;
			} else {

				_startSrv = nodeH.advertiseService("start_ctl", StartTmCtl);

				//Change active state
				_tmState = SYS_READY;
			}

		}
		break;

	case RESTORE_SAFE:

		if (_jntStatus[0].safe && _jntStatus[1].safe && _jntStatus[2].safe) {

			if (!_jntStatus[0].index || !_jntStatus[1].index
					|| !_jntStatus[2].index) {

				//Change active state
				_tmState = UNINDEXED;
			} else {

				_startSrv = nodeH.advertiseService("start_ctl", StartTmCtl);

				//Change active state
				_tmState = SYS_READY;
			}
		}
		break;

	default:

		break;
	}
}

void PublishTMStatus(void) {

	SYS_STATUS tmStatusMsg;

	tmStatusMsg.j1_stat = _jntStatus[0];
	tmStatusMsg.j2_stat = _jntStatus[1];
	tmStatusMsg.j3_stat = _jntStatus[2];

	switch (_tmState) {
	case JNTS_OUT_OF_SYNCH:
		tmStatusMsg.sys_status = SYS_STATUS::JNTS_OUT_OF_SYNCH;
		break;
	case UNINDEXED:
		tmStatusMsg.sys_status = SYS_STATUS::UNINDEXED;
		break;
	case SYS_READY:
		tmStatusMsg.sys_status = SYS_STATUS::SYS_READY;
		break;
	case RUNNING:
		tmStatusMsg.sys_status = SYS_STATUS::RUNNING;
		break;
	case MONITOR_BREAK:
		tmStatusMsg.sys_status = SYS_STATUS::MONITOR_BREAK;
		break;
	case STOP_JOINTS:
		tmStatusMsg.sys_status = SYS_STATUS::STOP_JOINTS;
		break;
	case RESTORE_SAFE:
		tmStatusMsg.sys_status = SYS_STATUS::RESTORE_SAFE;
		break;
	default:
		break;
	}

	_tmStatusPub.publish(tmStatusMsg);
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "tm_manager");

	ros::NodeHandle nodeH;

	TM_Init(nodeH);

	ros::Rate loop_rate(5);

	while (ros::ok()) {

		RunLoopTM(nodeH);

		ros::spinOnce();

		PublishTMStatus();

		loop_rate.sleep();
	}

	return 0;
}

