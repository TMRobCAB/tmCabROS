/*
 * joint_tray_interp.cpp
 *
 *  Created on: May 6, 2016
 *      Author: l_vis
 */

#include "tm_cab/joint_tray_interp.h"

#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include "msg_shared.hpp"

#include <time.h>

using namespace boost::interprocess;

ros::Publisher _speedP1;
ros::Publisher _speedP2;
ros::Publisher _speedP3;

jnt_ref_shared * _jnt1RefMsg;
jnt_ref_shared * _jnt2RefMsg;
jnt_ref_shared * _jnt3RefMsg;

int main(int argc, char **argv) {

	JointTrayInterp trayInterp(100);

	ros::init(argc, argv, "joint_traj_interp");

	ros::NodeHandle n;

	ros::Subscriber sysStatusS = n.subscribe("/sys_status", 10,
			&JointTrayInterp::sysStatusCallback, &trayInterp);

	ros::ServiceServer srvSetFs = n.advertiseService("/exe_jnt_traj",
			&JointTrayInterp::ExeTraj, &trayInterp);

	_speedP1 = n.advertise<tm_cab::POS_REF>("/joint_1_speed_ref",
			100);
	_speedP2 = n.advertise<tm_cab::POS_REF>("/joint_2_speed_ref",
			100);
	_speedP3 = n.advertise<tm_cab::POS_REF>("/joint_3_speed_ref",
			100);

	bool fMemMapped;

	// Shared ref objects j1
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

	//Map the whole shared memory in this process
	mapped_region region1(jnt1RefShm, read_write);

	//Get the address of the mapped region
	void * jnt1Refaddr = region1.get_address();

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
	mapped_region region2(jnt2RefShm, read_write);

	//Get the address of the mapped region
	void * jnt2Refaddr = region2.get_address();

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
	mapped_region region3(jnt3RefShm, read_write);

	//Get the address of the mapped region
	void * jnt3Refaddr = region3.get_address();

	//Construct the shared structure in memory
	_jnt3RefMsg = static_cast<jnt_ref_shared*>(jnt3Refaddr);

	////////////////////////////////////////////////////////////////////
	while (ros::ok()) {

		ros::spin();

	}

	ROS_INFO("Joint Trajectory Interpolator shutting down");

	return 0;
}

void JointTrayInterp::sysStatusCallback(
		const tm_cab::SYS_STATUS & msg) {

	if (msg.sys_status == tm_cab::SYS_STATUS::RUNNING)
		_sysRunning = true;
	else
		_sysRunning = false;

}

bool JointTrayInterp::ExeTraj(tm_cab::jnt_traj::Request& req,
		tm_cab::jnt_traj::Response& res) {

	ros::NodeHandle m;

	ros::Publisher sPub;

	std::stringstream srvName;

	jnt_ref_shared * jntRefMsg;

	tm_cab::POS_REF speedRefMsg;

	if (!_sysRunning) {

		res.ack = false;

		ROS_ERROR("System not running");

		return false;
	}

	if (req.jntNum < 1 || req.jntNum > 3) {

		res.ack = false;

		ROS_ERROR("Joint number must be between 1 and 3");

		return false;
	}

	if (req.duration <= 2 * req.accT) {

		res.ack = false;

		ROS_ERROR(
				"Trajectory duration must last longer the 2 times the acceleration time");

		return false;
	}

	_dPos = req.dPos;

	_duration = req.duration;

	_accT = req.accT;

	_nSteps = req.nSteps;

	_wIni = 0;

	int jntNum = req.jntNum;

	switch (jntNum) {
	case 1:
		sPub = _speedP1;

		jntRefMsg = _jnt1RefMsg;
		break;
	case 2:
		sPub = _speedP2;

		jntRefMsg = _jnt2RefMsg;
		break;
	case 3:
		sPub = _speedP3;

		jntRefMsg = _jnt3RefMsg;
		break;
	default:
		break;
	}

	ROS_INFO("Publishing speed trajectory at %s", srvName.str().data());

	ros::Rate loop_rate(_fs);

	//bool firstPubFlag = true;

	//timespec pubTime;

	float * varId;

	float wMax;

	switch (req.varId) {
	case tm_cab::jnt_traj::Request::SPEED:

		_wFinal = _dPos / (_duration - _accT);

		_nSteps = 1;

		varId = &jntRefMsg->dq;
		break;
	case tm_cab::jnt_traj::Request::POS:

		_wFinal = _dPos / (_duration - _accT);

		_nSteps = 1;

		varId = &jntRefMsg->q;
		break;
	case tm_cab::jnt_traj::Request::PWM:

		_wFinal = _dPos;

		if (_nSteps != 1) {

			wMax = _duration;

			_wFinal = wMax * 2 / _nSteps;

			_duration = (_dPos / _wFinal) + _accT;
		}

		varId = &jntRefMsg->pwmComp;
		break;
	default:
		break;
	}

	for (int i = 0; i < _nSteps; i++) {

		_done = false;

		_t = 0;

		_breaking = false;

		while (ros::ok() && !_done) {

			switch (req.intType) {
			case tm_cab::jnt_traj::Request::ARC_TG:

				speedRefMsg.q_D = ComputeNextPointArc();

				break;
			case tm_cab::jnt_traj::Request::LIN:

				speedRefMsg.q_D = ComputeNextPointLin();

				break;
			default:
				break;
			}

			//////////////////////////////////////////////

			jntRefMsg->mutex.lock();

			*varId = (float) speedRefMsg.q_D;

			jntRefMsg->newRef = true;

			jntRefMsg->mutex.unlock();

			//////////////////////////////////////////////

			sPub.publish(speedRefMsg);

			loop_rate.sleep();
		}

		_wFinal = -_wIni;

		_wIni = 0;

		if (i % 2 == 1) {

			_wFinal += wMax * 2 / _nSteps;

			_duration -= (_dPos / _wFinal + _accT) * 2 / _nSteps;
		}

		//sleep(1);

	}

	res.ack = true;

	return true;
}

float JointTrayInterp::ComputeNextPointArc() {

	_t += 1 / _fs;

	if (_t <= _accT) {

		float w = atan((_t / _accT - 0.5) * ATAN_X_RANGE) * ATAN_SCALE / M_PI
				+ 0.5;
		;

		return (_wFinal - _wIni) * w + _wIni;

	} else if (_t > _accT && _t < _duration - _accT) {

		return _wFinal;

	} else if (_t > _duration - _accT && _t < _duration) {
		if (!_breaking) {

			_wIni = _wFinal;

			_wFinal = 0;

			_breaking = true;
		}

		float w = atan(
				((_t - (_duration - _accT)) / _accT - 0.5) * ATAN_X_RANGE)
				* ATAN_SCALE / M_PI + 0.5;

		return (_wFinal - _wIni) * w + _wIni;

	} else if (_t >= _duration) {

		_done = true;

		return 0;
	}

	_done = true;

	return 0;
}

float JointTrayInterp::ComputeNextPointLin() {

	_t += 1 / _fs;

	if (_t <= _accT) {

		return (_t * (_wFinal - _wIni) / _accT + _wIni);

	} else if (_t > _accT && _t < _duration - _accT) {

		return _wFinal;

	} else if (_t > _duration - _accT && _t < _duration) {
		if (!_breaking) {

			_wIni = _wFinal;

			_wFinal = 0;

			_breaking = true;
		}

		return ((_t - (_duration - _accT)) * (_wFinal - _wIni) / _accT + _wIni);

	} else if (_t >= _duration) {

		_done = true;

		return 0;
	}

	_done = true;

	return 0;
}
