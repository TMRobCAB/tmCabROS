#ifndef JNT_TRAY_INT_H_

#define JNT_TRAY_INT_H_

#include <std_msgs/Bool.h>

#include <stdio.h>

#include <iostream>

#include <stdint.h>

#include "stdint.h"

#include <sstream>

#include "ros/ros.h"

#include "tm_cab/POS_REF.h"

#include "tm_cab/SYS_STATUS.h"

#include <tm_cab/jnt_traj.h>

#include <math.h>

#include <string>

#define ATAN_X_RANGE 16.0
#define ATAN_SCALE 1.08597

class JointTrayInterp {
public:

	JointTrayInterp(float fs) :
			_sysRunning(false), _wIni(0), _wFinal(0), _fs(fs), _t(0), _dPos(0), _duration(
					0), _accT(0), _done(true), _breaking(false), _nSteps(0) {
	}
	;
	~JointTrayInterp() {
	}

	bool isDone() const {
		return _done;
	}

	float getFs() const {
		return _fs;
	}

	void setFs(float fs) {
		_fs = fs;
	}

	;

	bool ExeTraj(tm_cab::jnt_traj::Request &req,
			tm_cab::jnt_traj::Response &res);

	void sysStatusCallback(const tm_cab::SYS_STATUS & msg);

private:

	void ResetInterp(float vIni, float vFinal);

	float ComputeNextPointArc();

	float ComputeNextPointLin();

	float _fs;

	float _dPos;

	float _duration;

	float _accT;

	int _nSteps;

	float _t;

	float _wIni;

	float _wFinal;

	bool _done;

	bool _breaking;

	bool _sysRunning;
};

#endif
