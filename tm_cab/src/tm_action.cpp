#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>

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

#include "tm_cab/POS_REF.h"

#include "tm_cab/START.h"

#include "tm_cab/STOP.h"

#include "tm_cab/START_TM_CTL.h"

#include "tm_cab/STOP_TM_CTL.h"

#include "tm_cab/PTP_TM_CTL.h"

#include "tm_cab/CTL_MODE.h"

#include <std_srvs/Trigger.h>

#include <termios.h>

using namespace std;

int _jointNum;

char getchNonBlock() {
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	char c = getchar();  // read character (non-blocking)

	c = getchar();  // read character (non-blocking)

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

bool isFloat(string myString) {
	istringstream iss(myString);
	float f;
	iss >> noskipws >> f; // noskipws considers leading whitespace invalid
	// Check the entire string was consumed and if either failbit or badbit is set
	return iss.eof() && !iss.fail();
}

class CmdFunc {
public:
	string _cmd;
	string _desc;
	int (*_Fun)(int argc, char **argv, int argIdx);

	CmdFunc(string cmd, string desc,
			int (*Fun)(int argc, char **argv, int argIdx)) :
			_desc(desc), _cmd(cmd), _Fun(Fun) {
	}
	;
	~CmdFunc() { /* free(name); */
	}
};

int F_Ptp(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1]) || !isFloat(argv[argIdx + 2])
			|| !isFloat(argv[argIdx + 3])) {

		ROS_ERROR("Arguments must be floats");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<tm_cab::PTP_TM_CTL>(
			"tm_ptp");

	tm_cab::PTP_TM_CTL srv;

	srv.request.Xd = atof(argv[argIdx + 1]);
	srv.request.Yd = atof(argv[argIdx + 2]);
	srv.request.Zd = atof(argv[argIdx + 3]);
	if (client.call(srv)) {

		if (srv.response.ack) {

			//ROS_INFO("PTP set");
		}

	} else {

		ROS_ERROR(
				"Failed to command PTP. Sys not running or point not reachable");

		return -1;
	}

	return 0;
}

int F_CtlModeForce(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1])) {

		ROS_ERROR("Arguments must be a float");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<tm_cab::CTL_MODE>(
			"tm_ctl_set_mode");

	tm_cab::CTL_MODE srv;

	srv.request.mode = tm_cab::CTL_MODE::Request::FORCE_POS;
	srv.request.force_k = atof(argv[argIdx + 1]);

	if (client.call(srv)) {

		if (srv.response.ack)
			ROS_INFO("Ctl mode set");

	} else {

		ROS_ERROR("Failed to set controller mode");

		return -1;
	}

	return 0;
}

int F_CtlModePTP(int argc, char **argv, int argIdx) {

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<tm_cab::CTL_MODE>(
			"tm_ctl_set_mode");

	tm_cab::CTL_MODE srv;

	srv.request.mode = tm_cab::CTL_MODE::Request::PTP;

	//ROS_INFO("calling set ptp");

	if (client.call(srv)) {

		//ROS_INFO("set ptp called");

		if (srv.response.ack)
			ROS_INFO("Ctl mode set");

	} else {

		ROS_ERROR("Failed to set controller mode");

		return -1;
	}

	return 0;

}

int F_SetSafeCurr(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1]) || !isFloat(argv[argIdx + 2])) {

		ROS_ERROR("Arguments must be floats");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SAFE_CURR srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_safe_curr";

		client = n.serviceClient<tm_cab::SAFE_CURR>(srvName.str());

		srv.request.maxC = atof(argv[argIdx + 1]);
		srv.request.minC = atof(argv[argIdx + 2]);
		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Current Safety Bounds Set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SetSafeForce(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1]) || !isFloat(argv[argIdx + 2])) {

		ROS_ERROR("Arguments must be floats");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SAFE_FORCE srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_safe_force";

		client = n.serviceClient<tm_cab::SAFE_FORCE>(srvName.str());

		srv.request.maxF = atof(argv[argIdx + 1]);
		srv.request.minF = atof(argv[argIdx + 2]);
		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Force Safety Bounds Set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SetSafePosE(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1]) || !isFloat(argv[argIdx + 2])) {

		ROS_ERROR("Arguments must be floats");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SAFE_POS_E srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_safe_Pos_E";

		client = n.serviceClient<tm_cab::SAFE_POS_E>(srvName.str());

		srv.request.SPEM = (float) atof(argv[argIdx + 1]);
		srv.request.SPEm = (float) atof(argv[argIdx + 2]);
		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Position Error Safety Bounds Set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SetSafePos(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1]) || !isFloat(argv[argIdx + 2])) {

		ROS_ERROR("Arguments must be floats");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SAFE_POS srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_safe_Pos";

		client = n.serviceClient<tm_cab::SAFE_POS>(srvName.str());

		srv.request.maxPos = atof(argv[argIdx + 1]);
		srv.request.minPos = atof(argv[argIdx + 2]);
		if (client.call(srv)) {

			float maxP = atof(argv[argIdx + 1]), minP = atof(argv[argIdx + 2]);

			if (srv.response.ack)
				ROS_INFO("Position Safety Bounds Set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SetSafePWM(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1]) || !isFloat(argv[argIdx + 2])) {

		ROS_ERROR("Arguments must be floats");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SAFE_PWM srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_safe_PWM";

		client = n.serviceClient<tm_cab::SAFE_PWM>(srvName.str());

		srv.request.maxP = atof(argv[argIdx + 1]);
		srv.request.minP = atof(argv[argIdx + 2]);
		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("PWM Safety Bounds Set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SetSafeSpeed(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1]) || !isFloat(argv[argIdx + 2])) {

		ROS_ERROR("Arguments must be floats");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SAFE_SPEED srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName.str("");
		srvName << "/joint_" << i << "_set_safe_Speed";

		client = n.serviceClient<tm_cab::SAFE_SPEED>(srvName.str());

		srv.request.maxS = atof(argv[argIdx + 1]);
		srv.request.minS = atof(argv[argIdx + 2]);
		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Speed Safety Bounds Set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SetInter(int argc, char **argv, int argIdx) {

	if (!isFloat(string(argv[argIdx + 2]))) {

		ROS_ERROR("Second arguments must be a float");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SET_INTER srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_inter";

		client = n.serviceClient<tm_cab::SET_INTER>(srvName.str());

		switch (*argv[argIdx + 1]) {
		case 'l':
			srv.request.id = tm_cab::SET_INTER::Request::FLIN;
			break;
		case 'a':
			srv.request.id = tm_cab::SET_INTER::Request::FARC;
			break;
		default:
			ROS_ERROR("First arguments must l (linear) or a (arctg)");

			return -1;

			break;
		}

		srv.request.time_change = atof(argv[argIdx + 2]);
		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Break interpolator's parameters set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SetParsPosPID(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 2])) {

		ROS_ERROR("Second argument must be a float");

		//return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SET_PAR_PID srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_par_pid_pos";

		client = n.serviceClient<tm_cab::SET_PAR_PID>(
				srvName.str());

		if (strcmp(argv[argIdx + 1], "kp") == 0) {

			srv.request.par_id = tm_cab::SET_PAR_PID::Request::KP;
		} else if (strcmp(argv[argIdx + 1], "ki") == 0) {

			srv.request.par_id = tm_cab::SET_PAR_PID::Request::KI;
		} else if (strcmp(argv[argIdx + 1], "kd") == 0) {

			srv.request.par_id = tm_cab::SET_PAR_PID::Request::KD;
		} else if (strcmp(argv[argIdx + 1], "sat") == 0) {

			srv.request.par_id = tm_cab::SET_PAR_PID::Request::SAT;
		} else if (strcmp(argv[argIdx + 1], "isat") == 0) {

			srv.request.par_id = tm_cab::SET_PAR_PID::Request::ISAT;
		} else {

			ROS_ERROR("First argument must be among: kp ki kd sat isat)");

			//return -1;
		}

		srv.request.pid_id = tm_cab::SET_PAR_PID::Request::POS_PID;

		srv.request.value = atof(argv[argIdx + 2]);
		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Pos PID parameter set Set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SelCtl(int argc, char **argv, int argIdx) {

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;

	tm_cab::SELECT_CTL srv;
	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_select_Ctl";

		client = n.serviceClient<tm_cab::SELECT_CTL>(srvName.str());

		if (strcmp(argv[argIdx + 1], "break") == 0) {

			srv.request.id = tm_cab::SELECT_CTL::Request::BREAK_CTL;
		} else if (strcmp(argv[argIdx + 1], "pos") == 0) {

			srv.request.id = tm_cab::SELECT_CTL::Request::POS_CTL;
		} else if (strcmp(argv[argIdx + 1], "speed") == 0) {

			srv.request.id = tm_cab::SELECT_CTL::Request::SPEED_CTL;
		} else if (strcmp(argv[argIdx + 1], "force") == 0) {

			srv.request.id = tm_cab::SELECT_CTL::Request::FORCE_CTL;
		} else if (strcmp(argv[argIdx + 1], "idx") == 0) {

			srv.request.id = tm_cab::SELECT_CTL::Request::IDX_CTL;
		} else {

			ROS_ERROR("Argument must be among: break pos speed force idx)");

			return -1;
		}

		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Controller selected Set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_SetFS(int argc, char **argv, int argIdx) {

	if (!isFloat(argv[argIdx + 1])) {

		ROS_ERROR("Argument must be a float");

		return -1;
	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SET_SYS_FS srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_sys_FS";

		client = n.serviceClient<tm_cab::SET_SYS_FS>(srvName.str());

		srv.request.sysFS = atoi(argv[argIdx + 1]);
		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Controller FS set");

		} else {

			ROS_ERROR("Failed to call service %s", srvName.str().data());

			//return -1;
		}
	}
	return 0;
}

int F_StartTM(int argc, char **argv, int argIdx) {

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::START_TM_CTL srv;

	srvName << "start_ctl";

	client = n.serviceClient<tm_cab::START_TM_CTL>(srvName.str());

	if (client.call(srv)) {

		if (srv.response.ack)
			ROS_INFO("TM Controller Started");

	} else {

		ROS_ERROR("Failed to call service %s", srvName.str().data());

		//return -1;
	}
	return 0;
}

int F_StopTM(int argc, char **argv, int argIdx) {

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::STOP_TM_CTL srv;

	srvName << "stop_ctl";

	client = n.serviceClient<tm_cab::STOP_TM_CTL>(srvName.str());

	if (client.call(srv)) {

		if (srv.response.ack)
			ROS_INFO("TM Controller Stopped");

	} else {

		ROS_ERROR("Failed to call service %s", srvName.str().data());

		//return -1;
	}
	return 0;
}

int F_IdxJoint(int argc, char **argv, int argIdx) {

	int jNum = atoi(argv[argIdx + 1]);

	if (!((unsigned) (jNum - 1) <= (3 - 1))) {

		ROS_ERROR("Invalid joint num");

		//return -1;

	}

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;

	srvName.str("");
	srvName << "/joint_" << jNum << "_start";

	client = n.serviceClient<tm_cab::START>(srvName.str());

	tm_cab::START startSrv;

	if (client.call(startSrv)) {

		if (startSrv.response.ack)
			ROS_INFO(
					"Controller Started: \n		uArrow -> +rot \n		dArrow -> -rot  \n		esc -> stop");

	} else {

		ROS_ERROR("Failed to call service %s", srvName.str().data());

		//return -1;
	}

	stringstream pubName;

	pubName.clear();
	pubName << "/joint_" << jNum << "_pos_ref";

	ros::Publisher keyPub = n.advertise<tm_cab::POS_REF>(
			pubName.str(), 1000);

	tm_cab::POS_REF refMsg;

	char key = 0;

	ros::Rate loop_rate(10);

	refMsg.q_D = 0;

	while (key != 27) {

		key = getchar();

		switch (key) {
		case '+':    // key up

			refMsg.q_D = 1;
			break;
		case '-':    // key down

			refMsg.q_D = -1;
			break;
		case '\n':    // key down

			break;
		case 27:

			break;

		default:

			refMsg.q_D = 0;

			break;
		}

		keyPub.publish(refMsg);

		loop_rate.sleep();
	}

	stringstream srvS;

	srvS << "/joint_" << jNum << "_stop";

	client.shutdown();

	client = n.serviceClient<tm_cab::STOP>(srvS.str());

	tm_cab::STOP stopSrv;

	if (client.call(stopSrv)) {

		if (stopSrv.response.ack)
			ROS_INFO("Idx Controller Stoped");

	} else {

		ROS_ERROR("Failed to call service %s", srvS.str().data());

		//return -1;
	}

	return 0;
}

int F_SetJoinSafe(int argc, char **argv, int argIdx) {

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	tm_cab::SAFE_STATE srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_set_safe";

		client = n.serviceClient<tm_cab::SAFE_STATE>(srvName.str());

		if (client.call(srv)) {

			if (srv.response.ack)
				ROS_INFO("Joint %d safety acknowledge", i);
			else
				ROS_WARN("Joint %d already safe", i);

		} else {

			ROS_ERROR("Error Calling joint_%d safe acknowledge", i);

			//return -1;
		}
	}
	return 0;
}

int F_Update(int argc, char **argv, int argIdx) {

	ros::NodeHandle n;
	ros::ServiceClient client;

	stringstream srvName;
	std_srvs::Trigger srv;

	for (int i = 1; i < 4; ++i) {

		if (_jointNum != 0 && _jointNum != i)
			continue;

		srvName.str("");
		srvName << "/joint_" << i << "_update";

		client = n.serviceClient<std_srvs::Trigger>(srvName.str());

		if (client.call(srv)) {

			if (srv.response.success)
				ROS_INFO("Joint %d Updating...", i);
			else
				ROS_WARN("Unable to update Joint %d", i);

		} else {

			ROS_ERROR("Error Calling joint_%d update", i);

			//return -1;
		}
	}
	return 0;
}

vector<CmdFunc> _cmdList;

void PrintUsage(void) {

	cout << "Usage: tm_action {-j | <joint_number>} <command> <[args]>" << endl;
	cout << endl;
	cout
			<< "		-j	Specify the joint to which the command will be applied. If not set, the command is applied to all joints. Has no effect when executing start, stop, idx"
			<< endl;

	for (int i = 0; i < _cmdList.size(); ++i) {
		cout << endl;
		cout << "		" << _cmdList[i]._cmd << "	" << _cmdList[i]._desc << endl;
	}

}

int main(int argc, char **argv) {

	_cmdList.push_back(
			CmdFunc("ptp",
					" Set a new desired point. Usage: ptp <Xd> <Yd> <zD> ",
					F_Ptp));
	_cmdList.push_back(
			CmdFunc("cS", " Set Current Safe Bounds. Usage: cS <max> <min> ",
					F_SetSafeCurr));
	_cmdList.push_back(
			CmdFunc("fS", " Set force Safe Bounds. Usage: fS <max> <min> ",
					F_SetSafeForce));
	_cmdList.push_back(
			CmdFunc("pS", " Set position Safe Bounds. Usage: pS <max> <min> ",
					F_SetSafePos));
	_cmdList.push_back(
			CmdFunc("peS",
					" Set position error Safe Bounds. peS: curr <max> <min> ",
					F_SetSafePosE));
	_cmdList.push_back(
			CmdFunc("pwmS", " Set PWM Safe Bounds. Usage: pwmS <max> <min> ",
					F_SetSafePWM));
	_cmdList.push_back(
			CmdFunc("sS", " Set speed Safe Bounds. Usage: sS <max> <min> ",
					F_SetSafeSpeed));
	_cmdList.push_back(
			CmdFunc("inter",
					"Set break interpolator parameters. Usage: inter <func: l - lineal, a - arctg> <T_change>",
					F_SetInter));
	_cmdList.push_back(
			CmdFunc("PID",
					"Set position PID parameters. Usage: PID <ParId: kp, ki, kd, sat, isat> <value>",
					F_SetParsPosPID));
	_cmdList.push_back(
			CmdFunc("sel",
					"Select a controller. Usage: sel <CtlId: break, pos, speed, force>",
					F_SelCtl));
	_cmdList.push_back(
			CmdFunc("fs", "Set the controllers FS. Usage: fs <Value>",
					F_SetFS));
	_cmdList.push_back(
			CmdFunc("start", "Start TM system. Usage: start", F_StartTM));
	_cmdList.push_back(
			CmdFunc("stop", "Stop TM system. Usage: stop", F_StopTM));
	_cmdList.push_back(
			CmdFunc("idx", "Run joint index process. Usage: idx <jNum>",
					F_IdxJoint));
	_cmdList.push_back(
			CmdFunc("ackSafe", "Acknowledge joint safety", F_SetJoinSafe));
	_cmdList.push_back(
			CmdFunc("modPTP", "Select PTP control mode", F_CtlModePTP));
	_cmdList.push_back(
			CmdFunc("modForce", "Select force control mode (sensor referenced)",
					F_CtlModeForce));
	_cmdList.push_back(
			CmdFunc("update",
					"Update joint driver's code (.bin file must be located in [package/tiva_app/joint_n.bin]))",
					F_Update));

	ros::init(argc, argv, "tm_action");

	int argIdx = 0;

	if (argc < 2) {

		PrintUsage();

		return -1;
	}

	_jointNum = 0;

	if (strcmp(argv[++argIdx], "-j") == 0) {
		// We know the next argument *should* be the filename:

		int jn = atoi(argv[++argIdx]);

		if (jn < 1 || jn > 3) {

			argIdx--;
		} else {

			_jointNum = jn;

			argIdx++;
		}
	}

	for (int i = 0; i < _cmdList.size(); ++i) {

		if (strcmp(argv[argIdx], _cmdList[i]._cmd.data()) == 0)
			return _cmdList[i]._Fun(argc, argv, argIdx);
	}

	PrintUsage();

	return -1;
}
