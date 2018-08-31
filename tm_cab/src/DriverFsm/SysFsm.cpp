#include "ros/ros.h"

#include "SysFsm.h"

SysFSM::SysFSM(ComCmds * comCmds, uint32_t selSchemeIdx) {

	//TODO obtener el esquema del driver

	_selSchemeIdx = selSchemeIdx;

	_comCmds = comCmds;

	_loggedVars.clear();

	Rst();
}

int32_t SysFSM::RunLoop(void) {

	int32_t fsmRet = 0;

	if (_fSynched) {

		if (_comCmds->CmdLineWrite() == COM_CMD_ERR_SEND) {
			_errCnt++;

			ROS_ERROR("WriteError");
		}
		fsmRet = _comCmds->CmdLineRead(this);

		if (fsmRet == COM_CMD_ERR_CRC) {
			_errCnt++;

			ROS_ERROR("ReadError");
		}
	}
	if (_errCnt >= COM_ERR_DESYNCH_NUM) {

		Rst();

		ROS_INFO("Driver Out Of Synch: com errors");

		return COM_CMD_ERR_SYNCH;

	}

	if (_state != OUT_OF_SYNCH && !Ping()) {

		Rst();

		ROS_INFO("Driver Out Of Synch: Ping timeout");

		return COM_CMD_ERR_SYNCH;

		//fsmRet = FSM_PING_TIMEOUT;
	}

	switch (_state) {

	case OUT_OF_SYNCH:

		S_OutOfSynch();

		if (_fStop)
			_fStop = false;

		// If this slave got synched: notify the master
		if (_fSynched){

			_fJustSynched = true;

			//Change active state
			_state = UNINDEXED;
		}

		break;

	case UNINDEXED:

		S_Unindexed();

		if (_fStop)
			_fStop = false;

		else if (_fIndex)

			//Change active state
			_state = IDLE;

		else if (_fStart) {

			//Clear flag
			_fStart = false;

			//Change active state
			_state = RUNNING;
		}


		break;

	case IDLE:

		if (_fStart) {

			//Clear flag
			_fStart = false;

			//Change active state
			_state = RUNNING;
		}

		break;

	case RUNNING:

		//fsmRet = S_RunScheme();

		// Stop scheme if requested of limits are exceeded
		if (!_fSafe) {

			//Change active state
			_state = UNSAFE;

		} else if (_fStop) {

			//Clear flag
			_fStop = false;

			//Change active state
			_state = MONITOR_BREAK;

		}

		break;

	case UNSAFE:

		//fsmRet = S_Unsafe();

		if (_fSafe && _fIndex)

			//Change active state
			_state = IDLE;

		else if (_fSafe && !_fIndex)

			_state = UNINDEXED;

		break;

	case MONITOR_BREAK:

		S_MonitorBreak();

		// If this slave got synched: notify the master
		if (_fMotorOff && _fIndex)

			//Change active state
			_state = IDLE;

		else if (_fMotorOff && !_fIndex)

			_state = UNINDEXED;

		break;

	default:
		break;
	}

	return fsmRet;
}

int32_t SysFSM::S_OutOfSynch(void) {

	if (_comCmds->Synch()) {

		_fSynched = true;

		_pingAnsw = true;

		usleep(100000);

		_pingTOut = g_SysClk.GetTime();
	}

	return 0;
}

int32_t SysFSM::S_Unindexed(void) {

	//_comCmds->AddCmd(CMDM_IS_INDEXED, 0);

	return 0;
}

int32_t SysFSM::S_RunScheme(void) {

	return 0;
}

int32_t SysFSM::S_MonitorBreak(void) {
	return 0;
}
int32_t SysFSM::S_Unsafe(void) {
	return 0;
}

bool SysFSM::Ping(void) {

	if (g_SysClk.GetTime() - _pingTOut >= FSM_PING_TIMEOUT) {

		if (!_pingAnsw)
			return false;

		_comCmds->AddCmd(CMDM_PING, 0);

		if (_state == UNINDEXED)
			_comCmds->AddCmd(CMDM_IS_INDEXED, 0);

		_pingTOut = g_SysClk.GetTime();

		_pingAnsw = false;

		//ROS_INFO("Driver is alive");

	}

	return true;
}

void SysFSM::Rst(void) {

	_state = OUT_OF_SYNCH;

	_fSynched = false;
	_fStart = false;
	_fUpdate = false;
	_fSelect = false;
	_fRunning = false;
	_fStop = false;
	_fSafe = true;
	_fMotorOff = true;
	_fVarsAvail = false;
	_fIndex = false;
	_fJustSynched = false;
	_safeErrorID = 0x00;

	_errCnt = 0;

	_pingTOut = 0;

	_pingAnsw = true;
}

void SysFSM::AddVar(unsigned char varID) {

	_loggedVars.resize(_loggedVars.size() + 1);

	std::vector<struct ctlVar>::iterator it = _loggedVars.end();

	struct ctlVar var2add = { varID, 0 };

	_loggedVars.insert(it, var2add);
}

void SysFSM::RmvVar(unsigned char varID) {

	std::vector<struct ctlVar>::iterator it;

	for (it = _loggedVars.begin(); it != _loggedVars.end(); ++it) {

		if ((*it).ID == varID)
			_loggedVars.erase(it);
	}
}

void SysFSM::CleanVars(void) {

	_loggedVars.clear();
}

bool SysFSM::GetVar(unsigned char varID, float * value) {

	std::vector<struct ctlVar>::iterator it;

	for (it = _loggedVars.begin(); it != _loggedVars.end(); ++it) {

		if ((*it).ID == varID) {

			*value = (*it).value;

			return true;
		}
	}

	return false;
}
bool SysFSM::SetVar(unsigned char varID, float value) {

	std::vector<struct ctlVar>::iterator it;

	for (it = _loggedVars.begin(); it != _loggedVars.end(); ++it) {

		if ((*it).ID == varID) {

			(*it).value = value;

			_fVarsAvail = true;

			return true;
		}
	}

	return false;
}
