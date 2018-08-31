#ifndef SYSFSM_H_

#define SYSFSM_H_

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "ComPort.h"
#include <vector>
#include "SysClk.h"
#include "ComCmds.h"

#define FSM_SYNCH_PERIOD 1.0f
#define FSM_PING_TIMEOUT 1.0f

#define FSM_ERR_NOT_INIT 		-1
#define FSM_ERR_FSM_POINT_NULL	-2
#define	FSM_ERR_SEND_SYNCH		-3
#define	FSM_ERR_SEND_SYNCH_ACK	-4
#define FSM_CMD_RST				1

enum StateVector {
	OUT_OF_SYNCH, UNINDEXED, IDLE, RUNNING, MONITOR_BREAK, UNSAFE
};

class ComCmds;

class SysFSM {

public:

	SysFSM(ComCmds * comCmds, uint32_t selSchemeIdx);
	~SysFSM() {
	}
	;

	int32_t RunLoop(void);

	void AddVar(unsigned char varID);
	void RmvVar(unsigned char varID);

	void CleanVars(void);

	bool GetVar(unsigned char varID, float * value);bool SetVar(
			unsigned char varID, float value);

	uint32_t getErrCnt() const {
		return _errCnt;
	}

	void setErrCnt(uint32_t errCnt) {
		_errCnt = errCnt;
	}

	bool isMotorOff() const {
		return _fMotorOff;
	}

	void setMotorOff(bool motorOff) {
		_fMotorOff = motorOff;
	}

	bool isRunning() const {
		return _fRunning;
	}

	void setRunning(bool running) {
		_fRunning = running;
	}

	bool isSafe() const {
		return _fSafe;
	}

	void setSafe(bool safe) {
		_fSafe = safe;
	}

	bool isSelect() const {
		return _fSelect;
	}

	void setSelect(bool select) {
		_fSelect = select;
	}

	bool isStart() const {
		return _fStart;
	}

	void setStart(bool start) {
		_fStart = start;
	}

	bool isStop() const {
		return _fStop;
	}

	void setStop(bool stop) {
		_fStop = stop;
	}

	bool isSynched() const {
		return _fSynched;
	}

	void setSynched(bool synched) {
		_fSynched = synched;
	}

	bool isUpdate() const {
		return _fUpdate;
	}

	void setUpdate(bool update) {
		_fUpdate = update;
	}

	uint32_t getSelSchemeIdx() const {
		return _selSchemeIdx;
	}

	void setSelSchemeIdx(uint32_t selSchemeIdx) {
		_selSchemeIdx = selSchemeIdx;
	}

	bool isPingAnsw() const {
		return _pingAnsw;
	}

	void setPingAnsw(bool pingAnsw) {
		_pingAnsw = pingAnsw;
	}

	StateVector getState() const {
		return _state;
	}

	unsigned char getSafeErrorId() const {
		return _safeErrorID;
	}

	void setSafeErrorId(unsigned char safeErrorId) {
		_safeErrorID = safeErrorId;
	}

	bool isVarsAvail() const {
		return _fVarsAvail;
	}

	void VarsPublished() {
		_fVarsAvail = false;
	}

	bool isIndex() const {
		return _fIndex;
	}

	void setIndex(bool index = false) {
		_fIndex = index;
	}

	void Rst(void);

	bool isJustSynched() const {
		return _fJustSynched;
	}

	void setJustSynched(bool justSynched = false) {
		_fJustSynched = justSynched;
	}

private:

	StateVector _state;

	/////////////////////////////////
	// FSM Flags
	/////////////////////////////////

	bool _fSynched = false;bool _fStart = false;bool _fUpdate = false;bool _fSelect =
	false;bool _fRunning = false;bool _fStop = false;bool _fSafe = true;bool _fMotorOff =
	true;bool _fVarsAvail = false;bool _fIndex = false;bool _fJustSynched =	false;

	uint32_t _errCnt;

	uint32_t _selSchemeIdx;

	ComCmds * _comCmds;

	unsigned char _safeErrorID;

	double _pingTOut;

	bool _pingAnsw;

	std::vector<struct ctlVar> _loggedVars;

	int32_t S_OutOfSynch(void);
	int32_t S_Unindexed(void);
	int32_t S_Update(void);

	int32_t S_SelectScheme(void);

	int32_t S_StartScheme(void);
	int32_t S_RunScheme(void);
	int32_t S_StopScheme(void);

	int32_t S_MonitorBreak(void);
	int32_t S_Unsafe(void);

	int32_t S_UpdateSys(void);

	bool Ping(void);

};

extern SysClk g_SysClk;

#endif
