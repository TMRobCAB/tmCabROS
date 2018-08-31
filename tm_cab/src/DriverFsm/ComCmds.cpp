#include "ComCmds.h"

#include "ros/ros.h"

///////////////////////////////////////
//
// Public Functions
//
///////////////////////////////////////

ComCmds::ComCmds(ComPort * comPort) {

	_comPort = comPort;

	_sysFSM = NULL;

	_cmdNum = 0;
}

int32_t ComCmds::CmdLineRead(SysFSM * sysFSM) {

	_sysFSM = sysFSM;

	int32_t cmdStatus = 0;
	int cmdLength;

	if (!_comPort->IsOpen())
		return COM_CMD_ERR_COM_MOD_DIS;

	if (_comPort->BytesAvail() <= 0)
		return COM_CMD_SYNCHED;

	cmdLength = _comPort->GetCmdLine((char *) _cInput);

	if (cmdLength > 0) {

		cmdStatus = CmdLineProcess(_cInput, cmdLength);

		//TODO fix error return
		if (cmdStatus == COM_CMD_ERR_BAD_CMD)
			return 0; //COM_CMD_ERR_CRC;

		if (cmdStatus == COM_CMD_ERR_CRC)
					return COM_CMD_ERR_CRC;

		return cmdStatus;
	}

	if (cmdLength == CMD_NOT_SYNCHED)
		return COM_CMD_NOT_SYNCHED;

	return 0;
}

int32_t ComCmds::CmdLineWrite(void) {

	int32_t cmdStatus = 0;

	if (!_comPort->IsOpen())
		return COM_CMD_ERR_COM_MOD_DIS;

	if (_cmdNum <= 0)
		return 0;

	//ROS_ERROR(" cmdNum %d", _cmdNum);

	cmdStatus = SendCmd(_cmdIDs, _cmdArgs, _cmdNum);

	_cmdNum = 0;

	return cmdStatus;
}

void ComCmds::AddCmd(unsigned char cmdIDs, float cmdArgs) {

	memcpy(_cmdIDs + _cmdNum, &cmdIDs, 1);

	memcpy(_cmdArgs + _cmdNum, &cmdArgs, 1 * sizeof(float));

	_cmdNum++;
}

void ComCmds::AddCmds(unsigned char * cmdIDs, float * cmdArgs, uint8_t CmdNum) {

	memcpy(_cmdIDs + _cmdNum, cmdIDs, CmdNum);

	memcpy(_cmdArgs + _cmdNum, cmdArgs, CmdNum * sizeof(float));

	_cmdNum += CmdNum;
}

///////////////////////////////////////
//
// Private Functions
//
///////////////////////////////////////

int ComCmds::CmdLineProcess(unsigned char *pcCmdLine, uint32_t cmdLength) {
	unsigned char *pcChar;
	uint16_t cmdCRC, computedCRC;
	uint32_t cmdIndex = 0;
	bool badCmd = false;

	pcChar = pcCmdLine;

	cmdCRC = *((uint16_t*) (pcChar + cmdLength - 2));;

	computedCRC = CRCCCITT(pcChar, cmdLength - 2, 0xffff, 0);

//	pcChar[cmdLength - 2] = (unsigned char) ((computedCRC >> 8) & 0xff);
//
//	pcChar[cmdLength - 1] = (unsigned char) (computedCRC & 0xff);
//
//	computedCRC = CRCCCITT(pcChar, cmdLength, 0xffff, 0);

	if (cmdCRC != computedCRC)
		return COM_CMD_ERR_CRC;

	cmdLength -= 2;

	cmdIndex = 1;

	float argu;

	//
	// Advance through the command line until its end.
	//
	while (cmdIndex < cmdLength) {

		//ROS_INFO("CmdLength %x", (uint8_t) pcChar[cmdIndex]);

		///////////////////////////////////////
		//
		// Command Execution
		//
		///////////////////////////////////////

		argu = *((float*) (pcChar + cmdIndex + 1));

		if ((int8_t) pcChar[cmdIndex] < 0) {

			_sysFSM->SetVar(pcChar[cmdIndex], argu);

		}

		switch ((int8_t) pcChar[cmdIndex]) {

		case CMDS_PAR_SET:

		case CMDS_UPDATING:
		case CMDS_SET_REF:
		case CMDS_SCHEME_SELECTED:
		case CMDS_ERR_SEL_SCH:

		case CMDS_ERR_REF_NOT_FOUND:
		case CMDS_ERR_SAFE_NOT_FOUND:
		case CMDS_ERR_BAD_CMD:
		case CMDS_ERR_CTL_MOD_NOT_FOUND:
		case CMDS_SYS_RUNNING:
		case CMDS_SAFE_ACK:

			return pcChar[cmdIndex];
			break;

		case CMDS_CTL_STARTED:

			_sysFSM->setStart(true);
			_sysFSM->setMotorOff(false);

			return pcChar[cmdIndex];
			break;

		case CMDS_MOTOR_OFF:

			_sysFSM->setMotorOff(true);

			//return pcChar[cmdIndex];
			break;

		case CMDS_CTL_STOPPED:

			_sysFSM->setStop(true);

			return pcChar[cmdIndex];
			break;

		case CMDS_ERR_SAFE_POS:
		case CMDS_ERR_SAFE_SPEED:
		case CMDS_ERR_SAFE_PWM:
		case CMDS_ERR_SAFE_CURR:
		case CMDS_ERR_SAFE_SPEED_E:
		case CMDS_ERR_SAFE_POS_E:
		case CMDS_ERR_SAFE_CURR_E:
		case CMDS_ERR_SAFE_FORCE:
		case CMDS_ERR_SAFE_FORCE_E:
		case CMDS_ERR_LIMIT_SWITCH:

			_sysFSM->setSafe(false);

			_sysFSM->setSafeErrorId(pcChar[cmdIndex]);

			break;

		case CMDS_IDX:


			_sysFSM->setIndex((bool) argu);

			//ROS_INFO("Index command %f", argu);

			break;

		case CMDS_PING:

			//ROS_INFO("Ping");

			F_Ping((int8_t) pcChar[cmdIndex], argu);
			break;

		default:

			badCmd = true;

			break;
		}

		cmdIndex += 5;
	}

	if (badCmd)
		return (COM_CMD_ERR_BAD_CMD);
	else
		return COM_CMD_SUCCESS;
}

//////////////////////////////////////////////
// SLAVE (Tiva) Cmd Functions
//////////////////////////////////////////////

int ComCmds::SendCmd(const unsigned char * cmdIDs, float * cmdArgs,
		uint32_t cmdsNum) {

	unsigned char * outBuf;
	uint16_t cmdCRC;
	int i;

	uint8_t cmdLength = sizeof(uint8_t)
			+ cmdsNum * (sizeof(unsigned char) + sizeof(float))
			+ sizeof(uint16_t);

	if (cmdLength >= CMD_LENGTH_MAX)
		return COM_CMD_ERR_SEND_TOO_LONG;

	outBuf = (unsigned char *) malloc(cmdLength);

	outBuf[0] = (unsigned char) cmdLength;

	for (i = 0; i < (int) cmdsNum; ++i) {

		outBuf[1 + i * 5] = cmdIDs[i];

		memcpy(outBuf + (2 + i * 5), cmdArgs + i, sizeof(float));
	}

	cmdCRC = CRCCCITT(outBuf, cmdLength - 2, 0xffff, 0);

	memcpy(outBuf + cmdLength - 2, &cmdCRC, sizeof(uint16_t));

//	outBuf[cmdLength - 2] = (unsigned char) (cmdCRC & 0xff);
//
//	outBuf[cmdLength - 1] = (unsigned char) ((cmdCRC >> 8) & 0xff);

//	cmdCRC = CRCCCITT(outBuf, cmdLength, 0xffff, 0);
//
//	outBuf[cmdLength - 2] = (unsigned char) ((cmdCRC >> 8) & 0xff);
//
//	outBuf[cmdLength - 1] = (unsigned char) (cmdCRC & 0xff);

	//TODO fix! wait for write to complete
	if (_comPort->WriteBuf((char *) outBuf, cmdLength) != cmdLength)
		return 0;		// COM_CMD_ERR_SEND;

	free(outBuf);

	cmdsNum = 0;

	return 0;
}

int ComCmds::SendData(char * bData, uint dLen) {

	return _comPort->WriteBuf(bData, dLen);
}

bool ComCmds::Synch(void) {

	uint32_t msgAck = SYNCH_MSG_ACK;

	if (_comPort->Synch() == CMD_SYNCHED) {

		SendData((char *) &msgAck, sizeof(uint32_t));

		//usleep(FSM_USLEEP_AFTER_SYNCH);

		ROS_INFO("Driver Synched");

		return true;
	}

	return false;
}

uint16_t CRCCCITT(unsigned char *data, uint32_t length, uint16_t seed,
		uint16_t final) {

	uint32_t count;
	uint32_t crc = seed;
	uint32_t temp;

	for (count = 0; count < length; ++count) {
		temp = (*data++ ^ (crc >> 8)) & 0xff;
		crc = crc_table[temp] ^ (crc << 8);
	}

	return (unsigned short) (crc ^ final);

}

////////////////////////////////////////////////////////////////////////////////
//
// CMD Methodes
//
////////////////////////////////////////////////////////////////////////////////

int ComCmds::F_Ping(int8_t cmd, float arg) {

	_sysFSM->setPingAnsw(true);

	return (0);
}
