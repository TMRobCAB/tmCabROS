#include "ComPort.h"
#include "ros/ros.h"

int ComPort::WriteBuf(char * bData, uint dLen) {
	return write(_fd, bData, dLen);
}

int ComPort::ReadBuf(char *bData, uint dLen) {
	return read(_fd, bData, dLen);
}

int ComPort::BytesAvail(void) {

	int bytes_avail;

	ioctl(_fd, FIONREAD, &bytes_avail);

	return bytes_avail;
}

bool ComPort::Open(const char *sPortName){

	strcpy(_sPortName, sPortName);

	return Open();
}

bool ComPort::Open(void) {

	struct termios newtio;

	_fd = open(_sPortName, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if ((int) _fd < 0) {
		return false;
	}

	//ROS_INFO("Port open %d", fd);

	newtio.c_cflag = B115200| CS8 | CLOCAL | CREAD;
	newtio.c_iflag = IGNPAR;
	newtio.c_oflag = 0;
	newtio.c_lflag = 0;       //ICANON;
	newtio.c_cc[VMIN] = 0;
	newtio.c_cc[VTIME] = 0;
	tcflush(_fd, TCIFLUSH);
	tcsetattr(_fd, TCSANOW, &newtio);


	return true;
}

int ComPort::FlushRx(void) {

	return tcflush(_fd, TCIFLUSH);
}

int ComPort::FlushTx(void) {

	return tcflush(_fd, TCOFLUSH);
}

bool ComPort::IsOpen(void) {

	return _fd >= 0;
}

int ComPort::Peek(char query) {

	//TODO

	char cmdLine[256];

	int bAvail = BytesAvail();

	int bRead = read(_fd, cmdLine, bAvail);

	lseek(_fd, -bAvail, SEEK_CUR);

	if (bRead != bAvail)
		return COM_ERR_PEEK;

	for (int i = 0; i < bAvail; ++i) {

		if (query == cmdLine[i])
			return i;
	}

	return COM_PEEK_NOT_FOUND;
}

int ComPort::GetCmdLine(char * cmdLine) {

	if (_cmdLength == CMD_LENGTH_MAX) {

			ReadBuf((char*) &_cmdLength, sizeof(uint8_t));
	}

	if (BytesAvail() >= _cmdLength - 1) {

		cmdLine[0] = (unsigned char) _cmdLength;

		ReadBuf((char*) cmdLine + 1, _cmdLength - 1);

		_cmdLength = CMD_LENGTH_MAX;

		return cmdLine[0];
	}

	return 0;
}

//TODO implement timeout when the ack_msg is sent but there's no response

int ComPort::Synch(void) {

	int  i, peekIdx = -1;

	char query = (char) (SYNCH_MSG >> 24);

	char cmdLine[256];

	uint32_t synchMsg;

	_cmdLength = CMD_LENGTH_MAX;

	int bAvail = BytesAvail();

	if (bAvail < sizeof(int32_t) * 2)
		return CMD_NOT_SYNCHED;

	if(bAvail > 256)
		bAvail = 256;

	int bRead = read(_fd, cmdLine, bAvail);

	//ROS_INFO("bytes read %d", bRead);

	if (bRead != bAvail)
		return CMD_NOT_SYNCHED;

	for (i = 0; i < bAvail; ++i) {

		if (query == cmdLine[i]){

			peekIdx = i;
		}
	}

	if (peekIdx == -1)
		return CMD_NOT_SYNCHED;

	if (peekIdx < 3)
		return CMD_NOT_SYNCHED;

	memcpy(&synchMsg, cmdLine + (peekIdx - 3), sizeof(uint32_t));

	if (synchMsg == (uint32_t) SYNCH_MSG)
		return CMD_SYNCHED;

	return CMD_NOT_SYNCHED;
}

bool ComPort::ReOpen(void) {

	if(close(_fd) != 0)
		return false;

	if(!Open())
		return false;

	return true;
}
