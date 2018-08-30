#ifndef COMPORT_H
#define COMPORT_H

#include "incfiles.h"

#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <sys/types.h>



#define TOUT_SEC 3

#define COM_ERR_PEEK -1

#define COM_PEEK_NOT_FOUND -2

class ComPort {


private:

	int _fd;
	fd_set set;
	struct timeval timeout;
	uint8_t _cmdLength;

	char _sPortName[64];

	bool Open(void);

public:
	ComPort(): _cmdLength(CMD_LENGTH_MAX), _fd(-1){};
	~ComPort(){close(_fd);};

	bool Open(const char *sPortName);
	bool ReOpen(void);
	int ReadBuf(char * bData, uint dLen);
	int WriteBuf(char * bData, uint dLen);
	int BytesAvail(void);
	bool IsOpen(void);
	int Peek(char query);
	int Synch(void);
	int GetCmdLine(char * cmdLine);

	int FlushRx(void);
	int FlushTx(void);
};

#endif // COMPORT_H
