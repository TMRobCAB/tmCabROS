#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <math.h>

#include <libusb/pmd.h>
#include <libusb/usb-7204.h>

#include <string>
#include <vector>
#include <iostream>

#define DAQ_GAIN BP_10_00V
#define DAQ_BITS 0x0C
#define DAQ_ALC 0x01000

#define DAQ_FX_CHANNEL 0x00
#define DAQ_FY_CHANNEL 0x01
#define DAQ_FZ_CHANNEL 0x02

#define SENSOR_FX_SLOPE (1.1199 * 48.46) //50.0f
#define SENSOR_FX_OFFSET 0.0f

#define SENSOR_FY_SLOPE (1.323 * 54.24) //50.0f
#define SENSOR_FY_OFFSET 0.0f

#define SENSOR_FZ_SLOPE (0.6697 * 61.43) //50.0f
#define SENSOR_FZ_OFFSET 0.0f

class ForceSample {
public:

	ForceSample(void) :	_fX(0), _fY(0), _fZ(0) {};

	~ForceSample(void){};

	float getFX() const {
		return _fX;
	}

	void setFX(float fX) {
		_fX = fX;
	}

	float getFY() const {
		return _fY;
	}

	void setFY(float fY) {
		_fY = fY;
	}

	float getFZ() const {
		return _fZ;
	}

	void setFZ(float fZ) {
		_fZ = fZ;
	}

	void setF(float fX, float fY, float fZ) {
		_fX = fX;
		_fY = fY;
		_fZ = fZ;
	}

private:

	float _fX;
	float _fY;
	float _fZ;
};

class TmForceSensor {
public:

	TmForceSensor(void) : _udev(0), _init(false), flag(0) {

//		if (this->Init() != 0)
//			throw "TmForceNotInit";
	};

	~TmForceSensor(void);

	int Init(void);

	int GetForce(ForceSample & fVal);

	int GetForceX(ForceSample & fVal);
	int GetForceY(ForceSample & fVal);
	int GetForceZ(ForceSample & fVal);

	int Reset(void);

private:

	bool _init;

	libusb_device_handle *_udev;

	Calibration_AIN _table_AIN[NMODE][NGAINS_USB7204][NCHAN_USB7204];

	int flag;
};
