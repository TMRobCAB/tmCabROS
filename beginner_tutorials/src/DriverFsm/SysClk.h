/*
 * SysClk.h
 *
 *  Created on: Feb 6, 2016
 *      Author: l_vis
 */

#include "incfiles.h"

#ifndef SYSCLK_H_
#define SYSCLK_H_

class SysClk {
public:

	SysClk();
	~SysClk(){};


	double GetTime(void);

	void Rst(void);

private:

	struct timespec _t0;
	struct timespec _tCurr;

	double diffT(struct timespec *tv1, struct timespec *tv2);
};



#endif /* SYSCLK_H_ */
