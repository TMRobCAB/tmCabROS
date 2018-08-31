#include "SysClk.h"

SysClk::SysClk(void) {

	Rst();
}

double SysClk::GetTime() {

	clock_gettime(CLOCK_MONOTONIC, &_tCurr);

	return diffT(&_tCurr, &_t0);
}

void SysClk::Rst(void) {

	clock_gettime(CLOCK_MONOTONIC, &_t0);
}

double SysClk::diffT(struct timespec *tv1, struct timespec *tv2) {

	return (tv1->tv_sec - tv2->tv_sec) + (tv1->tv_nsec - tv2->tv_nsec) / 1.0e9;

}

