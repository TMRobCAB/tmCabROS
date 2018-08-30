#ifndef INCFILES_H
#define INCFILES_H

#ifndef _REENTRANT
#define _REENTRANT
#endif

#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include <iostream>

#include "CommDefines.h"

#include <string.h>

#include <asm/types.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

#endif // INCFILES_H
