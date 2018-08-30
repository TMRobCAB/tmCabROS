/*
 * sflash.h
 *
 *  Created on: Apr 22, 2016
 *      Author: l_vis
 */

#ifndef BEGINNER_TUTORIALS_SRC_SFLASH_SFLASH_H_
#define BEGINNER_TUTORIALS_SRC_SFLASH_SFLASH_H_

#include <stdbool.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>
#include "uart_handler.h"
#include "packet_handler.h"

#include <unistd.h>

#include <string.h> /* memset */

int32_t UpdateJointFlash(int32_t argc, char **argv);

#endif /* BEGINNER_TUTORIALS_SRC_SFLASH_SFLASH_H_ */
