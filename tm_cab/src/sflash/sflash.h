/*
 * sflash.h
 *
 *  Created on: Apr 22, 2016
 *      Author: l_vis
 */

#ifndef TM_CAB_SRC_SFLASH_SFLASH_H_
#define TM_CAB_SRC_SFLASH_SFLASH_H_

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

#endif /* TM_CAB_SRC_SFLASH_SFLASH_H_ */
