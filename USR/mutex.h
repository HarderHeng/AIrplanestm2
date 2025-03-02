#ifndef _MUTEX_H
#define _MUTEX_H

#include "ucos_ii.h"

BOOLEAN Mutex_Init(void);

extern OS_EVENT *attitude_mutex;
extern INT8U attitude_err;

#endif