#include "mutex.h"

OS_EVENT *attitude_mutex;
INT8U attitude_err;

BOOLEAN Mutex_Init(void)
{
    attitude_mutex = OSMutexCreate(OS_PRIO_MUTEX_CEIL_DIS, &attitude_err);
    if (attitude_err != OS_ERR_NONE)
        return OS_FALSE;
    return OS_TRUE;
}