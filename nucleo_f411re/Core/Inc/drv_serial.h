#pragma once

#include "core/pif_comm.h"

typedef struct serialPort
{
	int port;
    uint32_t baudRate;
    PifComm comm;
} serialPort_t;
