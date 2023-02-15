/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"

#include "rc/pif_rc_sbus.h"


#define SBUS_MAX_CHANNEL    8


static PifRcSbus s_sbus;
static uint32_t sbusChannelData[SBUS_MAX_CHANNEL];


static uint16_t sbusReadRawRC(uint8_t chan)
{
    return sbusChannelData[mcfg.rcmap[chan]];
}

static void _evtSbusReceive(PifRc* p_owner, uint16_t* channel, PifIssuerP p_issuer)
{
    PifTask* p_task;

    // internal failsafe enabled and rx failsafe flag set
    if (feature(FEATURE_FAILSAFE) && pifRc_CheckFailSafe(p_owner)) return;

    failsafeCnt = 0; // clear FailSafe counter
	for (int i = 0; i < SBUS_MAX_CHANNEL; i++) {
		sbusChannelData[i] = channel[i];
	}

	p_task = (PifTask*)p_issuer;
	pifTask_SetTrigger(p_task);
}

BOOL sbusInit(int uart, rcReadRawDataPtr *callback)
{
    int b;

    for (b = 0; b < SBUS_MAX_CHANNEL; b++)
        sbusChannelData[b] = mcfg.midrc;

    core.rcvrport = uartOpen(uart, 100000, (portMode_t)(MODE_RX | MODE_SBUS), 2);	// 2ms
    if (!core.rcvrport) return FALSE;

    if (!pifRcSbus_Init(&s_sbus, PIF_ID_AUTO)) return FALSE;
    pifRc_AttachEvtReceive(&s_sbus.parent, _evtSbusReceive, g_task_compute_rc);
    pifRcSbus_AttachComm(&s_sbus, &core.rcvrport->comm);

    if (callback)
        *callback = sbusReadRawRC;

    serialStartReceiveFunc(&core.rcvrport->comm);

    return TRUE;
}
