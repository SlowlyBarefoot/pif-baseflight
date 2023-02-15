#include "board.h"
#include "mw.h"

#include "core/pif_log.h"
#include "rc/pif_rc_ibus.h"


#define IBUS_MAX_CHANNEL    8


static PifRcIbus s_ibus;
static uint16_t ibusChannelData[IBUS_MAX_CHANNEL];


static uint16_t ibusReadRawRC(uint8_t chan)
{
    return ibusChannelData[mcfg.rcmap[chan]];
}

static void _evtIbusReceive(PifRc* p_owner, uint16_t* channel, PifIssuerP p_issuer)
{
    PifTask* p_task;

    // internal failsafe enabled and rx failsafe flag set
    if (feature(FEATURE_FAILSAFE) && pifRc_CheckFailSafe(p_owner)) return;

    failsafeCnt = 0; // clear FailSafe counter
	for (int i = 0; i < IBUS_MAX_CHANNEL; i++) {
		ibusChannelData[i] = channel[i];
	}

	p_task = (PifTask*)p_issuer;
	pifTask_SetTrigger(p_task);
}

BOOL ibusInit(int uart, rcReadRawDataPtr *callback)
{
    int b;

    for (b = 0; b < IBUS_MAX_CHANNEL; b++)
        ibusChannelData[b] = mcfg.midrc;

    core.rcvrport = uartOpen(uart, 115200, MODE_RX, 2);	// 2ms
    if (!core.rcvrport) return FALSE;

    if (!pifRcIbus_Init(&s_ibus, PIF_ID_AUTO)) return FALSE;
    pifRc_AttachEvtReceive(&s_ibus.parent, _evtIbusReceive, g_task_compute_rc);
    pifRcIbus_AttachComm(&s_ibus, &core.rcvrport->comm);

    if (callback)
        *callback = ibusReadRawRC;

    serialStartReceiveFunc(&core.rcvrport->comm);

    return TRUE;
}
