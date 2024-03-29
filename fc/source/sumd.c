#include "board.h"
#include "mw.h"

#include "rc/pif_rc_sumd.h"


#define SUMD_MAX_CHANNEL    8


static PifRcSumd s_sumd;
static uint32_t sumdChannelData[SUMD_MAX_CHANNEL];


static uint16_t sumdReadRawRC(uint8_t chan)
{
    return sumdChannelData[mcfg.rcmap[chan]];
}

static void _evtSumdReceive(PifRc* p_owner, uint16_t* channel, PifIssuerP p_issuer)
{
    PifTask* p_task;

    // internal failsafe enabled and rx failsafe flag set
    if (feature(FEATURE_FAILSAFE) && pifRc_CheckFailSafe(p_owner)) return;

	failsafeCnt = 0;
	for (int b = 0; b < SUMD_MAX_CHANNEL; b++)
		sumdChannelData[b] = channel[b];

	p_task = (PifTask*)p_issuer;
	pifTask_SetTrigger(p_task);
}

BOOL sumdInit(int uart, rcReadRawDataPtr *callback)
{
    core.rcvrport = uartOpen(uart, 115200, MODE_RX, 2);		// 2ms
    if (!core.rcvrport) return FALSE;

    if (!pifRcSumd_Init(&s_sumd, PIF_ID_AUTO)) return FALSE;
    pifRc_AttachEvtReceive(&s_sumd.parent, _evtSumdReceive, g_task_compute_rc);
    pifRcSumd_AttachComm(&s_sumd, &core.rcvrport->comm);

    if (callback)
        *callback = sumdReadRawRC;

    serialStartReceiveFunc(&core.rcvrport->comm);

    return TRUE;
}
