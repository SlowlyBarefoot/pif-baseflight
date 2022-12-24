/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 */

#include "board.h"
#include "mw.h"

#include "core/pif_log.h"
#include "rc/pif_rc_spektrum.h"


#define SPEK_2048_MAX_CHANNEL       8


static PifRcSpektrum s_spektrum;
static uint32_t spekChannelData[SPEK_2048_MAX_CHANNEL];


static uint16_t spektrumReadRawRC(uint8_t chan)
{
	uint16_t data;

    if (chan >= s_spektrum.parent._channel_count) {
        data = mcfg.midrc;
    } else {
        data = spekChannelData[mcfg.rcmap[chan]];
    }

    return data;
}

static void _evtSpektrumReceive(PifRc* p_owner, uint16_t* channel, PifIssuerP p_issuer)
{
	int i;
    PifTask* p_task;

    // internal failsafe enabled and rx failsafe flag set
    if (feature(FEATURE_FAILSAFE) && pifRc_CheckFailSafe(p_owner)) return;

    failsafeCnt = 0;
    for (i = 0; i < p_owner->_channel_count; i++) {
        spekChannelData[i] = channel[i];
    }

	p_task = (PifTask*)p_issuer;
	if (!p_task->_running) p_task->immediate = TRUE;
}

BOOL spektrumInit(int uart, rcReadRawDataPtr *callback)
{
    int b;
    uint8_t Protocol_id;

    for (b = 0; b < SPEK_2048_MAX_CHANNEL; b++)
        spekChannelData[b] = mcfg.midrc;

    switch (mcfg.serialrx_type) {
    case SERIALRX_SPEKTRUM2048:
        Protocol_id = PIF_SPEKTRUM_PROTOCOL_ID_22MS_2048_DSMS;
        break;

    case SERIALRX_SPEKTRUM1024:
        Protocol_id = PIF_SPEKTRUM_PROTOCOL_ID_22MS_1024_DSM2;
        break;

    default:
        return FALSE;
    }

    // spekUart is set by spektrumBind() which is called very early at startup
    core.rcvrport = uartOpen(uart, 115200, MODE_RX);
    if (!core.rcvrport) return FALSE;

    if (!pifRcSpektrum_Init(&s_spektrum, PIF_ID_AUTO, Protocol_id)) return FALSE;
    pifRc_AttachEvtReceive(&s_spektrum.parent, _evtSpektrumReceive, g_task_compute_rc);
    pifRcSpektrum_AttachComm(&s_spektrum, &core.rcvrport->comm);

    if (callback)
        *callback = spektrumReadRawRC;

    serialStartReceiveFunc(&core.rcvrport->comm);

    return TRUE;
}
