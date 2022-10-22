#include "board.h"
#include "mw.h"


static uint16_t mspReadRawRC(uint8_t chan)
{
    return rcData[chan];
}

void mspFrameRecieve(void)
{
    failsafeCnt = 0; // clear FailSafe counter

    g_task_compute_rc->immediate = TRUE;
}

void mspInit(rcReadRawDataPtr *callback)
{
    if (callback)
        *callback = mspReadRawRC;
}
