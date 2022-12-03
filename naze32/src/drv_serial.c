#include "board.h"
#include "mw.h"

#include "drv_serial.h"


void serialPrint(serialPort_t *instance, const char *str)
{
    uint8_t ch;
    while ((ch = *(str++)) != 0) {
        serialWrite(instance, ch);
    }
}

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    serialPortVTable_t* vTable;

    if (instance->p_param) {
        vTable = (serialPortVTable_t*)instance->p_param;
        vTable->serialWrite(instance, ch);
    }
    else {
        pifComm_SendTxData(&instance->comm, &ch, 1);
    }
}

uint8_t serialTotalBytesWaiting(serialPort_t *instance)
{
    serialPortVTable_t* vTable;

    if (instance->p_param) {
        vTable = (serialPortVTable_t*)instance->p_param;
        return vTable->serialTotalBytesWaiting(instance);
    }
    else {
        return pifComm_GetRemainSizeOfRxBuffer(&instance->comm);
    }
}

uint8_t serialRead(serialPort_t *instance)
{
    serialPortVTable_t* vTable;
    uint8_t ch;

    if (instance->p_param) {
        vTable = (serialPortVTable_t*)instance->p_param;
        return vTable->serialRead(instance);
    }
    else {
        return pifComm_ReceiveRxData(&instance->comm, &ch, 1);
    }
}

BOOL serialSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
    serialPortVTable_t* vTable;

    if (instance->p_param) {
        vTable = (serialPortVTable_t*)instance->p_param;
        vTable->serialSetBaudRate(instance, baudRate);
    }
    else {
    	(*instance->comm.act_set_baudrate)(&instance->comm, baudRate);
    }
    return TRUE;
}

bool isSerialTransmitBufferEmpty(serialPort_t *instance)
{
    serialPortVTable_t* vTable;

    if (instance->p_param) {
        vTable = (serialPortVTable_t*)instance->p_param;
        return vTable->isSerialTransmitBufferEmpty(instance);
    }
    else {
        return pifComm_GetFillSizeOfTxBuffer(&instance->comm) == 0;
    }
}

inline void serialSetMode(serialPort_t *instance, portMode_t mode)
{
    serialPortVTable_t* vTable;

    if (instance->p_param) {
        vTable = (serialPortVTable_t*)instance->p_param;
        vTable->setMode(instance, mode);
    }
}

