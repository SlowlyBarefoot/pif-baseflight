#pragma once

#include "core/pif_comm.h"


typedef struct serialPortVTable {
    void (*serialWrite)(serialPort_t *instance, uint8_t ch);

    uint8_t (*serialTotalBytesWaiting)(serialPort_t *instance);

    uint8_t (*serialRead)(serialPort_t *instance);

    // Specified baud rate may not be allowed by an implementation, use serialGetBaudRate to determine actual baud rate in use.
    BOOL (*serialSetBaudRate)(serialPort_t *instance, uint32_t baudRate);

    bool (*isSerialTransmitBufferEmpty)(serialPort_t *instance);

    void (*setMode)(serialPort_t *instance, portMode_t mode);
} serialPortVTable_t;


void serialWrite(serialPort_t *instance, uint8_t ch);
uint8_t serialTotalBytesWaiting(serialPort_t *instance);
uint8_t serialRead(serialPort_t *instance);
BOOL serialSetBaudRate(serialPort_t *instance, uint32_t baudRate);
void serialSetMode(serialPort_t *instance, portMode_t mode);
bool isSerialTransmitBufferEmpty(serialPort_t *instance);
void serialPrint(serialPort_t *instance, const char *str);
