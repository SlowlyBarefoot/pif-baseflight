/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 *
 * DMA UART routines idea lifted from AutoQuad
 * Copyright 2011  Bill Nesbitt
*/
#include "board.h"
#include "mw.h"


// FIXME this is a uart_t really.  Move the generic properties into a separate structure (serialPort_t) and update the code to use it
typedef struct {
    serialPort_t port;
	int num;
} uartPort_t;


static uartPort_t uartPort[3];


static BOOL actSerial1SetBaudRate(PifComm* p_comm, uint32_t baudrate)
{
	Serial1.end();
	Serial1.begin(baudrate);

	pifTask_ChangePeriod(p_comm->_p_task, 115200 / baudrate);
	return TRUE;
}

static uint16_t actSerial1SendData(PifComm* p_comm, uint8_t* p_buffer, uint16_t size)
{
	(void)p_comm;

    return Serial1.write((char *)p_buffer, size);
}

static BOOL actSerial1ReceiveData(PifComm* p_comm, uint8_t* p_data)
{
	int rx_data;

	(void)p_comm;

	rx_data = Serial1.read();
	if (rx_data >= 0) {
		*p_data = rx_data;
		return TRUE;
	}
	return FALSE;
}

static uartPort_t *serialUSART1(portMode_t mode)
{
    uartPort_t *s;

    s = &uartPort[0];

	if (!pifComm_Init(&s->port.comm, PIF_ID_UART(0))) return NULL;
	s->port.comm.act_set_baudrate = actSerial1SetBaudRate;
	if (mode & MODE_RX) {
		s->port.comm.act_receive_data = actSerial1ReceiveData;
	}
	if (mode & MODE_TX) {
		s->port.comm.act_send_data = actSerial1SendData;
	}

    return s;
}

static BOOL actSerial2SetBaudRate(PifComm* p_comm, uint32_t baudrate)
{
	Serial2.end();
	Serial2.begin(baudrate);

	pifTask_ChangePeriod(p_comm->_p_task, 115200 / baudrate);
	return TRUE;
}

static uint16_t actSerial2SendData(PifComm* p_comm, uint8_t* p_buffer, uint16_t size)
{
	(void)p_comm;

	return Serial2.write((char *)p_buffer, size);
}

static BOOL actSerial2ReceiveData(PifComm* p_comm, uint8_t* p_data)
{
	int rx_data;

	(void)p_comm;

	rx_data = Serial2.read();
	if (rx_data >= 0) {
		*p_data = rx_data;
		return TRUE;
	}
	return FALSE;
}

static uartPort_t *serialUSART2(portMode_t mode)
{
    uartPort_t *s;

    s = &uartPort[1];

	if (!pifComm_Init(&s->port.comm, PIF_ID_UART(1))) return NULL;
	s->port.comm.act_set_baudrate = actSerial2SetBaudRate;
	if (mode & MODE_RX) {
		s->port.comm.act_receive_data = actSerial2ReceiveData;
	}
	if (mode & MODE_TX) {
		s->port.comm.act_send_data = actSerial2SendData;
	}

    return s;
}


static BOOL actSerial3SetBaudRate(PifComm* p_comm, uint32_t baudrate)
{
	Serial3.end();
	Serial3.begin(baudrate);

	pifTask_ChangePeriod(p_comm->_p_task, 115200 / baudrate);
	return TRUE;
}

static uint16_t actSerial3SendData(PifComm* p_comm, uint8_t* p_buffer, uint16_t size)
{
	(void)p_comm;

    return Serial3.write((char *)p_buffer, size);
}

static BOOL actSerial3ReceiveData(PifComm* p_comm, uint8_t* p_data)
{
	int rx_data;

	(void)p_comm;

	rx_data = Serial3.read();
	if (rx_data >= 0) {
		*p_data = rx_data;
		return TRUE;
	}
	return FALSE;
}

static uartPort_t *serialUSART3(portMode_t mode)
{
    uartPort_t *s;

    s = &uartPort[2];

	if (!pifComm_Init(&s->port.comm, PIF_ID_UART(2))) return NULL;
	s->port.comm.act_set_baudrate = actSerial3SetBaudRate;
	if (mode & MODE_RX) {
		s->port.comm.act_receive_data = actSerial3ReceiveData;
	}
	if (mode & MODE_TX) {
		s->port.comm.act_send_data = actSerial3SendData;
	}

    return s;
}

serialPort_t *uartOpen(int num, uint32_t baudRate, portMode_t mode)
{
    uartPort_t *s = NULL;

    switch (num) {
    case 1:
    	if (mode & MODE_SBUS) {
    		Serial1.begin(baudRate, (UARTClass::UARTModes)(US_MR_CHRL_8_BIT | US_MR_NBSTOP_2_BIT | UART_MR_PAR_EVEN));
    	}
    	else {
			Serial1.begin(baudRate);
		}
        s = serialUSART1(mode);
        break;

    case 2:
    	if (mode & MODE_SBUS) {
    		Serial2.begin(baudRate, (UARTClass::UARTModes)(US_MR_CHRL_8_BIT | US_MR_NBSTOP_2_BIT | UART_MR_PAR_EVEN));
    	}
    	else {
			Serial2.begin(baudRate);
		}
        s = serialUSART2(mode);
        break;

    case 3:
    	if (mode & MODE_SBUS) {
    		Serial3.begin(baudRate, (UARTClass::UARTModes)(US_MR_CHRL_8_BIT | US_MR_NBSTOP_2_BIT | UART_MR_PAR_EVEN));
    	}
    	else {
    		Serial3.begin(baudRate);
    	}
        s = serialUSART3(mode);
        break;

    default:
    	return NULL;
    }
    if (!s) return NULL;

	if (!pifComm_AttachTask(&s->port.comm, TM_PERIOD_MS, (uint16_t)(115200UL / baudRate), TRUE)) return NULL;	// 1ms = 115200bps

	s->num = num;
    s->port.mode = mode;

    return &s->port;
}

BOOL serialSetBaudRate(serialPort_t *port, uint32_t baudRate)
{
   	return (*port->comm.act_set_baudrate)(&port->comm, baudRate);
}

