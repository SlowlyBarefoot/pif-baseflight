/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 *
 * DMA UART routines idea lifted from AutoQuad
 * Copyright 2011  Bill Nesbitt
*/
#include "main.h"
#include "board.h"
#include "link_driver.h"

#include "drv_uart.h"

#include "core/pif_comm.h"
#include "core/pif_log.h"


#define UART_TX_FRAME_SIZE			8


// FIXME this is a uart_t really.  Move the generic properties into a separate structure (serialPort_t) and update the code to use it
typedef struct {
    serialPort_t port;

	UART_HandleTypeDef* p_huart;
	uint16_t tx_frame_size;
	uint8_t rx_frame;
	uint8_t rx_threshold;
	BOOL init;
} uartPort_t;


#ifdef __PIF_DEBUG__
	static PifComm s_comm_log;
	static uint16_t s_usLogTx;
#endif
static uartPort_t uartPort[3];


static BOOL actUartSetBaudRate(PifComm* p_comm, uint32_t baudrate)
{
	uartPort_t* p_uart = &uartPort[PIF_ID_UART_2_IDX(p_comm->_id)];

	if (p_uart->p_huart->Init.BaudRate == baudrate) return TRUE;

	HAL_UART_DeInit(p_uart->p_huart);
	p_uart->p_huart->Init.BaudRate = baudrate;
	if (HAL_UART_Init(p_uart->p_huart) != HAL_OK) {
		return FALSE;
	}
	return TRUE;
}

static BOOL actUartStartTransfer(PifComm* p_comm)
{
	uint8_t* p_data;
	uint8_t state;
	uartPort_t* p_uart = &uartPort[PIF_ID_UART_2_IDX(p_comm->_id)];

	p_uart->tx_frame_size = UART_TX_FRAME_SIZE;
	state = pifComm_StartGetTxData(p_comm, &p_data, &p_uart->tx_frame_size);
	if (state & PIF_COMM_SEND_DATA_STATE_DATA) {
		HAL_UART_Transmit_IT(p_uart->p_huart, p_data, p_uart->tx_frame_size);
		return TRUE;
	}
	return FALSE;
}

static BOOL serialUSART(uartPort_t* s, uint32_t baudRate, PifId pif_id)
{
	s->init = TRUE;
	if (!pifComm_Init(&s->port.comm, pif_id)) return FALSE;
	if (!pifComm_AttachTask(&s->port.comm, TM_PERIOD_MS, 1, TRUE)) return FALSE;	// 1ms
	if (!pifComm_AllocRxBuffer(&s->port.comm, 64, s->rx_threshold)) return FALSE;
	if (!pifComm_AllocTxBuffer(&s->port.comm, 64)) return FALSE;
	s->port.comm.act_set_baudrate = actUartSetBaudRate;
	s->port.comm.act_start_transfer = actUartStartTransfer;
	return TRUE;
}

#ifdef __PIF_DEBUG__

static BOOL actLogStartTransfer(PifComm* p_comm)
{
	uint8_t *p_data, state;

	s_usLogTx = 0;
	state = pifComm_StartGetTxData(p_comm, &p_data, &s_usLogTx);
	if (state & PIF_COMM_SEND_DATA_STATE_DATA) {
		HAL_UART_Transmit_IT(&huart2, p_data, s_usLogTx);
		return TRUE;
	}
	return FALSE;
}

BOOL logOpen()
{
	pifLog_Init();

	if (!pifComm_Init(&s_comm_log, PIF_ID_AUTO)) return FALSE;
	if (!pifComm_AttachTask(&s_comm_log, TM_PERIOD_MS, 1, TRUE)) return FALSE;			// 1ms
	if (!pifComm_AllocTxBuffer(&s_comm_log, 256)) return FALSE;
	s_comm_log.act_start_transfer = actLogStartTransfer;

	if (!pifLog_AttachComm(&s_comm_log)) return FALSE;
	return TRUE;
}

#endif

serialPort_t *uartOpen(int port, uint32_t baudRate, portMode_t mode)
{
    uartPort_t *s = NULL;

    if (port == UART_PORT_1) {
        s = &uartPort[0];
        s->p_huart = &huart1;
        s->rx_threshold = 25;
    }
#ifndef __PIF_DEBUG__
    else if (port == UART_PORT_2) {
        s = &uartPort[1];
        s->p_huart = &huart2;
        s->rx_threshold = 10;
    }
#endif
    else if (port == UART_PORT_3) {
        s = &uartPort[2];
        s->p_huart = &huart6;
        s->rx_threshold = 25;
    }
    else return NULL;

    if (!serialUSART(s, baudRate, PIF_ID_UART(port - 1))) return FALSE;

    // callback for IRQ-based RX ONLY
    if (!serialSetBaudRate(&s->port, baudRate)) {
    	pifComm_Clear(&s->port.comm);
    	return FALSE;
    }

	s->init = FALSE;
    return (serialPort_t *)s;
}

// Handlers

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t* p_data;
	uint8_t state;
	uartPort_t* p_uart;

	if (huart->Instance == USART1) {
		p_uart = &uartPort[0];
	}
	else if (huart->Instance == USART2) {
#ifdef __PIF_DEBUG__
		state = pifComm_EndGetTxData(&s_comm_log, s_usLogTx);
		if (state & PIF_COMM_SEND_DATA_STATE_EMPTY) {
			pifComm_FinishTransfer(&s_comm_log);
		}
		else {
			s_usLogTx = 0;
			state = pifComm_StartGetTxData(&s_comm_log, &p_data, &s_usLogTx);
			if (state & PIF_COMM_SEND_DATA_STATE_DATA) {
				HAL_UART_Transmit_IT(huart, p_data, s_usLogTx);
			}
		}
		return;
#else
		p_uart = &uartPort[1];
#endif
	}
	else if (huart->Instance == USART6) {
		p_uart = &uartPort[2];
	}
	else return;

	state = pifComm_EndGetTxData(&p_uart->port.comm, p_uart->tx_frame_size);
	if (state & PIF_COMM_SEND_DATA_STATE_EMPTY) {
		pifComm_FinishTransfer(&p_uart->port.comm);
	}
	else {
		p_uart->tx_frame_size = UART_TX_FRAME_SIZE;
		state = pifComm_StartGetTxData(&p_uart->port.comm, &p_data, &p_uart->tx_frame_size);
		if (state & 1) {
			HAL_UART_Transmit_IT(huart, p_data, p_uart->tx_frame_size);
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uartPort_t* p_uart;

	if (huart->Instance == USART1) {
		p_uart = &uartPort[0];
	}
	else if (huart->Instance == USART2) {
		p_uart = &uartPort[1];
	}
	else if (huart->Instance == USART6) {
		p_uart = &uartPort[2];
	}
	else return;

	if (p_uart->init) return;

	pifComm_PutRxByte(&p_uart->port.comm, p_uart->rx_frame);
	HAL_UART_Receive_IT(huart, &p_uart->rx_frame, 1);
}

BOOL serialSetBaudRate(serialPort_t* instance, uint32_t baudRate)
{
   	return (*instance->comm.act_set_baudrate)(&instance->comm, baudRate);
}

BOOL serialStartReceiveFunc(PifComm* p_comm)
{
	uartPort_t* p_uart = &uartPort[PIF_ID_UART_2_IDX(p_comm->_id)];

	if (HAL_UART_Receive_IT(p_uart->p_huart, &p_uart->rx_frame, 1) != HAL_OK) return FALSE;
	return TRUE;
}

BOOL serialStopReceiveFunc(PifComm* p_comm)
{
	uartPort_t* p_uart = &uartPort[PIF_ID_UART_2_IDX(p_comm->_id)];

	HAL_UART_Abort_IT(p_uart->p_huart);
	return TRUE;
}
