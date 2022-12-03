/*
 * This file is part of baseflight
 * Licensed under GPL V3 or modified DCL - see https://github.com/multiwii/baseflight/blob/master/README.md
 *
 * DMA UART routines idea lifted from AutoQuad
 * Copyright 2011  Bill Nesbitt
*/
#include "board.h"
#include "mw.h"

#include "drv_gpio.h"
#include "drv_uart.h"


static uartPort_t uartPort[3];


BOOL actCommSetBaudRate(PifComm* p_comm, uint32_t baudrate)
{
    uartSetBaudRate(&uartPort[PIF_ID_UART_2_IDX(p_comm->_id)], baudrate);
    return TRUE;
}

BOOL actUartStartTransfer(PifComm* p_comm)
{
    USART_ITConfig(uartPort[PIF_ID_UART_2_IDX(p_comm->_id)].USARTx, USART_IT_TXE, ENABLE);
    return TRUE;
}

// USART1 - Telemetry (RX/TX by DMA)
uartPort_t *serialUSART1(portMode_t mode)
{
    uartPort_t *s;
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    s = &uartPort[0];

    if (!pifComm_Init(&s->port.comm, PIF_ID_UART(0))) return NULL;
    if (mode & MODE_RX) {
        if (!pifComm_AllocRxBuffer(&s->port.comm, 16, 25)) return NULL;
    }
    if (mode & MODE_TX) {
        if (!pifComm_AllocTxBuffer(&s->port.comm, 16)) return NULL;
        s->port.comm.act_start_transfer = actUartStartTransfer;
    }
    s->port.comm.act_set_baudrate = actCommSetBaudRate;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    // USART1_TX    PA9
    // USART1_RX    PA10
    gpio.speed = Speed_2MHz;
    gpio.pin = Pin_9;
    gpio.mode = Mode_AF_PP;
    if (mode & MODE_TX)
        gpioInit(GPIOA, &gpio);
    gpio.pin = Pin_10;
    gpio.mode = Mode_IPU;
    if (mode & MODE_RX)
        gpioInit(GPIOA, &gpio);

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}

// USART2 - GPS or Spektrum or ?? (RX + TX by IRQ)
uartPort_t *serialUSART2(portMode_t mode)
{
    uartPort_t *s;
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    s = &uartPort[1];

    if (!pifComm_Init(&s->port.comm, PIF_ID_UART(1))) return NULL;
    if (mode & MODE_RX) {
        if (!pifComm_AllocRxBuffer(&s->port.comm, 16, 25)) return NULL;
    }
    if (mode & MODE_TX) {
        if (!pifComm_AllocTxBuffer(&s->port.comm, 16)) return NULL;
        s->port.comm.act_start_transfer = actUartStartTransfer;
    }
    s->port.comm.act_set_baudrate = actCommSetBaudRate;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    // USART2_TX    PA2
    // USART2_RX    PA3
    gpio.speed = Speed_2MHz;
    gpio.pin = Pin_2;
    gpio.mode = Mode_AF_PP;
    if (mode & MODE_TX)
        gpioInit(GPIOA, &gpio);
    gpio.pin = Pin_3;
    gpio.mode = Mode_IPU;
    if (mode & MODE_RX)
        gpioInit(GPIOA, &gpio);

    // RX/TX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}

// USART3 - Telemetry (RX/TX by DMA + REMAP)
uartPort_t *serialUSART3(portMode_t mode)
{
    uartPort_t *s;
    gpio_config_t gpio;
    NVIC_InitTypeDef NVIC_InitStructure;

    s = &uartPort[2];

    if (!pifComm_Init(&s->port.comm, PIF_ID_UART(2))) return NULL;
    if (mode & MODE_RX) {
        if (!pifComm_AllocRxBuffer(&s->port.comm, 16, 10)) return NULL;
    }
    if (mode & MODE_TX) {
        if (!pifComm_AllocTxBuffer(&s->port.comm, 16)) return NULL;
        s->port.comm.act_start_transfer = actUartStartTransfer;
    }
    s->port.comm.act_set_baudrate = actCommSetBaudRate;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    // USART3_TX    PB10
    // USART3_RX    PB11
    gpio.speed = Speed_2MHz;
    gpio.pin = Pin_10;
    gpio.mode = Mode_AF_PP;
    if (mode & MODE_TX)
        gpioInit(GPIOB, &gpio);
    gpio.pin = Pin_11;
    gpio.mode = Mode_IPU;
    if (mode & MODE_RX)
        gpioInit(GPIOB, &gpio);

    // RX Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return s;
}

serialPort_t *uartOpen(int port, uint32_t baudRate, portMode_t mode)
{
    USART_InitTypeDef USART_InitStructure;
    uartPort_t *s = NULL;

    if (!port)
        return NULL;

    if (port == UART_PORT_1) {
        s = serialUSART1(mode);
        s->USARTx = USART1;
    }
    else if (port == UART_PORT_2) {
        s = serialUSART2(mode);
        s->USARTx = USART2;
    }
    else if (port == UART_PORT_3) {
        s = serialUSART3(mode);
        s->USARTx = USART3;
    }
    else return NULL;

    s->port.p_param = NULL;

    if (!pifComm_AttachTask(&s->port.comm, TM_PERIOD_MS, 1, TRUE)) return NULL;

    // callback for IRQ-based RX ONLY
    s->port.mode = mode;

    USART_InitStructure.USART_BaudRate = baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    if (mode & MODE_SBUS) {
        USART_InitStructure.USART_StopBits = USART_StopBits_2;
        USART_InitStructure.USART_Parity = USART_Parity_Even;
    } else {
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
    }
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (mode & MODE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (mode & MODE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;
    USART_Init(s->USARTx, &USART_InitStructure);
    USART_Cmd(s->USARTx, ENABLE);

    // Receive DMA or IRQ
    if (mode & MODE_RX) {
        USART_ITConfig(s->USARTx, USART_IT_RXNE, ENABLE);
    }

    // Transmit DMA or IRQ
    if (mode & MODE_TX) {
        USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
    }

    return (serialPort_t *)s;
}

void uartSetBaudRate(uartPort_t *instance, uint32_t baudRate)
{
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = 0;
    if (instance->port.mode & MODE_RX)
        USART_InitStructure.USART_Mode |= USART_Mode_Rx;
    if (instance->port.mode & MODE_TX)
        USART_InitStructure.USART_Mode |= USART_Mode_Tx;
    USART_Init(instance->USARTx, &USART_InitStructure);
}


// Handlers

// USART1 Tx IRQ Handler
void USART1_IRQHandler(void)
{
    uartPort_t *s = &uartPort[0];
    uint16_t SR = s->USARTx->SR;
    uint8_t state, data;

    if (SR & USART_FLAG_RXNE) {
        pifComm_PutRxByte(&s->port.comm, s->USARTx->DR);
    }
    if (SR & USART_FLAG_TXE) {
        state = pifComm_GetTxByte(&s->port.comm, &data);
        if (state & PIF_COMM_SEND_DATA_STATE_DATA) {
            s->USARTx->DR = data;
        }
        if (state & PIF_COMM_SEND_DATA_STATE_EMPTY) {
            pifComm_FinishTransfer(&s->port.comm);
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}

// USART2 Rx/Tx IRQ Handler
void USART2_IRQHandler(void)
{
    uartPort_t *s = &uartPort[1];
    uint16_t SR = s->USARTx->SR;
    uint8_t state, data;

    if (SR & USART_FLAG_RXNE) {
        pifComm_PutRxByte(&s->port.comm, s->USARTx->DR);
    }
    if (SR & USART_FLAG_TXE) {
        state = pifComm_GetTxByte(&s->port.comm, &data);
        if (state & PIF_COMM_SEND_DATA_STATE_DATA) {
            s->USARTx->DR = data;
        }
        if (state & PIF_COMM_SEND_DATA_STATE_EMPTY) {
            pifComm_FinishTransfer(&s->port.comm);
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}

// USART3 Rx IRQ Handler
void USART3_IRQHandler(void)
{
    uartPort_t *s = &uartPort[2];
    uint16_t SR = s->USARTx->SR;
    uint8_t state, data;

    if (SR & USART_FLAG_RXNE) {
        pifComm_PutRxByte(&s->port.comm, s->USARTx->DR);
    }
    if (SR & USART_FLAG_TXE) {
        state = pifComm_GetTxByte(&s->port.comm, &data);
        if (state & PIF_COMM_SEND_DATA_STATE_DATA) {
            s->USARTx->DR = data;
        }
        if (state & PIF_COMM_SEND_DATA_STATE_EMPTY) {
            pifComm_FinishTransfer(&s->port.comm);
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}
