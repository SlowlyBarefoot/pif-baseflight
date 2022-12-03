#pragma once

#define UART_BUFFER_SIZE    64

#define UART1_RX_BUFFER_SIZE    256
#define UART1_TX_BUFFER_SIZE    256
#define UART2_RX_BUFFER_SIZE    128
#define UART2_TX_BUFFER_SIZE    64
#define UART3_RX_BUFFER_SIZE    256
#define UART3_TX_BUFFER_SIZE    256
#define MAX_SERIAL_PORTS        3

// FIXME this is a uart_t really.  Move the generic properties into a separate structure (serialPort_t) and update the code to use it
typedef struct {
    serialPort_t port;

    USART_TypeDef *USARTx;
} uartPort_t;

// serialPort API
void uartSetBaudRate(uartPort_t *s, uint32_t baudRate);
