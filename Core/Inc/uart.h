/*
 * uart.h
 *
 *  Created on: Apr 7, 2023
 *      Author: 320068681
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "stm32f4xx_hal.h"
#include "stdio.h"

/* change the size of the buffer */
#define UART_BUFFER_SIZE 256

typedef struct
{ unsigned char buffer[UART_BUFFER_SIZE];
  volatile unsigned int head;
  volatile unsigned int tail;
} ring_buffer;

void UARTringbuffer(void);

void Uartstore_char(unsigned char c, ring_buffer *buffer);

int Uart_read(void);
void Uart_write(int c);
void UartSendBytes (uint8_t *byte, uint8_t number);
void Uart_sendstring(const char *s);
void Uart_printbase (long n, uint8_t base);
int UartIsDataAvailable(void);
void UartGetDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto);
void Uart_flush (void);
void UartBufferRemovePrefix(char pos);
int Uart_peek();
uint8_t UartPeekRXByte(uint8_t index);
int UartCopy_upto (char *string, char *buffertocopyinto);
int UartGet_after (char *string, uint8_t numberofchars, char *buffertosave);
int UartWait_for (char *string);
uint8_t UartBufferFindByte(uint8_t needle);
void Uart_isr (UART_HandleTypeDef *huart);

void UART_Printf(const char* fmt, ...);
void UART_Printf_debug(char* buffer);



#endif /* INC_UART_H_ */
