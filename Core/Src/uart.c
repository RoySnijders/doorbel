#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include "stdio.h"

#include "main.h"
#include "uart.h"

extern UART_HandleTypeDef huart1;
//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart6;
#define uart1 &huart1
//#define uart2 &huart2
//#define uart6 &huart6


ring_buffer rx_buffer = { { 0 }, 0, 0};
ring_buffer tx_buffer = { { 0 }, 0, 0};

ring_buffer *_rx_buffer;
ring_buffer *_tx_buffer;


// --------------------------------------------------------------------------------
// Initialize the ring buffer
// --------------------------------------------------------------------------------
void UARTringbuffer(void)
{
  _rx_buffer = &rx_buffer;
  _tx_buffer = &tx_buffer;

  /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
  __HAL_UART_ENABLE_IT(uart1, UART_IT_ERR);

  /* Enable the UART Data Register not empty Interrupt */
  __HAL_UART_ENABLE_IT(uart1, UART_IT_RXNE);
}



// --------------------------------------------------------------------------------
// Store Char in Buffer
// --------------------------------------------------------------------------------
void Uartstore_char(unsigned char c, ring_buffer *buffer)
{	int i = (unsigned int)(buffer->head + 1) % UART_BUFFER_SIZE;

	// if we should be storing the received character into the location
	// just before the tail (meaning that the head would advance to the
	// current location of the tail), we're about to overflow the buffer
	// and so we don't write the character or advance the head.
	if(i != buffer->tail)
	{	buffer->buffer[buffer->head] = c;
		buffer->head = i;
	}
}


// reads the data in the rx_buffer and increment the tail count in rx_buffer
int Uart_read(void)
{
  // if the head isn't ahead of the tail, we don't have any characters
  if(_rx_buffer->head == _rx_buffer->tail)
  {
    return -1;
  }
  else
  {
    unsigned char c = _rx_buffer->buffer[_rx_buffer->tail];
    _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
    return c;
  }
}


//Checks if the data is available to read in the rx_buffer
//@return =0 if 0 bytes, # bytes will be returned
int UartIsDataAvailable(void)
{
  return (uint16_t)(UART_BUFFER_SIZE + _rx_buffer->head - _rx_buffer->tail) % UART_BUFFER_SIZE;
}



// writes the data to the tx_buffer and increment the head count in tx_buffer
void Uart_write(int c)
{
	if (c>=0)
	{
		int i = (_tx_buffer->head + 1) % UART_BUFFER_SIZE;

		// If the output buffer is full, there's nothing for it other than to
		// wait for the interrupt handler to empty it a bit
		// ???: return 0 here instead?
		while (i == _tx_buffer->tail);

		_tx_buffer->buffer[_tx_buffer->head] = (uint8_t)c;
		_tx_buffer->head = i;

		__HAL_UART_ENABLE_IT(uart1, UART_IT_TXE); // Enable UART transmission interrupt
	}
}



//function to send the string to the uart
void Uart_sendstring (const char *s)
{
	while(*s) Uart_write(*s++);
}

//function to send the string to the uart
void UartSendBytes (uint8_t *byte, uint8_t number)
{	uint8_t i=0;

	while(i!=number)
		Uart_write(byte[i++]);
}


//Print a number with any base, base can be 10, 8,2
void Uart_printbase (long n, uint8_t base)
{
  char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
  char *s = &buf[sizeof(buf) - 1];

  *s = '\0';

  // prevent crash if called with base == 1
  if (base < 2) base = 10;

  do {
    unsigned long m = n;
    n /= base;
    char c = m - base * n;
    *--s = c < 10 ? c + '0' : c + 'A' - 10;
  } while(n);

  while(*s) Uart_write(*s++);
}

uint8_t UartPeekRXByte(uint8_t index)
{	index+=(uint8_t) _rx_buffer->tail;
	if(index>=64) index-=64;
	return (uint8_t) _rx_buffer->buffer[index];
}

/* Copies the required data from a buffer
 * @startString: the string after which the data need to be copied
 * @endString: the string before which the data need to be copied
 * @USAGE:: GetDataFromBuffer ("name=", "&", buffertocopyfrom, buffertocopyinto);
 */
void UartGetDataFromBuffer (char *startString, char *endString, char *buffertocopyfrom, char *buffertocopyinto)
{
	int startStringLength = strlen (startString);
	int endStringLength   = strlen (endString);
	int so_far = 0;
	int indx = 0;
	int startposition = 0;
	int endposition = 0;

repeat1:
	while (startString[so_far] != buffertocopyfrom[indx]) indx++;
	if (startString[so_far] == buffertocopyfrom[indx])
	{
		while (startString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == startStringLength) startposition = indx;
	else
	{
		so_far =0;
		goto repeat1;
	}

	so_far = 0;

repeat2:
	while (endString[so_far] != buffertocopyfrom[indx]) indx++;
	if (endString[so_far] == buffertocopyfrom[indx])
	{
		while (endString[so_far] == buffertocopyfrom[indx])
		{
			so_far++;
			indx++;
		}
	}

	if (so_far == endStringLength) endposition = indx-endStringLength;
	else
	{
		so_far =0;
		goto repeat2;
	}

	so_far = 0;
	indx=0;

	for (int i=startposition; i<endposition; i++)
	{
		buffertocopyinto[indx] = buffertocopyfrom[i];
		indx++;
	}
}

/* Resets the entire ring buffer, the new data will start from position 0 */
void Uart_flush (void)
{	memset(_rx_buffer->buffer,'\0', UART_BUFFER_SIZE);
	_rx_buffer->head = 0;
	_rx_buffer->tail = 0;
}


void UartBufferRemovePrefix(char pos)
{	_rx_buffer->tail=(unsigned int) pos;	//set tail pointer to pos
}


/* Peek for the data in the Rx Bffer without incrementing the tail count
* Returns the last character
* USAGE: if (Uart_peek () == 'M') do something
*/
int Uart_peek()
{
  if(_rx_buffer->head == _rx_buffer->tail)
	  return -1;
  else
	  return _rx_buffer->buffer[_rx_buffer->tail];
}


/* Copy the data from the Rx buffer into the bufferr, Upto and including the entered string
* This copying will take place in the blocking mode, so you won't be able to perform any other operations
* Returns 1 on success and -1 otherwise
* USAGE: while (!(Copy_Upto ("some string", buffer)));
*/
int UartCopy_upto (char *string, char *buffertocopyinto)
{
	int so_far =0;
	int len = strlen (string);
	int indx = 0;

again:
	while (!UartIsDataAvailable());
	while (Uart_peek() != string[so_far])
		{
			buffertocopyinto[indx] = _rx_buffer->buffer[_rx_buffer->tail];
			_rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
			indx++;
			while (!UartIsDataAvailable());

		}
	while (Uart_peek() == string [so_far])
	{
		so_far++;
		buffertocopyinto[indx++] = Uart_read();
		if (so_far == len) return 1;
		while (!UartIsDataAvailable());
	}

	if (so_far != len)
	{
		so_far = 0;
		goto again;
	}

	if (so_far == len) return 1;
	else return -1;
}

/* Copies the entered number of characters (blocking mode) from the Rx buffer into the buffer, after some particular string is detected
* Returns 1 on success and -1 otherwise
* USAGE: while (!(Get_after ("some string", 6, buffer)));
*/
int UartGet_after (char *string, uint8_t numberofchars, char *buffertosave)
{

	while (UartWait_for(string) != 1);
	for (int indx=0; indx<numberofchars; indx++)
	{
		while (!(UartIsDataAvailable()));
		buffertosave[indx] = Uart_read();
	}
	return 1;
}



/* Wait until a paricular string is detected in the Rx Buffer
* Return 1 on success and -1 otherwise
* USAGE: while (!(Wait_for("some string")));
*/
int UartWait_for (char *string)
{
	int so_far =0;
	int len = strlen (string);

again:
	while (!UartIsDataAvailable());
	while (Uart_peek() != string[so_far]) _rx_buffer->tail = (unsigned int)(_rx_buffer->tail + 1) % UART_BUFFER_SIZE;
	while (Uart_peek() == string [so_far])
	{	so_far++;
		Uart_read();
		if (so_far == len) return 1;
		while (!UartIsDataAvailable());
	}

	if (so_far != len)
	{	so_far = 0;
		goto again;
	}

	if (so_far == len) return 1;
	else return -1;
}



// ------------------------------------------------------------------------------
// Check Buffer if it contains a certain byte (char)
// Return where first postion has been found, Other wise return 255
//------------------------------------------------------------------------------
uint8_t UartBufferFindByte(uint8_t needle)
{	uint8_t head = (uint8_t) _rx_buffer->head;
	uint8_t tail = (uint8_t) _rx_buffer->tail;
	uint8_t i=tail;

	while (i != head)
	{	if(_rx_buffer->buffer[i] == (unsigned char) needle)
			return i;

		if(i<UART_BUFFER_SIZE)
			i++;
		else
			i=0;	// tail is before head so start from begin buffer to search further
	}
	return 255;
}


// ------------------------------------------------------------------------------
//the ISR for the uart. put it in the IRQ handler
// ------------------------------------------------------------------------------
void Uart_isr (UART_HandleTypeDef *huart)
{
	  uint32_t srflags   = READ_REG(huart->Instance->ISR);
	  uint32_t cr1its     = READ_REG(huart->Instance->CR1);

    /* if DR is not empty and the Rx Int is enabled */
    if (((srflags & USART_ISR_RXNE_RXFNE) != RESET) && ((cr1its & USART_CR1_RXNEIE_RXFNEIE) != RESET))
    {
    	 /******************
    	    	      *  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
    	    	      *          error) and IDLE (Idle line detected) flags are cleared by software
    	    	      *          sequence: a read operation to USART_SR register followed by a read
    	    	      *          operation to USART_DR register.
    	    	      * @note   RXNE flag can be also cleared by a read to the USART_DR register.
    	    	      * @note   TC flag can be also cleared by software sequence: a read operation to
    	    	      *          USART_SR register followed by a write operation to USART_DR register.
    	    	      * @note   TXE flag is cleared only by a write to the USART_DR register.

    	 *********************/
		huart->Instance->ISR;                       /* Read status register */
        unsigned char c = huart->Instance->RDR;     /* Read data register */
        Uartstore_char (c, _rx_buffer);  				// store data in buffer
        return;
    }

    /*If interrupt is caused due to Transmit Data Register Empty */
    if (((srflags & USART_ISR_TXFE) != RESET) && ((cr1its & USART_CR1_TXFEIE) != RESET))
    {	if(tx_buffer.head == tx_buffer.tail)
    		__HAL_UART_DISABLE_IT(huart, UART_IT_TXE);	// Buffer empty, so disable interrupts
    	else
    	{	// There is more data in the output buffer. Send the next byte
    		unsigned char c = tx_buffer.buffer[tx_buffer.tail];
    	    tx_buffer.tail = (tx_buffer.tail + 1) % UART_BUFFER_SIZE;

			/******************
			*  @note   PE (Parity error), FE (Framing error), NE (Noise error), ORE (Overrun
			*          error) and IDLE (Idle line detected) flags are cleared by software
			*          sequence: a read operation to USART_SR register followed by a read
			*          operation to USART_DR register.
			* @note   RXNE flag can be also cleared by a read to the USART_DR register.
			* @note   TC flag can be also cleared by software sequence: a read operation to
			*          USART_SR register followed by a write operation to USART_DR register.
			* @note   TXE flag is cleared only by a write to the USART_DR register.
			*********************/
			huart->Instance->ISR;
			huart->Instance-> RDR = c;
    	}
    	return;
    }

}



void UART_Printf(const char* fmt, ...)
{   char buff[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buff, sizeof(buff), fmt, args);
    //HAL_UART_Transmit(&huart1, (uint8_t*)buff, strlen(buff), HAL_MAX_DELAY);
    Uart_sendstring(buff);
    va_end(args);
}

void UART_Printf_debug(char* buffer)
{	//#define ASCII_ESC 27
	//#define EC_CLEARSCREEN	"[2J"
	//UART_Printf("%c%[2J", 27);	//Clear Screen
	UART_Printf("\r\n");	//Enter 2x
	UART_Printf("-------------------------------------------------------------\r\n");	//
	Uart_sendstring(buffer);
	UART_Printf("-------------------------------------------------------------\r\n");	//
}

