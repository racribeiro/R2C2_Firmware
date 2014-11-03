/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include        <stdarg.h>

#include "uart.h"
#include "lpc17xx_uart.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"

char   uart_str_ox[] = "0x";

void uart_init(int baud_rate)
{
	// UART Configuration structure variable
	UART_CFG_Type UARTConfigStruct;
	// UART FIFO configuration Struct variable
	UART_FIFO_CFG_Type UARTFIFOConfigStruct;
	// Pin configuration for UART
	PINSEL_CFG_Type PinCfg;

	/*
	* Initialize UART3 pin connect: P4.28 -> TXD3; P4.29 -> RXD3
	* or P0.2 -> TXD0, P0.3 -> RXD0
	*/
	PinCfg.OpenDrain = PINSEL_PINMODE_NORMAL;
	PinCfg.Pinmode = PINSEL_PINMODE_PULLUP;
#if DBG_UART_NUM == 3	
	PinCfg.Funcnum = PINSEL_FUNC_3;
	PinCfg.Portnum = 4;
	PinCfg.Pinnum = 28; // TX
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 29; // RX
	PINSEL_ConfigPin(&PinCfg);
	
    GPIO_SetDir(4, ((uint32_t)1 << 28), 1); // OUTPUT = 1
	GPIO_SetDir(4, ((uint32_t)1 << 29), 0); // INPUT = 1
#else
	PinCfg.Funcnum = PINSEL_FUNC_1;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 3;
	PINSEL_ConfigPin(&PinCfg);
	
    GPIO_SetDir(0, (1 << 2), 1);
	GPIO_SetDir(0, (1 << 3), 0);
#endif	

	/* Initialize UART Configuration parameter structure to default state:
		* Baudrate = as below
		* 8 data bit
		* 1 Stop bit
		* None parity
		*/
	UART_ConfigStructInit(&UARTConfigStruct);
	UARTConfigStruct.Baud_rate = baud_rate;

	// Initialize UART peripheral with given to corresponding parameter
	UART_Init(DBG_UART, &UARTConfigStruct);

	/* Initialize FIFOConfigStruct to default state:
	*                              - FIFO_DMAMode = DISABLE
	*                              - FIFO_Level = UART_FIFO_TRGLEV0
	*                              - FIFO_ResetRxBuf = ENABLE
	*                              - FIFO_ResetTxBuf = ENABLE
	*                              - FIFO_State = ENABLE
	*/
	UART_FIFOConfigStructInit(&UARTFIFOConfigStruct);

	// Initialize FIFO for UART peripheral
	UART_FIFOConfig(DBG_UART, &UARTFIFOConfigStruct);

	// Enable UART Transmit
	UART_TxCmd(DBG_UART, ENABLE);	
}

char uart_data_available(void)
{
	return (DBG_UART->LSR & UART_LSR_RDR);
}

char uart_receive(void)
{
	return (UART_ReceiveByte(DBG_UART));
}

void uart_send(char byte)
{
	while ( (DBG_UART->LSR & UART_LSR_THRE) == 0) ;
	UART_SendByte(DBG_UART, byte);
}

void uart_writestr(char *data)
{
	uint8_t i = 0;
	char r;

 	while ((r = data[i++]))
		uart_send(r);
}

void uart_hex4(uint8_t v) {
	v &= 0xF;
	if (v < 10)
		uart_writechar('0' + v);
	else
		uart_writechar('A' - 10 + v);
}

void uart_hex8(uint8_t v) {
	uart_hex4(v >> 4);
	uart_hex4(v & 0x0F);
}

void uart_hex16(uint16_t v) {
	uart_hex8(v >> 8);
	uart_hex8(v & 0xFF);
}

void uart_hex32(uint32_t v) {
	uart_hex16(v >> 16);
	uart_hex16(v & 0xFFFF);
}

void uart_uint32(uint32_t v) {
	uint8_t t = 0;
	if (v >= 1000000000) {
		for (t = 0; v >= 1000000000; v -= 1000000000, t++);
		uart_writechar(t + '0');
	}

	if (v >= 100000000) {
		for (t = 0; v >= 100000000; v -= 100000000, t++);
		uart_writechar(t + '0');
	}
	else if (t != 0)
		uart_writechar('0');

	if (v >= 10000000) {
		for (t = 0; v >= 10000000; v -= 10000000, t++);
		uart_writechar(t + '0');
	}
	else if (t != 0)
		uart_writechar('0');

	if (v >= 1000000) {
		for (t = 0; v >= 1000000; v -= 1000000, t++);
		uart_writechar(t + '0');
	}
	else if (t != 0)
		uart_writechar('0');

	if (v >= 100000) {
		for (t = 0; v >= 100000; v -= 100000, t++);
		uart_writechar(t + '0');
	}
	else if (t != 0)
		uart_writechar('0');

	if (v >= 10000) {
		for (t = 0; v >= 10000; v -= 10000, t++);
		uart_writechar(t + '0');
	}
	else if (t != 0)
		uart_writechar('0');

	if (v >= 1000) {
		for (t = 0; v >= 1000; v -= 1000, t++);
		uart_writechar(t + '0');
	}
	else if (t != 0)
		uart_writechar('0');

	if (v >= 100) {
		t = v / 100;
		uart_writechar(t + '0');
		v -= (t * 100);
	}
	else if (t != 0)
		uart_writechar('0');

	if (v >= 10) {
	        /* 99 > v > 10 */
		t = v / 10;
		uart_writechar(t + '0');
		v -= (t * 10);
	}
	else if (t != 0)
		uart_writechar('0');

	uart_writechar(v + '0');
}

void uart_int32(int32_t v) {
	if (v < 0) {
		uart_writechar('-');
		v = -v;
	}

	uart_uint32(v);
}

void uart_double(double v)
{
  if (v < 0)
  {
    uart_writechar ('-');
    v = -v;
  }
  
  /* print first part before '.' */
  uart_uint32((uint32_t) v);

  /* print the '.' */
  uart_writechar('.');

  /* print last part after '.' */
  v = v - (int32_t)v;

  v = v * 1000.0;
  if (v < 100.0)
  	uart_writechar('0');
  if (v < 10.0)
  	uart_writechar('0');
  uart_uint32((uint32_t) v);  	
  
}



void uart_sendf(char *format, ...)
{ 
  va_list args;
  va_start(args, format);
		
          unsigned int i = 0;
        unsigned char c, j = 0;
        while ((c = format[i++])) {
                if (j) {
                        switch(c) {
                                case 'l':
                                        j = 4;
                                        break;
                                case 'u':
                                        if (j == 4)
                                                uart_uint32(va_arg(args, unsigned int));
                                        else
                                                uart_uint16(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
                                case 'd':
                                        if (j == 4)
                                                uart_int32(va_arg(args, int));
                                        else
                                                uart_int16(va_arg(args, int));
                                        j = 0;
                                        break;

                                /* print a double in normal notation */
                                case 'g':
                                uart_double(va_arg(args, double));
                                j = 0;
                                break;

                                case 'p':
                                case 'x':
                                        uart_writestr(uart_str_ox);
                                        if (j == 4)
                                                uart_hex32(va_arg(args, unsigned int));
                                        else
                                                uart_hex16(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
                                case 'c':
                                        uart_send(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
                                case 's':
                                        uart_writestr(va_arg(args, char *));
                                        j = 0;
                                        break;
                                default:
                                        j = 0;
                                        break;
                        }
                }
                else {
                        if (c == '%') {
                                j = 2;
                        }
                        else {
                                uart_send(c);
                        }
                }
        }

}
