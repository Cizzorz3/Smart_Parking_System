/*
 * uart.h
 *
 *  Created on: Oct 10, 2022
 *      Author: Mohamed Adel
 */

#ifndef INC_STM32_F103_C6_USART_H_
#define INC_STM32_F103_C6_USART_H_

#include "STM32f103C6.h"
#include "STM32_F103_C6_RCC.h"
#include "STM_F103C6_GPIO_Driver.h"


//================================
/*
Bit 3 TE: Transmitter enable
This bit enables the transmitter. It is set and cleared by software.
0: Transmitter is disabled
1: Transmitter is enabled

Bit 2 RE: Receiver enable
This bit enables the receiver. It is set and cleared by software.
0: Receiver is disabled
1: Receiver is enabled and begins searching for a start bit
*/
#define UART_Mode_Rx    (uint32_t(1<<2))
#define UART_Mode_Tx    (uint32_t(1<<3))
#define UART_Mode_Tx_RX ((uint32_t)(1<<3 | 1<<2))

//baud rate
#define UART_Baudrate_2400	    2400
#define UART_Baudrate_9600	    9600
#define UART_Baudrate_19200	    19200
#define UART_Baudrate_57600	    57600
#define UART_Baudrate_115200    115200
#define UART_Baudrate_230400	230400
#define UART_Baudrate_460800	460800
#define UART_Baudrate_921600	921600

//======================================
/*
Bit 12 M: Word length
This bit determines the word length. It is set or cleared by software.
0: 1 Start bit, 8 Data bits, n Stop bit
1: 1 Start bit, 9 Data bits, n Stop bit
note: The M bit must not be modified during a data transfer (both transmission and reception)
*/
#define Payload_length_8 		(uint32_t)(0)
#define Payload_length_9 		(uint32_t)(1<<12)

//==================================
/*
Bit 10 PCE: Parity control enable
This bit selects the hardware parity control (generation and detection). When the parity
control is enabled, the computed parity is inserted at the MSB position (9th bit if M=1; 8th bit
if M=0) and parity is checked on the received data. This bit is set and cleared by software.
Once it is set, PCE is active after the current byte (in reception and in transmission).
0: Parity control disabled
1: Parity control enabled
*/
#define  UART_Parity_None		(uint32_t)(0)
#define  UART_Parity_even		(uint32_t)(1<<10)
#define  UART_Parity_odd		((uint32_t)(1<<10 | 1<<9))
//==================================

/*
Bits 13:12 STOP: STOP bits
These bits are used for programming the stop bits.
00: 1 Stop bit
01: 0.5 Stop bit
10: 2 Stop bits
11: 1.5 Stop bit
The 0.5 Stop bit and 1.5 Stop bit are not available for UART4 & UART5.
*/

//====================================
#define Stop_bits_half			(uint32_t)(1<<12)
#define stop_bits_1				(uint32_t)(0)
#define stop_bits_1_half		(uint32_t)(3<<12)
#define stop_bits_2				(uint32_t)(2<<12)

//==========================================================
/*
Bit 9 CTSE: CTS enable
0: CTS hardware flow control disabled
1: CTS mode enabled, data is only transmitted when the CTS input is asserted (tied to 0). If
the CTS input is deasserted while a data is being transmitted, then the transmission is
completed before stopping. If a data is written into the data register while CTS is deasserted,
the transmission is postponed until CTS is asserted.
This bit is not available for UART4 & UART5.

Bit 8 RTSE: RTS enable
0: RTS hardware flow control disabled
1: RTS interrupt enabled, data is only requested when there is space in the receive buffer.
The transmission of data is expected to cease after the current character has been
transmitted. The RTS output is asserted (tied to 0) when a data can be received.
This bit is not available for UART4 & UART5.
*/
#define  HW_flwctl_None			(uint32_t)(0)
#define  HW_flwctl_RTS			(uint32_t)(1<<8)
#define  HW_flwctl_CTS			((uint32_t)(1<<9))
#define  HW_flwctl_CTS_RTS		((uint32_t)(1<<8 | 1<<9))

//===========================================================

/*
Bit 8 PEIE: PE interrupt enable
This bit is set and cleared by software.
0: Interrupt is inhibited
1: A USART interrupt is generated whenever PE=1 in the USART_SR register

Bit 7 TXEIE: TXE interrupt enable
This bit is set and cleared by software.
0: Interrupt is inhibited
1: A USART interrupt is generated whenever TXE=1 in the USART_SR register
*/
//===================UART IRQ Macros=========================
#define UART_IRQ_Enable_None	(uint32_t)(0)
#define UART_IRQ_Enable_TXE		(uint32_t)(1<<7)
#define UART_IRQ_Enable_TC		(uint32_t)(1<<6)
#define UART_IRQ_Enable_RXE		(uint32_t)(1<<5)
#define UART_IRQ_Enable_PE		(uint32_t)(1<<8)


//===========================================================

//Baud Rate calculation
//USARTDIV = fclk / (16 * BaudRate)
//USARTDIV_MUL100 = 25 * fclk / (4 * BaudRate)
//DIV_Mantissa_MUL100 = Integer part of USARTDIV_MUL100 * 100
//DIV_Mantissa = Integer part of USARTDIV
//DIV_Fraction = ((USARTDIV_MUL100 - DIV_Mantissa_MUL100 )*16) / 100

#define USARTDIV(_pclk_,_BAUD_)					            (uint32_t)(_pclk_ / (16 * _BAUD_))
#define USARTDIV_MUL100(_pclk_,_BAUD_)			            (uint32_t)(25 * _pclk_ / (4 * _BAUD_))
#define DIV_Mantissa_MUL100(_pclk_,_BAUD_)					(uint32_t) (USARTDIV(_pclk_,_BAUD_)	* 100)
#define DIV_Mantissa(_pclk_,_BAUD_)							(uint32_t) (USARTDIV(_pclk_,_BAUD_)	)
#define DIV_Fraction(_pclk_,_BAUD_)							(uint32_t) (((USARTDIV_MUL100(_pclk_,_BAUD_) - DIV_Mantissa_MUL100(_pclk_,_BAUD_) )*16) / 100	)
#define UART_BRR_Register(_pclk_,_BAUD_)					((DIV_Mantissa(_pclk_,_BAUD_)) << 4) | ( (DIV_Fraction(_pclk_,_BAUD_)) & 0xf)

//===========================================================
//config struct
typedef struct
{
	uint8_t USART_Mode; //Specify TX/RX Mode
						//Note this parameter will be set based on @ref

	uint32_t BaudRate ;  // Specify baudrate
						//Note this parameter will be set based on @ref

	uint8_t Payload_length; //Specify payload length 8 or 9

	uint8_t Parity ; //Specify if there is a parity or not and it's type

	uint8_t stop_bits ;

	uint8_t HW_flow_control ;

	uint8_t IRQ_Enable ;

	void (*P_IRQ_Callback)(void);
}UART_config;

enum Polling_mechanism {enable , disable };

///APIS

void MCAL_UART_Init(USART_typedef * Usartx , UART_config * UART_config);
void MCAL_UART_Denit(USART_typedef * Usartx);
void MCAL_UART_Set_GPIO_Pins(USART_typedef * Usartx);
void MCAL_UART_Send_Data(USART_typedef * Usartx , uint16_t * ptxbuffer , enum Polling_mechanism Polling_en);
void MCAL_UART_Recieve_Data(USART_typedef * Usartx , uint16_t * prxbuffer , enum Polling_mechanism Polling_en);
void MCAL_UART_Wait_TC(USART_typedef * Usartx);




#endif /* INC_STM32_F103_C6_USART_H_ */