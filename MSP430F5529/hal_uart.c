/*
 * hal_uart.c
 *
 *  Created on: Feb 23, 2018
 *      Author: Aaron
 */

#include "hal_uart.h"
#include "msp430f5529.h"

void hal_UART_Init(void) {
	// Set pins for UARTA0 and A1
	P3SEL |= BIT3 + BIT4;
	P4SEL |= BIT4 + BIT5;

	// Set software reset pin
	UCA0CTL1 = UCSWRST;
	// Set clock to SMCLK
	UCA0CTL1 |= UCSSEL1;
	// Set baud rate, 9600 baud, Assuming 16MHz / (131 + 6*256) = ~9600
	UCA0BR0 = 131;
	UCA0BR1 = 6;
	// Turn on UART
	UCA0CTL1 &= ~UCSWRST;
	// Enable Interrupts
	UCA0IE |= UCRXIE;

	// Set software reset pin
	UCA1CTL1 = UCSWRST;
	// Set clock to SMCLK
	UCA1CTL1 |= UCSSEL1;
	// Set baud rate, 115200 baud, Assuming 16MHz / (139 + 0*256) = ~115200
	UCA1BR0 = 139;
	UCA1BR1 = 0;
	// Turn on UART
	UCA1CTL1 &= ~UCSWRST;
	// Enable Interrupts
	UCA1IE |= UCRXIE;
}

void hal_UART_TxByte(uint8_t byte, uint8_t channel) {
	switch (channel) {
	case 0:
		UCA0TXBUF = byte;
		break;
	case 1:
		UCA1TXBUF = byte;
		break;
	default:
		break;
	}
}

uint8_t hal_UART_RxByte(uint8_t channel) {
	switch (channel) {
	case 0:
		return UCA0RXBUF;
	case 1:
		return UCA1RXBUF;
	default:
		return 0;
	}
}

uint8_t hal_UART_DataAvailable(uint8_t channel) {
	switch (channel) {
	case 0:
		return UCA0IFG & UCRXIFG;
	case 1:
		return UCA1IFG & UCRXIFG;
	default:
		return 0;
	}
}

uint8_t hal_UART_SpaceAvailable(uint8_t channel) {
	switch (channel) {
	case 0:
		return UCA0IFG & UCTXIFG;
	case 1:
		return UCA1IFG & UCTXIFG;
	default:
		return 0;
	}
}


#pragma vector=USCI_A0_VECTOR
__interrupt void UART_A0(void) {
	switch(__even_in_range(UCA0IV,4))
	{
	case USCI_NONE:
		break;
	case USCI_UCRXIFG:
		lastByte0 = UCA0RXBUF;
		break;
	case USCI_UCTXIFG:
		break;
	default:
		break;
	}
}

#pragma vector=USCI_A1_VECTOR
__interrupt void UART_A1(void) {
	switch(__even_in_range(UCA1IV,4))
	{
	case USCI_NONE:
		break;
	case USCI_UCRXIFG:
		lastByte1 = UCA1RXBUF;
		break;
	case USCI_UCTXIFG:
		break;
	default:
		break;
	}
}