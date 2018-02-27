#include <msp430.h> 
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "MTK3339.h"
#include "hal_uart.h"

/*
 * main.c
 */

void setClock16MHz(void);
void initMessage(void);
void sendGPSMessage(char* message);
void sendGPSData(gpsData_t* gps);
uint8_t genChecksum(char* message);
void sendUARTA0(char* bytes, uint32_t length);
void sendUARTA1(char* bytes, uint32_t length);


int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer
    setClock16MHz();

    // Enable interrupts
    __enable_interrupt();

    // Setup UART on A0, 9600 baud, used for GPS
    // Setup UART on A1, 115200 baud, used for computer
    hal_UART_Init();

    // Setup UART on A1, 115200 baud, talk to computer
    initMessage();

    // Send messages to GPS
    sendGPSMessage(PMTK_API_SET_NMEA_OUTPUT_RMC_GGA);
    sendGPSMessage(PMTK_SET_NMEA_UPDATERATE_1000);

    // Receive messages from GPS

    gpsData_t gps;

    while(1) {
    	// Check if a message is coming from the GPS
    	if (lastByte0 == '$') {
    		// Initialize a buffer
    		char bufferStart[150] = {};
    		char* buffer = bufferStart;
    		*buffer = '$';
    		buffer++;
    		volatile uint8_t counter = 1;
    		// The GPS messages end with a new line character
    		// so loop until that happens
    		while (*buffer != '\n') {
    			if (counter > 150) {
    				break;
    			}
    			// Increment the buffer
    			buffer++;
    			// Add the new character to the buffer
    			*buffer = lastByte0;
    			// Wait for the next byte
    			while(!hal_UART_DataAvailable(0));

    			counter++;
    		}

    		// Parse the respone from the GPS
    		readResponse(bufferStart, &gps);
    		// Send the response over UART to computer
    		sendGPSData(&gps);
    	}
    }

}

void setClock16MHz(void) {
	UCSCTL0 = 0x00;
	UCSCTL1 = DCORSEL_6;
	UCSCTL2 = FLLD_0 + 488;

	UCSCTL3 = SELREF_2;
	UCSCTL4 = SELA__REFOCLK | SELS__DCOCLK | SELM__DCOCLK ;

	 __bic_SR_register(SCG0);

	// Worst-case settling time for the DCO when the DCO range bits have been
	// changed is n x 32 x 32 x f_MCLK / f_FLL_reference. See UCS chapter in 5xx
	// UG for optimization.
	// 32 x 32 x 16 MHz / 32,768 Hz = 500000 = MCLK cycles for DCO to settle
	__delay_cycles(500000);
	// Loop until XT1,XT2 & DCO fault flag is cleared
	do {
		UCSCTL7 &= ~(XT2OFFG + XT1LFOFFG + DCOFFG); // Clear XT2,XT1,DCO fault flags
		SFRIFG1 &= ~OFIFG;                      // Clear fault flags
	} while (SFRIFG1&OFIFG);                   // Test oscillator fault flag

	return;
}

void initMessage(void) {
	char* helloMessage = "Press enter when ready to start\r\n";
	sendUARTA1(helloMessage, strlen(helloMessage));
	while (lastByte1 != '\r');
}

void sendGPSMessage(char* message) {
	char checksumBytes[3];
	uint32_t length = strlen(message);
	char buffer[150] = {};
	uint8_t checksum = genChecksum(message);
	sprintf(checksumBytes, "%d", checksum);

	// Create the message that will be sent to the GPS
	buffer[0] = '$';
	strcat(buffer, message);
	strcat(buffer, "*");
	strcat(buffer, checksumBytes);
	strcat(buffer, "\r\n");

	sendUARTA0(buffer, strlen(buffer));
}

void sendGPSData(gpsData_t* gps) {
	char buffer[150];
	sprintf(buffer, "Latitude: %f %c\tLongitude: %f %c\tAltitude: %f\r\n", gps->location.latitude,
			(gps->location.NS ? gps->location.NS : 'X'),
			gps->location.longitude,
			(gps->location.EW ? gps->location.EW : 'X'),
			gps->location.altitude);

	sendUARTA1(buffer, strlen(buffer));
}

uint8_t genChecksum(char* message) {
	uint8_t checksum = 0;

	while (*message != '\0') {
		checksum = checksum ^= *message;
		message++;
	}

	return checksum;
}

void sendUARTA0(char* bytes, uint32_t length) {
	volatile int i;
	for (i = 0; i < length; i++) {
		while(hal_UART_SpaceAvailable(0) == 0);
		hal_UART_TxByte((uint8_t)*bytes, 0);
		bytes++;
	}
}

void sendUARTA1(char* bytes, uint32_t length) {
	volatile int i;
	for (i = 0; i < length; i++) {
		while(hal_UART_SpaceAvailable(1) == 0);
		hal_UART_TxByte((uint8_t)*bytes, 1);
		bytes++;
	}
}

