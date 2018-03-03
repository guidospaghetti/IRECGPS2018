#include <msp430.h> 
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "MTK3339.h"
#include "uart.h"

/*
 * main.c
 */

void setClock16MHz(void);
void initMessage(void);
void sendGPSData(gpsData_t* gps);

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

    // Setup GPS
    gpsParams_t params;
    params.updateRate = 1000;
    params.outputFrames = PMTK_GGA | PMTK_RMC;
    initGPS(&params);

    // Receive messages from GPS

    gpsData_t gps;

    while(1) {
    	uint8_t update = checkForUpdate(&gps);

    	if (update) {
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



void sendGPSData(gpsData_t* gps) {
	char buffer[150];
	sprintf(buffer,
			"Latitude: %f %c Longitude: %f %c Altitude: %f\r\n"
			"Time: %f MM/DD/YY: %d/%d/%d Fix: %d\r\n",
			gps->location.latitude,
			(gps->location.NS ? gps->location.NS : 'X'),
			gps->location.longitude,
			(gps->location.EW ? gps->location.EW : 'X'),
			gps->location.altitude,
			gps->time.utcTime,
			gps->time.month,
			gps->time.day,
			gps->time.year,
			gps->fix);

	sendUARTA1(buffer, strlen(buffer));
}

