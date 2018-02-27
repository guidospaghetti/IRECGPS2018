#include <stdlib.h>
#include <string.h>
#include "MTK3339.h"



typedef enum parserState_t {
	START,
	COMMAND,
	DATA,
	CHECKSUM,
	END
} parserState_t;

void parseResponse(char* response, gpsData_t* gps);

void readResponse(char* response, gpsData_t* gps) {
	parseResponse(response, gps);
}

void parseResponse(char* response, gpsData_t* gps) {
	char* start = response;
	char buffer[100];
	char* bufferPtr = buffer;

	int numData = 0;
	int dataPos = 0;
	int curDataLen = 0;
	uint8_t checksumDone = 0;
	parserState_t state = START;

	while (*response != '\r') {
		if (*response == ',' || *response == '*') {
			numData++;
		}
		response++;
	}

	response = start;
	char* allData[50];

	while (*response != '\r') {
		switch (state) {
		case START:
			if (*response == '$') {
				state = COMMAND;
			}
			break;
		case COMMAND:
			if (*response != ',') {
				*bufferPtr = *response;
				bufferPtr++;
			}
			else {
				strncpy(allData[dataPos], buffer, PMTK_COMMAND_NAME_LENGTH);
				bufferPtr = buffer;
				state = DATA;
				dataPos++;
			}
			break;
		case DATA:
			if (*response == '*') {
				strncpy(allData[dataPos], buffer, curDataLen);
				bufferPtr = buffer;
				curDataLen = 0;
				dataPos++;
				state = CHECKSUM;
			}
			else if (*response == ',') {
				strncpy(allData[dataPos], buffer, curDataLen);
				bufferPtr = buffer;
				curDataLen = 0;
				dataPos++;
			}
			else {
				*bufferPtr = *response;
				bufferPtr++;
				curDataLen++;
			}
			break;
		case CHECKSUM:
			*bufferPtr = *response;
			bufferPtr++;
			if (checksumDone) {
				strncpy(allData[dataPos], buffer, PMTK_CHECKSUM_LENGTH);
				bufferPtr = buffer;
			}
			checksumDone = 1;
			break;
		default:
			break;
		}
		response++;
	}

	if (strcmp(allData[0], PMTK_GGA_RETURN) == 0) {
		gps->time.utcTime = atof(allData[PMTK_GGA_UTC_TIME]);
		gps->location.latitude = atof(allData[PMTK_GGA_LATITUDE]);
		gps->location.NS = *allData[PMTK_GGA_NS_INDICATOR];
		gps->location.longitude = atof(allData[PMTK_GGA_LONGITUDE]);
		gps->location.EW = *allData[PMTK_GGA_EW_INDICATOR];
		gps->fix = atoi(allData[PMTK_GGA_FIX]);
		gps->numSatellites = atoi(allData[PMTK_GGA_NUM_SATELLITES]);
		gps->location.altitude = atof(allData[PMTK_GGA_ALTITUDE]);
	}
	else if (strcmp(allData[0], PMTK_GSA_RETURN) == 0) {

	}
	else if (strcmp(allData[0], PMTK_GSV_RETURN) == 0) {
		uint8_t numMessages = atoi(allData[PMTK_GSV_NUM_MESSAGES]);
		uint8_t messageNum = atoi(allData[PMTK_GSV_MESSAGE_NUM]);
		volatile int i;
		for (i = 0; i < 4; i++) {
			gps->allSatellites[messageNum * 4 + i].satelliteID = atoi(allData[PMTK_GSV_SATELLITE_ID_1 + 4 * i]);
			gps->allSatellites[messageNum * 4 + i].elevation = atoi(allData[PMTK_GSV_ELEVATION_1 + 4 * i]);
			gps->allSatellites[messageNum * 4 + i].azimuth = atoi(allData[PMTK_GSV_AZIMUTH_1 + 4 * i]);
			gps->allSatellites[messageNum * 4 + i].SNR = atoi(allData[PMTK_GSV_SNR_1 + 4 * i]);
		}

	}
	else if (strcmp(allData[0], PMTK_RMC_RETURN) == 0) {
		gps->time.utcTime = atof(allData[PMTK_RMC_UTC_TIME]);
		gps->status = (*allData[PMTK_RMC_STATUS] == 'A' ? 1 : 0);
		gps->location.latitude = atof(allData[PMTK_RMC_LATITUDE]);
		gps->location.NS = *allData[PMTK_RMC_NS_INDICATOR];
		gps->location.longitude = atof(allData[PMTK_RMC_LONGITUDE]);
		gps->location.EW = *allData[PMTK_RMC_EW_INDICATOR];
		gps->speed = atof(allData[PMTK_RMC_SPEED_OVER_GND]);
		gps->heading = atof(allData[PMTK_RMC_COURSE_OVER_GND]);
		uint32_t date = atoi(allData[PMTK_RMC_DATE]);
		gps->time.year = date % 100;
		date /= 100;
		gps->time.month = date % 100;
		date /= 100;
		gps->time.day = date % 100;

	}
	else if (strcmp(allData[0], PMTK_VTG_RETURN) == 0) {

	}

}
