/*
 * JDY18.c
 *
 *  Created on: Dec 2, 2023
 *      Author: gusta
 */

#include "JDY18.h"

UART_HandleTypeDef *husart;

const char *JDY18_CommandPrefixStrings[] = { "PERM", // JDY18_Command_SetConfig
		"DEFAULT", // JDY18_Command_Reset
		"AT+NAME", // JDY18_Command_SetName
		"NAME", // JDY18_Command_GetName
		"ROLE", // JDY18_Command_SetRole
		"BAUD", // JDY18_Command_SetBaudRate
		"INQ" // JDY18_Command_Scan
		};

void JDY18_Setup(UART_HandleTypeDef *handle) {
	husart = handle;
}

void JDY18_SendCommand(JDY18_CommandPrefix_t commandIndex, char *commandParam) {
	const char *commandPrefix = JDY18_CommandPrefixStrings[commandIndex];
	char *command = "AT+";

	int commandSize = strlen(command) + strlen(commandPrefix)
			+ strlen(commandParam) + 3;
	command = malloc(commandSize);
	memset(command, 0, commandSize);

	strcat(command, "AT+");
	strcat(command, commandPrefix);
	strcat(command, commandParam);
	strcat(command, "\r\n");

	HAL_StatusTypeDef status = HAL_UART_Transmit(husart, (uint8_t*) command, commandSize, HAL_MAX_DELAY);
}

void JDY18_GetName(char *name) {
	name = malloc(24);
	memset(name, 0, 24);

	JDY18_SendCommand(JDY18_Command_GetName, "");

	HAL_UART_Receive(husart, (uint8_t*) name, 24, HAL_MAX_DELAY);
}

void JDY18_SetRole(JDY18_Role_t role) {
	char roleStr[2];
	itoa(role, roleStr, 10);

	JDY18_SendCommand(JDY18_Command_SetRole, roleStr);
}

void JDY18_SetBaudRate(JDY18_BaudRate_t baud) {
	char baudStr[5] = "";
	itoa(baud, baudStr, 10);

	JDY18_SendCommand(JDY18_Command_SetBaudRate, baudStr);
}

int JDY18_Scan(JDY18_Device_t *devices) {
//	char *data = (char *) malloc(JDY18_BUFFER_SZ * sizeof(char));
//	memset(data, 0, JDY18_BUFFER_SZ);

	//JDY18_SendCommand(JDY18_Command_Scan, "");

	HAL_Delay(500);

//	char tempBuffer[1];
//	HAL_StatusTypeDef readStatus = HAL_OK;
//	while(!__JDY18_GetScanStrIsComplete(data)) {
//		readStatus = HAL_UART_Receive(husart, tempBuffer, 1, 200);
//		strcat(data, tempBuffer);
//	}

	char *data = "OK\r\n+DEV:1=7FA17C2C055F,-32,PSE2022_B1\r\n+DEV:2=D4680F4C7542,-50,PSE2022_B2\r\n+DEV:3=D4680F4C7572,-50,PSE2022_B3\r\n+DEV:4=583364CA29DE,-82,[LG] webOS TV UJ6565\r\n+STOP:SCAN";

	int numDevices = __JDY18_GetDevicesFromScanStr(data, devices,
	JDY18_MAX_DEVICES);
	__JDY18_GetDistanceFromRssi(devices, numDevices);

	return numDevices;
}

int __JDY18_GetScanStrIsComplete(const char* data) {
	int containsStopStr = strstr(data, JDY18_START_STR) != NULL;
	int containsStartStr = strstr(data, JDY18_STOP_STR) != NULL;
	return containsStopStr && containsStartStr;
}

void __JDY18_GetDistanceFromRssi(JDY18_Device_t *devices, int8_t numDevices) {
	for (int i = 0; i < numDevices; i++) {
		devices[i].distance = pow(10,
				(JDY18_DEFAULT_POWER - devices[i].rssi) / (JDY18_N * 10));
	}
}

int __JDY18_GetDevicesFromScanStr(char *str, JDY18_Device_t *devices,
		int8_t maxDevices) {
	char inputCopy[JDY18_BUFFER_SZ];
	strcpy(inputCopy, str);

	const char *token = strtok(inputCopy, "\r\n");
	int deviceCount = 0;

	while (token != NULL) {
		int lineHasDeviceInfo = strstr(token, "+DEV:") != NULL;
		if (lineHasDeviceInfo) {
			int id, rssi;
			char name[18];

			char *deviceInfoRegex = "+DEV:%d=%*[^,],%d,%18[^\\r\\n]";

			if (sscanf(token, deviceInfoRegex, &id, &rssi, name) == 3) {
				if (deviceCount < maxDevices) {
					devices[deviceCount].id = id;
					devices[deviceCount].rssi = rssi;

					memset(devices[deviceCount].name, 0, 18);
					strcpy(devices[deviceCount].name, name);
					deviceCount++;
				}
			}
		}
		token = strtok(NULL, "\r\n");
	}

	return deviceCount;
}
