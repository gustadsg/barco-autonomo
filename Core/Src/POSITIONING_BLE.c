/*
 * Positioning_BLE.c
 *
 *  Created on: Nov 19, 2023
 *      Author: João
 */

#include "POSITIONING_BLE.h";

void POSITIONING_BLE_CreateConfig(POSITIONING_BLE_Config_t *config, POSITIONING_BLE_Devices_Info_t beaconsInfo, JDY18_Device_t* devicesList, int numOfDevices) {
	for (int i = 0; i < numOfDevices; i++) {
		const int isDepartureDevice = !strcmp(devicesList[i].name, beaconsInfo.departureDevice.name);
		if(isDepartureDevice) {
			config->depart_beacon.device = devicesList[i];
			config->depart_beacon.point.x = beaconsInfo.departureDevice.x;
			config->depart_beacon.point.y = beaconsInfo.departureDevice.y;

			continue;
		}

		const int isArrivalDevice = !strcmp(devicesList[i].name, beaconsInfo.arrivalDevice.name);
		if(isArrivalDevice) {
			config->arrival_beacon.device = devicesList[i];
			config->arrival_beacon.point.x = beaconsInfo.arrivalDevice.x;
			config->arrival_beacon.point.y = beaconsInfo.arrivalDevice.y;

			continue;
		}

		const int isOtherDevice = !strcmp(devicesList[i].name, beaconsInfo.otherDevice.name);
		if(isOtherDevice) {
			config->other_beacon.device = devicesList[i];
			config->other_beacon.point.x = beaconsInfo.otherDevice.x;
			config->other_beacon.point.y = beaconsInfo.otherDevice.y;

			continue;
		}
	}
}

POSITIONING_BLE_Cartesian_Point_t POSITIONING_BLE_GetPosition(
		POSITIONING_BLE_Config_t *config) {
	const float d1 = config->depart_beacon.device.distance;
	const float d2 = config->arrival_beacon.device.distance;
	const float d3 = config->other_beacon.device.distance;

	const POSITIONING_BLE_Cartesian_Point_t p1 = config->depart_beacon.point;
	const POSITIONING_BLE_Cartesian_Point_t p2 = config->arrival_beacon.point;
	const POSITIONING_BLE_Cartesian_Point_t p3 = config->other_beacon.point;

	const float d1_sq = d1 * d1;
	const float d2_sq = d2 * d2;
	const float d3_sq = d3 * d3;

	const float x1_sq = p1.x * p1.x;
	const float x2_sq = p2.x * p2.x;
	const float x3_sq = p3.x * p3.x;

	const float y1_sq = p1.y * p1.y;
	const float y2_sq = p2.y * p2.y;
	const float y3_sq = p3.y * p3.y;

	const float A = -2 * p1.x + 2 * p2.x;
	const float B = -2 * p1.y + 2 * p2.y;
	const float C = d1_sq - d2_sq - x1_sq + x2_sq - y1_sq + y2_sq;
	const float D = -2 * p2.x + 2 * p3.x;
	const float E = -2 * p2.y + 2 * p3.y;
	const float F = d2_sq - d3_sq - x2_sq + x3_sq - y2_sq + y3_sq;

	POSITIONING_BLE_Cartesian_Point_t result;
	result.x = (C * E - F * B) / (E * A - B * D);
	result.y = (C * D - A * F) / (B * D - A * E);
	return result;
}
;

float POSITIONING_BLE_CalculateDesiredAngleSetpoint(POSITIONING_BLE_Cartesian_Point_t currentPoint, POSITIONING_BLE_Cartesian_Point_t targetPoint) {
    float deltax = currentPoint.x - targetPoint.x;
    float deltay = currentPoint.y - targetPoint.y;

    float distanceToTarget = sqrt(pow(deltax,2)+pow(deltay,2));

    float gamma = asin(fabs(deltay)/distanceToTarget) * 180/M_PI;

    float theta = 90+gamma;

    if(deltax>0) {
        // first quadrant
        if(deltay>0) return 90+gamma;
        // fourth quadrant
        if(deltay<0) return 90-gamma;
    }
    if(deltax<0) {
        // second quadrant
        if(deltay>0) return 270-gamma;
        // third quadrant
        if(deltay<0) return 270+gamma;
    }

    return theta;
}
