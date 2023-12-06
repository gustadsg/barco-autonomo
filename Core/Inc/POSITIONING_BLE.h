/*
 * POSITIONING_BLE.h
 *
 *  Created on: Dec 5, 2023
 *      Author: gusta
 */

#ifndef INC_POSITIONING_BLE_H_
#define INC_POSITIONING_BLE_H_


#include "JDY18.h"
#include <string.h>

/**
 * @brief Structure representing the known information about a beacon.
 *        It includes the x and y coordinates along with the beacon's name.
 */
typedef struct
{
    // X-coordinate of the beacon.
    float x;
    // Y-coordinate of the beacon.
    float y;
    // Name of the beacon (up to 18 characters).
    char name[18];
} POSITIONING_BLE_Known_Device_Info_t;

/**
 * @brief Structure representing a group of beacons' information needed
 *        to retrieve the current position of the boat.
 */
typedef struct
{
    // Information about the departure beacon.
    POSITIONING_BLE_Known_Device_Info_t departureDevice;
    // Information about the arrival beacon.
    POSITIONING_BLE_Known_Device_Info_t arrivalDevice;
    // Information about another relevant beacon.
    POSITIONING_BLE_Known_Device_Info_t otherDevice;
} POSITIONING_BLE_Devices_Info_t;

/**
 * @brief Represents a point in Cartesian coordinates for positioning using BLE.
 */
typedef struct
{
    // @brief x-coordinate of the point.
    float x;
    // @brief y-coordinate of the point.
    float y;
} POSITIONING_BLE_Cartesian_Point_t;

/**
 * @brief Represents the location of a BLE beacon, including its device information.
 */
typedef struct
{
    // @brief JDY18 BLE device information.
    JDY18_Device_t device;
    // @brief Cartesian coordinates of the beacon.
    POSITIONING_BLE_Cartesian_Point_t point;
} POSITIONING_BLE_Beacon_Location_t;

/**
 * @brief Represents the configuration for BLE-based positioning, including beacon locations.
 */
typedef struct
{
    // @brief Location of the departure beacon.
    POSITIONING_BLE_Beacon_Location_t depart_beacon;
    // @brief Location of the arrival beacon.
    POSITIONING_BLE_Beacon_Location_t arrival_beacon;
    // @brief Location of another beacon.
    POSITIONING_BLE_Beacon_Location_t other_beacon;
} POSITIONING_BLE_Config_t;

/**
 * @brief Gets the current position based on the provided BLE positioning configuration.
 * @param config Pointer to the BLE positioning configuration.
 * @return The Cartesian coordinates representing the current position.
 */
POSITIONING_BLE_Cartesian_Point_t POSITIONING_BLE_GetPosition(POSITIONING_BLE_Config_t *config);

/**
 * @brief Creates a configuration for the positioning ble based on previous devices info and JDY18 scan list.
 * @param config Pointer to the BLE positioning configuration.
 * @param  beaconsInfo Structure composed by previous information about the beacons used as navigation reference.
 * @param  devicesList List of devices scanned by JDY18 module.
 * @param  numOfDevices Number of devices found by JDY18 module.
 */
void POSITIONING_BLE_CreateConfig(POSITIONING_BLE_Config_t *config, POSITIONING_BLE_Devices_Info_t beaconsInfo, JDY18_Device_t *devicesList, int numOfDevices);

/**
 * @brief Defines the angle the boat should make with north in order to achieve target position.
 * @param  currentPoint Cartesian point that describes boat current position.
 * @param  targetPoint 	Cartesian point that describes boat target position.
 * @return The desired angle with north.
 */
float POSITIONING_BLE_CalculateDesiredAngleSetpoint(POSITIONING_BLE_Cartesian_Point_t currentPoint, POSITIONING_BLE_Cartesian_Point_t targetPoint);

#endif /* INC_POSITIONING_BLE_H_ */
