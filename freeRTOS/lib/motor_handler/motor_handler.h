/**
 * @file motor_handler.h
 * @brief Header file for controlling motors using FreeRTOS tasks.
 * 
 * This module defines the interface for initializing motor control tasks
 * and handling motor status updates via CAN messages. It includes task
 * function prototypes and data structures used for motor control.
 */
#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H

#include "driver/twai.h"
#include "common.h"

#define MAX_SPEED 300  // Maximum motor speed for control calculations  

/**
 * @brief Structure to hold the status of a motor.
 * Includes motor ID, current, speed, position, and timestamp.
 * The timestamp can be used for tracking when the status was last updated.
 */
typedef struct {
    uint8_t id;
    int16_t current;
    int16_t speed;
    int16_t position;
} motor_status_t;

/**
 * @brief Structure to hold motor control commands.
 * Includes mode, intensity, minimum and maximum values for control.
 */
typedef struct {
    char mode;
    int16_t intensity;
    int16_t minVal;
    int16_t maxVal;
} motor_control_t;


/**
 * @brief Initializes 3 motor tasks.
 * Each task will control either the hip, knee or ankle motor.
 * When calling this function, the tasks and queues for each task will be created.
 */
uint8_t init_motor();

#endif