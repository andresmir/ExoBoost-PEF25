#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include "Arduino.h"

#define BNO08X_RST  2

/**
 * @brief Structure to hold IMU status data.
 * @returns 1 if initialization is successful, 0 otherwise.
 */
uint8_t init_imu();

/**
 * @brief Task to read IMU data and send it to the telemetry queue.
 * @param pvParameters Task parameters (not used).
 */
void imu_task(void *pvParameters);

#endif // IMU_HANDLER_H