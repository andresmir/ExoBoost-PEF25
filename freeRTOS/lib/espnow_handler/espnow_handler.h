#ifndef ESPNOW_HANDLER_H
#define ESPNOW_HANDLER_H

#include "Arduino.h"    

/**
 * @brief Initializes ESP-NOW task and related queues.
 * @return 1 if initialization is successful, 0 otherwise.
 */
uint8_t init_espnow();

/**
 * @brief Task function that handles ESP-NOW transmission.
 * @param pvParameters Pointer to task parameters (not used).
 * This task runs periodically at a fixed interval of 10ms. It is intended to send periodically
 * telemetry data of all sensors through ESP-NOW protocol to the UNITY interface.
 */
void espnow_tx_task(void *pvParameters);

/**
 * @brief Callback function that is called when an ESP-NOW message is received.
 * @param mac_addr Pointer to the MAC address of the sender.
 * @param incomingData Pointer to the received data.
 * @param len Length of the received data.
 * This function processes the incoming control packet and forwards it to the appropriate motor control queue.
 */
void onReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len);


#endif // ESPNOW_HANDLER_H