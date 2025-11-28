#ifndef CAN_HANDLER_H
#define CAN_HANDLER_H

#include "Arduino.h"

#define READ_CMD 0x9C

/**
 * @brief Initializes the CAN (Controller Area Network) task for FreeRTOS, and ISR. 
 * This function creates the CAN task that will get notified by the CAN ISR when a 
 * new message is received. The ISR event ensures that the CAN messages are handled
 * immediately when they arrive, minimizing latency and ensuring timely processing.
 */
uint8_t init_can();

/**
 * @brief CAN RX task.
 * This task is responsible for receiving CAN messages and processing them, and sending
 * them to the appropriate motor queues based on the motor ID in the message.
 * @note This task is normally on the Blocked state, waiting for CAN messages to arrive.
 */
void can_rx_task(void *pvParameters);


/**
 * @brief CAN transmission task.
 * This task handles the transmission of CAN messages.
 * It retrieves messages from a transmission queue and sends them over the CAN bus.
 * @note This task is normally on the Blocked state, waiting for messages to send.
 */
void can_tx_task(void *pvParameters);

#endif // CAN_HANDLER_H