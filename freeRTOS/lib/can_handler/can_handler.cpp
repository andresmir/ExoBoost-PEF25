#include "can_handler.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <Arduino.h>
#include "esp_log.h"
#include "common.h"

extern HardwareSerial Serial;
extern QueueHandle_t motor_queue_hip;
extern QueueHandle_t motor_queue_knee;
extern QueueHandle_t motor_queue_ankle;

// Queue for sending 
QueueHandle_t can_tx_queue = NULL;

// Task handles
TaskHandle_t can_rx_task_handle = NULL;
TaskHandle_t can_tx_task_handle = NULL;

uint8_t init_can() {

    can_tx_queue = xQueueCreate(6, sizeof(twai_message_t));

    // CAN RX on CORE_0 to balance load and route responses quickly
    BaseType_t can_rx_init = xTaskCreatePinnedToCore(
        can_rx_task,          // Task function
        "CAN_RX_Task",        // Name of the task
        STACK_SMALL,         // Stack size in words
        NULL,                  // Task input parameter
        PRIORITY_MEDIUM,      // Priority of the task
        &can_rx_task_handle,   // Task handle
        CORE_0              // Core where the task should run
    );

    // CAN TX on CORE_1 for tight coupling with motor tasks (eliminates cross-core latency)
    BaseType_t can_tx_init = xTaskCreatePinnedToCore(
        can_tx_task,          // Task function
        "CAN_TX_Task",        // Name of the task
        STACK_SMALL,         // Stack size in words
        NULL,                  // Task input parameter
        PRIORITY_MEDIUM,      // Priority of the task (runs after motor tasks send commands)
        &can_tx_task_handle,   // Task handle
        CORE_1                 // Core where the task should run
    );


    if (can_rx_init == pdPASS && can_tx_init == pdPASS) {
        return 1; 
    } else {
        return 0;
    }
}

void can_rx_task(void *pvParameters) {
    twai_message_t can_msg;
    uint8_t motor_id;

    for (;;) {
        // Non-blocking receive with short timeout to keep task responsive
        if (twai_receive(&can_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            motor_id = can_msg.identifier & 0x0F;
            
            // Use overwrite for single-element queues to always have latest data
            switch (motor_id) {
                case HIP_MOTOR_ID:
                    xQueueOverwrite(motor_queue_hip, &can_msg);
                    break;
                case KNEE_MOTOR_ID:
                    xQueueOverwrite(motor_queue_knee, &can_msg);
                    break;
                case ANKLE_MOTOR_ID:
                    xQueueOverwrite(motor_queue_ankle, &can_msg);
                    break;
                default:
                    // Reduce serial spam in normal operation
                    break;
            }
        }
        // Small yield to prevent hogging CPU
        taskYIELD();
    }
}

void can_tx_task(void *pvParameters) {
    twai_message_t can_msg;

    for (;;) {
        // Wait with timeout instead of blocking forever
         if (xQueueReceive(can_tx_queue, &can_msg, pdMS_TO_TICKS(20)) == pdTRUE) {
            // Transmit with 5ms timeout (enough time for TX buffer)
            if (twai_transmit(&can_msg, pdMS_TO_TICKS(5)) != ESP_OK) {
                Serial.printf("Failed CAN message - ID: 0x%X CMD: 0x%X\n", can_msg.identifier, can_msg.data[0]);
            }
        }
        // Yield to allow other tasks to run
        taskYIELD();
    }
}
