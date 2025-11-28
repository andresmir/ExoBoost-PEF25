#include "espnow_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <Arduino.h>
#include "esp_log.h"
#include "common.h"
#include <esp_now.h>
#include <WiFi.h>
#include "driver/twai.h"

extern HardwareSerial Serial;
extern QueueHandle_t motor_control_queue_hip;
extern QueueHandle_t motor_control_queue_knee;
extern QueueHandle_t motor_control_queue_ankle;

QueueHandle_t hip_telemetry_queue = NULL;
QueueHandle_t knee_telemetry_queue = NULL;
QueueHandle_t ankle_telemetry_queue = NULL;
QueueHandle_t imu_telemetry_queue = NULL;

TaskHandle_t espnow_tx_task_handle = NULL;

uint8_t init_espnow() {

    hip_telemetry_queue = xQueueCreate(1, sizeof(espnow_motor_status_t));
    knee_telemetry_queue = xQueueCreate(1, sizeof(espnow_motor_status_t));
    ankle_telemetry_queue = xQueueCreate(1, sizeof(espnow_motor_status_t));
    imu_telemetry_queue = xQueueCreate(1, sizeof(imu_status_t));

    // ESP-NOW TX on CORE_0 for WiFi stack affinity (telemetry, non-critical)
    BaseType_t espnow_tx_init = xTaskCreatePinnedToCore(
        espnow_tx_task,          // Task function
        "ESPNOW_Tx_Task",        // Name of the task
        STACK_SMALL,         // Stack size in words
        NULL,                  // Task input parameter
        PRIORITY_LOW,      // Priority of the task
        &espnow_tx_task_handle,  // Task handle
        CORE_0                 // Core where the task should run
    );

    if (espnow_tx_init == pdPASS) {
        return 1; 
    } else {
        return 0;
    }
}

void espnow_tx_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    espnow_motor_status_t hip_status, knee_status, ankle_status;
    imu_status_t imu_status;

    const uint8_t MOTOR_IDS[3] = {HIP_MOTOR_ID, KNEE_MOTOR_ID, ANKLE_MOTOR_ID};

    system_telemetry_packet telemetry_data = {
        .start = ESP32_SELECT,
        .qw = 0,
        .qx = 0,
        .qy = 0,
        .qz = 0,
        .pos = {0, 0, 0},
        .vel = {0, 0, 0},
        .cur = {0, 0, 0}
    };
    
    twai_message_t can_msg;

    for (;;) {
        // Handle ESP-NOW communication
        if (xQueueReceive(hip_telemetry_queue, &hip_status, pdMS_TO_TICKS(0)) == pdTRUE) {
            telemetry_data.pos[0] = hip_status.position;
            telemetry_data.vel[0] = hip_status.speed;
            telemetry_data.cur[0] = hip_status.current;
        }
        if (xQueueReceive(knee_telemetry_queue, &knee_status, pdMS_TO_TICKS(0)) == pdTRUE) {
            telemetry_data.pos[1] = knee_status.position;
            telemetry_data.vel[1] = knee_status.speed;
            telemetry_data.cur[1] = knee_status.current;
        }
        if (xQueueReceive(ankle_telemetry_queue, &ankle_status, pdMS_TO_TICKS(0)) == pdTRUE) {
            telemetry_data.pos[2] = ankle_status.position;
            telemetry_data.vel[2] = ankle_status.speed;
            telemetry_data.cur[2] = ankle_status.current;
        }
        if (xQueueReceive(imu_telemetry_queue, &imu_status, pdMS_TO_TICKS(0)) == pdTRUE) {
            telemetry_data.qw = imu_status.qw;
            telemetry_data.qx = imu_status.qx;
            telemetry_data.qy = imu_status.qy;
            telemetry_data.qz = imu_status.qz;
        }

        // Send telemetry data via ESP-NOW
        esp_err_t result = esp_now_send(NULL, (uint8_t *)&telemetry_data, sizeof(telemetry_data));
        if (result == ESP_OK) {
            // Serial.println("ESP-NOW telemetry data sent successfully");
            // También imprimir por Serial
            // Serial.printf("IMU:%d,%d,%d,%d | ", telemetry_data.qw, telemetry_data.qx, telemetry_data.qy, telemetry_data.qz);
            // for (int i = 0; i < 3; i++) {
            //     Serial.printf("M%d:%d,%d,%d ", MOTOR_IDS[i], telemetry_data.pos[i], telemetry_data.vel[i], telemetry_data.cur[i]);
            // }
            // Serial.println();
        } else {
            Serial.printf("Error sending ESP-NOW data: %d\n", result);
        }
        
        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
    }
}


void onReceive(const uint8_t *mac_addr, const uint8_t *incoming_data, int len) {
    if (len != sizeof(control_packet)) { return; } // Invalid packet size

    control_packet packet;
    memcpy(&packet, incoming_data, sizeof(packet));

    // Pass the control packet to the motor control task through the appropriate queue
    switch (packet.motorID) {
        case HIP_MOTOR_ID:
            // Control hip motor
            xQueueSend(motor_control_queue_hip, &packet, pdMS_TO_TICKS(1));
            // Serial.print("Motor ID: "); Serial.println(packet.motorID);
            // Serial.print("Mode: ");     Serial.println(packet.mode);
            // Serial.print("Intensity: ");Serial.println(packet.intensity);
            // Serial.print("Min: ");      Serial.println(packet.minVal);
            // Serial.print("Max: ");      Serial.println(packet.maxVal);
            break;
        case KNEE_MOTOR_ID:
            // Control knee motor
            xQueueSend(motor_control_queue_knee, &packet, pdMS_TO_TICKS(1));
            Serial.print("Motor ID: "); Serial.println(packet.motorID);
            Serial.print("Mode: ");     Serial.println(packet.mode);
            Serial.print("Intensity: ");Serial.println(packet.intensity);
            Serial.print("Min: ");      Serial.println(packet.minVal);
            Serial.print("Max: ");      Serial.println(packet.maxVal);
            break;
        case ANKLE_MOTOR_ID:
            // Control ankle motor
            xQueueSend(motor_control_queue_ankle, &packet, pdMS_TO_TICKS(1));
            // Serial.print("Motor ID: "); Serial.println(packet.motorID);
            // Serial.print("Mode: ");     Serial.println(packet.mode);
            // Serial.print("Intensity: ");Serial.println(packet.intensity);
            // Serial.print("Min: ");      Serial.println(packet.minVal);
            // Serial.print("Max: ");      Serial.println(packet.maxVal);
            break;
    }
}

