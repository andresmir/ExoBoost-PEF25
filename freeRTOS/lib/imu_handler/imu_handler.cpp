#include "imu_handler.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <Arduino.h>
#include "esp_log.h"
#include "common.h"
#include "Adafruit_BNO08x.h"
#include <Adafruit_Sensor.h>
#include <Wire.h>

extern HardwareSerial Serial;
extern QueueHandle_t imu_telemetry_queue;

TaskHandle_t imu_task_handle = NULL;

uint8_t init_imu() {

    // IMU task on CORE_0 (non-critical telemetry data, uses I2C sensor)
    BaseType_t imu_init = xTaskCreatePinnedToCore(
        imu_task,               // Task function
        "IMU_Task",             // Name of the task
        STACK_SMALL,         // Stack size in words
        NULL,                  // Task input parameter
        PRIORITY_LOW,           // Priority of the task
        &imu_task_handle,       // Task handle
        CORE_0                 // Core where the task should run
    );

    if (imu_init == pdPASS) {
        return 1; 
    } else {
        return 0;
    }
}

void imu_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();

    Adafruit_BNO08x bno08x(BNO08X_RST);
    sh2_SensorValue_t sensorValue;

    // Initialize the BNO08x sensor over I2C, if it fails, print an error and delete the task
    if (!bno08x.begin_I2C()) {
        Serial.println("No BNO08x detected ... Check your wiring or I2C ADDR!");
        vTaskDelete(NULL);
    }

    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 100);

    imu_status_t imu_status;

    float qw = 0;
    float qx = 0;
    float qy = 0;
    float qz = 0;

    // Task loop
    for (;;) {
        if (bno08x.getSensorEvent(&sensorValue)) {
            if (sensorValue.sensorId == SH2_ARVR_STABILIZED_RV) {
            qw = sensorValue.un.arvrStabilizedRV.real;
            qx = sensorValue.un.arvrStabilizedRV.i;
            qy = sensorValue.un.arvrStabilizedRV.j;
            qz = sensorValue.un.arvrStabilizedRV.k;
            }
        }

        imu_status.qw = (int16_t)(qw * 10000);
        imu_status.qx = (int16_t)(qx * 10000);
        imu_status.qy = (int16_t)(qy * 10000);
        imu_status.qz = (int16_t)(qz * 10000);

        // Send IMU status to the telemetry queue
        xQueueOverwrite(imu_telemetry_queue, &imu_status);

        xTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}