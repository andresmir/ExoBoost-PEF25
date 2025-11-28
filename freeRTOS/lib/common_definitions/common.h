#ifndef COMMON_DEFINITIONS_H
#define COMMON_DEFINITIONS_H

#include "Arduino.h"

// Define motor IDs 
#define HIP_MOTOR_ID  4
#define KNEE_MOTOR_ID 5
#define ANKLE_MOTOR_ID 6

// // CAN bus pins
// Tipo C mini mia 
// #define CAN_TX_PIN GPIO_NUM_21
// #define CAN_RX_PIN GPIO_NUM_22

// Tipo micro usb
// #define CAN_TX_PIN GPIO_NUM_14
// #define CAN_RX_PIN GPIO_NUM_27

// PCB Pin Out Definitions
// Can bus pins & I2C pins
  #define CAN_TX_PIN GPIO_NUM_5
  #define CAN_RX_PIN GPIO_NUM_4
  #define I2C_SDA_PIN GPIO_NUM_21
  #define I2C_SCL_PIN GPIO_NUM_22


// i2c pins and clock speed 
// #define I2C_SDA_PIN GPIO_NUM_16
// #define I2C_SCL_PIN GPIO_NUM_17
#define I2C_CLOCK_SPEED 100000 // 100kHz

// A 500000 baud rate is used for the serial communication.
// It is a good balance between speed and reliability.
// Higher baud rates may cause data loss or corruption, when tasks run simultaneously
// at the same time intervals. 
#define SERIAL_BAUD_RATE 500000

enum stack_size_t {
    STACK_SMALL = 2048,
    STACK_MEDIUM = 4096,
    STACK_LARGE = 8192
};

enum cpu_core_t {
    CORE_0 = 0,
    CORE_1 = 1
};

enum task_priority_t {
    PRIORITY_LOW = 1,
    PRIORITY_MEDIUM = 2,
    PRIORITY_HIGH = 3
};

struct __attribute__((packed)) system_telemetry_packet {
  uint8_t start;
  int16_t qw,qx,qy,qz;
  int16_t pos[3];            
  int16_t vel[3];            
  int16_t cur[3];            
};

// #define ESP32_SELECT 0xAA // Start byte for the left leg on the exoskeleton
#define ESP32_SELECT 0xBB // Start byte for the right leg on the exoskeleton

struct __attribute__((packed)) control_packet {
  int motorID;
  char mode;
  int intensity;
  int minVal;
  int maxVal;
};

// ESPNOW motor status structure
typedef struct {
  int16_t position;
  int16_t speed;
  int16_t current;
} espnow_motor_status_t;

// ESPNOW IMU status structure
typedef struct {
  float qw;
  float qx;
  float qy;
  float qz;
} imu_status_t;

/**
 * @brief Function to restart the system.
 * This function calls the esp_restart() function to reboot the ESP32.
 */
void restart_system();


#endif // COMMON_DEFINITIONS_H