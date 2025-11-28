#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"
#include "common.h"
#include "can_handler.h"
#include "motor_handler.h"
#include "espnow_handler.h"
#include "imu_handler.h"
#include <esp_now.h>
#include <WiFi.h>


void setup() {

  // Setup CPU frequency
  setCpuFrequencyMhz(240);
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);

  gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
  gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_25, 0);
  gpio_set_level(GPIO_NUM_26, 0);
  gpio_set_level(GPIO_NUM_27, 0);

  /**
   * Initialize all communication protocols:
   * - Serial
   * - I2C
   * - TWAI (CAN)
   * - ESP-NOW
   * If any of the communication protocols fails to initialize, the internal led will blink rapidly, and shortly after
   * reboot the esp32 in an attempt to recover.
   */
  Serial.begin(SERIAL_BAUD_RATE);
  delay(100); // Brief delay for serial initialization

  // Initialize I2C as master with 100kHz
  bool i2c_init = Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_SPEED);

   // TWAI (CAN) configuration at 1 Mbps, with RX alert enabled
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 6;
  g_config.rx_queue_len = 6;
  g_config.alerts_enabled = TWAI_ALERT_RX_DATA; 
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t twai_install = twai_driver_install(&g_config, &t_config, &f_config);
  esp_err_t twai_init = twai_start();

  
  // // ESP-NOW initialization
  uint8_t esp32_unity_mac[] = {0x78, 0x42, 0x1C, 0x6A, 0x90, 0x5C}; // ANDRES ESP32 MAC Address
  // uint8_t esp32_unity_mac[] = {0xC8, 0x2E, 0x18, 0xC3, 0x75, 0xD4}; // C8:2E:18:C3:75:D4

  WiFi.mode(WIFI_STA);
  esp_err_t espnow_init = esp_now_init();
  esp_now_register_recv_cb(onReceive);
  // esp_now_register_send_cb(onSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, esp32_unity_mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_err_t espnow_peer = esp_now_add_peer(&peerInfo);
  Serial.println(WiFi.macAddress());

  // Check if all communication protocols initialized successfully, if not restart system.
  if (twai_install != ESP_OK || twai_init != ESP_OK || espnow_init != ESP_OK || espnow_peer != ESP_OK || !i2c_init) {
    restart_system();
  }

  /**
   * Initialize FreeRTOS tasks:
   * - CAN communication task
   * - IMU reading task
   * - Motor control task
   * - ESP-NOW communication task
   * 
   * Each task will be initialized when calling the init function of each module.
   * The tasks will be pinned to a specific core to optimize performance.
   */
  uint8_t can_init = init_can();
  uint8_t motor_init = init_motor();
  uint8_t espnow_module_init = init_espnow();
  uint8_t imu_init = init_imu();

  if (can_init != 1 || motor_init != 1 || espnow_module_init != 1 || imu_init != 1) {
    restart_system();
  }

  Serial.println("Setup complete");
  gpio_set_level(GPIO_NUM_2, 1);
}

// Main loop does nothing, all operations are handled in FreeRTOS tasks.
void loop() {
  vTaskDelete(NULL);
}





