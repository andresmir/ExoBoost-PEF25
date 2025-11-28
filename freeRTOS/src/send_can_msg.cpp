#include "Arduino.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BUTTON_PIN GPIO_NUM_0
#define CAN_RX_PIN GPIO_NUM_22
#define CAN_TX_PIN GPIO_NUM_21

twai_message_t msg_1;
twai_message_t msg_2;

void setup() {
    Serial.begin(500000);
    while (!Serial) { ; }

    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) { Serial.println("TWAI driver instalado"); }
    if (twai_start() == ESP_OK) { Serial.println("TWAI iniciado"); }
}



void loop() {
  
    if (twai_receive(&msg_1, portMAX_DELAY) == ESP_OK) {
        uint8_t motor_id = msg_1.identifier & 0x0F;
        uint8_t command = msg_1.data[0];

        Serial.printf("Received CAN message - ID: 0x%X CMD: 0x%X\n", msg_1.identifier, command);
        
        if (motor_id == 2 && command == 0x9C) {

            msg_2.identifier = 0x242;
            msg_2.data_length_code = 8;
            msg_2.extd = 0;
            msg_2.rtr = 0;
            msg_2.data[0] = 0x9C;
            msg_2.data[1] = 0x32;
            msg_2.data[2] = 0x64;
            msg_2.data[3] = 0x00;
            msg_2.data[4] = 0xF4;
            msg_2.data[5] = 0x01;
            msg_2.data[6] = 0x2D;
            msg_2.data[7] = 0x00;
        
            vTaskDelay(pdMS_TO_TICKS(2));
            twai_transmit(&msg_2, portMAX_DELAY);
        }
        else if (motor_id == 2 && command == 0xA1) {

            msg_2.identifier = 0x242;
            msg_2.data_length_code = 8;
            msg_2.extd = 0;
            msg_2.rtr = 0;
            msg_2.data[0] = 0xA1;
            msg_2.data[1] = 0x32;
            msg_2.data[2] = 0x64;
            msg_2.data[3] = 0x00;
            msg_2.data[4] = 0xF4;
            msg_2.data[5] = 0x01;
            msg_2.data[6] = 0x2D;
            msg_2.data[7] = 0x00;

            twai_transmit(&msg_2, portMAX_DELAY);
        }
    }

}