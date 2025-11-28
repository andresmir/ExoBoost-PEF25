#include <Arduino.h>
#include "common.h"
#include "driver/gpio.h"
#include "esp_system.h"

void restart_system() {
    for (uint8_t i = 0; i < 10; i++) {
        gpio_set_level(GPIO_NUM_2, 1);
        delay(100);
        gpio_set_level(GPIO_NUM_2, 0);
        delay(100);
    }
    esp_restart();
}