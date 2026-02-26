#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"


#define LED 2
static uint8_t led_level = 0;

// Inicializador pin digital 
esp_err_t init_led(void){
    gpio_reset_pin(LED);
    esp_err_t err = gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    return err;   
}

esp_err_t blink_led(void){
    led_level = !led_level;
    return gpio_set_level(LED, led_level);
}

void app_main(void){
    ESP_ERROR_CHECK(init_led());
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(6000));
        ESP_ERROR_CHECK(blink_led());

    }
}