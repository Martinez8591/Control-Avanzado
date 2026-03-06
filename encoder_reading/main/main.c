#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

#include "encoder.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    encoder_init();

    while (1)
    {
        int32_t count = encoder_get_count();
        float rpm = encoder_get_rpm();

        const char *direction;

        if (rpm > 0)
            direction = "Horario";
        else if (rpm < 0)
            direction = "Antihorario";
        else
            direction = "Detenido";

        ESP_LOGI(TAG,
                 "Conteo: %ld pulsos | RPM: %.2f | Sentido: %s",
                 count,
                 rpm,
                 direction);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TIME_MS));
    }
}