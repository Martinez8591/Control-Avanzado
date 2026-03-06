#include "encoder.h"
#include "driver/pcnt.h"
#include "esp_log.h"

static const char *TAG = "ENCODER";

static int16_t last_count = 0;

esp_err_t encoder_init(void)
{
    pcnt_config_t pcnt_A = {
        .pulse_gpio_num = ENCODER_A,
        .ctrl_gpio_num = ENCODER_B,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768
    };

    pcnt_config_t pcnt_B = {
        .pulse_gpio_num = ENCODER_B,
        .ctrl_gpio_num = ENCODER_A,
        .channel = PCNT_CHANNEL_1,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_DEC,
        .neg_mode = PCNT_COUNT_INC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768
    };

    pcnt_unit_config(&pcnt_A);
    pcnt_unit_config(&pcnt_B);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);

    ESP_LOGI(TAG, "Encoder inicializado en modo x4");

    return ESP_OK;
}

int32_t encoder_get_count(void)
{
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);
    return count;
}

float encoder_get_rpm(void)
{
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count);

    int16_t delta = count - last_count;
    last_count = count;

    float pulses_per_sec = (float)delta * (1000.0 / SAMPLE_TIME_MS);

    float rpm = (pulses_per_sec * 60.0) / (ENCODER_PPR * 4);

    return rpm;
}