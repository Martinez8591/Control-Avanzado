#ifndef ENCODER_H
#define ENCODER_H

#include "esp_err.h"
#include <stdint.h>

// Pines del encoder
#define ENCODER_A 33
#define ENCODER_B 25

// Resolución del encoder 
#define ENCODER_PPR 11

// Tiempo de muestreo en ms 
#define SAMPLE_TIME_MS 100

// Funciones
esp_err_t encoder_init(void);
int32_t encoder_get_count(void);
float encoder_get_rpm(void);

#endif