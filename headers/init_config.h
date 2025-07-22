#ifndef INIT_CONFIG_H
#define INIT_CONFIG_H

#define OUT_PIN 7

#define BUZZER_PIN 21
#define BUZZER_FREQUENCY 100

#define LED_RED     13  // Pino para LED vermelho do RGB
#define LED_GREEN   11  // Pino para LED verde do RGB
#define LED_BLUE    12  // Pino para LED azul do RGB

#include "pio_matrix.pio.h"
#include "pico/stdlib.h"

uint pio_init(PIO pio);

void buzzer_pwm_config();

void leds_init();

#endif