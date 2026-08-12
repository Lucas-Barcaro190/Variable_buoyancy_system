#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Inicialização e amostragem do ADC do potenciômetro
void potentiometer_init(void);
uint16_t read_potentiometer_raw(void);
uint16_t read_potentiometer_median(void);

// Funções de conversão de posição e volume
float potToPistonPos(uint16_t pot);
uint16_t pistonPosToPot(float pos_mm);
float pistonPosToVolume(float pos_mm);
float volumeToPistonPos(float vol_cm3);
float potToVolume(uint16_t pot);

#ifdef __cplusplus
}
#endif
