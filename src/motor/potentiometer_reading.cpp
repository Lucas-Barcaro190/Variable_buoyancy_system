#include "src/motor/potentiometer_reading.h"
#include "src/config/hardware_config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

static inline float clampf(float v, float min, float max) {
    return (v < min) ? min : ((v > max) ? max : v);
}

static inline uint16_t clamp_u16(uint16_t v, uint16_t min, uint16_t max) {
    return (v < min) ? min : ((v > max) ? max : v);
}

void potentiometer_init(void) {
    adc_init();
    adc_gpio_init(POT_ADC_PIN);
    adc_select_input(POT_ADC_CHANNEL);
}

uint16_t read_potentiometer_raw(void) {
    uint32_t sample_sum = 0;
    for (int i = 0; i < POT_SAMPLE_COUNT; i++) {
        sample_sum += adc_read();
    }
    uint16_t current_val = (uint16_t)(sample_sum / POT_SAMPLE_COUNT);
    // Escala de 12-bit ADC (0-4095) para a faixa de 9-bit (0-511) usada no projeto
    current_val = current_val >> 3;
    return current_val;
}

float potToPistonPos(uint16_t pot) {
    pot = clamp_u16(pot, MINIMAL_THRESHOLD, MAXIMUM_THRESHOLD);
    return (((float)(pot - MINIMAL_THRESHOLD) / POT_RANGE) * PISTON_RANGE) - MAX_PISTON_POSITION; // tem algo errado na conta
}

uint16_t pistonPosToPot(float pos_mm) {
    pos_mm = clampf(pos_mm, -MAX_PISTON_POSITION, MAX_PISTON_POSITION);
    float fraction = (pos_mm + MAX_PISTON_POSITION) / PISTON_RANGE; // tem algo errado na conta
    return MINIMAL_THRESHOLD + (uint16_t)(fraction * POT_RANGE);
}

float pistonPosToVolume(float pos_mm) {
    return clampf(pos_mm * VOL_MULTIPLIER, -MAX_VOLUME, MAX_VOLUME); // tem algo errado na conta
}

float volumeToPistonPos(float vol_cm3) {
    return clampf(vol_cm3 / VOL_MULTIPLIER, -MAX_PISTON_POSITION, MAX_PISTON_POSITION); // tem algo errado na conta
}

float potToVolume(uint16_t pot) {
    return pistonPosToVolume(potToPistonPos(pot)); // tem algo errado na conta
}
