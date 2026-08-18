#include "src/motor/potentiometer_reading.h"
#include "src/config/hardware_config.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"

#define POT_MEDIAN_COUNT 11

static inline float clampf(float v, float min, float max) {
    return (v < min) ? min : ((v > max) ? max : v);
}

/*
Desc: Clamp a 16-bit unsigned integer between a minimum and maximum.
params:
    - [uint16_t] v: Value to clamp.
    - [uint16_t] min: Minimum allowed value.
    - [uint16_t] max: Maximum allowed value.
returns:
    - [uint16_t]: Clamped value within [min, max].
*/
static inline uint16_t clamp_u16(uint16_t v, uint16_t min, uint16_t max) {
    return (v < min) ? min : ((v > max) ? max : v);
}

/*
Desc: Compute the median value of a small buffer by sorting a temporary copy.
params:
    - [uint16_t[]] buf: Input buffer of potentiometer samples.
    - [int] count: Number of valid samples in the buffer.
returns:
    - [uint16_t]: The median sample value.
*/
static inline uint16_t median_of_buffer(uint16_t buf[], int count) {
    uint16_t temp[POT_MEDIAN_COUNT];
    for (int i = 0; i < count; ++i) {
        temp[i] = buf[i];
    }
    for (int i = 0; i < count - 1; ++i) {
        for (int j = i + 1; j < count; ++j) {
            if (temp[j] < temp[i]) {
                uint16_t swap = temp[i];
                temp[i] = temp[j];
                temp[j] = swap;
            }
        }
    }
    return temp[count / 2];
}

/*
Desc: Initialize the ADC input used for the potentiometer.
params:
    - none
returns:
    - [void]
*/
void potentiometer_init(void) {
    adc_init();
    adc_gpio_init(POT_ADC_PIN);
    adc_select_input(POT_ADC_CHANNEL);
}

/*
Desc: Read the potentiometer ADC multiple times and return the averaged 9-bit value.
params:
    - none
returns:
    - [uint16_t]: Filtered raw potentiometer value in the 0-511 range.
*/
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

/*
Desc: Read the potentiometer and return the median of the last 7 readings.
params:
    - none
returns:
    - [uint16_t]: Median filtered potentiometer value in the 0-511 range.
*/
uint16_t read_potentiometer_median(void) {
    static uint16_t history[POT_MEDIAN_COUNT] = {0};
    static int index = 0;
    static int filled = 0;

    uint16_t raw_value = read_potentiometer_raw();
    history[index] = raw_value;
    index = (index + 1) % POT_MEDIAN_COUNT;
    if (filled < POT_MEDIAN_COUNT) {
        filled++;
    }

    return median_of_buffer(history, filled);
}

/*
Desc: Convert a potentiometer reading into a piston position in millimeters.
params:
    - [uint16_t] pot: Raw potentiometer reading in the 0-511 range.
returns:
    - [float]: Piston position in millimeters, clamped to valid range.
*/
float potToPistonPos(uint16_t pot) {
    pot = clamp_u16(pot, MINIMAL_THRESHOLD, MAXIMUM_THRESHOLD);
    float fraction = 1.0f - ((float)(pot - MINIMAL_THRESHOLD) / POT_RANGE);
    return (fraction * PISTON_RANGE) - MAX_PISTON_POSITION;
}

/*
Desc: Convert a piston position in millimeters back to a potentiometer value.
params:
    - [float] pos_mm: Piston position in millimeters.
returns:
    - [uint16_t]: Estimated potentiometer reading in the 0-511 range.
*/
uint16_t pistonPosToPot(float pos_mm) {
    pos_mm = clampf(pos_mm, -MAX_PISTON_POSITION, MAX_PISTON_POSITION);
    float fraction = (pos_mm + MAX_PISTON_POSITION) / PISTON_RANGE;
    return MAXIMUM_THRESHOLD - (uint16_t)(fraction * POT_RANGE);
}

/*
Desc: Convert piston position to volume, using the configured multiplier.
params:
    - [float] pos_mm: Piston position in millimeters.
returns:
    - [float]: Estimated volume in cubic centimeters.
*/
float pistonPosToVolume(float pos_mm) {
    return clampf(pos_mm * VOL_MULTIPLIER, -MAX_VOLUME, MAX_VOLUME); // tem algo errado na conta
}

/*
Desc: Convert a target volume in cubic centimeters back to a piston position.
params:
    - [float] vol_cm3: Volume in cubic centimeters.
returns:
    - [float]: Piston position in millimeters, clamped to valid range.
*/
float volumeToPistonPos(float vol_cm3) {
    return clampf(vol_cm3 / VOL_MULTIPLIER, -MAX_PISTON_POSITION, MAX_PISTON_POSITION); // tem algo errado na conta
}

/*
Desc: Convert a potentiometer reading directly to volume.
params:
    - [uint16_t] pot: Raw potentiometer reading in the 0-511 range.
returns:
    - [float]: Estimated volume in cubic centimeters.
*/
float potToVolume(uint16_t pot) {
    return pistonPosToVolume(potToPistonPos(pot)); // tem algo errado na conta
}
