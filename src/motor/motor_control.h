#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "src/core/shared_state.h"

void setup_stepper_pio(void);
void send_stepper_pulses(uint32_t count);
bool is_stepper_busy(void);
void wait_stepper_done(void);
void stop_stepper_pio(void);
void sendStopCommand(void);
void sendEnableCommand(void);
void sendDisableCommand(void);
void setup_limit_switches_on_core1(void);
void gpio_limit_switches_callback(uint gpio, uint32_t events);
void evaluate_pending_limit_switches(void);
float potToPistonPos(uint16_t pot);
uint16_t pistonPosToPot(float pos_mm);
float pistonPosToVolume(float pos_mm);
float volumeToPistonPos(float vol_cm3);
