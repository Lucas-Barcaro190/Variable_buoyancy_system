#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VELOCITY_DEADBAND_MM      0.15f   // Zona morta de 0.15 mm
#define DEFAULT_VMAX_MM_S         2.0353f // Velocidade máxima (mm/s)
#define DEFAULT_ACCEL_TIME_S      3.0f    // Rampa de aceleração entre 2s e 5s

typedef struct {
    float h_start;        // Posição inicial (mm)
    float h_target;       // Posição alvo (mm)
    float t_start_sec;    // Instante de início da trajetória (s)
    float vmax;           // Velocidade máxima parametrizada (mm/s)
    float accel_time;     // Tempo de aceleração parametrizado (s)
    float a_max;          // Aceleração calculada (mm/s^2)
    
    float total_distance; // Distância total (mm)
    float dir;            // Direção (+1.0f para expansão, -1.0f para retração)
    bool is_triangular;   // true = rampa triangular; false = rampa trapezoidal
    float t1;             // Tempo da rampa de aceleração (s)
    float t2;             // Tempo de velocidade constante (s)
    float t3;             // Tempo da rampa de desaceleração (s)
    float t_total;        // Tempo total do perfil (s)
    float v_peak;         // Velocidade de pico atingida (mm/s)
    
    bool active;          // Trajetória ativa em execução
    bool is_deadband;     // Ativado se deslocamento < 0.15 mm
} VelocityGenerator_t;

typedef struct {
    float href;           // Posição de referência (mm)
    float vref;           // Velocidade de referência (mm/s)
    bool is_completed;    // Indica se o perfil terminou
} TrajectoryPoint_t;

void velocity_generator_init(VelocityGenerator_t *gen);
void velocity_generator_start(VelocityGenerator_t *gen, float h_start, float h_target, float t_now_sec, float vmax, float accel_time);
TrajectoryPoint_t velocity_generator_update(VelocityGenerator_t *gen, float t_now_sec);

#ifdef __cplusplus
}
#endif
