#include "src/motor/velocity_generator.h"
#include <math.h>

/*
Desc: Initialize a velocity generator object with default motion parameters.
params:
    - [VelocityGenerator_t*] gen: Pointer to the trajectory generator instance.
returns:
    - [void]
*/
void velocity_generator_init(VelocityGenerator_t *gen) {
    if (!gen) return;
    gen->h_start = 0.0f;
    gen->h_target = 0.0f;
    gen->t_start_sec = 0.0f;
    gen->vmax = DEFAULT_VMAX_MM_S;
    gen->accel_time = DEFAULT_ACCEL_TIME_S;
    gen->a_max = gen->vmax / gen->accel_time;
    gen->total_distance = 0.0f;
    gen->dir = 1.0f;
    gen->is_triangular = false;
    gen->t1 = 0.0f;
    gen->t2 = 0.0f;
    gen->t3 = 0.0f;
    gen->t_total = 0.0f;
    gen->v_peak = 0.0f;
    gen->active = false;
    gen->is_deadband = false;
}

/*
Desc: Start a motion trajectory from the current piston position to a target position.
params:
    - [VelocityGenerator_t*] gen: Pointer to the trajectory generator instance.
    - [float] h_start: Starting piston position in mm.
    - [float] h_target: Target piston position in mm.
    - [float] t_now_sec: Current time in seconds.
    - [float] vmax: Desired maximum velocity in mm/s.
    - [float] accel_time: Desired acceleration time in seconds.
returns:
    - [void]
*/
void velocity_generator_start(VelocityGenerator_t *gen, float h_start, float h_target, float t_now_sec, float vmax, float accel_time) {
    if (!gen) return;
    
    gen->h_start = h_start;
    gen->h_target = h_target;
    gen->t_start_sec = t_now_sec;
    
    float delta_h = fabsf(h_target - h_start);
    gen->total_distance = delta_h;
    
    // Verificação de Zona Morta (< 0.15 mm)
    if (delta_h < VELOCITY_DEADBAND_MM) {
        gen->active = false;
        gen->is_deadband = true;
        gen->t_total = 0.0f;
        gen->v_peak = 0.0f;
        return;
    }
    
    gen->is_deadband = false;
    gen->dir = (h_target >= h_start) ? 1.0f : -1.0f;
    
    // Limite de rampa de aceleração entre 2s e 5s
    if (accel_time < 2.0f) accel_time = 2.0f;
    if (accel_time > 5.0f) accel_time = 5.0f;
    gen->accel_time = accel_time;
    
    gen->vmax = (vmax > 0.0f) ? vmax : DEFAULT_VMAX_MM_S;
    gen->a_max = gen->vmax / gen->accel_time;
    
    // Distância mínima necessária para atingir vmax em aceleração e desaceleração
    float d_accel_decel = (gen->vmax * gen->vmax) / gen->a_max;
    
    if (delta_h < d_accel_decel) {
        // Perfil Triangular para deslocamentos curtos
        gen->is_triangular = true;
        gen->v_peak = sqrtf(gen->a_max * delta_h);
        gen->t1 = gen->v_peak / gen->a_max;
        gen->t2 = 0.0f;
        gen->t3 = gen->t1;
        gen->t_total = gen->t1 + gen->t3;
    } else {
        // Perfil Trapezoidal para deslocamentos longos
        gen->is_triangular = false;
        gen->v_peak = gen->vmax;
        gen->t1 = gen->accel_time;
        float d_cruise = delta_h - d_accel_decel;
        gen->t2 = d_cruise / gen->vmax;
        gen->t3 = gen->accel_time;
        gen->t_total = gen->t1 + gen->t2 + gen->t3;
    }
    
    gen->active = true;
}

/*
Desc: Compute the current trajectory reference position and velocity.
params:
    - [VelocityGenerator_t*] gen: Pointer to the active trajectory generator.
    - [float] t_now_sec: Current time in seconds.
returns:
    - [TrajectoryPoint_t]: Current reference position, speed, and completion flag.
*/
TrajectoryPoint_t velocity_generator_update(VelocityGenerator_t *gen, float t_now_sec) {
    TrajectoryPoint_t pt = {0.0f, 0.0f, true};
    
    if (!gen) return pt;
    
    if (gen->is_deadband || !gen->active) {
        pt.href = gen->h_target;
        pt.vref = 0.0f;
        pt.is_completed = true;
        return pt;
    }
    
    float t = t_now_sec - gen->t_start_sec;
    
    if (t <= 0.0f) {
        pt.href = gen->h_start;
        pt.vref = 0.0f;
        pt.is_completed = false;
        return pt;
    }
    
    if (t >= gen->t_total) {
        gen->active = false;
        pt.href = gen->h_target;
        pt.vref = 0.0f;
        pt.is_completed = true;
        return pt;
    }
    
    pt.is_completed = false;
    
    if (gen->is_triangular) {
        if (t <= gen->t1) {
            // Rampa de subida
            pt.vref = gen->dir * (gen->a_max * t);
            pt.href = gen->h_start + gen->dir * (0.5f * gen->a_max * t * t);
        } else {
            // Rampa de descida
            float t_prime = t - gen->t1;
            pt.vref = gen->dir * (gen->v_peak - gen->a_max * t_prime);
            float d_accel = 0.5f * gen->a_max * gen->t1 * gen->t1;
            pt.href = gen->h_start + gen->dir * (d_accel + gen->v_peak * t_prime - 0.5f * gen->a_max * t_prime * t_prime);
        }
    } else {
        if (t <= gen->t1) {
            // Fase 1: Aceleração
            pt.vref = gen->dir * (gen->a_max * t);
            pt.href = gen->h_start + gen->dir * (0.5f * gen->a_max * t * t);
        } else if (t <= (gen->t1 + gen->t2)) {
            // Fase 2: Velocidade constante
            float t_prime = t - gen->t1;
            pt.vref = gen->dir * gen->vmax;
            float d_accel = 0.5f * gen->a_max * gen->t1 * gen->t1;
            pt.href = gen->h_start + gen->dir * (d_accel + gen->vmax * t_prime);
        } else {
            // Fase 3: Desaceleração
            float t_double_prime = t - (gen->t1 + gen->t2);
            pt.vref = gen->dir * (gen->vmax - gen->a_max * t_double_prime);
            float d_accel = 0.5f * gen->a_max * gen->t1 * gen->t1;
            float d_cruise = gen->vmax * gen->t2;
            pt.href = gen->h_start + gen->dir * (d_accel + d_cruise + gen->vmax * t_double_prime - 0.5f * gen->a_max * t_double_prime * t_double_prime);
        }
    }
    
    return pt;
}
