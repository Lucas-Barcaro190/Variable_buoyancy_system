# Diagrama de Sequência - Variable Buoyancy System (VBS)

Este documento ilustra a sequência de execução e a interação entre os componentes de software e hardware do projeto VBS, abrangendo a inicialização do FreeRTOS, a recepção de comandos, a malha de controle no **Core 0**, a comunicação entre tarefas e a geração de pulsos PWM via PIO no RP2040.

---

## Diagrama de Sequência de Execução

```mermaid
sequenceDiagram
    autonumber
    actor User as Usuário / PC
    participant Parser as vParserTask (Core 1)
    participant Queue as xMotorCmdQueue
    participant ControlTask as vMotorControlTask (Core 0)
    participant Pot as potentiometer_reading
    participant VelGen as velocity_generator
    participant PID as PIDController
    participant MotorCtrl as motor_control
    participant PIO as State Machine PIO (pwm_pio)
    participant Hardware as Driver de Passo & Pistão

    %% 1. Inicialização
    Note over ControlTask, Parser: Sistema inicializado via initializeHardware() e initializeRTOS()

    %% 2. Envio de Comando
    User->>Parser: Envia comando via UART/USB (ex: "move_pot 250")
    Parser->>Parser: Converte comando para MotorCmd_t
    Parser->>Queue: xQueueSend(xMotorCmdQueue, &cmd)
    
    %% 3. Ciclo Periódico de Controle (20 Hz - Core 0)
    loop Ciclo Periódico de Controle (20 Hz / 50 ms)
        ControlTask->>Pot: read_potentiometer_raw()
        Pot-->>ControlTask: Valor cru do ADC (0-511)
        ControlTask->>Pot: potToPistonPos(current_val)
        Pot-->>ControlTask: Posicão medida h_medido (mm)

        %% Verificação de Novos Comandos
        opt Novo comando disponível na Fila
            ControlTask->>Queue: xQueueReceive(xMotorCmdQueue)
            Queue-->>ControlTask: Retorna MotorCmd_t (target_pot)
            ControlTask->>Pot: potToPistonPos(target_pot)
            Pot-->>ControlTask: h_target (mm)
            
            ControlTask->>VelGen: velocity_generator_start(h_start, h_target, t_now, vmax, accel_time)
            Note over VelGen: Verifica zona morta (<0.15mm)<br/>Define perfil Triangular ou Trapezoidal
            ControlTask->>PID: pid_reset()
        end

        %% Atualização da Trajetória e Malha PID
        alt Se Estado == MOVING
            ControlTask->>VelGen: velocity_generator_update(t_now)
            VelGen-->>ControlTask: TrajectoryPoint_t (href, vref, is_completed)
            
            alt Trajetória concluída e |erro| < 0.2 mm
                ControlTask->>MotorCtrl: stop_stepper_pio()
                MotorCtrl->>PIO: Desabilita SM / Limpa FIFO
                Note over ControlTask: Transita para MCTL_IDLE
            else Movimento em Andamento
                ControlTask->>PID: pid_compute(href, h_medido, dt)
                PID-->>ControlTask: Velocidade calculada v_control (mm/s)
                
                ControlTask->>MotorCtrl: set_stepper_speed_mm_s(v_control)
                MotorCtrl->>MotorCtrl: Calcula direção (GPIO 5) e período X
                Note over MotorCtrl: X = (f_pio / (2 * f_step)) - 3
                MotorCtrl->>PIO: pio_sm_put(period_x)
                
                %% Execução PIO
                PIO->>PIO: pull noblock (carrega X no OSR)
                PIO->>Hardware: Gera PWM 50% duty (GPIO 4 - PULSE)
                Hardware->>Hardware: Desloca Pistão / Altera Volume
            end
        else Se Estado == IDLE
            ControlTask->>MotorCtrl: stop_stepper_pio()
        end
    end

    %% 4. Tratamento de Fim de Curso
    opt Interrupção de Fim de Curso (SW_MIN / SW_MAX)
        Hardware-->>MotorCtrl: IRQ Edge Fall (Chave acionada)
        MotorCtrl->>ControlTask: Set flag_limit_hit = true
        ControlTask->>MotorCtrl: stop_stepper_pio()
        ControlTask->>Queue: xQueueReset()
        Note over ControlTask: Executa rotina de recuo de segurança
    end
```

---

## Descrição do Fluxo dos Componentes

1. **`vParserTask` (Core 1)**:
   - Recebe linhas de texto ou pacotes binários via USB/UART.
   - Converte os comandos (ex: `move_pot 250`) em estruturas `MotorCmd_t` e as envia para a fila `xMotorCmdQueue`.

2. **`xMotorCmdQueue`**:
   - Fila thread-safe do FreeRTOS que desacopla a recepção de comandos do ciclo de controle.

3. **`vMotorControlTask` (Core 0)**:
   - Executa estritamente no **Core 0** em um loop de alta prioridade a 20 Hz (período de 50 ms).
   - Lê a posição atual do atuador via `potentiometer_reading`.
   - Coleta novos alvos da fila e aciona o `velocity_generator`.
   - Executa o cálculo da malha PID em tempo discreto e comanda o acionamento dos passos via `motor_control`.

4. **`velocity_generator`**:
   - Gera os perfis contínuos de aceleração e desaceleração.
   - Aplica a zona morta de $0,15\text{ mm}$ para evitar ruído.
   - Seleciona automaticamente o perfil **Triangular** em deslocamentos curtos e **Trapezoidal** em deslocamentos longos.

5. **`PIDController`**:
   - Calcula o erro $e(t) = h_{ref} - h_{medido}$.
   - Aplica os termos Proporcional, Integral (com anti-windup) e Derivativo, limitando a velocidade de saída.

6. **`motor_control` e `pwm_pio`**:
   - Converte a velocidade de saída do PID ($mm/s$) em frequência de passos ($Hz$) e no valor de temporização $X$.
   - Alimenta o FIFO do PIO, que gera a onda quadrada (PWM 50% duty cycle) sem sobrecarregar a CPU.
