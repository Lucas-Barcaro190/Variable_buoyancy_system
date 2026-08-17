/*
Desc: This is a simple explanation of what this function does.

params:
    - [Type] param_name: Description of the parameter.
    - [Type] param_name: Description of the parameter.
returns:
    - [Type]: Description of the return value.
*/
int exampleFunction(int param1, float param2) {
    // Function implementation goes here.
    return 0; // Example return value.
}



/*
Comentarios aqui

================

- Velocidade do motor: 250 rpm
- Velocidade do fuso: 250 / 61.417 = 4.07 rpm
- Velocidade do potenciômetro: fuso / 2 = 2.035 rpm
- Delta do potenciômetro em 1 segundo: 2.035 / 60 = 0.0339 rps
- 511 unidades do potenciômetro em 10 voltas, ou seja, 51.1 unidades do potenciômetro por rps
- Delta do potenciômetro em 1 segundo: 0.0339 * 51.1 = 1.73 unidades do potenciômetro por segundo
- 20 medições por segundo (50ms de periodo)
- Um filtro de mediana de 7 amostras gera um atraso de 4 amostras, ou seja, 4/20 = 0.2 segundos
- Atraso em unidades do potenciômetro: 0.2 * 1.73 = 0.346
- Atraso é menor que a medição do potenciômetro (que é lido apenas o valor inteiro), então o erro é desprezível.

================

Calculo do alpha do filtro:

- Amostragem = 20 Hz (50ms)
- Quantização do sensor = 0,128mm
- Velocidade máxima do pistão = 0,2035 mm/s
- tau = quantização / velocidade = 0,128 / 0,2035 = 0,629 s
- alpha = exp(-1 / (tau * fs)) = exp(-1 / (0,629 * 20)) = 0,920
*/