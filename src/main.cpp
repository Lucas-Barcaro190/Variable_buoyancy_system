#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "src/init/init.h"

int main(void) {
    initializeHardware();
    initializeRTOS();
    //sleep_ms(2000); // Aguarda 2 segundos para estabilizar o sistema antes de iniciar o scheduler
    printf("Starting FreeRTOS scheduler...\n");
    vTaskStartScheduler();

    while (1) {
        printf("[ERROR] vTaskStartScheduler returned!\n");
        sleep_ms(1000);
    }

    return 0;
}
