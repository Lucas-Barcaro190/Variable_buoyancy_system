// src/helloworld.cpp
/**
 * Test program for verifying UART stdio communication in RP2040 emulator.
 */

#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();

    printf("\n=== HELLO WORLD START ===\n");
    fflush(stdout);

    while (1) {
        printf("Hello world from RP2040 ARM Emulator!\n");
        fflush(stdout);

        for (volatile int i = 0; i < 500000; i++) {
            // CPU delay loop without hardware timer dependency
        }
    }

    return 0;
}
