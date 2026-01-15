#include <stdint.h>
#include "xparameters.h"
#include "xgpio.h"

int main() {
    /* runs on softcore MicroBlaze/Nios
        setDirection: output */
    XGpio led;
    XGpio_Initialize(&led, XPAR_LED_DEVICE_ID);
    XGpio_SetDataDirection(&led, 1, 0x0);
    while (1)
        XGpio_DiscreteWrite(&led, 1, 0x1);
        for (volatile int i=0; i<50000000; i++);
        XGpio_DiscreteWrite(&led, 1, 0x0);
        for (volatile int i=0; i<50000000; i++);
}