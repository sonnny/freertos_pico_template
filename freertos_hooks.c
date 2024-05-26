#include "FreeRTOS.h" /* Must come first. */
#include "task.h"     /* RTOS task related API prototypes. */

void vApplicationMallocFailedHook(void) {
    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}

// void vApplicationTickHook(void) {
// }

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName ) {
    (void) pcTaskName;
    (void) pxTask;

    /* Force an assert. */
    configASSERT((volatile void *)NULL);
}
