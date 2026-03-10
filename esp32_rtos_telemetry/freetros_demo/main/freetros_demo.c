#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void blink_task(void *pvParameter)
{
    while (1) {
        printf("FreeRTOS task running!\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    xTaskCreate(
        blink_task,
        "blink_task",
        2048,
        NULL,
        5,
        NULL
    );
}
