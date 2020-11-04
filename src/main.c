#include <camerabytes.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main() {
	int i = 0;
	while (1) {
		printf("[%d] Hello world!\n", i);
		i++;
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}