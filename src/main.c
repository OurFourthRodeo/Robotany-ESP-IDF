#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "CameraFunctions.h"

void capture_image(uint8_t **buffer){
	start_capture();
	// TODO: Read the done flag.
	while(!get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK));
	uint32_t fifo_length = read_fifo_length();
	printf("FIFO Length: %d\n", fifo_length);
	if(fifo_length == 0){
		printf("FIFO length not set.\n");
	}
	
	//*buffer = (uint8_t*) malloc(fifo_length+1);
	
	if(*buffer == NULL){
		printf("Failed to allocate receive buffer.\n");
		while(1);
	}
	uint32_t bufferLength = 0;
	image_read(fifo_length, buffer, &bufferLength);
	
	for(int i = 0; i < bufferLength; i++){
		printf("%02x", (*buffer)[i]);
	}
	printf("\n");
	
	return;
}

void app_main() {
	// init camera
	cam_init();
	vTaskDelay(2000 / portTICK_PERIOD_MS);
	int i = 0;
	// init soft AP
	// init HTTP server
	while (1) {
		printf("[%d] Captured!\n", i);
		i++;
		uint8_t *rxBuf;
		capture_image(&rxBuf);
		free(rxBuf);
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}