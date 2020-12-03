#include "CameraFunctions.h"
#include "freertos/task.h"

static spi_device_handle_t spi = NULL;


esp_err_t cam_init(){
	uint8_t vid, pid, temp;
	spi_bus_config_t spi_cam_bus = {
		.miso_io_num = CAM_MISO_PIN,
		.mosi_io_num = CAM_MOSI_PIN,
		.sclk_io_num = CAM_SCK_PIN,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=MAX_FIFO_SIZE+1,
	};

	spi_device_interface_config_t cam_device_config = {
		.mode = 0,
		.queue_size=1,
		.spics_io_num=CAM_CS_PIN,
		.clock_speed_hz=4*1000*1000, // 4MHz
	};
	printf("Initializing SPI bus.\n");
	esp_err_t ret = spi_bus_initialize(HSPI_HOST, &spi_cam_bus, 1);
	ESP_ERROR_CHECK(ret);
	printf("Initializing SPI device.\n");
	ret = spi_bus_add_device(HSPI_HOST, &cam_device_config, &spi);
	ESP_ERROR_CHECK(ret);
	bus_write(ARDUCHIP_TEST1, 0x55);
	temp = bus_read(ARDUCHIP_TEST1);
	if (temp != 0x55)
	{
		//print error to serial
		printf("Failed to access camera buffer.\n");
		while (1);
	}

	// Initialize I2C
	i2c_config_t i2c_connection = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = CAM_SDA_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = CAM_SCL_PIN,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ
	};
	i2c_param_config(I2C_NUM_0, &i2c_connection);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

	// ensure the camera module is OV2640
	write_sensor_reg(0xff, 0x01);
	read_sensor_reg(OV2640_CHIPID_HIGH, &vid);
	read_sensor_reg(OV2640_CHIPID_LOW, &pid);
	if ((vid != 0x26) && (pid != 0x42)){
		// print to serial "can't find module"
		printf("Camera not OV2640.\n");
	}
	else{
		// print success to serial
		printf("Found OV2640 camera.\n");
	}
	// change capture mode to JPEG and initialize
	init_cam_regs(JPEG);
	
	OV2640_set_JPEG_size(OV2640_320x240);
	
	// clear FIFO flag
	clear_fifo_flag();

	return ret;
}

uint8_t bus_read(uint8_t addr){
	esp_err_t ret;
	uint8_t modelAddr[2]; 
	spi_transaction_t getData;
	modelAddr[0] = (addr & 0x7F);
	modelAddr[1] = 0x00;
	memset(&getData, 0, sizeof(getData));
	getData.length = 16;
	getData.tx_buffer = modelAddr;
	getData.rxlength = 16;
	getData.user=(void*)0;
	getData.flags = SPI_TRANS_USE_RXDATA;
	ret = spi_device_polling_transmit(spi, &getData);
	ESP_ERROR_CHECK(ret);
	return getData.rx_data[1];
}

void image_read(uint32_t fifoLength, uint8_t **rxBuf, uint32_t *rxLen){
	esp_err_t ret;
	uint8_t *txBuf = (uint8_t*) malloc(fifoLength+1);
	uint8_t *tempBuf = (uint8_t*) malloc(fifoLength+1);
	memset(txBuf, 0, fifoLength+1);
	txBuf[0] = 0x3C;

	if(txBuf == NULL){
		printf("Failed to allocate transmit buffer.\n");
		while(1);
	}
	if(tempBuf == NULL){
		printf("Failed to allocate temporary storage buffer.\n");
		while(1);
	}

	spi_transaction_t getImage = {
		.length = (fifoLength+1)*8,
		.tx_buffer = txBuf,
		.user = (void*)0,
		.rx_buffer = tempBuf
	};

	//printf("About to poll...\n");
	ret = spi_device_polling_transmit(spi, &getImage);
	ESP_ERROR_CHECK(ret);

	//printf("Transferring to temp buffer...\n");
	int length = 0;
	uint8_t prev = 0x00;
	bool copy = false;
	for(int i = 0; i < fifoLength+1; i++){
		uint8_t curr = tempBuf[i];
		if(prev == 0xff && curr == 0xd8){
			copy = true;
			tempBuf[length++] = prev;
		}
		if(copy){
			tempBuf[length++] = curr;
			//printf("Byte copied: %02x, Index: %d\n", curr, length);
		}
		if(prev == 0xff && curr == 0xd9 && copy){
			break;
		}
		prev = curr;
	}
	//printf("Copying %d bytes to rx buffer...\n", length);
	*rxBuf = (uint8_t*) malloc(length);
	memcpy((void*)(*rxBuf), (void*)tempBuf, length);
	*rxLen = length;
	//printf("Freeing tx and temp buffers...\n");
	free(txBuf);
	free(tempBuf);
}

esp_err_t bus_write(uint8_t addr, uint8_t data){
	// send transaction and check response
	esp_err_t ret;
	spi_transaction_t setData;
	uint8_t modelAddr[2]; 
	modelAddr[0] = (addr | 0x80);
	modelAddr[1] = data;
	memset(&setData, 0, sizeof(setData));
	setData.length = 16;
	setData.tx_buffer = modelAddr;
	setData.user=(void*)0;
	ret = spi_device_polling_transmit(spi, &setData);
	return ret;
}

// Write to configure camera sensor (I2C)
void write_sensor_reg(uint8_t addr, uint8_t data){
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, I2C_ADDRESS | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, addr, true);
	i2c_master_write_byte(cmd, data, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}


void write_many_sensor_regs(const struct sensor_reg s[]){
	// read in provided array
	// for each value (up until data or addr is 0xff)
		// write_sensor_reg(addr, data)
	int i = 0;
	while(1){
		uint8_t addr = s[i].reg;
		uint8_t data = s[i].val;
		write_sensor_reg(addr, data);
		if((addr == 0xff) && (data == 0xff)){
			break;
		}
		i++;
	}
}


void read_sensor_reg(uint8_t addr, uint8_t *buffer){
	// Set up I2C connection
	// Write addr & 0x00FF
	// Write data & 0x00FF
	// End I2C
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, I2C_ADDRESS | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, addr, true);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 50/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, I2C_ADDRESS | I2C_MASTER_READ, true);
	i2c_master_read_byte(cmd, buffer, I2C_MASTER_NACK);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 50/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

void clear_fifo_flag(){
	bus_write(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void start_capture(){
	clear_fifo_flag();
	bus_write(ARDUCHIP_FIFO, FIFO_START_MASK);
}

int read_fifo_length() {
	uint32_t len1,len2,len3,length=0;
	len1 = bus_read(FIFO_SIZE1);
	len2 = bus_read(FIFO_SIZE2);
	len3 = bus_read(FIFO_SIZE3) & 0x7f;
	length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return length;	
}

void set_fifo_burst() {
	// spi transfer 0x3c
	spi_transaction_t setData;
	uint8_t modelAddr[1]; 
	modelAddr[0] = 0x3C;
	memset(&setData, 0, sizeof(setData));
	setData.length = 8;
	setData.tx_buffer = modelAddr;
	setData.user=(void*)0;
	spi_device_polling_transmit(spi, &setData);
}

void OV2640_set_JPEG_size(){
	//this should be unnecessary, but added to match
	//the arducam code
	//write_many_sensor_regs(OV2640_320x240_JPEG);
}

void init_cam_regs(int format) {
	write_sensor_reg(0xff, 0x01);
	write_sensor_reg(0x12, 0x80);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	if (format == JPEG) {
		write_many_sensor_regs(OV2640_JPEG_INIT);
		write_many_sensor_regs(OV2640_YUV422);
		write_many_sensor_regs(OV2640_JPEG);
		write_sensor_reg(0xff, 0x01);
		write_sensor_reg(0x15, 0x00);
		write_many_sensor_regs(OV2640_320x240_JPEG);
	} else {
		write_many_sensor_regs(OV2640_QVGA);
	}
}

uint8_t get_bit(uint8_t addr, uint8_t bit){
	uint8_t temp;
	temp = bus_read(addr);
	return (temp & bit);
}