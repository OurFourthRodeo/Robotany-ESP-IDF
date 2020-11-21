/*
 * Init camera and take a photo.
*/

#include "Camerabytes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

static int format = JPEG;

spi_bus_config_t spi_connection = {
    .miso_io_num = CAM_MISO_PIN,
    .mosi_io_num = CAM_MOSI_PIN,
    .sclk_io_num = CAM_SCK_PIN,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=1
};

i2c_config_t i2c_connection = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = CAM_SDA_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = CAM_SCL_PIN,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = I2C_MASTER_FREQ
};

// global variables for writing the buffer
static const size_t bufferSize = 2048;
static uint8_t buffer[bufferSize] = {0xFF};
uint8_t temp = 0, temp_last = 0;
int i = 0;
bool is_header = false;

// specify our camera
// we're using C, so won't be using the class
// (it's also not super helpful with only one camera)

// Write single byte to register (SPI bus)
// Same as bus_write()
void bus_write(uint8_t addr, uint8_t data){
    // set CS low to select chip
    gpio_set_level((1ULL<<GPIO_NUM_17),0);
    // send address
    // send value
    // set CS high to de-select chip
    gpio_set_level((1ULL<<GPIO_NUM_17),0);
}

// Write multiple bytes to registers (SPI bus)
// Same as bus_read()
uint8_t bus_read(uint8_t addr){
    uint8_t value;
    gpio_set_level((1ULL<<GPIO_NUM_17),0);
    // send address
    // get value returned on next
        // value = spi
    // set CS high
    gpio_set_level((1ULL<<GPIO_NUM_17),1);
    return value;
}

// Write to configure camera sensor (I2C)
void write_sensor_reg(uint8_t addr, uint8_t data){
    // Set up I2C connection
    // Write addr & 0x00FF
    // Write data & 0x00FF
    // End I2C
}


void write_many_sensor_regs(struct sensor_reg s[]){
    // read in provided array
    // for each value (up until data or addr is 0xff)
        // write_sensor_reg(addr, data)
}

void read_sensor_reg(uint8_t addr, uint8_t *buffer){
    // Set up I2C connection
    // Write addr & 0x00FF
    // Write data & 0x00FF
    // End I2C
}

void clear_fifo_flag(){
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
}

void start_capture() {
    clear_fifo_flag();
    write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
}

int read_fifo_length(){
    uint32_t len1,len2,len3,length=0;
	len1 = read_reg(FIFO_SIZE1);
	len2 = read_reg(FIFO_SIZE2);
	len3 = read_reg(FIFO_SIZE3) & 0x7f;
	length = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
	return length;	
}

void OV2640_set_JPEG_size(){
    write_many_sensor_regs(OV2640_320x240);
}

void init_cam_regs(void) {
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


void setup_camera(int format) {
    uint8_t vid, pid, temp;
    esp_err_t ret;

    // set the CS as an output
    gpio_config_t cs_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = ((1ULL<<GPIO_NUM_17)),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&cs_conf);

    // TODO: configure/start I2C
  
    // TODO: initialize SPI
    ret = spi_bus_initialize(SPI2_HOST,&spi_connection,0);
    ESP_ERROR_CHECK(ret);

    // check if camera is attached (twice?)
    bus_write(ARDUCHIP_TEST1, 0x55);
    temp = bus_read(ARDUCHIP_TEST1);
    if (temp != 0x55)
    {
        // TODO: print error to serial
        while (1)
        ;
    }

    // ensure the camera module is OV2640
    write_sensor_reg(0xff, 0x01);
    read_sensor_reg(OV2640_CHIPID_HIGH, &vid);
    read_sensor_reg(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) 
        ; // TODO: print to serial "can't find module"
    else 
        ; // TODO: print success to serial
    
    // change capture mode to JPEG and initialize
    format = JPEG;
    init_cam_regs();
    
    OV2640_set_JPEG_size(OV2640_320x240);
    
    // clear FIFO flag
    clear_fifo_flag();

}

void capture_task() {
    uint32_t len = read_fifo_length();

    if (len >= MAX_FIFO_SIZE)
        ; // TODO: print oversize to serial
    if (len == 0)
        ; // TODO: print size 0 to serial
    
    gpio_set_level((1ULL<<GPIO_NUM_17),0);
    
    // TODO: set fifo burst (SPI transfer 0x3c)

    i = 0;
    while (len--)
    {
        temp_last = temp;
        temp = 0; 
        // TODO: SPI.transfer(0x00)

        // if we've reached the end, break the loop
        if ((temp == 0xD9) && (temp_last == 0xFF))
        {
            buffer[i++] = temp; // save the last 0xD9
            // TODO: do something with the data
            is_header = false;
            i = 0;
            gpio_set_level((1ULL<<GPIO_NUM_17),1);
            break;
        }
        if (is_header == true)
        {
            // white image data to buffer if not full
            if (i < bufferSize)
                buffer[i++] = temp;
            else 
            {
                // TODO: write to client
                i = 0;
                buffer[i++] = temp;
            }
        }
        else if ((temp == 0xD8) && (temp_last == 0xFF))
        {
            is_header = true;
            buffer[i++] = temp_last;
            buffer[i++] = temp;
        }
    }
}

