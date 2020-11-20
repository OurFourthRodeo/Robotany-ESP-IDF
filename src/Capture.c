/*
 * Init camera and take a photo.
*/

#include <driver/gpio.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "ArduCAM.h"

// SPI MISO = GPIO 19
#define CAM_MISO_PIN GPIO_NUM_19
// SPI MOSI = GPIO 23
#define CAM_MOSI_PIN GPIO_NUM_23
// SPI clock = GPIO 18
#define CAM_SCK_PIN GPIO_NUM_18

// I2C data = GPIO 21
#define CAM_SDA_PIN GPIO_NUM_21
// I2C clock = GPIO 22
#define CAM_SCL_PIN GPIO_NUM_22
// chip select = GPIO 17
#define CAM_CS_PIN GPIO_NUM_17

spi_bus_config_t connection = {
    .miso_io_num = CAM_MISO_PIN,
    .mosi_io_num = CAM_MOSI_PIN,
    .sclk_io_num = CAM_SCK_PIN,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
    .max_transfer_sz=1
};

// Configure for I2C


// OV2640-specific macros (may want to export to regs.h)
#define OV2640_CHIPID_HIGH  0x0A
#define OV2640_CHIPID_LOW   0x0B

// global variables for writing the buffer
static const size_t bufferSize = 2048;
static uint8_t buffer[bufferSize] = {0xFF};
uint8_t temp = 0, temp_last = 0;
int i = 0;
bool is_header = false;

// specify our camera
// we're using C, so won't be using the class
// (it's also not super helpful with only one camera)

void write_reg(uint8_t addr, uint8_t data){

}

uint8_t read_reg(uint8_t addr){

    return 0x00;
}

void write_sensor_reg(uint8_t addr, uint8_t data){

}

uint8_t read_sensor_reg(uint8_t addr){
    
    return 0x00;
}



void setup_camera() {
    uint8_t vid, pid, temp;

    // TODO: set the CS as an output, turn camera on
    // TODO: configure/start I2C
    // TODO: initialize SPI

    // check if camera is attached (twice?)
    write_reg(ARDUCHIP_TEST1, 0x55);
    temp = read_reg(ARDUCHIP_TEST1);
    if (temp != 0x55)
    {
        // TODO: print error to serial
        while (1)
        ;
    }

    // ensure the camera module is OV2640
    wrSensorReg8_8(0xff, 0x01);
    rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26) && ((pid != 0x41) || (pid != 0x42))) 
        ; // TODO: print to serial "can't find module"
    else 
        ; // TODO: print success to serial
    
    // change capture mode to JPEG and initialize
    cam.set_format(JPEG);
    cam.InitCAM();

    // clear FIFO flag
    cam.OV2640_set_JPEG_size(OV2640_320x240);
}

void start_capture() {
    cam.clear_fifo_flag();
    cam.start_capture();
}

void capture(ArduCAM myCam) {
    uint32_t len = cam.read_fifo_length();

    if (len >= MAX_FIFO_SIZE)
        ; // TODO: print oversize to serial
    if (len == 0)
        ; // TODO: print size 0 to serial
    
    // TODO: set CS low
    
    // TODO: set fifo burst (SPI transfer 0x3c)

    i = 0;
    while (len--)
    {
        temp_last = temp;
        temp = 0; // TODO: SPI.transfer(0x00)

        // if we've reached the end, break the loop
        if ((temp == 0xD9) && (temp_last == 0xFF))
        {
            buffer[i++] = temp; // save the last 0xD9
            // TODO: do something with the data
            is_header = false;
            i = 0;
            // TODO: set CS high
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

