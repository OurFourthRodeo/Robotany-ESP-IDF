#ifndef CAMERABYTES_H
#define CAMERABYTES_H
#include "driver/spi_master.h"
#include "driver/gpio.h"

// SPI Class
    // transfer(byte) -- returns response?

#define digitalPinToBitMask(pin)    (1UL << (((pin)>31)?((pin)-32):(pin)))
#define portOutputRegister(port)    ((volatile uint32_t*)((port)?GPIO_OUT1_REG:GPIO_OUT_REG))
#define digitalPinToPort(pin)       (((pin)>31)?1:0)

#define pgm_read_word(addr) ({ \
  typeof(addr) _addr = (addr); \
  *(const unsigned short *)(_addr); \
})

class SPIClass {
private: 
public:
    // set pin to input or output
    void pinMode(uint8_t pin, uint8_t mode);

};

class WireClass {
    uint8_t beginTransmission(int address);
    size_t write(uint8_t data);
    // endTransmission

};


void delay(uint32_t ms) {
    
    //vTaskDelay(ms / portTICK_PERIOD_MS);
}

#endif