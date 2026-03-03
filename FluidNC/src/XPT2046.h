#pragma once

#include <stdint.h>
#include "Pins/PinDetail.h"
#include "driver/spi_master.h"

// Basic point struct
struct TouchPoint {
    int16_t x;
    int16_t y;
    int16_t z; // Pressure
};

class XPT2046 {
public:
    XPT2046();
    ~XPT2046();

    // Required pins: cs_pin, irq_pin (optional but recommended)
    bool init(pinnum_t cs_pin, pinnum_t irq_pin);

    // Returns true if screen is being touched
    bool isTouched();

    // Returns raw ADC values (0-4095)
    TouchPoint getRawPoint();

    // Returns calibrated screen coordinates (mapped to width/height)
    // Needs calibration values
    TouchPoint getPoint(uint16_t displayWidth, uint16_t displayHeight, 
                        uint16_t xMin, uint16_t yMin, uint16_t xMax, uint16_t yMax);

private:
    spi_device_handle_t _spi;
    pinnum_t _cs_pin;
    pinnum_t _irq_pin;
    
    uint16_t transfer16(uint8_t cmd);
};
