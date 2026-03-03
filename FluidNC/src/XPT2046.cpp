#include "XPT2046.h"
#include "Logging.h"
#include "driver/gpio.h"

#ifdef CONFIG_IDF_TARGET_ESP32S3
#    define TS_SPI_HOST SPI2_HOST
#else
#    define TS_SPI_HOST HSPI_HOST
#endif

// Touch commands
#define CMD_X_READ  0xD0
#define CMD_Y_READ  0x90
#define CMD_Z1_READ 0xB0
#define CMD_Z2_READ 0xC0

XPT2046::XPT2046() : _spi(nullptr), _cs_pin(255), _irq_pin(255) {}

XPT2046::~XPT2046() {
    if (_spi) {
        spi_bus_remove_device(_spi);
    }
}

bool XPT2046::init(pinnum_t cs_pin, pinnum_t irq_pin) {
    _cs_pin  = cs_pin;
    _irq_pin = irq_pin;

    if (_cs_pin != 255) {
        gpio_config_t out_conf = {};
        out_conf.pin_bit_mask  = (1ULL << _cs_pin);
        out_conf.mode          = GPIO_MODE_OUTPUT;
        gpio_config(&out_conf);
        gpio_set_level((gpio_num_t)_cs_pin, 1);
    }

    if (_irq_pin != 255) {
        gpio_config_t in_conf = {};
        in_conf.pin_bit_mask  = (1ULL << _irq_pin);
        in_conf.mode          = GPIO_MODE_INPUT;
        in_conf.pull_up_en    = GPIO_PULLUP_ENABLE;
        gpio_config(&in_conf);
    }

    // Attach to SPI Bus - XPT2046 needs much slower SPI clock (1MHz-2MHz max)
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = 2 * 1000 * 1000; // 2 MHz
    devcfg.mode           = 0;
    devcfg.spics_io_num   = _cs_pin == 255 ? -1 : _cs_pin;
    devcfg.queue_size     = 1;
    devcfg.flags          = SPI_DEVICE_HALFDUPLEX;

    esp_err_t ret = spi_bus_add_device(TS_SPI_HOST, &devcfg, &_spi);
    if (ret != ESP_OK) {
        log_error("Failed to add XPT2046 to SPI bus");
        return false;
    }

    // Dummy read to wake up the controller
    transfer16(0x00);
    return true;
}

uint16_t XPT2046::transfer16(uint8_t cmd) {
    uint8_t rx_data[2] = {0, 0};
    
    spi_transaction_t t = {};
    t.length    = 8;  // send 8 bits
    t.rxlength  = 16; // receive 16 bits
    t.tx_buffer = &cmd;
    t.rx_buffer = rx_data;
    
    // We expect the CS pin to be handled automatically by spi_device_polling_transmit
    spi_device_polling_transmit(_spi, &t);
    
    // XPT2046 returns 12 bits data, shifted right by 3
    uint16_t res = (rx_data[0] << 8) | rx_data[1];
    return res >> 3;
}

bool XPT2046::isTouched() {
    if (_irq_pin != 255) {
        return gpio_get_level((gpio_num_t)_irq_pin) == 0;
    }
    // Alternatively check Z pressure
    return getRawPoint().z > 200;
}

TouchPoint XPT2046::getRawPoint() {
    TouchPoint p = {0, 0, 0};

    // Read X
    p.x = transfer16(CMD_X_READ);
    // Read Y
    p.y = transfer16(CMD_Y_READ);
    // Read Z (Pressure approximation)
    int z1 = transfer16(CMD_Z1_READ);
    int z2 = transfer16(CMD_Z2_READ);
    p.z    = z1 - z2; 
    
    return p;
}

TouchPoint XPT2046::getPoint(uint16_t displayWidth, uint16_t displayHeight, uint16_t xMin, uint16_t yMin, uint16_t xMax, uint16_t yMax) {
    TouchPoint raw = getRawPoint();
    TouchPoint cal = {0, 0, raw.z};

    // Calculate scaling
    if (xMax != xMin && yMax != yMin) {
        cal.x = (raw.x - xMin) * displayWidth / (xMax - xMin);
        cal.y = (raw.y - yMin) * displayHeight / (yMax - yMin);
    }

    // Clamp values
    if (cal.x < 0) cal.x = 0;
    if (cal.x >= displayWidth) cal.x = displayWidth - 1;
    if (cal.y < 0) cal.y = 0;
    if (cal.y >= displayHeight) cal.y = displayHeight - 1;

    return cal;
}
