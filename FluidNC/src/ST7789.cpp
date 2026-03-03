#include "ST7789.h"
#include "ST7789_Font.h"
#include "Logging.h"
#include "driver/gpio.h"
#include <string.h>

#ifdef CONFIG_IDF_TARGET_ESP32S3
#    define TS_SPI_HOST SPI2_HOST
#else
#    define TS_SPI_HOST HSPI_HOST
#endif

// ST7789 Standard Commands
#define ST7789_SWRESET 0x01
#define ST7789_SLPOUT 0x11
#define ST7789_COLMOD 0x3A
#define ST7789_MADCTL 0x36
#define ST7789_CASET 0x2A
#define ST7789_RASET 0x2B
#define ST7789_RAMWR 0x2C
#define ST7789_INVON 0x21
#define ST7789_NORON 0x13
#define ST7789_DISPON 0x29

// I'll use the font defined in ST7789_Font.h

// I'll remove text drawing for a second and just implement primitives.

ST7789::ST7789() : _spi(nullptr), _cs_pin(255), _dc_pin(255), _reset_pin(255), _backlight_pin(255) {}

ST7789::~ST7789() {
    if (_spi) {
        spi_bus_remove_device(_spi);
    }
}

bool ST7789::init(pinnum_t cs_pin, pinnum_t dc_pin, pinnum_t reset_pin, pinnum_t backlight_pin) {
    _cs_pin        = cs_pin;
    _dc_pin        = dc_pin;
    _reset_pin     = reset_pin;
    _backlight_pin = backlight_pin;

    // Output pins configuration
    gpio_config_t out_conf = {};
    out_conf.pin_bit_mask  = (1ULL << _dc_pin);
    if (_cs_pin != 255)
        out_conf.pin_bit_mask |= (1ULL << _cs_pin);
    if (_reset_pin != 255)
        out_conf.pin_bit_mask |= (1ULL << _reset_pin);
    if (_backlight_pin != 255)
        out_conf.pin_bit_mask |= (1ULL << _backlight_pin);
    out_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&out_conf);

    if (_cs_pin != 255)
        gpio_set_level((gpio_num_t)_cs_pin, 1);
    if (_backlight_pin != 255)
        gpio_set_level((gpio_num_t)_backlight_pin, 1);

    // Hard reset
    if (_reset_pin != 255) {
        gpio_set_level((gpio_num_t)_reset_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level((gpio_num_t)_reset_pin, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level((gpio_num_t)_reset_pin, 1);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

    // Attach to SPI Bus
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz                = 40 * 1000 * 1000;  // 40 MHz
    devcfg.mode                          = 0;                 // SPI mode 0
    devcfg.spics_io_num                  = _cs_pin == 255 ? -1 : _cs_pin;
    devcfg.queue_size                    = 7;
    devcfg.flags                         = SPI_DEVICE_NO_DUMMY;

    esp_err_t ret = spi_bus_add_device(TS_SPI_HOST, &devcfg, &_spi);
    if (ret != ESP_OK) {
        log_error("Failed to add ST7789 to SPI bus");
        return false;
    }

    // Initialization sequence
    sendCommand(ST7789_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(150));

    sendCommand(ST7789_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(500));

    sendCommand(ST7789_COLMOD);
    sendData(0x55);  // 16-bit RGB 5-6-5 format

    sendCommand(ST7789_MADCTL);
    sendData(0x60);  // Landscape rotation (adjust as needed, normally 0x00 for portrait, 0x60 for landscape)

    sendCommand(ST7789_INVON);  // Inversion ON (ST7789 normally needs this)
    vTaskDelay(pdMS_TO_TICKS(10));

    sendCommand(ST7789_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));

    sendCommand(ST7789_DISPON);
    vTaskDelay(pdMS_TO_TICKS(100));

    fillScreen(BLACK);
    return true;
}

void ST7789::sendCommand(uint8_t cmd) {
    gpio_set_level((gpio_num_t)_dc_pin, 0);  // DC Low for Command
    spi_transaction_t t = {};
    t.length            = 8;
    t.tx_buffer         = &cmd;
    spi_device_polling_transmit(_spi, &t);
}

void ST7789::sendData(uint8_t data) {
    gpio_set_level((gpio_num_t)_dc_pin, 1);  // DC High for Data
    spi_transaction_t t = {};
    t.length            = 8;
    t.tx_buffer         = &data;
    spi_device_polling_transmit(_spi, &t);
}

void ST7789::sendData(const uint8_t* data, int len) {
    if (len == 0)
        return;
    gpio_set_level((gpio_num_t)_dc_pin, 1);
    spi_transaction_t t = {};
    t.length            = len * 8;
    t.tx_buffer         = data;
    spi_device_polling_transmit(_spi, &t);
}

void ST7789::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    sendCommand(ST7789_CASET);
    uint8_t d1[4] = { (uint8_t)(x0 >> 8), (uint8_t)(x0 & 0xFF), (uint8_t)(x1 >> 8), (uint8_t)(x1 & 0xFF) };
    sendData(d1, 4);

    sendCommand(ST7789_RASET);
    uint8_t d2[4] = { (uint8_t)(y0 >> 8), (uint8_t)(y0 & 0xFF), (uint8_t)(y1 >> 8), (uint8_t)(y1 & 0xFF) };
    sendData(d2, 4);

    sendCommand(ST7789_RAMWR);
}

void ST7789::pushColors(uint16_t* data, uint32_t len) {
    gpio_set_level((gpio_num_t)_dc_pin, 1);

    // We send in chunks if necessary due to SPI DMA limits,
    // typically max_transfer_sz limits it. FluidNC sets max_transfer_sz to 4000 bytes.
    const uint32_t chunk_words = 2000;  // 4000 bytes
    uint32_t       words_sent  = 0;

    while (words_sent < len) {
        uint32_t to_send = len - words_sent;
        if (to_send > chunk_words)
            to_send = chunk_words;

        spi_transaction_t t = {};
        t.length            = to_send * 16;
        t.tx_buffer         = data + words_sent;
        spi_device_polling_transmit(_spi, &t);
        words_sent += to_send;
    }
}

void ST7789::fillScreen(uint16_t color) {
    fillRect(0, 0, WIDTH, HEIGHT, color);
}

void ST7789::fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if ((x >= WIDTH) || (y >= HEIGHT) || (w == 0) || (h == 0))
        return;
    if ((x + w - 1) >= WIDTH)
        w = WIDTH - x;
    if ((y + h - 1) >= HEIGHT)
        h = HEIGHT - y;

    setAddrWindow(x, y, x + w - 1, y + h - 1);

    // Create a color buffer for 1 line, swap endianness for ST7789 (expects Big Endian over SPI)
    uint16_t swapped = (color >> 8) | (color << 8);
    uint16_t line_buf[WIDTH];
    for (int i = 0; i < w; i++)
        line_buf[i] = swapped;

    gpio_set_level((gpio_num_t)_dc_pin, 1);
    for (int i = 0; i < h; i++) {
        spi_transaction_t t = {};
        t.length            = w * 16;
        t.tx_buffer         = line_buf;
        spi_device_polling_transmit(_spi, &t);
    }
}

void ST7789::drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    fillRect(x, y, w, 1, color);
    fillRect(x, y + h - 1, w, 1, color);
    fillRect(x, y, 1, h, color);
    fillRect(x + w - 1, y, 1, h, color);
}

void ST7789::setTextColor(uint16_t color, uint16_t bg) {
    _fg_color = color;
    _bg_color = bg;
}

void ST7789::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if ((x >= WIDTH) || (y >= HEIGHT))
        return;
    setAddrWindow(x, y, x, y);
    uint16_t swapped = (color >> 8) | (color << 8);
    sendData((uint8_t*)&swapped, 2);
}

void ST7789::drawChar(uint16_t x, uint16_t y, char c) {
    if (c < 32 || c > 127)
        return;

    for (int8_t i = 0; i < 5; i++) {
        uint8_t line = font5x7[(c - 32) * 5 + i];
        for (int8_t j = 0; j < 8; j++, line >>= 1) {
            if (line & 1) {
                if (_text_size == 1) {
                    drawPixel(x + i, y + j, _fg_color);
                } else {
                    fillRect(x + i * _text_size, y + j * _text_size, _text_size, _text_size, _fg_color);
                }
            } else if (_bg_color != _fg_color) {
                if (_text_size == 1) {
                    drawPixel(x + i, y + j, _bg_color);
                } else {
                    fillRect(x + i * _text_size, y + j * _text_size, _text_size, _text_size, _bg_color);
                }
            }
        }
    }
}

void ST7789::drawString(uint16_t x, uint16_t y, const char* str, uint8_t size) {
    _text_size = size;
    int cur_x  = x;
    while (*str) {
        drawChar(cur_x, y, *str);
        cur_x += 6 * size;
        str++;
    }
}
