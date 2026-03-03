#include "ST7789.h"
#include "ST7789_Font.h"
#include "Logging.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#ifdef CONFIG_IDF_TARGET_ESP32S3
#    define TS_SPI_HOST SPI2_HOST
#else
#    define TS_SPI_HOST HSPI_HOST
#endif

ST7789::ST7789() : _spi(nullptr), _cs_pin(nullptr), _dc_pin(nullptr), _reset_pin(nullptr), _backlight_pin(nullptr) {}

ST7789::~ST7789() {
    if (_spi) {
        spi_bus_remove_device(_spi);
    }
}

bool ST7789::init(Pin* cs_pin, Pin* dc_pin, Pin* reset_pin, Pin* backlight_pin) {
    _cs_pin        = cs_pin;
    _dc_pin        = dc_pin;
    _reset_pin     = reset_pin;
    _backlight_pin = backlight_pin;

    if (_cs_pin && _cs_pin->defined())
        _cs_pin->setAttr(Pin::Attr::Output);
    if (_dc_pin && _dc_pin->defined())
        _dc_pin->setAttr(Pin::Attr::Output);
    if (_reset_pin && _reset_pin->defined())
        _reset_pin->setAttr(Pin::Attr::Output);
    if (_backlight_pin && _backlight_pin->defined())
        _backlight_pin->setAttr(Pin::Attr::Output);

    // CS high (deselect), backlight off until init done
    if (_cs_pin && _cs_pin->defined())
        _cs_pin->on();

    // Hard reset - follow MKS TS35 pattern exactly
    if (_reset_pin && _reset_pin->defined()) {
        _reset_pin->on();
        vTaskDelay(pdMS_TO_TICKS(120));
        _reset_pin->off();
        vTaskDelay(pdMS_TO_TICKS(120));
        _reset_pin->on();
        vTaskDelay(pdMS_TO_TICKS(120));
    }

    // Attach to SPI Bus - using VSPI pins (18/19/23) so must match FluidNC spi.cpp host
    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz                = 10 * 1000 * 1000;  // 10 MHz
    devcfg.mode                          = 0;                 // SPI mode 0
    devcfg.spics_io_num                  = -1;                // Manual CS
    devcfg.queue_size                    = 7;
    devcfg.flags                         = SPI_DEVICE_HALFDUPLEX;

    esp_err_t ret = spi_bus_add_device(TS_SPI_HOST, &devcfg, &_spi);
    if (ret != ESP_OK) {
        log_error("ST7796: spi_bus_add_device failed");
        return false;
    }
    log_info("ST7796: SPI device added OK");

    spi_device_acquire_bus(_spi, portMAX_DELAY);
    if (_cs_pin && _cs_pin->defined())
        _cs_pin->off();  // Assert CS

    // ST7796 Initialization - exact sequence from MKS DLC32 firmware User_Setup.h
    vTaskDelay(pdMS_TO_TICKS(120));

    sendCommand(0x11);  // Sleep Out
    vTaskDelay(pdMS_TO_TICKS(20));

    sendCommand(0xF0);  // Command Set control - Enable extension command 2 part I
    sendData(0xC3);

    sendCommand(0xF0);  // Command Set control - Enable extension command 2 part II
    sendData(0x96);

    sendCommand(0x36);  // Memory Data Access Control
    sendData(0x68);     // Landscape MX|MV|BGR — only value that renders text correctly on TS24-R

    sendCommand(0x3A);  // Interface Pixel Format
    sendData(0x55);     // 16-bit color

    sendCommand(0xB4);  // Column Inversion
    sendData(0x01);     // 1-dot inversion

    sendCommand(0xB7);  // Entry Mode Set
    sendData(0xC6);

    sendCommand(0xE8);  // Display Output Ctrl Adjust
    sendData(0x40);
    sendData(0x8A);
    sendData(0x00);
    sendData(0x00);
    sendData(0x29);
    sendData(0x19);
    sendData(0xA5);
    sendData(0x33);

    sendCommand(0xC1);  // Power Control 2
    sendData(0x06);

    sendCommand(0xC2);  // Power Control 3
    sendData(0xA7);

    sendCommand(0xC5);  // VCOM Control
    sendData(0x18);

    sendCommand(0xE0);  // Positive Voltage Gamma Control
    sendData(0xF0);
    sendData(0x09);
    sendData(0x0B);
    sendData(0x06);
    sendData(0x04);
    sendData(0x15);
    sendData(0x2F);
    sendData(0x54);
    sendData(0x42);
    sendData(0x3C);
    sendData(0x17);
    sendData(0x14);
    sendData(0x18);
    sendData(0x1B);

    sendCommand(0xE1);  // Negative Voltage Gamma Control
    sendData(0xF0);
    sendData(0x09);
    sendData(0x0B);
    sendData(0x06);
    sendData(0x04);
    sendData(0x03);
    sendData(0x2D);
    sendData(0x43);
    sendData(0x42);
    sendData(0x3B);
    sendData(0x16);
    sendData(0x14);
    sendData(0x17);
    sendData(0x1B);

    sendCommand(0xF0);  // Disable extension command 2 part I
    sendData(0x3C);

    sendCommand(0xF0);  // Disable extension command 2 part II
    sendData(0x69);

    vTaskDelay(pdMS_TO_TICKS(120));

    sendCommand(0x29);  // Display ON
    vTaskDelay(pdMS_TO_TICKS(20));

    if (_cs_pin && _cs_pin->defined())
        _cs_pin->on();  // Release CS
    spi_device_release_bus(_spi);

    log_info("ST7796: Init sequence complete");

    // Turn on backlight
    if (_backlight_pin && _backlight_pin->defined())
        _backlight_pin->on();

    fillScreen(BLACK);  // Clear screen before UI layout is drawn
    return true;
}

void ST7789::sendCommand(uint8_t cmd) {
    if (_dc_pin && _dc_pin->defined())
        _dc_pin->off();
    spi_transaction_t t = {};
    t.flags             = SPI_TRANS_USE_TXDATA;
    t.length            = 8;
    t.tx_data[0]        = cmd;
    spi_device_polling_transmit(_spi, &t);
}

void ST7789::sendData(uint8_t data) {
    if (_dc_pin && _dc_pin->defined())
        _dc_pin->on();
    spi_transaction_t t = {};
    t.flags             = SPI_TRANS_USE_TXDATA;
    t.length            = 8;
    t.tx_data[0]        = data;
    spi_device_polling_transmit(_spi, &t);
}

void ST7789::sendData(const uint8_t* data, int len) {
    if (len == 0)
        return;
    for (int i = 0; i < len; i++)
        sendData(data[i]);
}

void ST7789::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    spi_device_acquire_bus(_spi, portMAX_DELAY);
    if (_cs_pin && _cs_pin->defined())
        _cs_pin->off();

    sendCommand(0x2A);  // CASET
    sendData((uint8_t)((x0 + COL_OFFSET) >> 8));
    sendData((uint8_t)((x0 + COL_OFFSET) & 0xFF));
    sendData((uint8_t)((x1 + COL_OFFSET) >> 8));
    sendData((uint8_t)((x1 + COL_OFFSET) & 0xFF));

    sendCommand(0x2B);  // RASET
    sendData((uint8_t)((y0 + ROW_OFFSET) >> 8));
    sendData((uint8_t)((y0 + ROW_OFFSET) & 0xFF));
    sendData((uint8_t)((y1 + ROW_OFFSET) >> 8));
    sendData((uint8_t)((y1 + ROW_OFFSET) & 0xFF));

    sendCommand(0x2C);  // RAMWR

    if (_cs_pin && _cs_pin->defined())
        _cs_pin->on();
    spi_device_release_bus(_spi);
}

void ST7789::pushColors(uint16_t* data, uint32_t len) {
    if (_dc_pin && _dc_pin->defined())
        _dc_pin->on();

    spi_device_acquire_bus(_spi, portMAX_DELAY);
    if (_cs_pin && _cs_pin->defined())
        _cs_pin->off();

    spi_transaction_t t = {};
    t.length            = len * 16;
    t.tx_buffer         = data;
    spi_device_polling_transmit(_spi, &t);

    if (_cs_pin && _cs_pin->defined())
        _cs_pin->on();
    spi_device_release_bus(_spi);
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

    uint16_t swapped = (color >> 8) | (color << 8);
    // Static buffer - safe for DMA (not on stack)
    static uint16_t line_buf[480];
    for (int i = 0; i < w; i++)
        line_buf[i] = swapped;

    if (_dc_pin && _dc_pin->defined())
        _dc_pin->on();

    spi_device_acquire_bus(_spi, portMAX_DELAY);
    if (_cs_pin && _cs_pin->defined())
        _cs_pin->off();

    for (int i = 0; i < h; i++) {
        spi_transaction_t t = {};
        t.length            = w * 16;
        t.tx_buffer         = line_buf;
        spi_device_polling_transmit(_spi, &t);
    }

    if (_cs_pin && _cs_pin->defined())
        _cs_pin->on();
    spi_device_release_bus(_spi);
}

void ST7789::drawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if ((x >= WIDTH) || (y >= HEIGHT))
        return;
    setAddrWindow(x, y, x, y);
    uint16_t swapped = (color >> 8) | (color << 8);
    sendData((uint8_t)(swapped >> 8));
    sendData((uint8_t)(swapped & 0xFF));
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

void ST7789::drawChar(uint16_t x, uint16_t y, char c) {
    if (c < 32 || c > 127)
        return;
    for (int8_t i = 0; i < 5; i++) {
        uint8_t line = font5x7[(c - 32) * 5 + i];
        for (int8_t j = 0; j < 8; j++, line >>= 1) {
            if (line & 1) {
                if (_text_size == 1)
                    drawPixel(x + i, y + j, _fg_color);
                else
                    fillRect(x + i * _text_size, y + j * _text_size, _text_size, _text_size, _fg_color);
            } else if (_bg_color != _fg_color) {
                if (_text_size == 1)
                    drawPixel(x + i, y + j, _bg_color);
                else
                    fillRect(x + i * _text_size, y + j * _text_size, _text_size, _text_size, _bg_color);
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
