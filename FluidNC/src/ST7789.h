#pragma once

#include <stdint.h>
#include "Pin.h"
#include "driver/spi_master.h"

class ST7789 {
public:
    ST7789();
    ~ST7789();

    // Required pins: dc, cs, reset, backlight
    // MOSI, MISO, SCK are handled by Machine::SPIBus initialization
    bool init(Pin* cs_pin, Pin* dc_pin, Pin* reset_pin, Pin* backlight_pin);

    void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
    void pushColors(uint16_t* data, uint32_t len);

    void fillScreen(uint16_t color);
    void drawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
    void drawPixel(uint16_t x, uint16_t y, uint16_t color);

    // Very basic text rendering
    void setTextColor(uint16_t color, uint16_t bg);
    void drawChar(uint16_t x, uint16_t y, char c);
    void drawString(uint16_t x, uint16_t y, const char* str, uint8_t size = 1);

    static const uint16_t WIDTH  = 320;
    static const uint16_t HEIGHT = 240;
    // ST7796 controller has 480x320 RAM; TS24 panel is 320x240.
    // Offsets shift our drawing into the visible portion of the RAM.
    static const uint16_t COL_OFFSET = 0;
    static const uint16_t ROW_OFFSET = 0;

    // Common Colors (RGB565)
    static const uint16_t BLACK   = 0x0000;
    static const uint16_t WHITE   = 0xFFFF;
    static const uint16_t RED     = 0xF800;
    static const uint16_t GREEN   = 0x07E0;
    static const uint16_t BLUE    = 0x001F;
    static const uint16_t YELLOW  = 0xFFE0;
    static const uint16_t CYAN    = 0x07FF;
    static const uint16_t MAGENTA = 0xF81F;
    static const uint16_t GRAY    = 0x8410;

private:
    spi_device_handle_t _spi;
    Pin*                _cs_pin;
    Pin*                _dc_pin;
    Pin*                _reset_pin;
    Pin*                _backlight_pin;

    uint16_t _fg_color  = WHITE;
    uint16_t _bg_color  = BLACK;
    uint8_t  _text_size = 1;

    void sendCommand(uint8_t cmd);
    void sendData(uint8_t data);
    void sendData(const uint8_t* data, int len);
    void clearHardwareRAM();  // Clears full ST7796 480-row RAM to remove power-on garbage
};
