#pragma once

#include "Config.h"
#include "Configuration/Configurable.h"
#include "Channel.h"
#include "Module.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

// Forward declarations for drivers
class ST7789;
class XPT2046;

class TS24 : public Channel, public ConfigurableModule {
public:
    TS24(const char* name) : Channel(name), ConfigurableModule(name) {}
    ~TS24() = default;

    void init() override;

    // Configurable parameters
    Pin _mosi_pin;
    Pin _miso_pin;
    Pin _sck_pin;
    Pin _cs_pin;
    Pin _dc_pin;
    Pin _reset_pin;
    Pin _backlight_pin;
    Pin _touch_cs_pin;
    Pin _touch_irq_pin;

    int32_t _baud_rate = 40000000;  // 40MHz default for ST7789

    // Channel overrides for outputting UI data
    size_t write(uint8_t data) override;
    int    read(void) override { return -1; }
    int    peek(void) override { return -1; }
    Error  pollLine(char* line) override;
    void   flushRx() override {}
    bool   lineComplete(char*, char) override { return false; }
    size_t timedReadBytes(char* buffer, size_t length, TickType_t timeout) override { return 0; }

    // Configuration
    void group(Configuration::HandlerBase& handler) override;
    void validate() override {}
    void afterParse() override;

private:
    ST7789*  _display;
    XPT2046* _touch;

    // UI State caching
    std::string _last_state;
    float       _last_mpos[3] = { 0.0f, 0.0f, 0.0f };
    bool        _is_homed     = false;

    // FreeRTOS queue for thread-safe command delivery from ui_task to polling loop
    QueueHandle_t    _cmdQueue   = nullptr;
    static const int CMD_MAX_LEN = 64;

    std::string _report;
    void        parse_report();
    void        parse_axes(std::string s, float* axes);

public:
    void parse_status_report();
    void render_ui();
    void handle_touch();
};
