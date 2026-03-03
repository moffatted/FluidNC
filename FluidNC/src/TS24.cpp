#include "TS24.h"
#include "ST7789.h"
#include "XPT2046.h"
#include "Machine/MachineConfig.h"
#include "Machine/SPIBus.h"
#include "Report.h"
#include "InputFile.h"
#include <string.h>
#include <Arduino.h>
#include "Serial.h"
#include "string_util.h"

static void ui_task(void* param);

void TS24::init() {
    _display = new ST7789();
    _touch   = new XPT2046();

    // The SPI Bus must already be initialized. FluidNC handles this via Config.
    if (!config->_spi || !config->_spi->defined()) {
        log_error("TS24 Display requires configured SPIBus");
        return;
    }

    pinnum_t cs  = _cs_pin.defined() ? _cs_pin.getNative(Pin::Capabilities::Output) : 255;
    pinnum_t dc  = _dc_pin.defined() ? _dc_pin.getNative(Pin::Capabilities::Output) : 255;
    pinnum_t rst = _reset_pin.defined() ? _reset_pin.getNative(Pin::Capabilities::Output) : 255;
    pinnum_t bl  = _backlight_pin.defined() ? _backlight_pin.getNative(Pin::Capabilities::Output) : 255;

    if (!_display->init(cs, dc, rst, bl)) {
        log_error("Failed to init ST7789 display");
        return;
    }

    pinnum_t tcs = _touch_cs_pin.defined() ? _touch_cs_pin.getNative(Pin::Capabilities::Output) : 255;
    pinnum_t irq = _touch_irq_pin.defined() ? _touch_irq_pin.getNative(Pin::Capabilities::Output) : 255;

    if (!_touch->init(tcs, irq)) {
        log_error("Failed to init XPT2046 touch controller");
        return;
    }

    log_info("TS24 Native SPI Display Initialized");

    // Clear Screen and Draw Initial Layout
    _display->fillScreen(ST7789::BLACK);
    render_ui();

    allChannels.registration(this);
    setReportInterval(200);

    // Start UI task to handle polling touch and redrawing DRO without blocking Stepper cores
    xTaskCreatePinnedToCore(ui_task,
                            "ts24_ui",
                            4096,  // Stack size
                            this,
                            1,  // Priority (Low)
                            NULL,
                            0  // Core 0 (Async / WebUI core, away from stepper core)
    );
}

void TS24::group(Configuration::HandlerBase& handler) {
    handler.item("mosi_pin", _mosi_pin);
    handler.item("miso_pin", _miso_pin);
    handler.item("sck_pin", _sck_pin);
    handler.item("cs_pin", _cs_pin);
    handler.item("dc_pin", _dc_pin);
    handler.item("reset_pin", _reset_pin);
    handler.item("backlight_pin", _backlight_pin);
    handler.item("touch_cs_pin", _touch_cs_pin);
    handler.item("touch_irq_pin", _touch_irq_pin);
}

void TS24::afterParse() {
    // Optional defaulting logic
}

size_t TS24::write(uint8_t data) {
    if (data == '\r')
        return 1;
    if (data == '\n') {
        parse_report();
        _report = "";
        return 1;
    }
    _report += (char)data;
    return 1;
}

Error TS24::pollLine(char* line) {
    return Channel::pollLine(line);
}

void TS24::parse_report() {
    if (_report.length() == 0)
        return;
    if (_report[0] == '<') {
        parse_status_report();
    }
}

void TS24::parse_status_report() {
    // Basic status report parser: <State|MPos:0.00,0.00,0.00|...>
    size_t pos  = 1;
    size_t next = _report.find('|', pos);
    if (next != std::string::npos) {
        _last_state = _report.substr(pos, next - pos);
    }

    size_t mpos_start = _report.find("MPos:", next);
    if (mpos_start == std::string::npos)
        mpos_start = _report.find("WPos:", next);

    if (mpos_start != std::string::npos) {
        mpos_start += 5;
        size_t mpos_end = _report.find('|', mpos_start);
        if (mpos_end == std::string::npos)
            mpos_end = _report.find('>', mpos_start);
        if (mpos_end != std::string::npos) {
            parse_axes(_report.substr(mpos_start, mpos_end - mpos_start), _last_mpos);
        }
    }
}

void TS24::parse_axes(std::string s, float* axes) {
    size_t pos     = 0;
    size_t nextpos = -1;
    int    axis    = 0;
    do {
        nextpos  = s.find_first_of(",", pos);
        auto num = s.substr(pos, nextpos - pos);
        if (axis < 3) {
            string_util::from_float(num, axes[axis++]);
        }
        pos = nextpos + 1;
    } while (nextpos != std::string::npos);
}

// --- UI Layout & Rendering Logic ---

void TS24::render_ui() {
    if (!_display)
        return;

    static std::string prev_state;
    static float       prev_mpos[3] = { -1e9, -1e9, -1e9 };

    bool changed = (prev_state != _last_state);
    for (int i = 0; i < 3; i++) {
        if (fabs(prev_mpos[i] - _last_mpos[i]) > 0.01)
            changed = true;
    }

    if (!changed)
        return;

    prev_state = _last_state;
    memcpy(prev_mpos, _last_mpos, sizeof(prev_mpos));

    // Top Bar (DRO)
    _display->fillRect(0, 0, 320, 40, ST7789::BLUE);
    _display->setTextColor(ST7789::WHITE, ST7789::BLUE);
    _display->drawString(10, 12, ("State: " + _last_state).c_str(), 2);

    // Coordinate Display
    _display->fillRect(0, 40, 320, 40, ST7789::BLACK);
    _display->setTextColor(ST7789::WHITE, ST7789::BLACK);
    char pos_buf[64];
    snprintf(pos_buf, sizeof(pos_buf), "X:%.2f Y:%.2f Z:%.2f", _last_mpos[0], _last_mpos[1], _last_mpos[2]);
    _display->drawString(10, 50, pos_buf, 2);

    // Jog Controls Grid
    int btn_w = 60;
    int btn_h = 50;

    // Y+
    _display->fillRect(60, 90, btn_w, btn_h, ST7789::GRAY);
    _display->setTextColor(ST7789::WHITE, ST7789::GRAY);
    _display->drawString(80, 105, "Y+", 2);

    // X-
    _display->fillRect(0, 140, btn_w, btn_h, ST7789::GRAY);
    _display->drawString(20, 155, "X-", 2);

    // X+
    _display->fillRect(120, 140, btn_w, btn_h, ST7789::GRAY);
    _display->drawString(140, 155, "X+", 2);

    // Y-
    _display->fillRect(60, 190, btn_w, btn_h, ST7789::GRAY);
    _display->drawString(80, 205, "Y-", 2);

    // Z Controls
    _display->fillRect(200, 90, btn_w, btn_h, ST7789::GRAY);  // Z+
    _display->drawString(220, 105, "Z+", 2);
    _display->fillRect(200, 190, btn_w, btn_h, ST7789::GRAY);  // Z-
    _display->drawString(220, 205, "Z-", 2);

    // Zero Controls
    _display->fillRect(260, 90, btn_w, btn_h, ST7789::CYAN);  // Zero XY
    _display->setTextColor(ST7789::BLACK, ST7789::CYAN);
    _display->drawString(270, 105, "0XY", 1);

    _display->fillRect(260, 190, btn_w, btn_h, ST7789::CYAN);  // Zero Z
    _display->drawString(270, 205, "0Z", 1);

    // Bottom Bar (Fixed buttons)
    _display->fillRect(0, 240 - 40, 150, 40, ST7789::GREEN);  // HOME ALL
    _display->setTextColor(ST7789::BLACK, ST7789::GREEN);
    _display->drawString(20, 210, "HOME ALL", 2);

    _display->fillRect(170, 240 - 40, 150, 40, ST7789::RED);  // STOP
    _display->setTextColor(ST7789::WHITE, ST7789::RED);
    _display->drawString(200, 210, "STOP", 2);
}

void TS24::handle_touch() {
    if (!_touch || !_touch->isTouched())
        return;

    // Get point (Width, Height, Calibration MinX, MinY, MaxX, MaxY)
    TouchPoint p = _touch->getPoint(320, 240, 200, 200, 3800, 3800);

    // A simple debouncer
    static uint32_t last_touch = 0;
    if (millis() - last_touch < 300)
        return;
    last_touch = millis();

    // Map Touch to Regions
    if (p.y >= 200) {
        if (p.x < 150) {  // HOME ALL
            push(std::string("$H\n"));
        } else if (p.x > 170) {         // STOP
            push(std::string("\x85"));  // Jog Cancel / Feedhold
        }
    } else if (p.x >= 0 && p.x <= 180) {
        // Jogging Area
        if (p.y >= 90 && p.y <= 140 && p.x >= 60 && p.x <= 120) {  // Y+
            push(std::string("$J=G91 Y1 F1000\n"));
        } else if (p.y >= 140 && p.y <= 190 && p.x >= 0 && p.x <= 60) {  // X-
            push(std::string("$J=G91 X-1 F1000\n"));
        } else if (p.y >= 140 && p.y <= 190 && p.x >= 120 && p.x <= 180) {  // X+
            push(std::string("$J=G91 X1 F1000\n"));
        } else if (p.y >= 190 && p.y <= 240 && p.x >= 60 && p.x <= 120) {  // Y-
            push(std::string("$J=G91 Y-1 F1000\n"));
        }
    } else if (p.x >= 200 && p.x <= 260) {
        // Z Jogging
        if (p.y >= 90 && p.y <= 140) {  // Z+
            push(std::string("$J=G91 Z1 F500\n"));
        } else if (p.y >= 190 && p.y <= 240) {  // Z-
            push(std::string("$J=G91 Z-1 F500\n"));
        }
    } else if (p.x >= 260) {
        // Zeroing
        if (p.y >= 90 && p.y <= 140) {  // Zero XY
            push(std::string("G10 L20 P1 X0 Y0\n"));
        } else if (p.y >= 190 && p.y <= 240) {  // Zero Z
            push(std::string("G10 L20 P1 Z0\n"));
        }
    }
}

static void ui_task(void* param) {
    TS24* ts24 = static_cast<TS24*>(param);
    while (true) {
        // Polling loop. Refresh DRO every 200ms or so, process touch events.
        ts24->handle_touch();

        // This is where we would conditionally call ts24->render_ui() if the _last_state changed.

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Register the module and channel to the system so FluidNC builds it automatically if configured
#include "Channel.h"
#include "Module.h"

ConfigurableModuleFactory::InstanceBuilder<TS24> ts24_module __attribute__((init_priority(105))) ("ts24");
