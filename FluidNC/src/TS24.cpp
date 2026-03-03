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

    if (!_display->init(&_cs_pin, &_dc_pin, &_reset_pin, &_backlight_pin)) {
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

    // Clear entire screen on first render to eliminate any init artifacts
    static bool first_render = true;
    if (first_render) {
        _display->fillScreen(ST7789::BLACK);
        first_render = false;
    }

    // Layout for 320x240 (MKS TS24-R)
    // MADCTL 0x68: Y=0 at physical BOTTOM, Y=239 at physical TOP
    const uint16_t W = 320;
    const uint16_t H = 240;

    // --- Header: State (physical top = high Y) ---
    _display->fillRect(0, 210, W, 30, ST7789::BLUE);
    _display->setTextColor(ST7789::WHITE, ST7789::BLUE);
    std::string state_str = _last_state.empty() ? "Idle" : _last_state;
    _display->drawString(8, 217, state_str.c_str(), 2);

    // --- Coord bar (below header = slightly lower Y) ---
    _display->fillRect(0, 182, W, 28, ST7789::BLACK);
    _display->setTextColor(ST7789::GREEN, ST7789::BLACK);
    char pos_buf[64];
    snprintf(pos_buf, sizeof(pos_buf), "X:%.2f Y:%.2f Z:%.2f", _last_mpos[0], _last_mpos[1], _last_mpos[2]);
    _display->drawString(5, 192, pos_buf, 1);

    // --- Jog buttons ---
    const uint16_t BTN_W = 60;
    const uint16_t BTN_H = 35;
    const uint16_t COL1  = 5;    // X- column
    const uint16_t COL2  = 70;   // Center column (Y+/Y-)
    const uint16_t COL3  = 135;  // X+ column
    const uint16_t ZCOL  = 220;  // Z column

    // High Y = physical top of jog area
    const uint16_t ROW_YP = 141;  // Y+ row (physical top)
    const uint16_t ROW_X  = 101;  // X-/X+ row (physical middle)
    const uint16_t ROW_YM = 61;   // Y- row (physical bottom)

    // Y+
    _display->fillRect(COL2, ROW_YP, BTN_W, BTN_H, ST7789::GRAY);
    _display->setTextColor(ST7789::WHITE, ST7789::GRAY);
    _display->drawString(COL2 + 18, ROW_YP + 10, "Y+", 2);

    // X-
    _display->fillRect(COL1, ROW_X, BTN_W, BTN_H, ST7789::GRAY);
    _display->drawString(COL1 + 18, ROW_X + 10, "X-", 2);

    // X+
    _display->fillRect(COL3, ROW_X, BTN_W, BTN_H, ST7789::GRAY);
    _display->drawString(COL3 + 18, ROW_X + 10, "X+", 2);

    // Y-
    _display->fillRect(COL2, ROW_YM, BTN_W, BTN_H, ST7789::GRAY);
    _display->drawString(COL2 + 18, ROW_YM + 10, "Y-", 2);

    // --- Z controls (right side) ---
    _display->fillRect(ZCOL, ROW_YP, BTN_W, BTN_H, 0x0578);  // Teal
    _display->setTextColor(ST7789::WHITE, 0x0578);
    _display->drawString(ZCOL + 18, ROW_YP + 10, "Z+", 2);

    _display->fillRect(ZCOL, ROW_YM, BTN_W, BTN_H, 0x0578);
    _display->drawString(ZCOL + 18, ROW_YM + 10, "Z-", 2);

    // Zero XY (physically above Zero Z = higher Y)
    _display->fillRect(ZCOL, ROW_X + BTN_H / 2, BTN_W, BTN_H / 2 - 2, ST7789::CYAN);
    _display->setTextColor(ST7789::BLACK, ST7789::CYAN);
    _display->drawString(ZCOL + 12, ROW_X + BTN_H / 2 + 3, "0 XY", 1);

    // Zero Z (physically below Zero XY = lower Y)
    _display->fillRect(ZCOL, ROW_X, BTN_W, BTN_H / 2 - 2, ST7789::YELLOW);
    _display->setTextColor(ST7789::BLACK, ST7789::YELLOW);
    _display->drawString(ZCOL + 15, ROW_X + 3, "0 Z", 1);

    // --- Bottom bar (physical bottom = low Y) ---
    _display->fillRect(0, 0, W / 2 - 3, 40, ST7789::GREEN);
    _display->setTextColor(ST7789::BLACK, ST7789::GREEN);
    _display->drawString(20, 12, "HOME ALL", 2);

    _display->fillRect(W / 2 + 3, 0, W / 2 - 3, 40, ST7789::RED);
    _display->setTextColor(ST7789::WHITE, ST7789::RED);
    _display->drawString(W / 2 + 40, 12, "STOP", 2);
}

void TS24::handle_touch() {
    if (!_touch || !_touch->isTouched())
        return;

    // Get point mapped to 320x240 display coordinates
    TouchPoint p = _touch->getPoint(320, 240, 200, 200, 3800, 3800);

    // A simple debouncer
    static uint32_t last_touch = 0;
    if (millis() - last_touch < 300)
        return;
    last_touch = millis();

    // Bottom bar: y >= 200
    if (p.y >= 200) {
        if (p.x < 157) {  // HOME ALL
            push(std::string("$H\n"));
        } else if (p.x >= 163) {        // STOP
            push(std::string("\x85"));  // Jog Cancel / Feedhold
        }
    }
    // XY Jog area: x < 200, y 64..179
    else if (p.x < 200 && p.y >= 64 && p.y < 180) {
        if (p.y >= 64 && p.y < 99 && p.x >= 70 && p.x < 130) {  // Y+
            push(std::string("$J=G91 Y1 F1000\n"));
        } else if (p.y >= 104 && p.y < 139 && p.x >= 5 && p.x < 65) {  // X-
            push(std::string("$J=G91 X-1 F1000\n"));
        } else if (p.y >= 104 && p.y < 139 && p.x >= 135 && p.x < 195) {  // X+
            push(std::string("$J=G91 X1 F1000\n"));
        } else if (p.y >= 144 && p.y < 179 && p.x >= 70 && p.x < 130) {  // Y-
            push(std::string("$J=G91 Y-1 F1000\n"));
        }
    }
    // Z Jog area: x >= 220
    else if (p.x >= 220 && p.x < 280) {
        if (p.y >= 64 && p.y < 99) {  // Z+
            push(std::string("$J=G91 Z1 F500\n"));
        } else if (p.y >= 144 && p.y < 179) {  // Z-
            push(std::string("$J=G91 Z-1 F500\n"));
        } else if (p.y >= 104 && p.y < 122) {  // Zero XY
            push(std::string("G10 L20 P1 X0 Y0\n"));
        } else if (p.y >= 122 && p.y < 139) {  // Zero Z
            push(std::string("G10 L20 P1 Z0\n"));
        }
    }
}

static void ui_task(void* param) {
    TS24* ts24 = static_cast<TS24*>(param);
    while (true) {
        ts24->handle_touch();
        ts24->render_ui();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Register the module and channel to the system so FluidNC builds it automatically if configured
#include "Channel.h"
#include "Module.h"

ConfigurableModuleFactory::InstanceBuilder<TS24> ts24_module __attribute__((init_priority(105))) ("ts24");
