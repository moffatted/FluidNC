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
#include "RealtimeCmd.h"

static void ui_task(void* param);

void TS24::init() {
    log_info("TS24: build " __DATE__ " " __TIME__);
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
    pinnum_t irq = _touch_irq_pin.defined() ? _touch_irq_pin.getNative(Pin::Capabilities::Input) : 255;

    log_info("TS24 Initializing Touch: CS=" << (int)tcs << " IRQ=" << (int)irq);

    if (!_touch->init(tcs, irq)) {
        log_error("Failed to init XPT2046 touch controller");
        return;
    }

    log_info("TS24 Native SPI Display Initialized");

    // Clear Screen and Draw Initial Layout
    _display->fillScreen(ST7789::BLACK);
    render_ui();

    // Create FreeRTOS queue for thread-safe command delivery from ui_task to polling loop
    _cmdQueue = xQueueCreate(4, CMD_MAX_LEN);

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
    // Check FreeRTOS command queue first (thread-safe cross-core delivery)
    if (line && _cmdQueue) {
        char cmd[CMD_MAX_LEN];
        if (xQueueReceive(_cmdQueue, cmd, 0) == pdTRUE) {
            // Check for realtime commands (single byte, >= 0x80 or specific chars)
            if (cmd[1] == '\0' && is_realtime_command((uint8_t)cmd[0])) {
                handleRealtimeCharacter((uint8_t)cmd[0]);
                return Error::NoData;
            }
            strncpy(line, cmd, Channel::maxLine);
            // Strip trailing \n and \r (lineComplete normally does this)
            size_t len = strlen(line);
            while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
                line[--len] = '\0';
            }
            return Error::Ok;
        }
    }
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

    size_t homed_start = _report.find("H:", next);
    if (homed_start != std::string::npos) {
        homed_start += 2;
        _is_homed = (_report[homed_start] != '0' && _report[homed_start] != '|' && _report[homed_start] != '>');
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

    bool changed = (prev_state != _last_state) || _force_redraw;
    for (int i = 0; i < 3; i++) {
        if (fabs(prev_mpos[i] - _last_mpos[i]) > 0.01)
            changed = true;
    }

    if (!changed)
        return;

    _force_redraw = false;
    prev_state    = _last_state;
    memcpy(prev_mpos, _last_mpos, sizeof(prev_mpos));

    // Clear entire screen on first render to eliminate any init artifacts
    static bool first_render = true;
    if (first_render) {
        _display->fillScreen(ST7789::BLACK);
        first_render = false;
    }

    // Layout for 320x240 (MKS TS24-R)
    const uint16_t W = 320;
    const uint16_t H = 240;

    // --- Header: State ---
    _display->fillRect(0, 215, W, 25, ST7789::BLUE);
    _display->setTextColor(ST7789::WHITE, ST7789::BLUE);
    std::string state_str = _last_state.empty() ? "Idle" : _last_state;
    _display->drawString(8, 220, state_str.c_str(), 2);

    // --- Coord bar ---
    _display->fillRect(0, 192, W, 23, ST7789::BLACK);
    _display->setTextColor(ST7789::GREEN, ST7789::BLACK);
    char pos_buf[64];
    snprintf(pos_buf, sizeof(pos_buf), "X:%.2f Y:%.2f Z:%.2f", _last_mpos[0], _last_mpos[1], _last_mpos[2]);
    _display->drawString(5, 196, pos_buf, 1);

    // --- Distance buttons ---
    const char*    dist_labels[5] = { ".1", "1", "5", "10", "100" };
    const uint16_t DIST_W         = 60;
    const uint16_t DIST_H         = 28;
    const uint16_t ROW_DIST       = 160;

    for (int i = 0; i < 5; i++) {
        uint16_t dcol       = 5 + i * (DIST_W + 4);
        uint16_t color      = (i == _jog_dist_idx) ? ST7789::GREEN : ST7789::GRAY;
        uint16_t text_color = (i == _jog_dist_idx) ? ST7789::BLACK : ST7789::WHITE;
        _display->fillRect(dcol, ROW_DIST, DIST_W, DIST_H, color);
        _display->setTextColor(text_color, color);
        int offset[] = { 19, 24, 24, 14, 8 };  // Fine-tuned offsets for 2-scale font roughly (~11px per char)
        _display->drawString(dcol + offset[i], ROW_DIST + 6, dist_labels[i], 2);
    }

    // --- Jog buttons ---
    const uint16_t BTN_W = 60;
    const uint16_t BTN_H = 34;  // Reduced from 35
    const uint16_t COL1  = 5;
    const uint16_t COL2  = 70;
    const uint16_t COL3  = 135;
    const uint16_t ZCOL  = 220;

    const uint16_t ROW_YP = 45;   // Moved up from 61
    const uint16_t ROW_X  = 83;   // Moved up from 101
    const uint16_t ROW_YM = 121;  // Moved up from 141

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

    // Z controls
    _display->fillRect(ZCOL, ROW_YP, BTN_W, BTN_H, 0x0578);
    _display->setTextColor(ST7789::WHITE, 0x0578);
    _display->drawString(ZCOL + 18, ROW_YP + 10, "Z+", 2);

    _display->fillRect(ZCOL, ROW_YM, BTN_W, BTN_H, 0x0578);
    _display->drawString(ZCOL + 18, ROW_YM + 10, "Z-", 2);

    // Zero XY
    _display->fillRect(ZCOL, ROW_X, BTN_W, BTN_H / 2 - 2, ST7789::CYAN);
    _display->setTextColor(ST7789::BLACK, ST7789::CYAN);
    _display->drawString(ZCOL + 12, ROW_X + 3, "0 XY", 1);

    // Zero Z
    _display->fillRect(ZCOL, ROW_X + BTN_H / 2, BTN_W, BTN_H / 2 - 2, ST7789::YELLOW);
    _display->setTextColor(ST7789::BLACK, ST7789::YELLOW);
    _display->drawString(ZCOL + 15, ROW_X + BTN_H / 2 + 3, "0 Z", 1);

    // --- Top bar ---
    const uint16_t TOP_BTN_W = 104;

    uint16_t home_color = ST7789::BLUE;
    uint16_t home_text  = ST7789::WHITE;
    if (_last_state.find("Home") != std::string::npos) {
        home_color = ST7789::YELLOW;
        home_text  = ST7789::BLACK;
    } else if (_is_homed) {
        home_color = ST7789::GREEN;
        home_text  = ST7789::BLACK;
    }

    _display->fillRect(0, 0, TOP_BTN_W, 40, home_color);
    _display->setTextColor(home_text, home_color);
    _display->drawString(29, 13, "HOME", 2);

    _display->fillRect(TOP_BTN_W + 4, 0, TOP_BTN_W, 40, ST7789::RED);
    _display->setTextColor(ST7789::WHITE, ST7789::RED);
    _display->drawString(TOP_BTN_W + 4 + 29, 13, "STOP", 2);

    bool     is_alarm     = (_last_state.find("Alarm") != std::string::npos);
    uint16_t unlock_color = is_alarm ? ST7789::RED : ST7789::GRAY;
    uint16_t unlock_text  = is_alarm ? ST7789::WHITE : ST7789::BLACK;

    _display->fillRect((TOP_BTN_W + 4) * 2, 0, TOP_BTN_W, 40, unlock_color);
    _display->setTextColor(unlock_text, unlock_color);
    _display->drawString(((TOP_BTN_W + 4) * 2) + 23, 13, "ALARM", 2);
}

void TS24::handle_touch() {
    if (!_touch)
        return;

    // Read raw point for diagnostics
    TouchPoint raw     = _touch->getRawPoint();
    bool       touched = (raw.z > 100 && raw.z < 4000);

    // Periodic idle-state log (every 2s)
    static uint32_t last_log = 0;
    if (millis() - last_log > 2000) {
        last_log = millis();
        log_info("TS24 POLL: rawX=" << raw.x << " rawY=" << raw.y << " Z=" << raw.z << " touched=" << touched);
    }

    if (!touched)
        return;

    // Get point mapped to 320x240 display coordinates
    TouchPoint p = _touch->getPoint(320, 240, 550, 600, 3650, 3450);

    log_info("TS24 TOUCH: raw(" << raw.x << "," << raw.y << "," << raw.z << ") -> screen(" << p.x << "," << p.y << ")");

    // Determine touch region for fast response tracking
    int cur_region = -1;
    if (p.y < 45)
        cur_region = 0;   // Top bar
    else if (p.y >= 210)  // State header is 215, giving leeway to 210
        cur_region = 1;   // State header
    else if (p.y >= 155 && p.y <= 192) {
        // Distance buttons
        int btn_idx = (p.x - 5) / 64;  // 64 spacing (60 w + 4 space)
        if (btn_idx >= 0 && btn_idx < 5) {
            cur_region = 30 + btn_idx;
        }
    } else if (p.x < 200) {
        if (p.y >= 45 && p.y <= 79)
            cur_region = 10;  // Y+
        else if (p.y >= 83 && p.y <= 117)
            cur_region = (p.x < 65) ? 11 : (p.x >= 135 ? 12 : -1);
        else if (p.y >= 121 && p.y <= 155)
            cur_region = 13;  // Y-
    } else if (p.x >= 210) {
        if (p.y >= 45 && p.y <= 79)
            cur_region = 20;  // Z+
        else if (p.y >= 121 && p.y <= 155)
            cur_region = 21;  // Z-
        else if (p.y >= 83 && p.y < 100)
            cur_region = 22;  // Zero XY
        else if (p.y >= 100 && p.y <= 117)
            cur_region = 23;  // Zero Z
    }

    uint32_t now     = millis();
    bool     is_jog  = (cur_region >= 10 && cur_region <= 21);
    bool     is_dist = (cur_region >= 30 && cur_region <= 34);

    // Debounce
    static uint32_t last_touch  = 0;
    static int      last_region = -1;

    // Dist buttons act differently—only trigger on new presses or very long hold to avoid flickering
    if (is_dist && cur_region == last_region && (now - last_touch) < 800) {
        return;  // Long debounce for dist buttons to prevent rapid toggle spam
    }

    uint32_t debounce_ms = is_jog ? 150 : 300;
    if (now - last_touch < debounce_ms)
        return;

    last_touch  = now;
    last_region = cur_region;

    if (is_dist) {
        _jog_dist_idx = cur_region - 30;
        _force_redraw = true;  // ensure UI will redraw the green highlight
        log_info("TS24 ACTION: Set Jog Dist to idx " << _jog_dist_idx);
        return;  // Don't do any other action
    }

    // Determine target distances and feedrates
    float dists[]  = { 0.1f, 1.0f, 5.0f, 10.0f, 100.0f };
    float jog_dist = dists[_jog_dist_idx];
    int   jog_feed = 2000;                                 // Keep a constant reasonable feedrate for manual positioning
    float z_dist   = jog_dist > 10.0f ? 10.0f : jog_dist;  // Limit Z jog stroke safety

    // Cancel any in-progress jog to be safe when clicking rapidly
    if (is_jog) {
        execute_realtime_command(Cmd::JogCancel, *this);
    }

    // Helper: send command via FreeRTOS queue
    auto safePush = [this](const char* cmd, const char* label) {
        log_info("TS24 ACTION: " << label << " -> " << cmd);
        if (_cmdQueue) {
            char buf[CMD_MAX_LEN] = {};
            strncpy(buf, cmd, CMD_MAX_LEN - 1);
            xQueueSend(_cmdQueue, buf, 0);
        }
    };

    // --- Top Bar ---
    if (p.y < 45) {
        const uint16_t TOP_BTN_W = 104;
        if (p.x < TOP_BTN_W) {
            safePush("$H\n", "HOME");
        } else if (p.x >= TOP_BTN_W + 4 && p.x < (TOP_BTN_W + 4) * 2) {
            // STOP: call execute_realtime_command directly for immediate effect in all states
            log_info("TS24 ACTION: STOP (Reset)");
            execute_realtime_command(Cmd::Reset, *this);
        } else if (p.x >= (TOP_BTN_W + 4) * 2) {
            if (_last_state.find("Alarm") != std::string::npos) {
                safePush("$X\n", "UNLOCK");
            }
        }
    }
    // --- State Header ---
    else if (p.y >= 200) {
        if (_last_state.find("Alarm") != std::string::npos) {
            safePush("$X\n", "UNLOCK_BAR");
        }
    }
    // --- Jog area ---
    else if (p.y >= 45 && p.y < 160) {
        char cmd[CMD_MAX_LEN];
        if (p.x < 200) {
            if (p.y < 79 && p.x >= 60 && p.x < 140) {
                snprintf(cmd, sizeof(cmd), "$J=G91 Y%.2f F%d\n", jog_dist, jog_feed);
                safePush(cmd, "Y+");
            } else if (p.y >= 83 && p.y < 117 && p.x >= 0 && p.x < 70) {
                snprintf(cmd, sizeof(cmd), "$J=G91 X-%.2f F%d\n", jog_dist, jog_feed);
                safePush(cmd, "X-");
            } else if (p.y >= 83 && p.y < 117 && p.x >= 125 && p.x < 200) {
                snprintf(cmd, sizeof(cmd), "$J=G91 X%.2f F%d\n", jog_dist, jog_feed);
                safePush(cmd, "X+");
            } else if (p.y >= 121 && p.x >= 60 && p.x < 140) {
                snprintf(cmd, sizeof(cmd), "$J=G91 Y-%.2f F%d\n", jog_dist, jog_feed);
                safePush(cmd, "Y-");
            }
        } else if (p.x >= 210) {
            if (p.y < 79) {
                snprintf(cmd, sizeof(cmd), "$J=G91 Z%.2f F%d\n", z_dist, jog_feed);
                safePush(cmd, "Z+");
            } else if (p.y >= 121 && p.y <= 155) {
                snprintf(cmd, sizeof(cmd), "$J=G91 Z-%.2f F%d\n", z_dist, jog_feed);
                safePush(cmd, "Z-");
            } else if (p.y < 100) {
                safePush("G10 L20 P1 X0 Y0\n", "ZERO_XY");
            } else {
                safePush("G10 L20 P1 Z0\n", "ZERO_Z");
            }
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
