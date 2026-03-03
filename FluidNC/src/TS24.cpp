#include "TS24.h"
#include "ST7789.h"
#include "XPT2046.h"
#include "Machine/MachineConfig.h"
#include "Machine/SPIBus.h"
#include "Report.h"
#include "InputFile.h"
#include <string.h>

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

    // Start UI task to handle polling touch and redrawing DRO without blocking Stepper cores
    xTaskCreatePinnedToCore(
        ui_task,
        "ts24_ui",
        4096, // Stack size
        this,
        1,    // Priority (Low)
        NULL,
        0     // Core 0 (Async / WebUI core, away from stepper core)
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
    // Ignore direct single byte writes.
    return 1;
}

Error TS24::pollLine(char* line) {
    // Intercept status reports from the internal FluidNC string reporting system
    if (line[0] == '<') {
        // Simple string finding to extract status.
        // E.g., <Idle|MPos:10.0,0.0,0.0|FS:0,0>
        char* stateEnd = strchr(line, '|');
        if (stateEnd) {
            _last_state = std::string(&line[1], stateEnd - &line[1]);
        }
        
        // Find MPos or WPos
        char* pos = strstr(line, "MPos:");
        if (!pos) pos = strstr(line, "WPos:");
        
        if (pos) {
            pos += 5; // skip "MPos:"
            sscanf(pos, "%f,%f,%f", &_last_mpos[0], &_last_mpos[1], &_last_mpos[2]);
        }
    }
    return Error::Ok; 
}


// --- UI Layout & Rendering Logic ---

void TS24::render_ui() {
    if (!_display) return;

    // We rely on static coordinate boxing. ST7789 Width=320, Height=240

    // Top Bar (DRO)
    _display->fillRect(0, 0, 320, 40, ST7789::BLUE);
    // Draw state & coords (requires a font, which is mocked in the current cpp file)
    // _display->drawString(10, 10, ("State: " + _last_state).c_str(), 2);
    // std::string coords = "X:" + std::to_string((int)_last_mpos[0]) + " Y:" + std::to_string((int)_last_mpos[1]) + " Z:" + std::to_string((int)_last_mpos[2]);
    // _display->drawString(150, 10, coords.c_str(), 2);

    // Jog Controls Grid
    int btn_w = 60;
    int btn_h = 50;
    
    // Y+
    _display->fillRect(60, 50, btn_w, btn_h, ST7789::GRAY);
    // X-
    _display->fillRect(0, 100, btn_w, btn_h, ST7789::GRAY);
    // X+
    _display->fillRect(120, 100, btn_w, btn_h, ST7789::GRAY);
    // Y-
    _display->fillRect(60, 150, btn_w, btn_h, ST7789::GRAY);

    // Z Controls
    _display->fillRect(200, 50, btn_w, btn_h, ST7789::GRAY); // Z+
    _display->fillRect(200, 150, btn_w, btn_h, ST7789::GRAY); // Z-

    // Zero Controls
    _display->fillRect(260, 50, btn_w, btn_h, ST7789::CYAN); // Zero XY
    _display->fillRect(260, 150, btn_w, btn_h, ST7789::CYAN); // Zero Z

    // Bottom Bar
    _display->fillRect(0, 200, 150, 40, ST7789::GREEN); // HOME ALL
    _display->fillRect(170, 200, 150, 40, ST7789::RED);   // STOP
}

void TS24::handle_touch() {
    if (!_touch || !_touch->isTouched()) return;
    
    // Assuming calibrated between 0x0 and 240x320
    TouchPoint p = _touch->getPoint(320, 240, 200, 200, 3800, 3800);

    // A simple debouncer
    static uint32_t last_touch = 0;
    if (millis() - last_touch < 300) return;
    last_touch = millis();

    // Map Touch to Regions. Because of FluidNC's input queue, we inject commands.
    // Example: fluidnc InputQueue::push("G0 X10\n");
    // Or system commands like InputQueue::push("\x85"); // Jog Cancel / Stop

    if (p.y >= 200) {
        if (p.x < 150) { // HOME ALL
            InputFile::push(std::string("$H\n"));
        } else if (p.x > 170) { // STOP
            InputFile::push(std::string("\x85")); // Jog Cancel / Feedhold
        }
    } else if (p.y >= 50 && p.y <= 100 && p.x >= 260) {
        // Zero XY
        InputFile::push(std::string("G10 L20 P1 X0 Y0\n"));
    } else if (p.y >= 150 && p.y <= 200 && p.x >= 260) {
        // Zero Z
        InputFile::push(std::string("G10 L20 P1 Z0\n"));
    } 
    // Add logic for jogging step sizes, currently X/Y/Z pad.
    // E.g.,
    // if (p.y >= 100 && p.y <= 150 && p.x < 60) { InputQueue::push(std::string("$J=G91 X-1 F1000\n")); } 
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
ChannelFactory::InstanceBuilder<TS24> ts24_channel __attribute__((init_priority(105))) ("ts24");
ConfigurableModuleFactory::InstanceBuilder<TS24> ts24_module __attribute__((init_priority(105))) ("ts24");
