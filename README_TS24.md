# MKS TS24 R v2.1 Display Integration for FluidNC

This module provides native support for the MKS TS24 R v2.1 SPI display and touch controller in FluidNC. It includes a custom GUI designed for CNC machine control, featuring real-time status updates and manual control overrides.

## Features

- **High-Performance SPI Driver**: Native implementation for the ST7789 display controller.
- **Resistive Touch Support**: Integrated driver for the XPT2046 touch controller with coordinate calibration.
- **Asynchronous UI Task**: The display logic and touch polling run on a background task (Core 0), ensuring that UI rendering does not interfere with the high-priority stepper motor timing (Core 1).
- **Real-Time DRO**: Displays machine state and coordinates (X, Y, Z) with optimized redrawing to minimize SPI bus traffic.
- **Manual Controls**:
    - **Jogging**: Directional movement for X, Y, and Z axes.
    - **Zeroing**: Quick zeroing for XY axes and Z axis.
    - **Homing**: One-touch "HOME ALL" command.
    - **Safety**: Dedicated "STOP" button (sends a Jog Cancel/Feedhold real-time command).

## Hardware Configuration (`config.yaml`)

To enable the TS24 display, you must define the SPI bus and the `ts24:` module in your configuration. The pins below are typical for an MKS DLC32 board; adjust them if your hardware differs.

```yaml
spi:
  miso_pin: gpio.19
  mosi_pin: gpio.23
  sck_pin: gpio.18

ts24:
  cs_pin: gpio.5
  dc_pin: gpio.2
  reset_pin: gpio.4
  backlight_pin: gpio.21
  touch_cs_pin: gpio.22
  touch_irq_pin: gpio.17
  baud_rate: 40000000
```

### Parameters

| Parameter | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `cs_pin` | pin | - | SPI chip select for the ST7789 display. |
| `dc_pin` | pin | - | Data/Command pin for the ST7789. |
| `reset_pin` | pin | - | Reset pin for the display. |
| `backlight_pin` | pin | - | Pin to enable the display backlight. |
| `touch_cs_pin` | pin | - | SPI chip select for the XPT2046 touch controller. |
| `touch_irq_pin` | pin | - | Interrupt pin for touch detection. |
| `baud_rate` | int | 40000000 | SPI frequency in Hz (40MHz recommended for ST7789). |

## Technical Implementation

### Architecture
The `TS24` class inherits from `Channel` and `ConfigurableModule`. 
- As a **Channel**, it automatically receives status reports and G-code feedback from the core FluidNC engine.
- As a **ConfigurableModule**, it integrates seamlessly with the `config.yaml` parsing system.

### Performance Optimizations
1. **Delta Rendering**: The `render_ui()` function maintains a local cache of the last machine state and coordinates. It only pushes pixel data to the display when a significant change (e.g., >0.01mm movement) is detected.
2. **Core Affinity**: By pinning the `ui_task` to Core 0, the CPU cycles required for font rendering and SPI transmission are kept away from the `Stepper` and `Planner` tasks, preventing potential step-loss or jitter.

## Developer Notes
- **Font**: Uses a lean 5x7 GLCD bitmapped font for maximum speed and minimal memory footprint.
- **Coordinate Mapping**: The touch controller is pre-calibrated for standard 320x240 orientation. If your touch is inverted, coordinate scaling in `handle_touch()` may need adjustment.
