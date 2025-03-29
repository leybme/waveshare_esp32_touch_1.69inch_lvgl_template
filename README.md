# waveshare_esp32_touch_1.69inch_lvgl_template

This is a simple template project to make it easier to use the **Waveshare ESP32 1.69" Touch Display** with **LVGL (Light and Versatile Graphics Library)**.

I created this template to help get started quickly with the basic setup and functions.

## Features

- LVGL support
- Basic screen brightness control
- Short and double touch press callbacks

## Usage

### Brightness Control

```cpp
void setBrightness(uint8_t brightness);
```
Set screen brightness (0–255).

---

### Touch Callbacks

```cpp
void setShortPressCallback(ButtonCallback cb);
```
Set a callback for **short press** events.

```cpp
void setDoublePressCallback(ButtonCallback cb);
```
Set a callback for **double press** events.

---

## Hardware

- **Display:** Waveshare 1.69" Touch LCD
- **Board:** ESP32-based (e.g., ESP32-S3)

Product page: [Waveshare ESP32-S3 Touch LCD 1.69"](https://www.waveshare.com/product/esp32-s3-touch-lcd-1.69.htm)  
Wiki/documentation: [Waveshare Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.69)

## Getting Started

1. Clone this repo  
2. Set up your ESP32 dev environment (PlatformIO or Arduino IDE)  
3. Flash to your board and start building your UI!

## License

MIT License – free to use and modify.

---

## Support this work

If you find this project helpful, you can [buy me a coffee](https://www.paypal.com/paypalme/ley995) ☕

## Contact

For work inquiries or collaborations, reach out via email: **nguyenleybme@gmail.com**
