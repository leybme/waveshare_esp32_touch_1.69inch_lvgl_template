# waveshare_esp32_touch_1.69inch_lvgl_template

This is a simple template project to make it easier to use the **Waveshare ESP32 1.69" Touch Display** with **LVGL (Light and Versatile Graphics Library)**.

I created this template to help get started quickly with the basic setup and functions.

## Features

- LVGL support
- Basic screen brightness control
- Short and double touch press callbacks
- Simple tone output support
- Periodic update via `Ticker`

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

### Tone Output

```cpp
void toneOut(int pitch, int duration);
```
Play a tone using defined pitch and duration (in ms).

Example pitch constants (defined in `tone.h`):
```cpp
#define TONE_AN 220   // A
#define TONE_AS 233   // A#
#define TONE_BN 247   // B
#define TONE_CN 261   // C
#define TONE_CS 277   // C#
#define TONE_DN 294   // D
#define TONE_DS 311   // D#
#define TONE_EN 330   // E
#define TONE_FN 349   // F
#define TONE_FS 370   // F#
#define TONE_GN 392   // G
#define TONE_GS 415   // G#
```

---

### Periodic Ticker Callback

Use `Ticker` to schedule functions at a regular interval:

```cpp
Ticker ticker;

void ticker_callback()
{
    static long count = millis();
    count = millis();
    Serial1.println("Ticker called! " + String(count));
    lv_label_set_text(label, ("millis:" + String(count)).c_str()); // Update the label text
}

// Call ticker_callback every 1 second
ticker.attach(1, ticker_callback);
```

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

## Support This Project

If you find this project helpful, you can [buy me a coffee](https://www.paypal.com/paypalme/ley995) ☕

## Contact

For work inquiries or collaborations, reach out via email: **nguyenleybme@gmail.com**
