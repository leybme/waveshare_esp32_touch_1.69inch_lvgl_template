/*
   MIT License

  Copyright (c) 2023 Felix Biego

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  ______________  _____
  ___  __/___  /_ ___(_)_____ _______ _______
  __  /_  __  __ \__  / _  _ \__  __ `/_  __ \
  _  __/  _  /_/ /_  /  /  __/_  /_/ / / /_/ /
  /_/     /_.___/ /_/   \___/ _\__, /  \____/
                              /____/

*/

#define LGFX_USE_V1
#include "Arduino.h"
#include <LovyanGFX.hpp>
#include <Timber.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include "app_hal.h"

#include "tone.h"

#include <lvgl.h>

#include "pins.h"
#include "FS.h"
#include "FFat.h"

#include "common/api.h"
#ifdef ENABLE_APP_QMI8658C
#include "FastIMU.h"
#define QMI_ADDRESS 0x6B
#endif

#ifdef ENABLE_RTC
#include <RtcPCF8563.h>
RtcPCF8563<TwoWire> Rtc(Wire);
#endif

#define FLASH FFat
#define F_NAME "FATFS"
#define buf_size 10

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_GC9A01 _panel_instance;
  lgfx::Light_PWM _light_instance;
  lgfx::Bus_SPI _bus_instance;
  lgfx::Touch_CST816S _touch_instance;

public:
  LGFX(void)
  {
    {
      auto cfg = _bus_instance.config();

      // SPI bus settings
      cfg.spi_host = SPI; // Select the SPI to use ESP32-S2,C3 : SPI2_HOST or SPI3_HOST / ESP32 : VSPI_HOST or HSPI_HOST
      // * Due to the ESP-IDF version upgrade, VSPI_HOST, The HSPI_HOST specification is deprecated, so if you get an error, use SPI2_HOST or SPI3_HOST instead.
      cfg.spi_mode = 0;                       // Set SPI communication mode (0 ~ 3)
      cfg.freq_write = 80000000;              // SPI time (up to 80MHz, four or five inputs divided by 80MHz to get an integer)
      cfg.freq_read = 20000000;               // SPI time when connected cfg.spi_3wire = true; // Set true if receiving is done via MOSI pin
      cfg.use_lock = true;                    // Usage lock time setting true
      cfg.dma_channel = SPI_DMA_CH_AUTO;      // Set the DMA channel to use (0=DMA not used / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=automatic setting) // * Due to the ESP-IDF version upgrade, SPI_DMA_CH_AUTO (automatic setting) is now recommended for the DMA channel. Specifying 1ch or 2ch is no longer recommended.
      cfg.pin_sclk = SCLK;                    // Set the SPI SCLK pin number
      cfg.pin_mosi = MOSI;                    // Set the SPI CLK pin number
      cfg.pin_miso = MISO;                    // Set the SPI MISO pin number (-1 = disable)
      cfg.pin_dc = DC;                        // Set the SPI D/C pin number (-1 = disable)
      _bus_instance.config(cfg);              // Reflect the setting value to the bus.
      _panel_instance.setBus(&_bus_instance); // Set the bus to the panel.
    }

    {                                      // Set the display panel control.
      auto cfg = _panel_instance.config(); // Get the structure for display panel settings.

      cfg.pin_cs = CS;   // Pin number to which CS is connected (-1 = disable)
      cfg.pin_rst = RST; // Pin number to which RST is connected (-1 = disable)
      cfg.pin_busy = -1; // Pin number to which BUSY is connected (-1 = disable)

      /* The following settings are set to general initial values ​​for each panel, so try commenting out any items you are unsure of. */

      cfg.memory_width = WIDTH;   // Maximum width supported by driver IC
      cfg.memory_height = HEIGHT; // Maximum height supported by driver IC
      cfg.panel_width = WIDTH;    // Actual displayable width
      cfg.panel_height = HEIGHT;  // Actual displayable height
      cfg.offset_x = OFFSET_X;    // Panel offset in X direction
      cfg.offset_y = OFFSET_Y;    // Panel offset in Y direction
      cfg.offset_rotation = 0;    // Value 0~7 in rotation direction (4~7 is inverted)
      cfg.dummy_read_pixel = 8;   // Virtual number of positions read before reading image
      cfg.dummy_read_bits = 1;    // The number of imaginary words other than the image element
      cfg.readable = false;       // As long as the number of acquisitions is as high as possible, the setting is true
      cfg.invert = true;          // As a result, the brightness and darkness of the face plate is reversed, and the setting is true
      cfg.rgb_order = RGB_ORDER;  // As a result, the red color and the blue color are replaced on the face plate, and the setting is true
      cfg.dlen_16bit = false;     // From 16th position to 16th position, the length of the number of transfers is set to true
      cfg.bus_shared = false;     // How to use drawJpgFile (e.g. summary control)

      _panel_instance.config(cfg);
    }

    { // Set backlight control. (delete if not necessary)

      auto cfg = _light_instance.config(); // Get the structure for backlight configuration.

      cfg.pin_bl = BL;     // pin number to which the backlight is connected
      cfg.invert = false;  // true to invert backlight brightness
      cfg.freq = 44100;    // backlight PWM frequency
      cfg.pwm_channel = 1; // PWM channel number to use

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance); // Sets the backlight to the panel.
    }

    { // Sets touchscreen control. (Delete if not needed)

      auto cfg = _touch_instance.config();
      cfg.x_min = 0;        // Minimum X value obtained from touch screen (raw value)
      cfg.x_max = WIDTH;    // Maximum X value obtained from touch screen (raw value)
      cfg.y_min = 0;        // Minimum Y value obtained from touch screen (raw value)
      cfg.y_max = HEIGHT;   // Maximum Y value obtained from touch screen (raw value)
      cfg.pin_int = TP_INT; // Pin number to which INT is connected
      cfg.pin_rst = TP_RST;
      cfg.bus_shared = true;   // Set true if using a common bus with the screen
      cfg.offset_rotation = 0; // Adjust if display and touch orientation do not match. Set to a value between 0 and 7

      cfg.i2c_port = 0;      // Select the I2C to use (0 or 1)
      cfg.i2c_addr = 0x15;   // I2C device address number
      cfg.pin_sda = I2C_SDA; // Pin number to which SDA is connected
      cfg.pin_scl = I2C_SCL; // Pin number to which SCL is connected
      cfg.freq = 400000;     // Set the I2C clock
      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance); // Set the touch screen to the panel.
    }

    setPanel(&_panel_instance); // Set the panel to use.
  }
};

LGFX tft;

Preferences prefs;

#ifdef ENABLE_APP_QMI8658C
QMI8658 qmi8658c;
calData calib = {0};
AccelData acc;
GyroData gyro;
#endif

static const uint32_t screenWidth = WIDTH;
static const uint32_t screenHeight = HEIGHT;

const unsigned int lvBufferSize = screenWidth * buf_size;
uint8_t lvBuffer[2][lvBufferSize];




void hal_setup(void);
void hal_loop(void);

void flashDrive_cb(lv_event_t *e);
void driveList_cb(lv_event_t *e);

String hexString(uint8_t *arr, size_t len, bool caps = false, String separator = "");
bool lvImgHeader(uint8_t *byteArray, uint8_t cf, uint16_t w, uint16_t h, uint16_t stride);

/**
 * @brief Flushes a portion of the display with the provided pixel data.
 *
 * This function is used by the LVGL library to update a specific area of the display.
 * It converts the pixel data to the appropriate format and sends it to the display
 * using DMA (Direct Memory Access) for efficient rendering.
 *
 * @param display Pointer to the LVGL display object.
 * @param area Pointer to the area of the display to be updated. Contains the coordinates
 *             of the rectangular region to flush.
 * @param data Pointer to the pixel data to be flushed to the display. The data is expected
 *             to be in RGB565 format.
 *
 * @note This function ensures that the display flushing process is completed by calling
 *       `lv_display_flush_ready()` once the operation is done.
 */
void my_disp_flush(lv_display_t *display, const lv_area_t *area, unsigned char *data)
{

  uint32_t w = lv_area_get_width(area);
  uint32_t h = lv_area_get_height(area);
  lv_draw_sw_rgb565_swap(data, w * h);

  if (tft.getStartCount() == 0)
  {
    tft.endWrite();
  }

  tft.pushImageDMA(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1, (uint16_t *)data);
  lv_display_flush_ready(display); /* tell lvgl that flushing is done */
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_t *indev_driver, lv_indev_data_t *data)
{
  bool touched;
  uint8_t gesture;
  uint16_t touchX, touchY;
  touched = tft.getTouch(&touchX, &touchY);
  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;

    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
    // screenTimer.time = millis();
    // screenTimer.active = true;
  }
}

#ifdef ELECROW_C3
// ELECROW C3 I2C IO extender
#define PI4IO_I2C_ADDR 0x43

// Extended IO function
void init_IO_extender()
{
  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x01); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  uint8_t rxdata = Wire.read();
  Serial.print("Device ID: ");
  Serial.println(rxdata, HEX);

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x03);                                                 // IO direction register
  Wire.write((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4)); // set pins 0, 1, 2 as outputs
  Wire.endTransmission();

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x07);                                                    // Output Hi-Z register
  Wire.write(~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4))); // set pins 0, 1, 2 low
  Wire.endTransmission();
}

void set_pin_io(uint8_t pin_number, bool value)
{

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  uint8_t rxdata = Wire.read();
  Serial.print("Before the change: ");
  Serial.println(rxdata, HEX);

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // Output register

  if (!value)
    Wire.write((~(1 << pin_number)) & rxdata); // set pin low
  else
    Wire.write((1 << pin_number) | rxdata); // set pin high
  Wire.endTransmission();

  Wire.beginTransmission(PI4IO_I2C_ADDR);
  Wire.write(0x05); // test register
  Wire.endTransmission();
  Wire.requestFrom(PI4IO_I2C_ADDR, 1);
  rxdata = Wire.read();
  Serial.print("after the change: ");
  Serial.println(rxdata, HEX);
}
#endif

#ifdef ENABLE_RTC
bool wasError(const char *errorTopic = "")
{
  uint8_t error = Rtc.LastError();
  if (error != 0)
  {
    // we have a communications error
    // see https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
    // for what the number means
    Serial.print("[");
    Serial.print(errorTopic);
    Serial.print("] WIRE communications error (");
    Serial.print(error);
    Serial.print(") : ");

    switch (error)
    {
    case Rtc_Wire_Error_None:
      Serial.println("(none?!)");
      break;
    case Rtc_Wire_Error_TxBufferOverflow:
      Serial.println("transmit buffer overflow");
      break;
    case Rtc_Wire_Error_NoAddressableDevice:
      Serial.println("no device responded");
      break;
    case Rtc_Wire_Error_UnsupportedRequest:
      Serial.println("device doesn't support request");
      break;
    case Rtc_Wire_Error_Unspecific:
      Serial.println("unspecified error");
      break;
    case Rtc_Wire_Error_CommunicationTimeout:
      Serial.println("communications timed out");
      break;
    }
    return true;
  }
  return false;
}
#endif

void toneOut(int pitch, int duration)
{ // pitch in Hz, duration in ms
#if defined(BUZZER) && (BUZZER != -1)
  int delayPeriod;
  long cycles, i;

  pinMode(BUZZER, OUTPUT);                        // turn on output pin
  delayPeriod = (500000 / pitch) - 7;             // calc 1/2 period in us -7 for overhead
  cycles = ((long)pitch * (long)duration) / 1000; // calc. number of cycles for loop

  for (i = 0; i <= cycles; i++)
  { // play note for duration ms
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(delayPeriod);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(delayPeriod - 1); // - 1 to make up for digitaWrite overhead
  }
  pinMode(BUZZER, INPUT); // shut off pin to avoid noise from other operations
#endif
}

String heapUsage()
{
  String usage;
  uint32_t total = ESP.getHeapSize();
  uint32_t free = ESP.getFreeHeap();
  usage += "Total: " + String(total);
  usage += "\tFree: " + String(free);
  usage += "\t" + String(((total - free) * 1.0) / total * 100, 2) + "%";
  return usage;
}

void *sd_open_cb(lv_fs_drv_t *drv, const char *path, lv_fs_mode_t mode)
{
  char buf[256];
  sprintf(buf, "/%s", path);
  // Serial.print("path : ");
  // Serial.println(buf);

  File f;

  if (mode == LV_FS_MODE_WR)
  {
    f = FLASH.open(buf, FILE_WRITE);
  }
  else if (mode == LV_FS_MODE_RD)
  {
    f = FLASH.open(buf);
  }
  else if (mode == (LV_FS_MODE_WR | LV_FS_MODE_RD))
  {
    f = FLASH.open(buf, FILE_WRITE);
  }

  if (!f)
  {
    return NULL; // Return NULL if the file failed to open
  }

  File *fp = new File(f); // Allocate the File object on the heap
  return (void *)fp;      // Return the pointer to the allocated File object
}

lv_fs_res_t sd_read_cb(lv_fs_drv_t *drv, void *file_p, void *buf, uint32_t btr, uint32_t *br)
{
  lv_fs_res_t res = LV_FS_RES_NOT_IMP;
  File *fp = (File *)file_p;
  uint8_t *buffer = (uint8_t *)buf;

  // Serial.print("name sd_read_cb : ");
  // Serial.println(fp->name());
  *br = fp->read(buffer, btr);

  res = LV_FS_RES_OK;
  return res;
}

lv_fs_res_t sd_seek_cb(lv_fs_drv_t *drv, void *file_p, uint32_t pos, lv_fs_whence_t whence)
{
  lv_fs_res_t res = LV_FS_RES_OK;
  File *fp = (File *)file_p;

  uint32_t actual_pos;

  switch (whence)
  {
  case LV_FS_SEEK_SET:
    actual_pos = pos;
    break;
  case LV_FS_SEEK_CUR:
    actual_pos = fp->position() + pos;
    break;
  case LV_FS_SEEK_END:
    actual_pos = fp->size() + pos;
    break;
  default:
    return LV_FS_RES_INV_PARAM; // Invalid parameter
  }

  if (!fp->seek(actual_pos))
  {
    return LV_FS_RES_UNKNOWN; // Seek failed
  }

  // Serial.print("name sd_seek_cb : ");
  // Serial.println(fp->name());

  return res;
}

lv_fs_res_t sd_tell_cb(lv_fs_drv_t *drv, void *file_p, uint32_t *pos_p)
{
  lv_fs_res_t res = LV_FS_RES_NOT_IMP;
  File *fp = (File *)file_p;

  *pos_p = fp->position();
  // Serial.print("name in sd_tell_cb : ");
  // Serial.println(fp->name());
  res = LV_FS_RES_OK;
  return res;
}

lv_fs_res_t sd_close_cb(lv_fs_drv_t *drv, void *file_p)
{
  File *fp = (File *)file_p;

  fp->close();
  // delete fp;  // Free the allocated memory

  return LV_FS_RES_OK;
}


/**
 * @brief Adjusts the screen brightness of the TFT display.
 * 
 * This function sets the brightness of the TFT display to the specified value.
 * 
 * @param value The brightness level to set, ranging from 0 (off) to 255 (maximum brightness).
 */
void screenBrightness(uint8_t value)
{
  tft.setBrightness(value);
#ifdef ELECROW_C3
  set_pin_io(2, value > 0); // ELECROW C3, no brightness control
#endif
}

String readFile(const char *path)
{
  String result;
  File file = FLASH.open(path);
  if (!file || file.isDirectory())
  {
    Serial.println("- failed to open file for reading");
    return result;
  }

  Serial.println("- read from file:");
  while (file.available())
  {
    result += (char)file.read();
  }
  file.close();
  return result;
}

void deleteFile(const char *path)
{
  Serial.printf("Deleting file: %s\r\n", path);
  if (FLASH.remove(path))
  {
    Serial.println("- file deleted");
  }
  else
  {
    Serial.println("- delete failed");
  }
}

bool setupFS()
{
  if (!FLASH.begin(true, "/ffat", MAX_FILE_OPEN))
  {
    FLASH.format();

    return false;
  }

  static lv_fs_drv_t sd_drv;
  lv_fs_drv_init(&sd_drv);
  sd_drv.cache_size = 512;

  sd_drv.letter = 'S';
  sd_drv.open_cb = sd_open_cb;
  sd_drv.close_cb = sd_close_cb;
  sd_drv.read_cb = sd_read_cb;
  sd_drv.seek_cb = sd_seek_cb;
  sd_drv.tell_cb = sd_tell_cb;
  lv_fs_drv_register(&sd_drv);

  return true;
}





void savePrefInt(const char *key, int value)
{
  prefs.putInt(key, value);
}

int getPrefInt(const char *key, int def_value)
{
  return prefs.getInt(key, def_value);
}



void onBrightnessChange(lv_event_t *e)
{
  // Your code here
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  int v = lv_slider_get_value(slider);
  screenBrightness(v);

  prefs.putInt("brightness", v);
}


void setTimeout(int i)
{
  // if (i == 4)
  // {
  //   screenTimer.duration = -1; // always on
  // }
  // else if (i == 0)
  // {
  //   screenTimer.duration = 5000; // 5 seconds
  //   screenTimer.active = true;
  // }
  // else if (i < 4)
  // {
  //   screenTimer.duration = 10000 * i; // 10, 20, 30 seconds
  //   screenTimer.active = true;
  // }
}










void imu_init()
{
#ifdef ENABLE_APP_QMI8658C
  int err = qmi8658c.init(calib, QMI_ADDRESS);
  if (err != 0)
  {
    // showError("IMU State", "Failed to init");
  }
#endif
}

imu_data_t get_imu_data()
{
  imu_data_t qmi;
#ifdef ENABLE_APP_QMI8658C

  qmi8658c.update();
  qmi8658c.getAccel(&acc);
  qmi8658c.getGyro(&gyro);

  qmi.ax = acc.accelX;
  qmi.ay = acc.accelY;
  qmi.az = acc.accelZ;
  qmi.gx = gyro.gyroX;
  qmi.gy = gyro.gyroY;
  qmi.gz = gyro.gyroZ;
  qmi.temp = qmi8658c.getTemp();
  qmi.success = true;
#else
  qmi.success = false;
#endif
  return qmi;
}

void imu_close()
{
#ifdef ENABLE_APP_QMI8658C

#endif
}

void logCallback(Level level, unsigned long time, String message)
{
  Serial.print(message);
  Serial1.print(message);
}

// void lv_log_register_print_cb(lv_log_print_g_cb_t print_cb) {
//   // Do nothing, not needed here!
// }

// void my_log_cb(lv_log_level_t level, const char *buf)
// {
//   Serial.write(buf, strlen(buf));
//   Serial1.write(buf, strlen(buf));
// }

int putchar(int ch)
{
  Serial.write(ch); // Send character to Serial
  return ch;
}


static uint32_t my_tick(void)
{
  return millis();
}

void hal_setup()
{

  Serial.begin(115200); /* prepare for possible serial debug */
  Serial1.begin(115200);
  Timber.setLogCallback(logCallback);
  Timber.i("Starting up device");
  prefs.begin("my-app");
  int rt = prefs.getInt("rotate", 0);

#ifdef ELECROW_C3
  Wire.begin(4, 5);
  init_IO_extender();
  delay(100);
  set_pin_io(2, true);
  set_pin_io(3, true);
  set_pin_io(4, true);
#endif
  toneOut(TONE_EN * 2, 170);
  toneOut(TONE_FS * 2, 170);
  toneOut(TONE_GN * 2, 170);

//Setup the display driver
  tft.init();
  tft.initDMA();
  tft.startWrite();
  tft.fillScreen(TFT_BLACK);
  tft.setRotation(rt);
  Timber.i(heapUsage());

  lv_init();
  lv_tick_set_cb(my_tick);

  static auto *lvDisplay = lv_display_create(screenWidth, screenHeight);
  lv_display_set_color_format(lvDisplay, LV_COLOR_FORMAT_RGB565);
  lv_display_set_flush_cb(lvDisplay, my_disp_flush);

  lv_display_set_buffers(lvDisplay, lvBuffer[0], lvBuffer[1], lvBufferSize, LV_DISPLAY_RENDER_MODE_PARTIAL);
//Setup the touchpad driver
  static auto *lvInput = lv_indev_create();
  lv_indev_set_type(lvInput, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(lvInput, my_touchpad_read);

  // lv_log_register_print_cb(my_log_cb);

  // _lv_fs_init();

  // ui_init();

  Serial.println(heapUsage());

  bool fsState = setupFS();
  if (fsState)
  {
    // driveList_cb(NULL);
    Serial.println("Setup FS success");
  }
  else
  {
    Serial.println("Setup FS failed");
    // showError(F_NAME, "Failed to mount the partition");
  }
  Serial.println(heapUsage());


  // screenBrightness(br);


  if (lv_fs_is_ready('S'))
  {
    Serial.println("Drive S is ready");
  }
  else
  {
    Serial.println("Drive S is not ready");
  }

  imu_init();

#ifdef ENABLE_RTC
  Rtc.Begin();

  if (!Rtc.GetIsRunning())
  {
    uint8_t error = Rtc.LastError();
    if (error != 0)
    {
      showError("RTC", "Error on RTC");
    }
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();

  watch.setTime(now.Second(), now.Minute(), now.Hour(), now.Day(), now.Month(), now.Year());

  Rtc.StopAlarm();
  Rtc.StopTimer();
  Rtc.SetSquareWavePin(PCF8563SquareWavePinMode_None);
#endif

}

void hal_loop()
{
  lv_timer_handler(); // Update the UI
  delay(5);

}




String hexString(uint8_t *arr, size_t len, bool caps, String separator)
{
  String hexString = "";
  for (size_t i = 0; i < len; i++)
  {
    char hex[3];
    sprintf(hex, caps ? "%02X" : "%02x", arr[i]);
    hexString += separator;
    hexString += hex;
  }
  return hexString;
}

String longHexString(unsigned long l)
{
  char buffer[9];             // Assuming a 32-bit long, which requires 8 characters for hex representation and 1 for null terminator
  sprintf(buffer, "%08x", l); // Format as 8-digit hex with leading zeros
  return String(buffer);
}

bool lvImgHeader(uint8_t *byteArray, uint8_t cf, uint16_t w, uint16_t h, uint16_t stride)
{

  byteArray[0] = LV_IMAGE_HEADER_MAGIC;
  byteArray[1] = cf;
  byteArray[2] = 0;
  byteArray[3] = 0;

  byteArray[4] = (w & 0xFF);
  byteArray[5] = (w >> 8) & 0xFF;

  byteArray[6] = (h & 0xFF);
  byteArray[7] = (h >> 8) & 0xFF;

  byteArray[8] = (stride & 0xFF);
  byteArray[9] = (stride >> 8) & 0xFF;

  byteArray[10] = 0;
  byteArray[11] = 0;

  return true;
}
