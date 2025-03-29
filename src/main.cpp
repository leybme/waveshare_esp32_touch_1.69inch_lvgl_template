
/*
   MIT License

  Copyright (c) 2024 Felix Biego
  modified by 2025 Nguyen Le Y

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

*/
#include "Arduino.h"
#include "app_hal.h"
#include "common/api.h"
#include "tone.h"
#include "lvgl.h"

TaskHandle_t otherTaskHandle = NULL;
lv_obj_t *label;
void otherTask(void *parameter)
{
    static long counter = 0;
    while (1)
    {
        counter = millis() / 1000;
        String str = "Counter" + String(counter);
        lv_label_set_text(label, str.c_str()); // Update the label text
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
}

void onShortPress()
{
    Serial.println("Short press detected!");
}

void onDoublePress()
{
    Serial.println("Double press detected!");
}

void setup()
{
    hal_setup(); // Initialize the hardware and peripherals
    // create a text in lvgl
    label = lv_label_create(lv_scr_act()); // Create a label on the active screen
    lv_label_set_text(label, "Hello World!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);                    // Align the label to the center of the screen
    lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), 0); // Set text color to red
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0);  // Set font to Montserrat 16
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);   // Center align the text

    setShortPressCallback(onShortPress);
    setDoublePressCallback(onDoublePress);
    xTaskCreatePinnedToCore(
        otherTask,        // Task function
        "Other Task",     // Task name
        2048,             // Stack size (words)
        NULL,             // Parameters
        1,                // Priority (1 is usually enough)
        &otherTaskHandle, // Task handle
        0                 // Core 0 or 1 (0 is good for simple tasks)
    );
}

void loop()
{
    hal_loop();
}