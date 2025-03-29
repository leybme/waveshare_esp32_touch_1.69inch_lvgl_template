
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
#include "app_hal.h"
#include "lvgl.h"

// esp32 hardware
#include <Arduino.h>
void setup(){
    hal_setup();

    //create a text in lvgl
    lv_obj_t * label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "Hello World!");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0); // Align the label to the center of the screen
    lv_obj_set_style_text_color(label, lv_color_hex(0xFF0000), 0); // Set text color to red
    lv_obj_set_style_text_font(label, &lv_font_montserrat_16, 0); // Set font to Montserrat 16
    lv_obj_set_style_text_letter_space(label, 2, 0); // Set letter spacing to 2 pixels
    lv_obj_set_style_text_line_space(label, 4, 0); // Set line spacing to 4 pixels
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0); // Center align the text
    lv_obj_set_style_text_decor(label, LV_TEXT_DECOR_UNDERLINE, 0); // Underline the text
    lv_obj_set_style_text_opa(label, LV_OPA_COVER, 0); // Set text opacity to cover
    

}

void loop(){
    hal_loop();
}