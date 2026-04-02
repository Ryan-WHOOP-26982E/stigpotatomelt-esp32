#include <Arduino.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#include <driver/rmt.h>
#pragma GCC diagnostic pop
#include "led.h"
#include "../melty_config.h"

rmt_item32_t led_data[6*8];
uint8_t pixel_color[6]; 

LED::LED() {
    rmt_config_t rmt_cfg = RMT_DEFAULT_CONFIG_TX(NEOPIXEL_PIN, NEOPIXEL_RMT);

    rmt_cfg.clk_div = 8; // slow us down to 10mhz

    rmt_config(&rmt_cfg);

    rmt_driver_install(rmt_cfg.channel, 0, 0);
}

void LED::leds_on_ready() {
    leds_on_rgb(0, 0, 255);
}

void LED::leds_on_low_battery() {
    leds_on_rgb(255, 0, 0);
}

void LED::leds_on_controller_stale() {
    long now = millis();
        now /= 100;
        if ((now % 10) < 8) {
            leds_on_rgb(255, 0, 0);
        } else {
            leds_off();
        }
}

void LED::leds_on_no_controller() {
    long now = millis();
        now /= 100;
        if ((now % 10) < 2) {
            leds_on_rgb(255, 0, 0);
        } else {
            leds_off();
        }
}

void LED::leds_on_gradient(int color) {
    int green = (color > 50) ? 255 : 255*color/50;
    int red = (color < 50) ? 255 : 255*(100-color)/50;
    leds_on_rgb(red, green, 0);
}

void LED::leds_off() {
    pixel_color[0] = pixel_color[1] = pixel_color[2] = 0;
    pixel_color[3] = pixel_color[4] = pixel_color[5] = 0;
    write_pixel();
}

void LED::leds_on_rgb(int red, int green, int blue) {
    // neopixels usually use GRB addressing rather than RGB
    pixel_color[0] = pixel_color[3] = green;
    pixel_color[1] = pixel_color[4] = red;
    pixel_color[2] = pixel_color[5] = blue;
    write_pixel();
}

void LED::write_pixel() {
    int index = 0;
    for (auto chan : pixel_color) {
      uint8_t value = chan;
      for (int bit = 7; bit >= 0; bit--) {
        if ((value >> bit) & 1) {
          led_data[index].level0 = 1;
          led_data[index].duration0 = 8;
          led_data[index].level1 = 0;
          led_data[index].duration1 = 4;
        } else {
          led_data[index].level0 = 1;
          led_data[index].duration0 = 4;
          led_data[index].level1 = 0;
          led_data[index].duration1 = 8;
        }
        index++;
      }
    }

    rmt_write_items(NEOPIXEL_RMT, led_data, 6*8, false);
  }