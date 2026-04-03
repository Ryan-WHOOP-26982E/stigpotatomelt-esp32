#include <Arduino.h>
#include "led.h"
#include "../melty_config.h"

LED::LED() {
    pinMode(LED_1_PIN, OUTPUT);
    pinMode(LED_2_PIN, OUTPUT);
    set_leds(false);
}

void LED::set_leds(bool on) {
    digitalWrite(LED_1_PIN, on ? HIGH : LOW);
    digitalWrite(LED_2_PIN, on ? HIGH : LOW);
}

// Solid on — controller connected, ready
void LED::leds_on_ready() {
    set_leds(true);
}

// Solid on — battery low (can't show color with basic LEDs)
void LED::leds_on_low_battery() {
    set_leds(true);
}

// Fast flash: mostly on with brief off — stale/failsafe
void LED::leds_on_controller_stale() {
    long t = millis() / 100;
    set_leds((t % 10) >= 2);
}

// Slow flash: mostly off with brief on — no controller
void LED::leds_on_no_controller() {
    long t = millis() / 100;
    set_leds((t % 10) < 2);
}

// Heading indicator during spin — both LEDs on
void LED::leds_on_gradient(int /*color*/) {
    set_leds(true);
}

void LED::leds_off() {
    set_leds(false);
}
