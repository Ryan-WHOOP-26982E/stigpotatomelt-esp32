// controller.cpp - SBUS receiver interface for Potatomelt
//
// Requires the "Bolder Flight Systems SBUS" library.
// Install via Arduino Library Manager: search for "sbus" by Bolder Flight Systems.

#include <Preferences.h>
#include "lib/SbusReader.h"

#include "controller.h"
#include "melty_config.h"
#include "subsystems/storage.h"

// SBUS receiver on UART1
HardwareSerial SBUSSerial(1);
SbusReader sbus_rx(&SBUSSerial, SBUS_RX_PIN, SBUS_TX_PIN);

// RPM targets - adjust the list and NUM_TARGET_RPMS to taste
int spin_target_rpms[] = {600, 800, 1000, 1200, 1500, 1800, 2100, 2500, 3000};
float translation_trims[] = {1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2, 10.0};
#define NUM_TARGET_RPMS 9
#define NUM_TRANS_TRIMS 13

int target_rpm_index = 3;
int target_trans_trim = 4;
bool reverse_spin = false;

long last_sbus_millis = 0;
prev_state previous_state;
ctrl_state current_state;

// Scale a raw SBUS value to the axis range used by the rest of the code (-512..512).
// SBUS range is 172-1811, center is 992.
static int sbus_to_axis(int16_t raw) {
    int centered = (int)raw - SBUS_CENTER;
    return constrain(centered * 512 / SBUS_HALF_RANGE, -512, 512);
}

void ctrl_init() {
    sbus_rx.begin();
}

bool is_connected() {
    return current_state.connected;
}

ctrl_state* ctrl_update(bool /*upd8*/) {
    if (!sbus_rx.read()) {
        // No new SBUS frame this cycle - check for timeout
        if (millis() - last_sbus_millis > CONTROL_UPDATE_TIMEOUT_MS) {
            current_state.alive = false;
        }
        // Trim outputs are edge-triggered; clear them each cycle with no new data
        current_state.trim_left  = false;
        current_state.trim_right = false;
        return &current_state;
    }

    const SbusReader::Data& data = sbus_rx.data();
    last_sbus_millis = millis();

    // Failsafe: receiver lost signal from transmitter
    if (data.failsafe || data.lost_frame) {
        current_state.connected = false;
        current_state.alive     = false;
        return &current_state;
    }

    current_state.connected = true;
    current_state.alive     = true;

    // --- Spin trigger ---
    current_state.spin_requested = data.ch[SBUS_CH_THROTTLE] > SBUS_SPIN_THRESHOLD;

    // --- Axis inputs ---
    current_state.translate_forback = sbus_to_axis(data.ch[SBUS_CH_TRANSLATE]);
    current_state.translate_lr      = sbus_to_axis(data.ch[SBUS_CH_TRANSLATE_LR]);
    current_state.turn_lr           = sbus_to_axis(data.ch[SBUS_CH_TURN]);

    // --- Reverse spin direction (toggle on switch rising edge) ---
    bool rev_pressed = data.ch[SBUS_CH_REVERSE] > SBUS_SWITCH_THRESHOLD;
    if (rev_pressed && !previous_state.reverse_spin_pressed) {
        reverse_spin = !reverse_spin;
    }
    previous_state.reverse_spin_pressed = rev_pressed;
    current_state.reverse_spin = reverse_spin;

    // --- Target RPM adjustment (stick/dial, with deadzone) ---
    int rpm_input = sbus_to_axis(data.ch[SBUS_CH_RPM_ADJUST]);
    if (abs(rpm_input) > CONTROL_SPIN_SPEED_DEADZONE && !previous_state.spin_target_rpm_changed) {
        previous_state.spin_target_rpm_changed = true;
        if (rpm_input < 0 && target_rpm_index < NUM_TARGET_RPMS - 1) {
            target_rpm_index++;
        } else if (rpm_input > 0 && target_rpm_index > 0) {
            target_rpm_index--;
        }
        get_active_store()->set_target_rpm(target_rpm_index);
    } else if (abs(rpm_input) < CONTROL_SPIN_SPEED_DEADZONE) {
        previous_state.spin_target_rpm_changed = false;
    }
    current_state.target_rpm = spin_target_rpms[target_rpm_index];

    // --- Translate trim (edge detect on momentary switches) ---
    current_state.trim_left  = false;
    current_state.trim_right = false;

    bool trim_up = data.ch[SBUS_CH_TRIM_TRANS_UP] > SBUS_SWITCH_THRESHOLD;
    if (trim_up != previous_state.increase_translate_pressed) {
        previous_state.increase_translate_pressed = trim_up;
        if (trim_up && target_trans_trim < NUM_TRANS_TRIMS - 1) {
            target_trans_trim++;
            get_active_store()->set_trans_trim(target_trans_trim);
        }
    }

    bool trim_down = data.ch[SBUS_CH_TRIM_TRANS_DOWN] > SBUS_SWITCH_THRESHOLD;
    if (trim_down != previous_state.decrease_translate_pressed) {
        previous_state.decrease_translate_pressed = trim_down;
        if (trim_down && target_trans_trim > 0) {
            target_trans_trim--;
            get_active_store()->set_trans_trim(target_trans_trim);
        }
    }

    current_state.translate_trim = translation_trims[target_trans_trim];

    // --- Motor trim (fire once on rising edge) ---
    bool tl = data.ch[SBUS_CH_TRIM_LEFT] > SBUS_SWITCH_THRESHOLD;
    if (tl && !previous_state.trim_left_pressed) {
        current_state.trim_left = true;
    }
    previous_state.trim_left_pressed = tl;

    bool tr = data.ch[SBUS_CH_TRIM_RIGHT] > SBUS_SWITCH_THRESHOLD;
    if (tr && !previous_state.trim_right_pressed) {
        current_state.trim_right = true;
    }
    previous_state.trim_right_pressed = tr;

    return &current_state;
}
