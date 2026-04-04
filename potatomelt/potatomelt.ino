#include <PID_v1.h>
#include <Preferences.h>
#include <Wire.h>
#include "src/robot.h"
#include "src/melty_config.h"
#include "src/controller.h"
#include "src/subsystems/storage.h"

TaskHandle_t hotloop;

Robot robot;

robot_status state;
spin_control_parameters_t control_params;
tank_control_parameters_t tank_params;

Storage store;

long last_logged_at = 0;

// Variables for the PID - it doesn't take args directly, just gets pointers to these
double pid_current_rpm = 0.0; // Input to the PID: The current RPM
double pid_target_rpm = 0.0;  // Setpoint for the PID: The target RPM
double pid_throttle_output = 0.0; // Output from the PID: How hard to run the throttle

// We're using a PID to control motor power, to chase a RPM set by the throttle channel
PID throttle_pid(&pid_current_rpm, &pid_throttle_output, &pid_target_rpm, PID_KP, PID_KI, PID_KD, P_ON_E, DIRECT);

// todo - translation trim

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.println("PotatoMelt startup");

    // set up I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    Wire.setTimeOut(10);

    // start data storage and recall
    store.init();

    // Configure the PID
    throttle_pid.SetOutputLimits(0.0, 1023.0);

    // start the robot subsystems
    robot.init();
    state = NO_CONTROLLER;

    // start the SBUS control interface
    ctrl_init();

    // and start the hot loop - it'll be managing LEDs and motors
    xTaskCreatePinnedToCore(
        hotloopFN, // the function
        "hotloop", // name the task
        10000,     // stack depth
        NULL,      // params
        1,         // priority
        &hotloop,  // task handle (if we want to interact with the task)
        0          // core affinity
    );

    //pinMode(2, OUTPUT);
    //digitalWrite(2, HIGH);
}

// This function is the core of the control loop
// it calculates all the parameters needed for a single rotation (motor phases, LED timing, etc)
void calculate_melty_params(spin_control_parameters_t* params, ctrl_state* c) {
    float rpm = robot.get_rpm(c->target_rpm);

    // don't lie to the PID
    pid_current_rpm = rpm;

    // but for the rest of the math, we're going to insist that we're spinning at least so fast
    // this puts a limit on the amount of time we'll spend in a single rotation
    rpm = max(rpm, (float) MIN_TRACKING_RPM);

    // because by default we're spinning clockwise, right turns = longer rotations = less RPM
    float rpm_adjustment_factor = c->turn_lr / 1024.0 / LEFT_RIGHT_HEADING_CONTROL_DIVISOR;

    rpm -= rpm*rpm_adjustment_factor;

    long rotation_us = (1.0f/rpm) * 60 * 1000 * 1000;

    params->rotation_interval_us = rotation_us;

    // and the LED settings
    float led_on_portion = rpm / MAX_TRACKING_RPM;
    if (led_on_portion < 0.10f) led_on_portion = 0.10f;
    if (led_on_portion > 0.90f) led_on_portion = 0.90f;

    double led_on_us = (long) (led_on_portion * rotation_us);
    double led_offset_us = (long) (LED_OFFSET_PERCENT * rotation_us / 100);

    // starts LED on time at point in rotation so it's "centered" on led offset
    params->led_start = led_offset_us - (led_on_us / 2);
    if (params->led_start < 0) {
        params->led_start += rotation_us;
    }
    
    params->led_stop = params->led_start + led_on_us;
    if (params->led_stop > rotation_us)
    {
        params->led_stop -= rotation_us;
    }

     // phase transition timing: Currently, only forwards/backwards, so we start the phases at 0 and 1/2 a rotation
    params->motor_start_phase_1 = 0;
    params->motor_start_phase_2 = rotation_us / 2;

    // The throttle PID control
    pid_target_rpm = c->target_rpm;
    throttle_pid.Compute();
    params->throttle_perk = (int) pid_throttle_output;

    params->max_throttle_offset = (int) c->translate_forback * params->throttle_perk * c->translate_trim / 1024;

    params->battery_percent = robot.get_battery();
}

// Arduino loop function. Runs in CPU 1.
// todo - low-battery state
void loop() {
    ctrl_state* c = ctrl_update(true);

    if (!c->connected) {
        throttle_pid.SetMode(MANUAL);
        state = NO_CONTROLLER;
    } else if (!c->alive) {
        throttle_pid.SetMode(MANUAL);
        state = CONTROLLER_STALE;
    } else if (c->spin_requested) {
        if (state != SPINNING) {
            // we're just starting to spin. Start the PID
            throttle_pid.SetMode(AUTOMATIC);
        }

        calculate_melty_params(&control_params, c);
        state = SPINNING;
    } else {
        throttle_pid.SetMode(MANUAL);
        state = READY;

        tank_params.translate_forback = c->translate_forback;
        tank_params.turn_lr = c->turn_lr;
    }

    if (c->trim_right) {
        robot.trim_accel(false, c->target_rpm);
    }

    if (c->trim_left) {
        robot.trim_accel(true, c->target_rpm);
    }

    if (millis() - last_logged_at > 200) {
        // state: 0=NO_CONTROLLER 1=CONTROLLER_STALE 2=READY 3=SPINNING
        Serial.printf(
            "state:%d conn:%d alive:%d | spin:%d fwd:%4d turn:%4d | accel1:%.3f accel2:%.3f rpm_calc:%.1f | trim:%.4f | bat:%d\n",
            (int)state,
            c->connected, c->alive,
            c->spin_requested, c->translate_forback, c->turn_lr,
            robot.get_accel_1_g(), robot.get_accel_2_g(),
            robot.get_rpm(c->target_rpm),
            robot.get_accel_trim(c->target_rpm),
            robot.get_battery()
        );
        last_logged_at = millis();
    }

    // The main loop must have some kind of "yield to lower priority task" event.
    // Otherwise, the watchdog will get triggered.
    // If your main loop doesn't have one, just add a simple `vTaskDelay(1)`.
    // Detailed info here:
    // https://stackoverflow.com/questions/66278271/task-watchdog-got-triggered-the-tasks-did-not-reset-the-watchdog-in-time

    vTaskDelay(10);
}

// The robot control loop. Runs in CPU 0.
void hotloopFN(void* parameter) {
    while(true) {
        // do the magic stuff
        robot.update_loop(state, &control_params, &tank_params);

        //TODO: Up
    }
}