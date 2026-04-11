#include <PID_v1.h>
#include <Preferences.h>
#include <Wire.h>
#include "src/robot.h"
#include "src/melty_config.h"
#include "src/controller.h"
#include "src/subsystems/storage.h"

#ifdef TELEMETRY_ENABLED
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP telemetry_udp;
IPAddress telemetry_host;
static int telemetry_send_counter = 0;
#endif

TaskHandle_t hotloop;

Robot robot;

robot_status state;
spin_control_parameters_t control_params;
tank_control_parameters_t tank_params;

Storage store;

long last_logged_at = 0;

// Spin data logger — captures RPM + throttle during spin, dumps to serial on stop
#define SPIN_LOG_SIZE 400
struct SpinLogEntry { uint32_t ms; float rpm; int throttle; };
SpinLogEntry spin_log[SPIN_LOG_SIZE];
int spin_log_idx = 0;
bool was_spinning = false;

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
    delay(1500); // wait for USB CDC serial to connect before printing boot messages

    Serial.println("PotatoMelt startup");

    // Print last spin log if one was saved before reset
    {
        Preferences prefs;
        prefs.begin("spinlog", true);
        int count = prefs.getInt("count", 0);
        if (count > 0) {
            uint32_t dur = prefs.getUInt("dur", 0);
            float maxrpm = prefs.getFloat("maxrpm", 0);
            float avgrpm = prefs.getFloat("avgrpm", 0);
            int maxthr = prefs.getInt("maxthr", 0);
            float loop_ms = dur > 0 ? (float)dur / count : 0;
            Serial.printf("=== LAST SPIN: %d entries / %ums = %.1fms/loop | maxRPM:%.0f avgRPM:%.0f maxThr:%d ===\n",
                count, dur, loop_ms, maxrpm, avgrpm, maxthr);
            prefs.end();
            prefs.begin("spinlog", false);
            prefs.putInt("count", 0);
        } else {
            Serial.println("=== No previous spin log ===");
        }
        prefs.end();
    }

    // set up I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    // start data storage and recall
    store.init();

    // Configure the PID
    throttle_pid.SetOutputLimits(0.0, 1023.0);

    // start the robot subsystems (SparkFun LIS331 library calls Wire.begin()
    // internally, which resets clock and timeout — so we re-apply them AFTER)
    robot.init();
    Wire.setClock(400000);  // back to 400kHz — faster reads, fewer retries
    Wire.setTimeOut(10);
    state = NO_CONTROLLER;

    // start the SBUS control interface
    ctrl_init();

#ifdef TELEMETRY_ENABLED
    WiFi.softAP(TELEMETRY_WIFI_SSID, TELEMETRY_WIFI_PASS);
    telemetry_host.fromString(TELEMETRY_UDP_HOST);
    telemetry_udp.begin(TELEMETRY_UDP_PORT);
    Serial.printf("Telemetry: connect to WiFi '%s' | UDP %s:%d\n",
        TELEMETRY_WIFI_SSID, TELEMETRY_UDP_HOST, TELEMETRY_UDP_PORT);
#endif

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
    params->reverse_spin = c->reverse_spin;
}

// Arduino loop function. Runs in CPU 1.
// todo - low-battery state
void loop() {
    // Measure actual time between loop() invocations (includes vTaskDelay)
    static uint32_t last_loop_start_us = 0;
    uint32_t loop_start_us = micros();
    uint32_t loop_elapsed_us = loop_start_us - last_loop_start_us;
    last_loop_start_us = loop_start_us;

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

    // Log RPM + throttle while spinning; dump to serial the moment spin stops
    if (state == SPINNING) {
        if (!was_spinning) { spin_log_idx = 0; was_spinning = true; } // reset on new spin
        if (spin_log_idx < SPIN_LOG_SIZE) {
            spin_log[spin_log_idx++] = { millis(), (float)pid_current_rpm, (int)pid_throttle_output };
        }
    } else if (was_spinning) {
        was_spinning = false;
        // Save summary to NVS so it survives the USB reset when plugging back in
        if (spin_log_idx > 1) {
            float max_rpm = 0, sum_rpm = 0;
            int max_thr = 0;
            uint32_t duration_ms = spin_log[spin_log_idx-1].ms - spin_log[0].ms;
            for (int i = 0; i < spin_log_idx; i++) {
                if (spin_log[i].rpm > max_rpm) max_rpm = spin_log[i].rpm;
                if (spin_log[i].throttle > max_thr) max_thr = spin_log[i].throttle;
                sum_rpm += spin_log[i].rpm;
            }
            Preferences prefs;
            prefs.begin("spinlog", false);
            prefs.putInt("count", spin_log_idx);
            prefs.putUInt("dur", duration_ms);
            prefs.putFloat("maxrpm", max_rpm);
            prefs.putFloat("avgrpm", sum_rpm / spin_log_idx);
            prefs.putInt("maxthr", max_thr);
            prefs.end();
        }
    }

    if (c->trim_right) {
        robot.trim_accel(false, c->target_rpm);
    }

    if (c->trim_left) {
        robot.trim_accel(true, c->target_rpm);
    }

    if (millis() - last_logged_at > 200) {
        // state: 0=SPINNING 1=READY 2=LOW_BAT 3=STALE 4=NO_CTRL
        // pid_current_rpm is updated each spin loop — no I2C read here
        Serial.printf(
            "state:%d conn:%d alive:%d | spin:%d fwd:%4d turn:%4d | rpm:%.1f | trim:%.4f | bat:%d | loop:%.1fms\n",
            (int)state,
            c->connected, c->alive,
            c->spin_requested, c->translate_forback, c->turn_lr,
            (float)pid_current_rpm,
            robot.get_accel_trim(c->target_rpm),
            robot.get_battery(),
            loop_elapsed_us / 1000.0f
        );
        last_logged_at = millis();
    }

#ifdef TELEMETRY_ENABLED
    // Send UDP telemetry packet every TELEMETRY_SEND_EVERY_N loops.
    // CSV format: ms,state,target_rpm,rpm,throttle,bat,connected,loop_us,trim
    // NOTE: WiFi task runs on core 0 alongside hotloop — disable for arena runs.
    if (++telemetry_send_counter >= TELEMETRY_SEND_EVERY_N) {
        telemetry_send_counter = 0;
        char pkt[128];
        int n = snprintf(pkt, sizeof(pkt), "%lu,%d,%d,%.1f,%.1f,%d,%d,%lu,%.4f,%.3f\n",
            (unsigned long)millis(),
            (int)state,
            (int)pid_target_rpm,
            (float)pid_current_rpm,
            (float)pid_throttle_output,
            robot.get_battery(),
            (int)c->connected,
            (unsigned long)loop_elapsed_us,
            robot.get_accel_trim(c->target_rpm),
            robot.get_accel_1_g()   // raw (unfiltered) g reading for comparison
        );
        telemetry_udp.beginPacket(telemetry_host, TELEMETRY_UDP_PORT);
        telemetry_udp.write((uint8_t*)pkt, n);
        telemetry_udp.endPacket();
    }
#endif

    // Yield 1 tick to prevent watchdog and allow lower-priority tasks to run.
    // vTaskDelay(10) was used here but caused ~35ms loops on ESP32-S3 because
    // Seeed's board package configures FreeRTOS at 250Hz (4ms/tick), so
    // vTaskDelay(10) = 40ms, not 10ms. vTaskDelay(1) yields the minimum
    // (1 tick = ~4ms), letting the I2C read and loop body set the natural rate.
    vTaskDelay(1);
}

// The robot control loop. Runs in CPU 0.
void hotloopFN(void* parameter) {
    while(true) {
        robot.update_loop(state, &control_params, &tank_params);
        vTaskDelay(1); // ~1ms yield - limits DShot to ~1kHz, well within ESC spec

      }
}
