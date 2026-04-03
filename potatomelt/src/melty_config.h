#include <Arduino.h>
#include <driver/rmt.h>

// This file has all the hard-coded settings for Potatomelt

// ------------ safety settings ----------------------
#define CONTROL_UPDATE_TIMEOUT_MS 3000

// ------------ Spin control settings ----------------
#define ACCELEROMETER_HARDWARE_RADIUS_CM 2.738f
#define LED_OFFSET_PERCENT 47

#define LEFT_RIGHT_HEADING_CONTROL_DIVISOR 2.0f   // How quick steering while melting is (larger values = slower)
#define MIN_TRACKING_RPM 400
#define MAX_TRACKING_ROTATION_INTERVAL_US (1.0f / MIN_TRACKING_RPM) * 60 * 1000 * 1000 // don't track heading if we are this slow

#define MAX_TRACKING_RPM 3000

// ------------ control parameters -------------------
#define CONTROL_TRANSLATE_DEADZONE 50
#define CONTROL_SPIN_SPEED_DEADZONE 200

#define TANK_FORBACK_POWER_SCALE 0.02f // Scale the power waaaaay down on tank mode
#define TANK_TURNING_POWER_SCALE 0.005f // because we're sitting on a pair of ungeared brushless motors

// ------------ PID tuning ---------------------------
// Tuning PIDs is an art. See: https://pidexplained.com/how-to-tune-a-pid-controller/

#define PID_KP 1.0                                  // Proportional Gain
#define PID_KI 0.4                                  // Integral
#define PID_KD 0.0                                  // Derivative

// ------------ SBUS receiver configuration ----------
// Receiver TX → D7 (GPIO44) on XIAO ESP32-S3
// Receiver RX → D6 (GPIO43) on XIAO ESP32-S3 (not needed for receive-only SBUS)
#define SBUS_RX_PIN D7              // Receiver TX → D7 on XIAO ESP32-S3
#define SBUS_TX_PIN D6              // Receiver RX → D6 (not used for receive-only)
#define SBUS_CENTER 992             // SBUS center value (full range: 172–1811)
#define SBUS_HALF_RANGE 820         // (1811 - 172) / 2
#define SBUS_SPIN_THRESHOLD 1400    // Throttle channel above this = spin requested
#define SBUS_SWITCH_THRESHOLD 1400  // Any switch channel above this = ON

// SBUS channel mapping (0-indexed: CH1 on transmitter = index 0)
// Adjust these to match your transmitter's channel assignments.
#define SBUS_CH_TURN            0   // CH1: Heading turn        (right stick X)
#define SBUS_CH_TRANSLATE       2   // CH2: Forward/back        (right stick Y)
#define SBUS_CH_THROTTLE        1   // CH3: Spin trigger        (left stick Y / throttle)
#define SBUS_CH_TRANSLATE_LR    3   // CH4: Left/right trans.   (reserved, not currently used)
#define SBUS_CH_REVERSE         4   // CH5: Reverse spin dir.   (switch)
#define SBUS_CH_RPM_ADJUST      9   // CH6: Target RPM adjust   (switch or dial)
#define SBUS_CH_TRIM_TRANS_UP   6   // CH7: Increase trans trim (momentary switch)
#define SBUS_CH_TRIM_TRANS_DOWN 7   // CH8: Decrease trans trim (momentary switch)
#define SBUS_CH_TRIM_LEFT       5   // CH9: Motor trim left     (momentary switch)
#define SBUS_CH_TRIM_RIGHT      8   // CH10: Motor trim right   (momentary switch)

// ------------ Pin and RMT Mappings -----------------

#define LED_1_PIN GPIO_NUM_2      // D1
#define LED_2_PIN GPIO_NUM_3      // D2
#define MOTOR_1_PIN GPIO_NUM_9    // D10
#define MOTOR_2_PIN GPIO_NUM_8    // D9

#define MOTOR_1_RMT RMT_CHANNEL_0
#define MOTOR_2_RMT RMT_CHANNEL_1

#define I2C_SDA_PIN 5
#define I2C_SCL_PIN 6
#define BATTERY_ADC_PIN 10

// ------------ Battery Configuration ---------------

#define BATTERY_ALERT_ENABLED                     // if enabled - heading LED will flicker when battery voltage is low
#define BATTERY_CRIT_HALT_ENABLED                 // if enabled - robot will halt when battery voltage is critically low
#define BATTERY_VOLTAGE_DIVIDER 8.24              // From the PCB - what's the voltage divider between the battery + and the sense line?
#define BATTERY_CELL_COUNT 3                      // 3S LiPo
#define BATTERY_CELL_FULL_VOLTAGE 4.2             // Fully-charged cell voltage (standard LiPo)
#define BATTERY_CELL_EMPTY_VOLTAGE 3.2            // Empty cell voltage cutoff
#define LOW_BAT_REPEAT_READS_BEFORE_ALARM 20      // Requires this many ADC reads below threshold before halting
