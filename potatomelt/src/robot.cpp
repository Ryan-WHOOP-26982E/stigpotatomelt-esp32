#include "robot.h"
#include "melty_config.h"

int perk2dshot(int throttle) {
  if (throttle == 0) {
    return 0;
  }
  
  if (throttle > 0) {
    return min(throttle, 999) + 48;
  }

  // otherwise, throttle_perk must be less than zero
  return min(throttle*-1, 998) + 1049;
}

Robot::Robot():
    motor1(MOTOR_1_PIN, MOTOR_1_RMT),
    motor2(MOTOR_2_PIN, MOTOR_2_RMT)  {
}

void Robot::update_loop(robot_status state, spin_control_parameters_t* spin_params, tank_control_parameters_t* tank_params) {
    switch(state) {
        default:
        case NO_CONTROLLER:
            leds.leds_on_no_controller();
            motors_stop();
            break;
        case CONTROLLER_STALE:
            leds.leds_on_controller_stale();
            motors_stop();
            break;
        case READY:
            leds.leds_on_ready();
            drive_tank(tank_params);
            break;
        case SPINNING:
            spin(spin_params);
    }
}

void Robot::spin(spin_control_parameters_t* spin_params) {
    long time_spent_this_rotation_us = micros() - rotation_started_at_us;

    if (time_spent_this_rotation_us > spin_params->rotation_interval_us) {
        time_spent_this_rotation_us -= spin_params->rotation_interval_us;
        rotation_started_at_us += spin_params->rotation_interval_us;
    }

    double throttle_offset = 0;

    // translation math time - first, how far into this phase of rotation are we?
    long micros_into_phase = time_spent_this_rotation_us % (spin_params->rotation_interval_us/2);
    float phase_progress = 2.0 * micros_into_phase / (spin_params->rotation_interval_us);

    // What does that mean the sine (approximation) of that distance into the phase is?
    // Using a parabolic approximation of half a sine wave, that goes from Y=0 to 1 and back in the range X=0..1
    float phase_offset_fraction = -4 * phase_progress * (phase_progress - 1);

    throttle_offset = (double) (phase_offset_fraction * spin_params->max_throttle_offset);

    if (time_spent_this_rotation_us >= spin_params->motor_start_phase_1 && time_spent_this_rotation_us <= spin_params->motor_start_phase_2) {
        motor1.sendThrottleValue(perk2dshot(spin_params->throttle_perk + throttle_offset));
        motor2.sendThrottleValue(perk2dshot(spin_params->throttle_perk - throttle_offset));
    } else {
        motor1.sendThrottleValue(perk2dshot(spin_params->throttle_perk - throttle_offset));
        motor2.sendThrottleValue(perk2dshot(spin_params->throttle_perk + throttle_offset));
    }
   
    // displays heading LED at correct location
    if (spin_params->led_start > spin_params->led_stop) {
    // the LED will be on across 0, so each loop we're shutting it off and then turning it on
    if (time_spent_this_rotation_us >= spin_params->led_start || time_spent_this_rotation_us <= spin_params->led_stop) {
      leds.leds_on_gradient(spin_params->battery_percent);
    } else {
      leds.leds_off();
    }
  } else {
    if (time_spent_this_rotation_us >= spin_params->led_start && time_spent_this_rotation_us <= spin_params->led_stop) {
      leds.leds_on_gradient(spin_params->battery_percent);
    } else {
      leds.leds_off();
    }
  }
}

void Robot::motors_stop() {
    motor1.sendThrottleValue(0);
    motor2.sendThrottleValue(0);
}

void Robot::drive_tank(tank_control_parameters_t* params) {
    if (abs(params->translate_forback) > CONTROL_TRANSLATE_DEADZONE || abs(params->turn_lr) > CONTROL_TRANSLATE_DEADZONE) {
        int forback = params->translate_forback * TANK_FORBACK_POWER_SCALE;
        int leftright = params->turn_lr * TANK_TURNING_POWER_SCALE;

        motor1.sendThrottleValue(perk2dshot(forback + leftright));
        motor2.sendThrottleValue(perk2dshot(-1 * (forback - leftright)));
    } else {
        motors_stop();
    }
}

float Robot::get_z_buffer() {
    return imu.z_accel_buffer;
}

float Robot::get_rpm(int target_rpm) {
    return imu.get_rpm(target_rpm);
}

int Robot::get_battery() {
    return battery.get_percent();
}

void Robot::trim_accel(bool increase, int target_rpm) {
  imu.trim(increase, target_rpm);
}

float Robot::get_accel_trim(int target_rpm) {
  return imu.get_trim(target_rpm);
}

float Robot::get_accel_1_g() {
  return imu.get_accel_1_g();
}

float Robot::get_accel_2_g() {
  return imu.get_accel_2_g();
}

void Robot::init() {
    imu.init();
    motor1.begin(DSHOT600);
    motor2.begin(DSHOT600);
}