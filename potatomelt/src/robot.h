#include "subsystems/accelerometer.h"
#include "subsystems/battery.h"
#include "subsystems/imu.h"
#include "subsystems/led.h"
#include "lib/DShotRMT.h"
#include "melty_config.h"

// The main struct shared by the robot side and the control side threads - contains the state of what we want the robot to do
typedef struct spin_control_parameters_t {
    int throttle_perk;               // stores throttle, out of 0-1000
    int max_throttle_offset;            // In a rotation, the furthest from the base throttle setting that each motor should be spun
    long rotation_interval_us; // time for 1 rotation of robot
    long led_start;            // offset for beginning of LED beacon
    long led_stop;             // offset for end of LED beacon
    long motor_start_phase_1;  // time offset for when motor 1 begins translating forwards
    long motor_start_phase_2;  // time offset for when motor 2 begins translating forwards
    int battery_percent;                // battery power remaining- where on the green->red slope we should be
};

typedef struct tank_control_parameters_t {
    int translate_forback;
    int turn_lr;
};

enum robot_status {
    SPINNING,
    READY,
    LOW_BATTERY,
    CONTROLLER_STALE,
    NO_CONTROLLER
};

// And the parent Robot class
class Robot {
    public:
        Robot();
        void update_loop(robot_status state, spin_control_parameters_t* spin_params, tank_control_parameters_t* tank_params);
        float get_z_buffer();
        float get_rpm(int target_rpm);
        void init();
        int get_battery();
        void trim_accel(bool increase, int target_rpm);
        float get_accel_trim(int target_rpm);
        float get_accel_1_g();
        float get_accel_2_g();
    private:
        void motors_stop();
        void drive_tank(tank_control_parameters_t* params);
        void spin(spin_control_parameters_t* params);
        unsigned long rotation_started_at_us;
        LED leds;
        Battery battery;
        DShotRMT motor1;
        DShotRMT motor2;
        IMU imu;
};