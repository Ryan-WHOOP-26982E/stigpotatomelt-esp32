#include <math.h>

#include "accelerometer.h"
#include "../lib/SparkFun_LIS331_ESP32.h"

Accelerometer::Accelerometer() { }

void Accelerometer::init(int addr) {
    lis.setI2CAddr(addr);
    lis.begin(LIS331ESP::USE_I2C);
    lis.setFullScale(LIS331ESP::HIGH_RANGE);
}

// Assumption: We're only calling this when the bot is at rest, right-side-up, in a 1g environment.
void Accelerometer::sample_offset() {
    int16_t x, y, z;
    lis.readAxes(x, y, z);

    float xg = lis.convertToG(400, x);
    float yg = lis.convertToG(400, y);
    float zg = lis.convertToG(400, z);

    sample_count++;
    summed_x_samples += xg;
    x_offset = summed_x_samples/sample_count;
    summed_y_samples += yg;
    y_offset = summed_y_samples/sample_count;
    summed_z_samples += zg;
    z_offset = summed_z_samples/sample_count - 1.0;
}

float Accelerometer::get_z_accel() {
    int16_t x, y, z;
    lis.readAxes(x, y, z);
    float zg = lis.convertToG(400, z);
    return zg - z_offset;
}

float Accelerometer::get_xy_accel() {
    int16_t x, y, z;
    lis.readAxes(x, y, z);
    float zg = lis.convertToG(400, z);
    // Accelerometer is mounted vertically: centripetal force acts on Z axis, not XY.
    // z_offset was calculated as avg_z - 1.0 (assuming Z was vertical/gravity axis),
    // so (z_offset + 1.0) recovers the true resting Z average to subtract as baseline.
    return fabs(zg - (z_offset + 1.0f));
}
