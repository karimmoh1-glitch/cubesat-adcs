// sensor_packets.hpp — wire-level packets the flight software ingests.
// These mirror what a driver would read from the bus: raw integer counts,
// not engineering units. Conversion happens inside the flight code.
#pragma once
#include <cstdint>

namespace adcs {

// MEMS gyro, Z axis only (SPI burst read).
struct ImuSample {
    uint32_t t_us;       // timestamp from sensor clock
    int16_t  gyro_z;     // raw LSB, kGyroLsbPerDps
    int16_t  temp_raw;   // die temperature, unused by the loop
    uint8_t  seq;        // rolling sequence counter
    uint8_t  status;     // bit0 = data ready, bit1 = saturated
};

// Absolute attitude fix (star tracker / sun sensor), low rate.
struct AttitudeFix {
    uint32_t t_us;
    int32_t  angle_urad; // microradians, wrapped to [-pi, pi]
    uint8_t  valid;
};

// Hall-sensor tachometer on the wheel motor.
struct WheelTach {
    int16_t rpm;         // signed, 1 rpm resolution
};

// Output to the motor driver.
struct MotorCommand {
    int16_t pwm;               // signed counts, +/- kPwmMax
    float   duty;              // pwm / kPwmMax
    float   torque_cmd;        // commanded torque on the BODY [N m]
    float   current_cmd;       // commanded winding current [A]
    float   wheel_speed_cmd;   // wheel speed target for next tick [rad/s]
    float   p, i, d;           // PID term breakdown for telemetry [N m]
};

}  // namespace adcs
