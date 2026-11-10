// sensors_mock.hpp — mock IMU, attitude fix and tachometer producing the
// same raw packets the flight drivers would read off the bus.
// Deterministic: seeded xorshift64*, Gaussian via Irwin-Hall (no libm).
#pragma once
#include <cstdint>
#include "../flight/adcs_config.hpp"
#include "../flight/sensor_packets.hpp"

namespace sim {

class Rng {
public:
    explicit Rng(uint64_t seed) : s_(seed ? seed : 0x9E3779B97F4A7C15ull) {}
    double uniform() {                       // [0, 1)
        s_ ^= s_ >> 12; s_ ^= s_ << 25; s_ ^= s_ >> 27;
        return static_cast<double>((s_ * 0x2545F4914F6CDD1Dull) >> 11) * (1.0 / 9007199254740992.0);
    }
    double gauss() {                         // ~N(0,1), sum of 12 uniforms
        double acc = 0.0;
        for (int k = 0; k < 12; ++k) acc += uniform();
        return acc - 6.0;
    }
private:
    uint64_t s_;
};

inline int16_t sat16(double x) {
    if (x >  32767.0) return  32767;
    if (x < -32768.0) return -32768;
    return static_cast<int16_t>(x >= 0 ? x + 0.5 : x - 0.5);
}

constexpr double kRad2Deg = 57.29577951308232;

// Gyro: true rate + constant bias + white noise, quantized to 16-bit LSB.
struct MockGyro {
    double bias_dps;        // turn-on bias
    double noise_dps;       // 1-sigma per sample
    Rng    rng;
    uint8_t seq = 0;

    adcs::ImuSample sample(double true_rate_rad_s, uint32_t t_us) {
        const double dps = true_rate_rad_s * kRad2Deg + bias_dps + noise_dps * rng.gauss();
        const double lsb = dps * adcs::cfg::kGyroLsbPerDps;
        adcs::ImuSample s{};
        s.t_us     = t_us;
        s.gyro_z   = sat16(lsb);
        s.temp_raw = 2500;
        s.seq      = seq++;
        s.status   = 0x1u | ((lsb > 32767.0 || lsb < -32768.0) ? 0x2u : 0u);
        return s;
    }
};

// Star tracker: absolute angle with white noise, microradian output.
struct MockAttitudeSensor {
    double noise_urad;
    Rng    rng;
    adcs::AttitudeFix sample(double true_angle_rad, uint32_t t_us) {
        adcs::AttitudeFix f{};
        f.t_us       = t_us;
        f.angle_urad = static_cast<int32_t>(true_angle_rad * 1e6 + noise_urad * rng.gauss());
        f.valid      = 1;
        return f;
    }
};

// Hall tach: 1 rpm quantization.
inline adcs::WheelTach tach_sample(double W_rad_s) {
    return adcs::WheelTach{sat16(W_rad_s * 60.0 / 6.283185307179586)};
}

}  // namespace sim
