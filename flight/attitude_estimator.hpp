// attitude_estimator.hpp — 1-axis gyro propagation with absolute-fix
// correction and online gyro-bias estimation (steady-state 2-state observer).
//
// The angle correction is spread over the fix interval as a rate term instead
// of applied as a step: a 10 Hz staircase in the attitude estimate would be
// chased by the 8 rad/s pointing loop and show up as body-rate jitter.
//
// Error dynamics per fix interval T, with x = angle error, y = T * bias error:
//   [x+]   [1-L1  -1] [x]
//   [y+] = [ L2    1] [y]       eig |lambda| = sqrt(1 - L1 + L2)
// L1 = 0.3, L2 = 0.05  ->  |lambda| = 0.87 per fix, settles in ~2 s at 10 Hz.
#pragma once
#include "adcs_config.hpp"

namespace adcs {

[[gnu::always_inline]] inline constexpr float wrap_pi(float a) {
    // Inputs are at most one turn out of range; no fmod needed.
    if (a >  cfg::kPi) a -= cfg::kTwoPi;
    if (a < -cfg::kPi) a += cfg::kTwoPi;
    return a;
}

class AttitudeEstimator {
public:
    static constexpr float kL1 = 0.3f;
    static constexpr float kL2 = 0.05f;
    static constexpr float kFixPeriod = cfg::kDt * cfg::kStarTrackerDivider;
    static constexpr float kL1OverT  = kL1 / kFixPeriod;
    static constexpr float kL2OverT  = kL2 / kFixPeriod;

    // Bias comes from ground calibration; the angle is seeded by the first fix
    // (an unseeded estimator would read the initial offset as gyro bias).
    constexpr void init(float bias) { bias_ = bias; corr_rate_ = 0.0f; seeded_ = false; }

    // Called every control tick with the calibrated gyro rate [rad/s].
    [[gnu::always_inline]] inline void propagate(float gyro_rate) {
        rate_  = gyro_rate - bias_;
        angle_ = wrap_pi(angle_ + (rate_ + corr_rate_) * cfg::kDt);
    }

    // Called when an absolute fix arrives.
    inline void correct(float measured_angle) {
        if (!seeded_) { angle_ = measured_angle; seeded_ = true; return; }
        const float innov = wrap_pi(measured_angle - angle_);
        corr_rate_ = kL1OverT * innov;     // applied over the next T seconds
        bias_     -= kL2OverT * innov;
    }

    constexpr float angle() const { return angle_; }
    constexpr float rate()  const { return rate_; }
    constexpr float bias()  const { return bias_; }

private:
    float angle_ = 0.0f, rate_ = 0.0f, bias_ = 0.0f, corr_rate_ = 0.0f;
    bool  seeded_ = false;
};

}  // namespace adcs
