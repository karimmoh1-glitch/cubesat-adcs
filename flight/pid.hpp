// pid.hpp — allocation-free, branch-light PID for a fixed-rate control ISR.
//
//  * derivative on measurement: the caller passes the error RATE from the
//    gyro, so there is no numerical differentiation and no setpoint kick
//  * first-order low-pass on the D path (precomputed alpha)
//  * error clamp on the P path for saturation-aware slewing
//  * integral separation zone + conditional-integration anti-windup
//  * all coefficients pre-multiplied by dt at construction
#pragma once

namespace adcs {

[[gnu::always_inline]] inline constexpr float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

struct PidGains {
    float kp, ki, kd;
    float out_limit;      // symmetric output saturation
    float integ_limit;    // symmetric integrator state limit (output units)
    float err_clamp;      // symmetric clamp on error fed to P
    float integ_zone;     // integrate only while |err| < integ_zone
    float d_alpha;        // D-path LPF coefficient in (0, 1]; 1 = no filter
};

class Pid {
public:
    constexpr Pid(const PidGains& g, float dt)
        : kp_(g.kp), ki_dt_(g.ki * dt), kd_(g.kd),
          out_lim_(g.out_limit), i_lim_(g.integ_limit),
          e_lim_(g.err_clamp), zone_(g.integ_zone), alpha_(g.d_alpha) {}

    // err      = setpoint - measurement
    // err_rate = d(err)/dt  (= rate_ref - measured_rate)
    [[gnu::always_inline]] inline float update(float err, float err_rate) {
        const float e = clampf(err, -e_lim_, e_lim_);
        d_filt_ += alpha_ * (err_rate - d_filt_);

        p_ = kp_ * e;
        d_ = kd_ * d_filt_;
        const float u_raw = p_ + integ_ + d_;
        const float u     = clampf(u_raw, -out_lim_, out_lim_);

        // Integrate only near the setpoint, and never further into saturation.
        const bool in_zone = (err < zone_) & (err > -zone_);
        const bool wind_hi = (u_raw >  out_lim_) & (e > 0.0f);
        const bool wind_lo = (u_raw < -out_lim_) & (e < 0.0f);
        if (in_zone & !(wind_hi | wind_lo))
            integ_ = clampf(integ_ + ki_dt_ * e, -i_lim_, i_lim_);

        return u;
    }

    constexpr void reset() { integ_ = d_filt_ = p_ = d_ = 0.0f; }

    constexpr float p_term() const { return p_; }
    constexpr float i_term() const { return integ_; }
    constexpr float d_term() const { return d_; }

private:
    float kp_, ki_dt_, kd_, out_lim_, i_lim_, e_lim_, zone_, alpha_;
    float integ_  = 0.0f;
    float d_filt_ = 0.0f;
    float p_ = 0.0f, d_ = 0.0f;
};

}  // namespace adcs
