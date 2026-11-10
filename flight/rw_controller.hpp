// rw_controller.hpp — reaction-wheel attitude control law.
//
// Per tick:  sensors -> estimator -> PID (body torque) -> wheel torque
//            -> winding current -> PWM with back-EMF + friction feed-forward.
//
// Physics used to map body torque to motor drive:
//   Body:   J_b * dw_b/dt = -tau_wheel            (Newton's 3rd law on the rotor)
//   Wheel:  J_w * dW/dt   =  tau_wheel - tau_friction(W)
//   Motor:  tau = Kt * i,  i = (V - Ke * W) / R,  V = duty * Vbus
// so to produce body torque tau_b we need wheel torque -tau_b, current
// -tau_b / Kt, and a voltage that also cancels the back-EMF Ke * W.
#pragma once
#include "adcs_config.hpp"
#include "attitude_estimator.hpp"
#include "pid.hpp"
#include "sensor_packets.hpp"

namespace adcs {

class ReactionWheelController {
public:
    static constexpr PidGains kGains{
        cfg::kKp, cfg::kKi, cfg::kKd,
        cfg::kTorqueMax, cfg::kIntegralLimit, cfg::kErrClamp, cfg::kIntegralZone,
        // alpha = dt / (dt + 1/(2 pi fc))
        cfg::kDt / (cfg::kDt + 1.0f / (cfg::kTwoPi * cfg::kDerivCutoffHz)),
    };

    constexpr ReactionWheelController() : pid_(kGains, cfg::kDt) {}

    constexpr void set_target(float angle_rad, float rate_rad_s = 0.0f) {
        ref_angle_ = angle_rad;
        ref_rate_  = rate_rad_s;
    }
    constexpr void init_estimator(float gyro_bias) { est_.init(gyro_bias); }

    // ---- Sensor ingestion (from driver ISRs / DMA callbacks) -------------
    inline void ingest(const ImuSample& s) {
        if (s.status & 0x1u) gyro_rate_ = static_cast<float>(s.gyro_z) * cfg::kGyroRadPerLsb;
    }
    inline void ingest(const AttitudeFix& f) {
        if (f.valid) { fix_ = static_cast<float>(f.angle_urad) * 1.0e-6f; fix_pending_ = true; }
    }
    inline void ingest(const WheelTach& t) {
        wheel_speed_ = static_cast<float>(t.rpm) * cfg::kRpmToRad;
    }

    // ---- Control step: call at exactly cfg::kControlHz ----------------------
    inline MotorCommand step() {
        est_.propagate(gyro_rate_);
        if (fix_pending_) { est_.correct(fix_); fix_pending_ = false; }

        const float err      = wrap_pi(ref_angle_ - est_.angle());
        const float err_rate = ref_rate_ - est_.rate();
        const float tau_body = pid_.update(err, err_rate);

        // Wheel torque is the reaction; add friction the motor must overcome.
        const float W = wheel_speed_;
        const float sgn = W / ((W < 0.0f ? -W : W) + 1.0f);   // smooth sign, no libm
        const float tau_fric = cfg::kFrictionCoulomb * sgn + cfg::kFrictionViscous * W;
        const float tau_wheel = -tau_body;

        const float i_cmd = clampf((tau_wheel + tau_fric) * kInvKt, -cfg::kImax, cfg::kImax);
        const float volts = i_cmd * cfg::kWindingR + cfg::kKe * W;
        const float duty  = clampf(volts * kInvVbus, -1.0f, 1.0f);

        const float pwm_f = duty * static_cast<float>(cfg::kPwmMax);
        const int16_t pwm = static_cast<int16_t>(pwm_f + (pwm_f >= 0.0f ? 0.5f : -0.5f));

        MotorCommand cmd;
        cmd.pwm             = pwm;
        cmd.duty            = static_cast<float>(pwm) * kInvPwmMax;
        cmd.torque_cmd      = tau_body;
        cmd.current_cmd     = i_cmd;
        cmd.wheel_speed_cmd = W + tau_wheel * kDtOverJw;
        cmd.p = pid_.p_term();
        cmd.i = pid_.i_term();
        cmd.d = pid_.d_term();
        return cmd;
    }

    constexpr const AttitudeEstimator& estimator() const { return est_; }

private:
    static constexpr float kInvKt     = 1.0f / cfg::kKt;
    static constexpr float kInvVbus   = 1.0f / cfg::kVbus;
    static constexpr float kInvPwmMax = 1.0f / static_cast<float>(cfg::kPwmMax);
    static constexpr float kDtOverJw  = cfg::kDt / cfg::kWheelInertia;

    AttitudeEstimator est_;
    Pid   pid_;
    float ref_angle_ = 0.0f, ref_rate_ = 0.0f;
    float gyro_rate_ = 0.0f, wheel_speed_ = 0.0f, fix_ = 0.0f;
    bool  fix_pending_ = false;
};

}  // namespace adcs
