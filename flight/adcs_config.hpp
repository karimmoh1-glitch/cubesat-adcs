// adcs_config.hpp — compile-time configuration for the 1-axis reaction wheel ADCS.
//
// Flight code is freestanding: the only header it may include is <cstdint>
// (available with -ffreestanding). No heap, no exceptions, no RTTI, no libm.
// Every gain and reciprocal is folded at compile time so the control step
// contains no divisions.
#pragma once
#include <cstdint>

namespace adcs::cfg {

inline constexpr float kPi       = 3.14159265358979323846f;
inline constexpr float kTwoPi    = 2.0f * kPi;
inline constexpr float kDegToRad = kPi / 180.0f;
inline constexpr float kRadToDeg = 180.0f / kPi;
inline constexpr float kRpmToRad = kTwoPi / 60.0f;
inline constexpr float kRadToRpm = 60.0f / kTwoPi;

// ---- Timing ---------------------------------------------------------------
inline constexpr uint32_t kControlHz = 1000;                 // control ISR rate
inline constexpr float    kDt        = 1.0f / kControlHz;    // [s]
inline constexpr uint32_t kStarTrackerDivider = 100;         // 10 Hz absolute fix

// ---- Rigid body (6U CubeSat, rotation about body Z) ----------------------
inline constexpr float kBodyInertia  = 0.050f;               // J_b [kg m^2]

// ---- Reaction wheel + BLDC motor (nominal datasheet values) --------------
inline constexpr float kWheelInertia = 5.0e-5f;              // J_w [kg m^2]
inline constexpr float kKt           = 0.010f;               // torque const [N m / A]
inline constexpr float kKe           = 0.010f;               // back-EMF const [V s / rad]
inline constexpr float kWindingR     = 2.0f;                 // [ohm]
inline constexpr float kVbus         = 8.0f;                 // [V]
inline constexpr float kImax         = 2.0f;                 // driver current limit [A]
inline constexpr float kTorqueMax    = kKt * kImax;          // 20 mN m
inline constexpr float kFrictionCoulomb = 2.0e-4f;           // [N m]  (feed-forward model)
inline constexpr float kFrictionViscous = 2.0e-7f;           // [N m s / rad]

// PWM timer: 80 MHz / 20 kHz = 4000 counts full scale, signed for H-bridge direction.
inline constexpr int16_t kPwmMax = 4000;

// ---- Sensors ---------------------------------------------------------------
inline constexpr float kGyroLsbPerDps = 65.5f;               // +/-500 dps range, 16-bit
inline constexpr float kGyroRadPerLsb = kDegToRad / kGyroLsbPerDps;

// ---- Loop design: PID + integrator pole placement --------------------------
// Closed loop  J s^3 + Kd s^2 + Kp s + Ki  placed at
//   (s^2 + 2 zeta wn s + wn^2)(s + p)
inline constexpr float kWn   = 8.0f;     // [rad/s]
inline constexpr float kZeta = 0.9f;
inline constexpr float kPint = 2.0f;     // integrator pole [rad/s]

inline constexpr float kKp = kBodyInertia * (kWn * kWn + 2.0f * kZeta * kWn * kPint);
inline constexpr float kKi = kBodyInertia * (kWn * kWn * kPint);
inline constexpr float kKd = kBodyInertia * (2.0f * kZeta * kWn + kPint);

// Max approach rate under saturation. The P-path error is clamped so that a
// large error commands a constant slew rate instead of winding the loop into a
// saturated overshoot:  e_max = w_max * Kd / Kp.
inline constexpr float kMaxSlewRate = 0.12f;                 // [rad/s]
inline constexpr float kErrClamp    = kMaxSlewRate * kKd / kKp;

inline constexpr float kIntegralLimit = 0.25f * kTorqueMax;  // integrator authority [N m]
// Integral separation: accumulate only in the fine-pointing zone, so large
// transients (knocks, slews) cannot charge the integrator and cause a slow tail.
inline constexpr float kIntegralZone  = 0.15f * kDegToRad;   // [rad]
inline constexpr float kDerivCutoffHz = 20.0f;               // D-path low-pass (~15x loop bandwidth)

}  // namespace adcs::cfg
