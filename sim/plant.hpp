// plant.hpp — "truth" physics for the ground harness (double precision).
//
// State: body angle theta, body rate w_b, wheel speed W relative to the body.
//   motor:   i = clamp((duty*Vbus - Ke*W) / R, +/-Imax),  tau_m = Kt * i
//   bearing: tau_f = c * sgn(W) + b * W
//   body:    J_b * dw_b/dt = -(tau_m - tau_f) + tau_ext
//   wheel:   J_w * (dw_b/dt + dW/dt) = tau_m - tau_f     (absolute rotor accel)
// Total angular momentum H = J_b w_b + J_w (w_b + W) changes only by tau_ext.
// Integrated with fixed-step RK4.
#pragma once

namespace sim {

struct MotorParams { double kt, ke, r, vbus, imax, coulomb, viscous; };
struct PlantParams { double Jb, Jw; MotorParams m; };
struct PlantState  { double theta, w_b, W; };

class Plant {
public:
    Plant(const PlantParams& p, const PlantState& s0) : p_(p), s_(s0) {}

    void step(double duty, double tau_ext, double dt) {
        auto f = [&](const PlantState& s) { return deriv(s, duty, tau_ext); };
        const PlantState k1 = f(s_);
        const PlantState k2 = f(add(s_, k1, 0.5 * dt));
        const PlantState k3 = f(add(s_, k2, 0.5 * dt));
        const PlantState k4 = f(add(s_, k3, dt));
        s_.theta += dt / 6.0 * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta);
        s_.w_b   += dt / 6.0 * (k1.w_b   + 2 * k2.w_b   + 2 * k3.w_b   + k4.w_b);
        s_.W     += dt / 6.0 * (k1.W     + 2 * k2.W     + 2 * k3.W     + k4.W);
        const double twopi = 6.283185307179586;
        if (s_.theta >  0.5 * twopi) s_.theta -= twopi;
        if (s_.theta < -0.5 * twopi) s_.theta += twopi;
        last_current_ = current(s_.W, duty);
    }

    const PlantState& state() const { return s_; }
    double momentum() const { return p_.Jb * s_.w_b + p_.Jw * (s_.w_b + s_.W); }
    double current_draw() const { return last_current_; }
    double wheel_torque() const { return p_.m.kt * last_current_; }

private:
    double current(double W, double duty) const {
        double i = (duty * p_.m.vbus - p_.m.ke * W) / p_.m.r;
        if (i >  p_.m.imax) i =  p_.m.imax;
        if (i < -p_.m.imax) i = -p_.m.imax;
        return i;
    }
    PlantState deriv(const PlantState& s, double duty, double tau_ext) const {
        const double tau_m  = p_.m.kt * current(s.W, duty);
        const double absW   = s.W < 0 ? -s.W : s.W;
        const double tau_f  = p_.m.coulomb * s.W / (absW + 1.0) + p_.m.viscous * s.W;
        const double tau_net = tau_m - tau_f;
        const double wb_dot = (-tau_net + tau_ext) / p_.Jb;
        const double W_dot  = tau_net / p_.Jw - wb_dot;
        return {s.w_b, wb_dot, W_dot};
    }
    static PlantState add(const PlantState& a, const PlantState& k, double h) {
        return {a.theta + h * k.theta, a.w_b + h * k.w_b, a.W + h * k.W};
    }

    PlantParams p_;
    PlantState  s_;
    double last_current_ = 0.0;
};

// Triangular torque pulse: models a knock (debris strike, deployment snag,
// thruster misfire). Impulse = peak * duration / 2.
struct Knock {
    double t_start, duration, peak_torque;
    double torque(double t) const {
        const double x = t - t_start;
        if (x < 0.0 || x > duration) return 0.0;
        const double half = 0.5 * duration;
        return peak_torque * (x < half ? x / half : (duration - x) / half);
    }
    double impulse() const { return 0.5 * peak_torque * duration; }
};

}  // namespace sim
