// harness.cpp — closed-loop ground test for the reaction-wheel ADCS.
//
// Runs the flight controller (float, freestanding) against the double-precision
// truth plant with mocked sensors, injects knocks, checks the < 1.5 s
// convergence requirement, and logs telemetry CSVs. Exit code 0 = all pass.
#include <chrono>
#include <cstdio>
#include <cstring>

#include "../flight/rw_controller.hpp"
#include "../sim/plant.hpp"
#include "../sim/sensors_mock.hpp"

namespace {

using adcs::cfg::kDt;
using sim::kRad2Deg;

constexpr int    kSubsteps       = 10;           // physics at 10 kHz
constexpr double kSettleLimitS   = 1.5;          // requirement
constexpr double kAngleTolDeg    = 0.10;         // settled pointing band
constexpr double kRateTolDps     = 0.05;         // settled rate band
constexpr double kPostKnockRunS  = 4.0;
constexpr double kDisturbance    = 5.0e-6;       // SRP + gravity gradient [N m]
constexpr int    kMonteCarloRuns = 25;           // noise seeds per scenario; worst case reported

// True hardware deviates from the nominal values the flight code assumes.
const sim::PlantParams kTruth{
    0.050, 5.0e-5,
    {0.0097, 0.0103, 2.10, 8.0, 2.0, 2.5e-4, 2.5e-7},
};

int g_failures = 0;
void check(bool ok, const char* what) {
    std::printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) ++g_failures;
}
double absd(double x) { return x < 0 ? -x : x; }

// ---------------------------------------------------------------------------
struct Scenario {
    const char* name;
    const char* description;
    double wheel_rpm0;
    double theta0_deg;
    sim::Knock knocks[2];
    int    n_knocks;
    double t_eval;   // settle clock starts here (first knock onset or t=0)
};

struct Result {
    double settle_s, peak_err_deg, peak_rate_dps, peak_wheel_rpm, sat_fraction;
    double final_err_deg;
};

Result run_scenario(const Scenario& sc, const char* out_dir, uint64_t seed, bool log) {
    sim::Plant plant(kTruth, {sc.theta0_deg / kRad2Deg, 0.0, sc.wheel_rpm0 * 6.283185307179586 / 60.0});
    sim::MockGyro gyro{0.25, 0.067, sim::Rng(seed)};
    sim::MockAttitudeSensor tracker{50.0, sim::Rng(seed * 7 + 1)};

    adcs::ReactionWheelController ctrl;
    ctrl.set_target(0.0f);
    ctrl.init_estimator(0.20f * adcs::cfg::kDegToRad);   // ground-cal bias, 0.05 dps off

    double t_end = sc.t_eval + kPostKnockRunS;
    for (int k = 0; k < sc.n_knocks; ++k) {
        const double e = sc.knocks[k].t_start + sc.knocks[k].duration + kPostKnockRunS;
        if (e > t_end) t_end = e;
    }
    const long n_ticks = static_cast<long>(t_end / kDt + 0.5);

    char path[512];
    std::snprintf(path, sizeof path, "%s/%s.csv", out_dir, sc.name);
    FILE* f = log ? std::fopen(path, "w") : nullptr;
    if (f) std::fprintf(f,
        "t_s,theta_true_deg,theta_est_deg,err_deg,rate_true_dps,rate_est_dps,gyro_raw,"
        "bias_est_dps,wheel_rpm,wheel_cmd_rpm,pwm,duty,torque_cmd_mNm,torque_wheel_mNm,"
        "current_A,p_mNm,i_mNm,d_mNm,tau_ext_mNm,H_mNms\n");

    Result r{};
    double last_violation = sc.t_eval;
    long sat_ticks = 0, eval_ticks = 0;

    for (long n = 0; n < n_ticks; ++n) {
        const double t = n * kDt;
        const auto& s = plant.state();
        const uint32_t t_us = static_cast<uint32_t>(t * 1e6);

        // 1. Sensors -> flight software
        const adcs::ImuSample imu = gyro.sample(s.w_b, t_us);
        ctrl.ingest(imu);
        if (n % adcs::cfg::kStarTrackerDivider == 0) ctrl.ingest(tracker.sample(s.theta, t_us));
        ctrl.ingest(sim::tach_sample(s.W));

        // 2. Control law
        const adcs::MotorCommand cmd = ctrl.step();

        // 3. Physics over one control period (ZOH on PWM)
        double tau_ext_log = 0.0;
        for (int k = 0; k < kSubsteps; ++k) {
            const double tk = t + k * (kDt / kSubsteps);
            double tau_ext = kDisturbance;
            for (int j = 0; j < sc.n_knocks; ++j) tau_ext += sc.knocks[j].torque(tk);
            if (absd(tau_ext) > absd(tau_ext_log)) tau_ext_log = tau_ext;
            plant.step(cmd.duty, tau_ext, kDt / kSubsteps);
        }

        // 4. Verification against truth (post-step state)
        const auto& s2 = plant.state();
        const double err_deg  = -s2.theta * kRad2Deg;
        const double rate_dps = s2.w_b * kRad2Deg;
        const double t2 = t + kDt;
        if (t2 >= sc.t_eval) {
            ++eval_ticks;
            if (cmd.current_cmd >= adcs::cfg::kImax || cmd.current_cmd <= -adcs::cfg::kImax) ++sat_ticks;
            if (absd(err_deg)  > r.peak_err_deg)  r.peak_err_deg  = absd(err_deg);
            if (absd(rate_dps) > r.peak_rate_dps) r.peak_rate_dps = absd(rate_dps);
            if (absd(err_deg) > kAngleTolDeg || absd(rate_dps) > kRateTolDps) last_violation = t2;
        }
        const double rpm = s2.W * 60.0 / 6.283185307179586;
        if (absd(rpm) > r.peak_wheel_rpm) r.peak_wheel_rpm = absd(rpm);
        r.final_err_deg = err_deg;

        if (f) std::fprintf(f,
            "%.4f,%.5f,%.5f,%.5f,%.5f,%.5f,%d,%.4f,%.2f,%.2f,%d,%.5f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.6f\n",
            t2, s2.theta * kRad2Deg, ctrl.estimator().angle() * kRad2Deg, err_deg, rate_dps,
            ctrl.estimator().rate() * kRad2Deg, imu.gyro_z, ctrl.estimator().bias() * kRad2Deg,
            rpm, cmd.wheel_speed_cmd * adcs::cfg::kRadToRpm, cmd.pwm, cmd.duty,
            cmd.torque_cmd * 1e3, plant.wheel_torque() * 1e3, plant.current_draw(),
            cmd.p * 1e3, cmd.i * 1e3, cmd.d * 1e3, tau_ext_log * 1e3, plant.momentum() * 1e3);
    }
    if (f) std::fclose(f);

    r.settle_s     = last_violation - sc.t_eval;
    r.sat_fraction = eval_ticks ? static_cast<double>(sat_ticks) / eval_ticks : 0.0;
    return r;
}

// ---------------------------------------------------------------------------
void test_pid_antiwindup() {
    std::printf("\n== Unit: PID anti-windup & limits ==\n");
    adcs::Pid pid(adcs::ReactionWheelController::kGains, kDt);
    float u = 0;
    for (int k = 0; k < 20000; ++k) u = pid.update(1.0f, 0.0f);       // 20 s saturated
    check(u <= adcs::cfg::kTorqueMax + 1e-9f, "output clamped to wheel torque limit");
    check(pid.i_term() <= adcs::cfg::kIntegralLimit + 1e-9f, "integrator bounded under sustained saturation");
    check(pid.p_term() <= adcs::cfg::kKp * adcs::cfg::kErrClamp + 1e-9f, "P path uses clamped error");
    std::printf("    integrator after 20 s of saturated 57 deg error: %.3g mN m\n", pid.i_term() * 1e3);
    check(pid.i_term() == 0.0f, "no wind-up outside the integral zone");

    adcs::Pid fine(adcs::ReactionWheelController::kGains, kDt);
    for (int k = 0; k < 2000; ++k) fine.update(0.001f, 0.0f);        // 0.057 deg, inside zone
    check(fine.i_term() > 0.0f && fine.i_term() <= adcs::cfg::kIntegralLimit,
          "integrates small steady error, bounded by integrator limit");
}

void test_momentum_conservation() {
    std::printf("\n== Unit: plant conserves angular momentum ==\n");
    sim::Plant p(kTruth, {0.0, 0.01, 100.0});
    sim::Rng rng(42);
    const double H0 = p.momentum();
    double duty = 0.0;
    for (int k = 0; k < 100000; ++k) {                  // 10 s, random PWM
        if (k % 500 == 0) duty = rng.uniform() * 2.0 - 1.0;
        p.step(duty, 0.0, 1e-4);
    }
    const double drift = absd(p.momentum() - H0) / absd(H0);
    std::printf("    relative drift %.3e\n", drift);
    check(drift < 1e-9, "|dH/H| < 1e-9 with no external torque");

    sim::Plant q(kTruth, {0.0, 0.0, 100.0});
    const sim::Knock kn{0.01, 0.01, 0.5};
    const double Hq0 = q.momentum();
    for (int k = 0; k < 1000; ++k) q.step(0.0, kn.torque(k * 1e-4), 1e-4);
    const double dH = q.momentum() - Hq0;
    std::printf("    knock impulse %.6f mNms, measured dH %.6f mNms\n", kn.impulse() * 1e3, dH * 1e3);
    check(absd(dH - kn.impulse()) < 1e-3 * kn.impulse(), "knock delivers its impulse to H (0.1%)");
}

void test_estimator_bias() {
    std::printf("\n== Unit: estimator learns gyro bias ==\n");
    sim::MockGyro g{0.25, 0.067, sim::Rng(3)};
    sim::MockAttitudeSensor st{50.0, sim::Rng(4)};
    adcs::AttitudeEstimator est;
    for (int n = 0; n < 5000; ++n) {                    // 5 s stationary
        est.propagate(static_cast<float>(g.sample(0.0, 0).gyro_z) * adcs::cfg::kGyroRadPerLsb);
        if (n % adcs::cfg::kStarTrackerDivider == 0)
            est.correct(static_cast<float>(st.sample(0.0, 0).angle_urad) * 1e-6f);
    }
    const double b = est.bias() * kRad2Deg;
    std::printf("    bias estimate %.4f dps (truth 0.2500)\n", b);
    check(absd(b - 0.25) < 0.01, "bias within 0.01 dps after 5 s from a cold start");
    check(absd(est.angle() * kRad2Deg) < 0.02, "angle estimate within 0.02 deg");
}

void benchmark() {
    std::printf("\n== Benchmark: control step cost (host CPU) ==\n");
    adcs::ReactionWheelController ctrl;
    constexpr long N = 20'000'000;
    volatile int16_t sink = 0;
    adcs::ImuSample imu{0, 0, 0, 0, 1};
    const auto t0 = std::chrono::steady_clock::now();
    for (long n = 0; n < N; ++n) {
        imu.gyro_z = static_cast<int16_t>((n * 37) & 0x3FF) - 512;
        ctrl.ingest(imu);
        ctrl.ingest(adcs::WheelTach{static_cast<int16_t>(1000 + (n & 0xFF))});
        if ((n & 127) == 0) ctrl.ingest(adcs::AttitudeFix{0, static_cast<int32_t>(n & 0xFFF), 1});
        sink = ctrl.step().pwm;
    }
    const double ns = std::chrono::duration<double, std::nano>(std::chrono::steady_clock::now() - t0).count() / N;
    (void)sink;
    std::printf("    %.1f ns per tick (ingest x3 + estimator + PID + PWM map)\n", ns);
    std::printf("    sizeof(ReactionWheelController) = %zu bytes, 0 heap allocations\n",
                sizeof(adcs::ReactionWheelController));
    check(ns < 1000.0, "step cost < 1 us on host (budget at 1 kHz: 1 ms)");
}

}  // namespace

int main(int argc, char** argv) {
    const char* out_dir = argc > 1 ? argv[1] : "telemetry";
    std::printf("1-axis CubeSat reaction-wheel ADCS — ground test harness\n");
    std::printf("Gains: Kp=%.4f N m/rad  Ki=%.4f N m/(rad s)  Kd=%.4f N m s/rad  | torque limit %.1f mN m\n",
                adcs::cfg::kKp, adcs::cfg::kKi, adcs::cfg::kKd, adcs::cfg::kTorqueMax * 1e3);

    test_pid_antiwindup();
    test_momentum_conservation();
    test_estimator_bias();

    const Scenario scenarios[] = {
        {"knock_nominal",     "+2.5 mNms knock, wheel at 1000 rpm",
         1000, 0, {{3.0, 0.010,  0.5}}, 1, 3.0},
        {"knock_heavy_neg",   "-6.0 mNms knock (6.9 deg/s kick), saturates wheel torque",
         1000, 0, {{3.0, 0.012, -1.0}}, 1, 3.0},
        {"knock_zero_cross",  "+2.5 mNms knock drives wheel through 0 rpm (stiction region)",
         150,  0, {{3.0, 0.010, -0.5}}, 1, 3.0},
        {"knock_double",      "two knocks 0.4 s apart, opposite sign",
         1000, 0, {{3.0, 0.010, 0.5}, {3.4, 0.010, -0.6}}, 2, 3.0},
        {"acquire_3deg",      "3 deg initial pointing error, no knock",
         1000, 3.0, {{0, 0, 0}}, 0, 0.0},
    };

    std::printf("\n== Closed-loop scenarios: %d noise seeds each, worst case shown ==\n"
                "   settled = |err|<%.2f deg & |rate|<%.2f deg/s and stays there; requirement < %.1f s\n",
                kMonteCarloRuns, kAngleTolDeg, kRateTolDps, kSettleLimitS);
    char path[512];
    std::snprintf(path, sizeof path, "%s/summary.csv", out_dir);
    FILE* sum = std::fopen(path, "w");
    if (sum) std::fprintf(sum, "scenario,worst_settle_s,peak_err_deg,peak_rate_dps,peak_wheel_rpm,torque_sat_pct,final_err_deg,pass\n");
    std::printf("  %-18s %9s %9s %10s %11s %10s %7s\n", "scenario", "mean[s]", "worst[s]", "peak[deg]",
                "peak[dps]", "wheel[rpm]", "sat[%]");

    for (const Scenario& sc : scenarios) {
        Result r{};
        double mean = 0.0;
        for (int m = 0; m < kMonteCarloRuns; ++m) {
            const Result rm = run_scenario(sc, out_dir, 1000 + 17 * m, m == 0);   // log seed 0 only
            mean += rm.settle_s / kMonteCarloRuns;
            if (m == 0 || rm.settle_s > r.settle_s) r = rm;
        }
        const bool pass = r.settle_s < kSettleLimitS;
        std::printf("  %-18s %9.3f %9.3f %10.3f %11.3f %10.0f %7.1f  %s\n", sc.name, mean, r.settle_s,
                    r.peak_err_deg, r.peak_rate_dps, r.peak_wheel_rpm, r.sat_fraction * 100, pass ? "PASS" : "FAIL");
        if (sum) std::fprintf(sum, "%s,%.4f,%.4f,%.4f,%.1f,%.2f,%.5f,%d\n", sc.name, r.settle_s, r.peak_err_deg,
                              r.peak_rate_dps, r.peak_wheel_rpm, r.sat_fraction * 100, r.final_err_deg, pass);
        if (!pass) ++g_failures;
    }
    if (sum) std::fclose(sum);

    benchmark();

    std::printf("\nTelemetry written to %s/*.csv\n%s: %d failure(s)\n", out_dir,
                g_failures ? "FAILED" : "ALL TESTS PASSED", g_failures);
    return g_failures ? 1 : 0;
}
